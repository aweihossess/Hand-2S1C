#!/usr/bin/env python3
"""Low-tension slack take-up collector for MCP tendon motors.

This script is separate from the main desktop GUI and firmware control path.
It talks to the existing firmware text monitor over serial, enters direct motor
mode, and commands selected motors with:

    target = current_motor_abs + tighten_dir * bias

The bias is adjusted from servo current feedback. The resulting CSV is meant for
offline fitting of a tendon/motor feedforward map.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

try:
    import serial
except ImportError:  # pragma: no cover - runtime dependency message
    serial = None


DEFAULT_CONFIG = Path(__file__).with_name("config_mcp.json")
CURRENT_MA_PER_COUNT = 6.5

PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF
CMD_MOTOR_POS_ABS = 0xD3
MOTOR_COUNT = 22
MOTOR_ABS_CMD_MIN = -30719
MOTOR_ABS_CMD_MAX = 30719

SERVO_RE = re.compile(r"<<<SERVO M(\d+) motor_abs=(-?\d+) .* online=(\d+)>>>")
LOAD_RE = re.compile(
    r"<<<LOAD M(\d+) load=(-?\d+) current=(-?\d+) speed=(-?\d+) voltage=(\d+) temperature=(\d+) online=(\d+)>>>"
)
ENC_RE = re.compile(
    r"<<<ENC J(\d+) raw=(\d+) raw_valid=(\d+) mapped=(-?\d+) mapped_valid=(\d+) deg=(-?\d+(?:\.\d+)?)>>>"
)


@dataclass
class MotorConfig:
    channel: int
    name: str
    tighten_dir: int


@dataclass
class JointConfig:
    index: int
    name: str
    safe_min_deg: Optional[float] = None
    safe_max_deg: Optional[float] = None


@dataclass
class Config:
    baud: int
    period_s: float
    idle_samples: int
    i_low_counts: int
    i_high_counts: int
    tighten_step: int
    release_step: int
    bias_max_each: int
    bias_total_max: int
    max_target_step: int
    motor_abs_guard: int
    motors: List[MotorConfig]
    joints: List[JointConfig]


@dataclass
class MotorState:
    motor_abs: Optional[int] = None
    load: Optional[int] = None
    current: Optional[int] = None
    speed: Optional[int] = None
    online: bool = False
    idle_current: float = 0.0
    idle_load: float = 0.0
    bias: int = 0
    target: Optional[int] = None
    effective_current: float = 0.0
    effective_load: float = 0.0


@dataclass
class JointState:
    deg: Optional[float] = None
    mapped_valid: bool = False


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def load_config(path: Path) -> Config:
    data = json.loads(path.read_text(encoding="utf-8"))
    motors = [
        MotorConfig(
            channel=int(item["channel"]),
            name=str(item.get("name", f"M{int(item['channel']):02d}")),
            tighten_dir=1 if int(item.get("tighten_dir", 1)) >= 0 else -1,
        )
        for item in data["motors"]
    ]
    joints = [
        JointConfig(
            index=int(item["index"]),
            name=str(item.get("name", f"J{int(item['index']):02d}")),
            safe_min_deg=None if item.get("safe_min_deg") is None else float(item["safe_min_deg"]),
            safe_max_deg=None if item.get("safe_max_deg") is None else float(item["safe_max_deg"]),
        )
        for item in data.get("joints", [])
    ]
    return Config(
        baud=int(data.get("baud", 921600)),
        period_s=float(data.get("period_s", 0.02)),
        idle_samples=int(data.get("idle_samples", 40)),
        i_low_counts=int(data.get("i_low_counts", 20)),
        i_high_counts=int(data.get("i_high_counts", 45)),
        tighten_step=int(data.get("tighten_step", 1)),
        release_step=int(data.get("release_step", 4)),
        bias_max_each=int(data.get("bias_max_each", 120)),
        bias_total_max=int(data.get("bias_total_max", 240)),
        max_target_step=int(data.get("max_target_step", 160)),
        motor_abs_guard=int(data.get("motor_abs_guard", 6400)),
        motors=motors,
        joints=joints,
    )


class TextFirmwareClient:
    def __init__(self, port_name: str, baud: int, timeout: float = 0.02):
        if serial is None:
            raise RuntimeError("Missing dependency: install pyserial")
        self.port = serial.Serial(port_name, baud, timeout=timeout, write_timeout=0.2)
        self._buffer = bytearray()

    def close(self) -> None:
        self.port.close()

    def write_line(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("ascii")
        self.port.write(payload)
        self.port.flush()

    def write_binary_frame(self, frame: bytes) -> None:
        self.port.write(frame)
        self.port.flush()

    def read_available_lines(self, wait_s: float = 0.0) -> List[str]:
        deadline = time.time() + wait_s
        lines: List[str] = []
        while True:
            chunk = self.port.read(4096)
            if chunk:
                self._buffer.extend(chunk)
            while b"\n" in self._buffer:
                raw, _, rest = self._buffer.partition(b"\n")
                self._buffer = bytearray(rest)
                text = raw.decode("utf-8", errors="ignore").strip()
                if text:
                    lines.append(text)
            if wait_s <= 0.0 or time.time() >= deadline:
                break
        return lines

    def command_and_collect(self, command: str, settle_s: float = 0.03) -> List[str]:
        self.write_line(command)
        return self.read_available_lines(wait_s=settle_s)


class StartupSafetyError(RuntimeError):
    """Raised before motor output is enabled when the requested posture is unsafe."""


def build_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    frame_len = 1 + len(payload) + 1
    return bytes([PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF]) + payload + bytes([PROTOCOL_TAIL])


def build_motor_pos_abs_cmd(motor_abs: List[int]) -> bytes:
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    payload = bytearray()
    for value in motor_abs:
        v = clamp(int(value), MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX)
        payload.extend(int(v).to_bytes(2, byteorder="big", signed=True))
    return build_command_frame(CMD_MOTOR_POS_ABS, bytes(payload))


def parse_lines(
    lines: Iterable[str],
    motors: Dict[int, MotorState],
    joints: Dict[int, JointState],
) -> None:
    for line in lines:
        m = SERVO_RE.match(line)
        if m:
            ch = int(m.group(1))
            state = motors.setdefault(ch, MotorState())
            state.motor_abs = int(m.group(2))
            state.online = m.group(3) == "1"
            continue

        m = LOAD_RE.match(line)
        if m:
            ch = int(m.group(1))
            state = motors.setdefault(ch, MotorState())
            state.load = int(m.group(2))
            state.current = int(m.group(3))
            state.speed = int(m.group(4))
            state.online = m.group(7) == "1"
            continue

        m = ENC_RE.match(line)
        if m:
            idx = int(m.group(1))
            state = joints.setdefault(idx, JointState())
            state.deg = float(m.group(6))
            state.mapped_valid = m.group(5) == "1"


def refresh_snapshot(
    client: TextFirmwareClient,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
) -> None:
    lines: List[str] = []
    lines.extend(client.command_and_collect("servo", settle_s=0.025))
    lines.extend(client.command_and_collect("load", settle_s=0.025))
    lines.extend(client.command_and_collect("encoder", settle_s=0.025))
    lines.extend(client.read_available_lines(wait_s=0.005))
    parse_lines(lines, motor_states, joint_states)


def estimate_idle_current(
    client: TextFirmwareClient,
    cfg: Config,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
) -> None:
    current_samples: Dict[int, List[int]] = {m.channel: [] for m in cfg.motors}
    load_samples: Dict[int, List[int]] = {m.channel: [] for m in cfg.motors}
    print(f"Estimating idle current/load from {cfg.idle_samples} samples...")
    for _ in range(cfg.idle_samples):
        refresh_snapshot(client, motor_states, joint_states)
        for motor in cfg.motors:
            state = motor_states.setdefault(motor.channel, MotorState())
            if state.current is not None:
                current_samples[motor.channel].append(state.current)
            if state.load is not None:
                load_samples[motor.channel].append(state.load)
        time.sleep(cfg.period_s)

    for motor in cfg.motors:
        current_values = current_samples[motor.channel]
        load_values = load_samples[motor.channel]
        state = motor_states.setdefault(motor.channel, MotorState())
        state.idle_current = sum(current_values) / len(current_values) if current_values else 0.0
        state.idle_load = sum(load_values) / len(load_values) if load_values else 0.0
        print(
            f"  {motor.name} M{motor.channel:02d}: "
            f"idle_current={state.idle_current:.1f} idle_load={state.idle_load:.1f}"
        )

    if all(current_samples[m.channel] and all(v == 0 for v in current_samples[m.channel]) for m in cfg.motors):
        print(
            "WARNING: selected servo current feedback is exactly zero in all idle samples. "
            "The take-up loop will still be angle-guarded, but current may not stop tightening. "
            "Check firmware current telemetry or use very small tie-current/tie-step values."
        )


def enforce_total_bias_budget(cfg: Config, motor_states: Dict[int, MotorState]) -> None:
    total = sum(max(0, motor_states[m.channel].bias) for m in cfg.motors)
    if total <= cfg.bias_total_max or total <= 0:
        return
    scale = cfg.bias_total_max / total
    for motor in cfg.motors:
        state = motor_states[motor.channel]
        state.bias = int(math.floor(state.bias * scale))


def update_motor_targets(cfg: Config, motor_states: Dict[int, MotorState]) -> None:
    for motor in cfg.motors:
        state = motor_states.setdefault(motor.channel, MotorState())
        if state.current is None or state.motor_abs is None or not state.online:
            state.target = None
            continue

        state.effective_current = abs(float(state.current) - state.idle_current)
        if state.effective_current < cfg.i_low_counts:
            state.bias += cfg.tighten_step
        elif state.effective_current > cfg.i_high_counts:
            state.bias -= cfg.release_step

        state.bias = clamp(state.bias, 0, cfg.bias_max_each)

    enforce_total_bias_budget(cfg, motor_states)

    for motor in cfg.motors:
        state = motor_states[motor.channel]
        if state.motor_abs is None or not state.online:
            state.target = None
            continue
        raw_target = state.motor_abs + motor.tighten_dir * state.bias
        raw_target = clamp(raw_target, -cfg.motor_abs_guard, cfg.motor_abs_guard)
        raw_target = clamp(raw_target, state.motor_abs - cfg.max_target_step, state.motor_abs + cfg.max_target_step)
        state.target = raw_target


def build_full_motor_targets(motor_states: Dict[int, MotorState]) -> List[int]:
    targets = [0] * MOTOR_COUNT
    for ch in range(MOTOR_COUNT):
        state = motor_states.setdefault(ch, MotorState())
        if state.target is not None:
            targets[ch] = int(state.target)
        elif state.motor_abs is not None:
            targets[ch] = int(state.motor_abs)
    return targets


def send_targets(client: TextFirmwareClient, motor_states: Dict[int, MotorState]) -> None:
    client.write_binary_frame(build_motor_pos_abs_cmd(build_full_motor_targets(motor_states)))


def send_selected_targets(client: TextFirmwareClient, motor_states: Dict[int, MotorState]) -> None:
    send_targets(client, motor_states)


def clear_unselected_motor_targets(cfg: Config, motor_states: Dict[int, MotorState]) -> None:
    selected = {m.channel for m in cfg.motors}
    for ch, state in motor_states.items():
        if ch not in selected:
            state.target = None


def set_hold_targets_for_selected(cfg: Config, motor_states: Dict[int, MotorState]) -> None:
    for motor in cfg.motors:
        state = motor_states[motor.channel]
        if state.target is None and state.motor_abs is not None:
            state.target = state.motor_abs


def build_csv_header(cfg: Config) -> List[str]:
    header = ["time_s", "mode", "action", "phase", "max_angle_error_deg", "safe_violation"]
    for joint in cfg.joints:
        header.extend([f"{joint.name.lower()}_deg", f"{joint.name.lower()}_valid"])
    for motor in cfg.motors:
        prefix = f"m{motor.channel:02d}"
        header.extend(
            [
                f"{prefix}_name",
                f"{prefix}_abs",
                f"{prefix}_current",
                f"{prefix}_current_mA",
                f"{prefix}_idle_current",
                f"{prefix}_effective_current",
                f"{prefix}_effective_current_mA",
                f"{prefix}_torque_proxy_mA",
                f"{prefix}_load",
                f"{prefix}_idle_load",
                f"{prefix}_effective_load",
                f"{prefix}_speed",
                f"{prefix}_bias",
                f"{prefix}_target",
                f"{prefix}_online",
            ]
        )
    return header


def build_csv_row(
    cfg: Config,
    start_time: float,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
    mode: str = "",
    action: str = "",
    phase: str = "",
    max_angle_error_deg: Optional[float] = None,
    safe_violation: bool = False,
) -> Dict[str, object]:
    row: Dict[str, object] = {
        "time_s": time.time() - start_time,
        "mode": mode,
        "action": action,
        "phase": phase,
        "max_angle_error_deg": "" if max_angle_error_deg is None else f"{max_angle_error_deg:.4f}",
        "safe_violation": int(bool(safe_violation)),
    }
    for joint in cfg.joints:
        state = joint_states.setdefault(joint.index, JointState())
        row[f"{joint.name.lower()}_deg"] = "" if state.deg is None else f"{state.deg:.4f}"
        row[f"{joint.name.lower()}_valid"] = int(state.mapped_valid)
    for motor in cfg.motors:
        state = motor_states.setdefault(motor.channel, MotorState())
        prefix = f"m{motor.channel:02d}"
        row[f"{prefix}_name"] = motor.name
        row[f"{prefix}_abs"] = "" if state.motor_abs is None else state.motor_abs
        row[f"{prefix}_current"] = "" if state.current is None else state.current
        row[f"{prefix}_current_mA"] = "" if state.current is None else f"{state.current * CURRENT_MA_PER_COUNT:.2f}"
        row[f"{prefix}_idle_current"] = f"{state.idle_current:.2f}"
        row[f"{prefix}_effective_current"] = f"{state.effective_current:.2f}"
        row[f"{prefix}_effective_current_mA"] = f"{state.effective_current * CURRENT_MA_PER_COUNT:.2f}"
        row[f"{prefix}_torque_proxy_mA"] = f"{state.effective_current * CURRENT_MA_PER_COUNT:.2f}"
        row[f"{prefix}_load"] = "" if state.load is None else state.load
        row[f"{prefix}_idle_load"] = f"{state.idle_load:.2f}"
        row[f"{prefix}_effective_load"] = f"{state.effective_load:.2f}"
        row[f"{prefix}_speed"] = "" if state.speed is None else state.speed
        row[f"{prefix}_bias"] = state.bias
        row[f"{prefix}_target"] = "" if state.target is None else state.target
        row[f"{prefix}_online"] = int(state.online)
    return row


def print_status(cfg: Config, motor_states: Dict[int, MotorState], joint_states: Dict[int, JointState]) -> None:
    joint_bits = []
    for joint in cfg.joints:
        state = joint_states.setdefault(joint.index, JointState())
        if state.deg is None:
            joint_bits.append(f"{joint.name}=na")
        else:
            joint_bits.append(f"{joint.name}={state.deg:.1f}")
    motor_bits = []
    for motor in cfg.motors:
        state = motor_states.setdefault(motor.channel, MotorState())
        motor_bits.append(
            f"M{motor.channel:02d} abs={state.motor_abs} load={state.load} cur={state.current} "
            f"ieff={state.effective_current:.1f} leff={state.effective_load:.1f} "
            f"bias={state.bias} tgt={state.target}"
        )
    print(" | ".join(joint_bits + motor_bits))


def prepare_text_monitor(client: TextFirmwareClient) -> None:
    client.read_available_lines(wait_s=0.1)
    client.command_and_collect("text", settle_s=0.05)


def enable_direct_output(client: TextFirmwareClient, skip_start: bool) -> None:
    if not skip_start:
        client.command_and_collect("start", settle_s=0.05)
    client.command_and_collect("direct", settle_s=0.05)


def confirm_start_if_requested(confirm_start: bool) -> None:
    if not confirm_start:
        return
    print()
    print("CONFIRMATION REQUIRED")
    print("Motors are still stopped. Prepare your hand/load now.")
    print("Type YES and press Enter to enable motor output, or press Ctrl+C to cancel.")
    try:
        answer = input("> ")
    except EOFError as exc:
        raise StartupSafetyError("No confirmation received; refusing to enable output.") from exc
    if answer.strip() != "YES":
        raise StartupSafetyError("Confirmation was not YES; refusing to enable output.")


def selected_joint_targets(
    cfg: Config,
    joint_states: Dict[int, JointState],
    target_j00: Optional[float],
    target_j01: Optional[float],
) -> Dict[int, float]:
    targets: Dict[int, float] = {}
    for joint in cfg.joints:
        explicit: Optional[float] = None
        if joint.index == 0:
            explicit = target_j00
        elif joint.index == 1:
            explicit = target_j01

        if explicit is not None:
            targets[joint.index] = float(explicit)
            continue

        state = joint_states.setdefault(joint.index, JointState())
        if state.deg is None or not state.mapped_valid:
            raise RuntimeError(
                f"Cannot capture target for {joint.name}: no valid encoder angle. "
                "Pass --target-j00/--target-j01 or check encoder telemetry."
            )
        targets[joint.index] = float(state.deg)
    return targets


def max_joint_error(
    joint_states: Dict[int, JointState],
    targets: Dict[int, float],
) -> float:
    max_err = 0.0
    for joint_idx, target_deg in targets.items():
        state = joint_states.setdefault(joint_idx, JointState())
        if state.deg is None or not state.mapped_valid:
            return float("inf")
        max_err = max(max_err, abs(float(state.deg) - target_deg))
    return max_err


def build_joint_safe_bounds(
    cfg: Config,
    targets: Dict[int, float],
    safe_window_deg: Optional[float],
    j00_safe_min: Optional[float],
    j00_safe_max: Optional[float],
    j01_safe_min: Optional[float],
    j01_safe_max: Optional[float],
) -> Dict[int, tuple]:
    bounds: Dict[int, tuple] = {}
    for joint in cfg.joints:
        lo = joint.safe_min_deg
        hi = joint.safe_max_deg

        if safe_window_deg is not None:
            target = targets.get(joint.index)
            if target is not None:
                lo = target - abs(float(safe_window_deg))
                hi = target + abs(float(safe_window_deg))

        if joint.index == 0:
            if j00_safe_min is not None:
                lo = float(j00_safe_min)
            if j00_safe_max is not None:
                hi = float(j00_safe_max)
        elif joint.index == 1:
            if j01_safe_min is not None:
                lo = float(j01_safe_min)
            if j01_safe_max is not None:
                hi = float(j01_safe_max)

        if lo is not None or hi is not None:
            bounds[joint.index] = (lo, hi)
    return bounds


def joint_safety_violation(
    joint_states: Dict[int, JointState],
    bounds: Dict[int, tuple],
) -> bool:
    for joint_idx, (lo, hi) in bounds.items():
        state = joint_states.setdefault(joint_idx, JointState())
        if state.deg is None or not state.mapped_valid:
            return True
        if lo is not None and state.deg < lo:
            return True
        if hi is not None and state.deg > hi:
            return True
    return False


def describe_joint_bounds_violation(
    joint_states: Dict[int, JointState],
    bounds: Dict[int, tuple],
) -> Optional[str]:
    for joint_idx, (lo, hi) in sorted(bounds.items()):
        state = joint_states.setdefault(joint_idx, JointState())
        if state.deg is None or not state.mapped_valid:
            return f"J{joint_idx:02d} has no valid encoder angle"
        if lo is not None and state.deg < lo:
            return f"J{joint_idx:02d}={state.deg:.2f} is below safe_min={lo:.2f}"
        if hi is not None and state.deg > hi:
            return f"J{joint_idx:02d}={state.deg:.2f} is above safe_max={hi:.2f}"
    return None


def describe_target_bounds_violation(
    targets: Dict[int, float],
    bounds: Dict[int, tuple],
) -> Optional[str]:
    for joint_idx, target in sorted(targets.items()):
        lo, hi = bounds.get(joint_idx, (None, None))
        if lo is not None and target < lo:
            return f"J{joint_idx:02d}_target={target:.2f} is below safe_min={lo:.2f}"
        if hi is not None and target > hi:
            return f"J{joint_idx:02d}_target={target:.2f} is above safe_max={hi:.2f}"
    return None


def update_effective_currents(cfg: Config, motor_states: Dict[int, MotorState]) -> None:
    for motor in cfg.motors:
        state = motor_states.setdefault(motor.channel, MotorState())
        if state.current is None:
            state.effective_current = 0.0
        else:
            state.effective_current = abs(float(state.current) - state.idle_current)
        if state.load is None:
            state.effective_load = 0.0
        else:
            state.effective_load = abs(float(state.load) - state.idle_load)


def motor_target_from_relative_step(cfg: Config, state: MotorState, motor: MotorConfig, step: int) -> Optional[int]:
    if state.motor_abs is None or not state.online:
        return None
    raw_target = state.motor_abs + motor.tighten_dir * int(step)
    raw_target = clamp(raw_target, -cfg.motor_abs_guard, cfg.motor_abs_guard)
    raw_target = clamp(raw_target, state.motor_abs - cfg.max_target_step, state.motor_abs + cfg.max_target_step)
    return raw_target


def run_collect_loop(
    client: TextFirmwareClient,
    cfg: Config,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
    out_path: Path,
    duration_s: float,
    dry_run: bool,
    skip_start: bool,
    confirm_start: bool,
    stop_requested_cb,
) -> bool:
    output_enabled = False
    if not dry_run:
        confirm_start_if_requested(confirm_start)
        enable_direct_output(client, skip_start=skip_start)
        output_enabled = True
    header = build_csv_header(cfg)
    print(f"Writing CSV: {out_path}")
    print("Dry run: targets will not be sent and output will not be enabled." if dry_run else "Commanding motors in direct mode.")

    start_time = time.time()
    next_tick = start_time
    next_print = start_time
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        while not stop_requested_cb():
            now = time.time()
            if duration_s > 0 and now - start_time >= duration_s:
                break
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += cfg.period_s

            refresh_snapshot(client, motor_states, joint_states)
            update_motor_targets(cfg, motor_states)
            clear_unselected_motor_targets(cfg, motor_states)
            if not dry_run:
                send_selected_targets(client, motor_states)
            writer.writerow(
                build_csv_row(
                    cfg,
                    start_time,
                    motor_states,
                    joint_states,
                    mode="collect",
                    action="takeup",
                )
            )

            if now >= next_print:
                print_status(cfg, motor_states, joint_states)
                next_print = now + 1.0
    return output_enabled


def run_hold_probe_loop(
    client: TextFirmwareClient,
    cfg: Config,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
    out_path: Path,
    duration_s: float,
    dry_run: bool,
    skip_start: bool,
    confirm_start: bool,
    stop_requested_cb,
) -> bool:
    refresh_snapshot(client, motor_states, joint_states)
    for motor in cfg.motors:
        state = motor_states.setdefault(motor.channel, MotorState())
        if state.motor_abs is None or not state.online:
            raise StartupSafetyError(f"Cannot hold M{motor.channel:02d}: no valid online motor_abs.")
        state.bias = 0
        state.target = state.motor_abs

    output_enabled = False
    header = build_csv_header(cfg)
    print(f"Writing CSV: {out_path}")
    print("Hold-probe: selected motors hold their current positions while current/load are recorded.")
    print("Pull or release the tendon externally and watch cur/load/leff. Press Ctrl+C to stop.")
    if not dry_run:
        confirm_start_if_requested(confirm_start)
        enable_direct_output(client, skip_start=skip_start)
        output_enabled = True
        send_selected_targets(client, motor_states)
    else:
        print("Dry run: hold targets will not be sent and output will not be enabled.")

    start_time = time.time()
    next_tick = start_time
    next_print = start_time
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        while not stop_requested_cb():
            now = time.time()
            if duration_s > 0 and now - start_time >= duration_s:
                break
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += cfg.period_s

            refresh_snapshot(client, motor_states, joint_states)
            update_effective_currents(cfg, motor_states)
            for motor in cfg.motors:
                state = motor_states.setdefault(motor.channel, MotorState())
                if state.target is None and state.motor_abs is not None:
                    state.target = state.motor_abs
            clear_unselected_motor_targets(cfg, motor_states)
            if not dry_run:
                send_selected_targets(client, motor_states)

            writer.writerow(
                build_csv_row(
                    cfg,
                    start_time,
                    motor_states,
                    joint_states,
                    mode="hold-probe",
                    action="hold",
                )
            )

            if now >= next_print:
                print_status(cfg, motor_states, joint_states)
                next_print = now + 0.5
    return output_enabled


def run_pulse_probe_loop(
    client: TextFirmwareClient,
    cfg: Config,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
    out_path: Path,
    duration_s: float,
    dry_run: bool,
    skip_start: bool,
    confirm_start: bool,
    pulse_step: int,
    pulse_max_travel: int,
    pulse_current_stop: float,
    pulse_load_stop: float,
    pulse_angle_stop: float,
    pulse_stage_prompt: bool,
    pulse_stage_travel: int,
    pulse_phase_sequence: str,
    stop_requested_cb,
) -> bool:
    if len(cfg.motors) != 1:
        raise StartupSafetyError("pulse-probe expects exactly one motor in the config.")

    motor = cfg.motors[0]
    refresh_snapshot(client, motor_states, joint_states)
    state = motor_states.setdefault(motor.channel, MotorState())
    if state.motor_abs is None or not state.online:
        raise StartupSafetyError(f"Cannot pulse M{motor.channel:02d}: no valid online motor_abs.")

    initial_abs = int(state.motor_abs)
    initial_joints: Dict[int, float] = {}
    for joint in cfg.joints:
        joint_state = joint_states.setdefault(joint.index, JointState())
        if joint_state.deg is not None and joint_state.mapped_valid:
            initial_joints[joint.index] = float(joint_state.deg)

    state.bias = 0
    state.target = initial_abs
    header = build_csv_header(cfg)
    print(f"Writing CSV: {out_path}")
    print(
        f"Pulse-probe M{motor.channel:02d}: start_abs={initial_abs}, "
        f"tighten_dir={motor.tighten_dir}, step={pulse_step}, max_travel={pulse_max_travel}."
    )
    print(
        "The motor will move in small tightening steps and stop on max travel, "
        "load/current threshold, angle change threshold, Ctrl+C, or duration."
    )
    phases = [p.strip() for p in pulse_phase_sequence.split(",") if p.strip()]
    if not phases and pulse_stage_prompt:
        phases = ["loose", "loaded"]
    staged = len(phases) > 1

    if staged:
        print(
            f"Staged mode: phases={phases}. Each non-final phase runs for "
            f"{pulse_stage_travel} counts, then holds and waits for confirmation."
        )

    output_enabled = False
    if not dry_run:
        confirm_start_if_requested(confirm_start)
        enable_direct_output(client, skip_start=skip_start)
        output_enabled = True
        send_selected_targets(client, motor_states)
    else:
        print("Dry run: pulse targets will not be sent and output will not be enabled.")

    start_time = time.time()
    next_tick = start_time
    next_print = start_time
    cumulative_cmd = 0
    peak_current = 0.0
    peak_load = 0.0
    stop_reason = ""
    stage_index = 0
    phase = phases[stage_index] if staged else ""

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        while not stop_requested_cb():
            now = time.time()
            if duration_s > 0 and now - start_time >= duration_s:
                stop_reason = "duration_stop"
                break
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += cfg.period_s

            refresh_snapshot(client, motor_states, joint_states)
            update_effective_currents(cfg, motor_states)
            state = motor_states.setdefault(motor.channel, MotorState())
            peak_current = max(peak_current, state.effective_current)
            peak_load = max(peak_load, state.effective_load)

            angle_delta = 0.0
            for joint_idx, initial_deg in initial_joints.items():
                joint_state = joint_states.setdefault(joint_idx, JointState())
                if joint_state.deg is not None and joint_state.mapped_valid:
                    angle_delta = max(angle_delta, abs(float(joint_state.deg) - initial_deg))

            action = "pulse"
            if not state.online or state.motor_abs is None:
                stop_reason = "motor_offline_stop"
            elif staged and stage_index < (len(phases) - 1) and cumulative_cmd >= max(1, int(pulse_stage_travel)) * (stage_index + 1):
                action = "stage_pause"
                state.target = state.motor_abs if state.motor_abs is not None else None
                clear_unselected_motor_targets(cfg, motor_states)
                if not dry_run:
                    send_selected_targets(client, motor_states)
                writer.writerow(
                    build_csv_row(
                        cfg,
                        start_time,
                        motor_states,
                        joint_states,
                        mode="pulse-probe",
                        action=action,
                        phase=phase,
                        max_angle_error_deg=angle_delta,
                    )
                )
                print_status(cfg, motor_states, joint_states)
                next_phase = phases[stage_index + 1]
                expected = str(stage_index + 2)
                print(
                    f"  action={action} travel_cmd={cumulative_cmd} "
                    f"phase={phase}. Prepare phase '{next_phase}', then type {expected} and press Enter."
                )
                try:
                    answer = input(f"phase {expected}> ")
                except EOFError as exc:
                    raise StartupSafetyError("No phase confirmation received; stopping pulse-probe.") from exc
                if answer.strip() != expected:
                    stop_reason = "stage_cancel_stop"
                else:
                    stage_index += 1
                    phase = phases[stage_index]
                    next_tick = time.time()
                    next_print = next_tick
                    continue
            elif pulse_current_stop > 0 and state.effective_current >= pulse_current_stop:
                stop_reason = f"current_stop_{state.effective_current:.1f}"
            elif pulse_load_stop > 0 and state.effective_load >= pulse_load_stop:
                stop_reason = f"load_stop_{state.effective_load:.1f}"
            elif pulse_angle_stop > 0 and angle_delta >= pulse_angle_stop:
                stop_reason = f"angle_stop_{angle_delta:.1f}"
            elif cumulative_cmd >= pulse_max_travel:
                stop_reason = "travel_stop"

            if stop_reason:
                action = stop_reason
                state.target = state.motor_abs if state.motor_abs is not None else None
            else:
                step = min(max(1, abs(int(pulse_step))), max(0, int(pulse_max_travel) - cumulative_cmd))
                state.target = motor_target_from_relative_step(cfg, state, motor, step)
                cumulative_cmd += step
                state.bias = cumulative_cmd
                action = f"pulse_m{motor.channel:02d}"

            clear_unselected_motor_targets(cfg, motor_states)
            if not dry_run:
                send_selected_targets(client, motor_states)

            writer.writerow(
                build_csv_row(
                    cfg,
                    start_time,
                    motor_states,
                    joint_states,
                    mode="pulse-probe",
                    action=action,
                    phase=phase,
                    max_angle_error_deg=angle_delta,
                    safe_violation=bool(stop_reason),
                )
            )

            if now >= next_print:
                print_status(cfg, motor_states, joint_states)
                print(
                    f"  action={action} travel_cmd={cumulative_cmd} "
                    f"phase={phase or 'single'} angle_delta={angle_delta:.2f} "
                    f"peak_cur={peak_current:.1f} peak_load={peak_load:.1f}"
                )
                next_print = now + 0.5

            if stop_reason:
                break

    if stop_reason:
        print(
            f"Pulse-probe stopped: {stop_reason}. "
            f"commanded_travel={cumulative_cmd}, peak_current={peak_current:.1f}, peak_load={peak_load:.1f}"
        )
    return output_enabled


def run_tie_angle_loop(
    client: TextFirmwareClient,
    cfg: Config,
    motor_states: Dict[int, MotorState],
    joint_states: Dict[int, JointState],
    out_path: Path,
    duration_s: float,
    dry_run: bool,
    stop_requested_cb,
    target_j00: Optional[float],
    target_j01: Optional[float],
    angle_tolerance_deg: float,
    safe_window_deg: Optional[float],
    j00_safe_min: Optional[float],
    j00_safe_max: Optional[float],
    j01_safe_min: Optional[float],
    j01_safe_max: Optional[float],
    tie_current_counts: int,
    tie_step: int,
    angle_release_step: int,
    ignore_target_drift: bool,
    skip_start: bool,
    confirm_start: bool,
) -> bool:
    refresh_snapshot(client, motor_states, joint_states)
    targets = selected_joint_targets(cfg, joint_states, target_j00, target_j01)
    safe_bounds = build_joint_safe_bounds(
        cfg,
        targets,
        safe_window_deg,
        j00_safe_min,
        j00_safe_max,
        j01_safe_min,
        j01_safe_max,
    )
    target_text = ", ".join(f"J{idx:02d}={deg:.2f}" for idx, deg in sorted(targets.items()))
    print(f"Tie-angle target: {target_text}")
    if safe_bounds:
        bounds_text = ", ".join(
            f"J{idx:02d}=[{('-inf' if lo is None else f'{lo:.2f}')}, {('inf' if hi is None else f'{hi:.2f}')} ]"
            for idx, (lo, hi) in sorted(safe_bounds.items())
        )
        print(f"Absolute joint safe bounds: {bounds_text}")
    else:
        print("Absolute joint safe bounds: disabled. Use --safe-window or --j00-safe-min/--j00-safe-max.")

    target_bad = describe_target_bounds_violation(targets, safe_bounds)
    if target_bad:
        raise StartupSafetyError(
            f"Refusing to enable output: target posture violates absolute safety bounds ({target_bad})."
        )
    current_bad = describe_joint_bounds_violation(joint_states, safe_bounds)
    if current_bad:
        raise StartupSafetyError(
            f"Refusing to enable output: current posture violates absolute safety bounds ({current_bad})."
        )

    initial_err = max_joint_error(joint_states, targets)
    if not ignore_target_drift and (not math.isfinite(initial_err) or initial_err > angle_tolerance_deg):
        raise StartupSafetyError(
            f"Refusing to enable output: current posture is {initial_err:.2f} deg from target, "
            f"larger than angle_tolerance={angle_tolerance_deg:.2f}. Move the finger to target first."
        )

    if ignore_target_drift:
        print(
            "Loose-tendon take-up: target drift release is disabled. "
            "The tool still releases on absolute joint safety bounds."
        )
    else:
        print(
            "Move/hold the finger near this posture. The tool tightens one tendon at a time "
            "and releases if the joint angle drifts."
    )
    if not dry_run:
        confirm_start_if_requested(confirm_start)
        enable_direct_output(client, skip_start=skip_start)
        output_enabled = True
    else:
        print("Dry run: targets will not be sent and output will not be enabled.")
        output_enabled = False

    header = build_csv_header(cfg)
    start_time = time.time()
    next_tick = start_time
    next_print = start_time
    active_motor_index = 0

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        while not stop_requested_cb():
            now = time.time()
            if duration_s > 0 and now - start_time >= duration_s:
                break
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += cfg.period_s

            refresh_snapshot(client, motor_states, joint_states)
            update_effective_currents(cfg, motor_states)
            err = max_joint_error(joint_states, targets)
            safe_bad = joint_safety_violation(joint_states, safe_bounds) if safe_bounds else False
            action = "hold"

            if safe_bad:
                action = "absolute_safe_release"
                for motor in cfg.motors:
                    state = motor_states.setdefault(motor.channel, MotorState())
                    release = min(max(0, state.bias), max(1, angle_release_step))
                    state.bias -= release
                    if release > 0:
                        state.target = motor_target_from_relative_step(cfg, state, motor, -release)
                    else:
                        state.target = state.motor_abs if state.motor_abs is not None else None
            elif not ignore_target_drift and (not math.isfinite(err) or err > angle_tolerance_deg):
                action = "target_drift_release"
                for motor in cfg.motors:
                    state = motor_states.setdefault(motor.channel, MotorState())
                    release = min(max(0, state.bias), max(1, angle_release_step))
                    state.bias -= release
                    if release > 0:
                        state.target = motor_target_from_relative_step(cfg, state, motor, -release)
                    else:
                        state.target = state.motor_abs if state.motor_abs is not None else None
            else:
                candidate: Optional[MotorConfig] = None
                motor_count = len(cfg.motors)
                for offset in range(motor_count):
                    motor = cfg.motors[(active_motor_index + offset) % motor_count]
                    state = motor_states.setdefault(motor.channel, MotorState())
                    if not state.online or state.motor_abs is None or state.current is None:
                        continue
                    if state.effective_current < tie_current_counts and state.bias < cfg.bias_max_each:
                        candidate = motor
                        active_motor_index = (active_motor_index + offset + 1) % motor_count
                        break

                if candidate is None:
                    action = "tied_hold"
                    set_hold_targets_for_selected(cfg, motor_states)
                else:
                    total_bias = sum(max(0, motor_states[m.channel].bias) for m in cfg.motors)
                    budget_left = max(0, cfg.bias_total_max - total_bias)
                    step = min(max(1, tie_step), cfg.bias_max_each - motor_states[candidate.channel].bias, budget_left)
                    if step <= 0:
                        action = "bias_budget_hold"
                    else:
                        state = motor_states[candidate.channel]
                        state.bias += step
                        state.target = motor_target_from_relative_step(cfg, state, candidate, step)
                        action = f"tighten_m{candidate.channel:02d}"
                        for motor in cfg.motors:
                            if motor.channel != candidate.channel:
                                other = motor_states.setdefault(motor.channel, MotorState())
                                other.target = other.motor_abs if other.motor_abs is not None else None

            clear_unselected_motor_targets(cfg, motor_states)
            if not dry_run:
                send_selected_targets(client, motor_states)

            writer.writerow(
                build_csv_row(
                    cfg,
                    start_time,
                    motor_states,
                    joint_states,
                    mode="tie-angle",
                    action=action,
                    max_angle_error_deg=err,
                    safe_violation=safe_bad,
                )
            )

            if now >= next_print:
                print_status(cfg, motor_states, joint_states)
                print(f"  action={action} max_angle_error={err:.3f} deg")
                next_print = now + 1.0
    return output_enabled


def main() -> int:
    parser = argparse.ArgumentParser(description="Low-tension slack take-up collector")
    parser.add_argument("--port", required=True, help="Firmware serial port, for example COM10")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="JSON config path")
    parser.add_argument(
        "--mode",
        choices=["collect", "tie-angle", "hold-probe", "pulse-probe"],
        default="collect",
        help=(
            "collect: low-tension data collection; "
            "tie-angle: tighten slack while holding a target posture; "
            "hold-probe: hold current motor position and record external-load response; "
            "pulse-probe: step one motor while recording load/current peaks"
        ),
    )
    parser.add_argument("--baud", type=int, help="Override baud from config")
    parser.add_argument("--duration", type=float, default=60.0, help="Run duration in seconds; <=0 runs until Ctrl+C")
    parser.add_argument("--out", type=Path, default=Path("run_data/low_tension_collect.csv"), help="CSV output path")
    parser.add_argument("--dry-run", action="store_true", help="Compute and record but do not command motors")
    parser.add_argument("--skip-start", action="store_true", help="Do not send START before DIRECT")
    parser.add_argument("--confirm-start", action="store_true", help="Require typing YES before motor output is enabled")
    parser.add_argument("--target-j00", type=float, help="Tie-angle target for J00. Default: capture current angle")
    parser.add_argument("--target-j01", type=float, help="Tie-angle target for J01. Default: capture current angle")
    parser.add_argument("--angle-tolerance", type=float, default=1.5, help="Max allowed angle drift in tie-angle mode")
    parser.add_argument(
        "--safe-window",
        type=float,
        help="Absolute safety window around the target for all configured joints. Overrides config joint bounds.",
    )
    parser.add_argument("--j00-safe-min", type=float, help="Absolute J00 lower safety bound")
    parser.add_argument("--j00-safe-max", type=float, help="Absolute J00 upper safety bound")
    parser.add_argument("--j01-safe-min", type=float, help="Absolute J01 lower safety bound")
    parser.add_argument("--j01-safe-max", type=float, help="Absolute J01 upper safety bound")
    parser.add_argument("--tie-current", type=int, default=25, help="Effective current target for tie-angle mode")
    parser.add_argument("--tie-step", type=int, default=2, help="Motor counts per tie-angle tightening step")
    parser.add_argument("--angle-release-step", type=int, default=8, help="Release step if angle drifts in tie-angle mode")
    parser.add_argument("--pulse-step", type=int, default=40, help="Motor counts per pulse-probe step")
    parser.add_argument("--pulse-max-travel", type=int, default=500, help="Max cumulative pulse-probe travel in motor counts")
    parser.add_argument(
        "--pulse-current-stop",
        type=float,
        default=0.0,
        help="Stop pulse-probe when effective current reaches this value; <=0 disables",
    )
    parser.add_argument(
        "--pulse-load-stop",
        type=float,
        default=0.0,
        help="Stop pulse-probe when effective load reaches this value; <=0 disables",
    )
    parser.add_argument(
        "--pulse-angle-stop",
        type=float,
        default=0.0,
        help="Stop pulse-probe when any configured joint changes this many degrees; <=0 disables",
    )
    parser.add_argument(
        "--pulse-stage-prompt",
        action="store_true",
        help="Pause pulse-probe after the first stage and wait for typing 2 before continuing",
    )
    parser.add_argument(
        "--pulse-stage-travel",
        type=int,
        default=4096,
        help="Cumulative travel for the first pulse-probe stage before prompting",
    )
    parser.add_argument(
        "--pulse-phase-sequence",
        default="",
        help="Comma-separated staged pulse-probe phase labels, for example loaded_1,loose,loaded_2",
    )
    parser.add_argument(
        "--ignore-target-drift",
        action="store_true",
        help="Disable relative target drift release in tie-angle mode; absolute joint safety bounds still apply",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.baud:
        cfg.baud = args.baud

    args.out.parent.mkdir(parents=True, exist_ok=True)
    motor_states: Dict[int, MotorState] = {m.channel: MotorState() for m in cfg.motors}
    joint_states: Dict[int, JointState] = {j.index: JointState() for j in cfg.joints}
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    client = TextFirmwareClient(args.port, cfg.baud)
    output_enabled = False
    try:
        prepare_text_monitor(client)
        estimate_idle_current(client, cfg, motor_states, joint_states)
        if args.mode == "collect":
            output_enabled = run_collect_loop(
                client,
                cfg,
                motor_states,
                joint_states,
                args.out,
                args.duration,
                args.dry_run,
                args.skip_start,
                args.confirm_start,
                lambda: stop_requested,
            )
        elif args.mode == "tie-angle":
            output_enabled = run_tie_angle_loop(
                client,
                cfg,
                motor_states,
                joint_states,
                args.out,
                args.duration,
                args.dry_run,
                lambda: stop_requested,
                target_j00=args.target_j00,
                target_j01=args.target_j01,
                angle_tolerance_deg=args.angle_tolerance,
                safe_window_deg=args.safe_window,
                j00_safe_min=args.j00_safe_min,
                j00_safe_max=args.j00_safe_max,
                j01_safe_min=args.j01_safe_min,
                j01_safe_max=args.j01_safe_max,
                tie_current_counts=args.tie_current,
                tie_step=args.tie_step,
                angle_release_step=args.angle_release_step,
                ignore_target_drift=args.ignore_target_drift,
                skip_start=args.skip_start,
                confirm_start=args.confirm_start,
            )
        elif args.mode == "hold-probe":
            output_enabled = run_hold_probe_loop(
                client,
                cfg,
                motor_states,
                joint_states,
                args.out,
                args.duration,
                args.dry_run,
                args.skip_start,
                args.confirm_start,
                lambda: stop_requested,
            )
        else:
            output_enabled = run_pulse_probe_loop(
                client,
                cfg,
                motor_states,
                joint_states,
                args.out,
                args.duration,
                args.dry_run,
                args.skip_start,
                args.confirm_start,
                pulse_step=args.pulse_step,
                pulse_max_travel=args.pulse_max_travel,
                pulse_current_stop=args.pulse_current_stop,
                pulse_load_stop=args.pulse_load_stop,
                pulse_angle_stop=args.pulse_angle_stop,
                pulse_stage_prompt=args.pulse_stage_prompt,
                pulse_stage_travel=args.pulse_stage_travel,
                pulse_phase_sequence=args.pulse_phase_sequence,
                stop_requested_cb=lambda: stop_requested,
            )
    except StartupSafetyError as exc:
        print(f"SAFETY: {exc}", file=sys.stderr)
        return 2
    finally:
        if output_enabled:
            print("Stopping output and leaving firmware in STOP state.")
            try:
                client.command_and_collect("stop", settle_s=0.05)
            except Exception as exc:  # pragma: no cover - best effort shutdown
                print(f"WARNING: failed to send stop: {exc}", file=sys.stderr)
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
