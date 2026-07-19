#!/usr/bin/env python3
"""Direct COM6/COM7 sine/step test with synchronized force and control logging.

This deliberately bypasses Tk target dispatch.  It uses the same binary angle
target frame as the upper computer and sends it every 100 ms.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from desktop.force_sensor_monitor import (
    DISPLAY_CHANNELS,
    ENCODER_COUNT,
    FORCE_CHANNEL_TO_MOTOR,
    NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS,
    NULLSPACE_PRETENSION_VECTOR,
    allocate_nullspace_internal_tension,
    build_read_holding_registers,
    build_upper_angle_cmd,
    decode_live_weight_response,
    load_force_zero_baselines,
    nullspace_alpha_count_step,
    nullspace_pretension_bias_counts,
    open_serial_port,
    read_exact,
    weight_register_for_channel,
)


INITIAL_TARGETS = (10.0, 10.0, 20.0, 10.0)
JOINT_LIMITS_DEG = ((-60.0, 60.0), (0.0, 60.0), (0.0, 50.0), (-10.0, 100.0))
# Measured-angle aborts use the same mechanical safety envelope as commanded
# targets.  J01 still keeps its strict-positive experiment requirement below.
ACTUAL_GUARD_LIMITS_DEG = JOINT_LIMITS_DEG
CTRL_RE = re.compile(r"\[MCP5 CTRL\]\s+(.*)")
JOINT_RE = re.compile(r"J(\d\d) target=([-0-9.]+) actual=([-0-9.]+)")
MOTOR_RE = re.compile(
    r"M(\d\d) len=([-0-9.]+)/([-0-9.]+)/([-0-9.]+) "
    r"map=([-0-9.]+) solver=([-0-9]+) cmd=([-0-9]+) now=([-0-9]+) "
    r"load=([-0-9]+) cur=([-0-9]+) bias=([-0-9]+)"
)


def default_output_path(mode: str = "sine") -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("run_data", f"direct_{mode}_{stamp}.csv")


def parse_four_targets(text: str) -> tuple[float, float, float, float]:
    values = [part.strip() for part in re.split(r"[,;]", text) if part.strip()]
    if len(values) != 4:
        raise argparse.ArgumentTypeError("base targets must contain four J00-J03 angles")
    try:
        parsed = tuple(float(value) for value in values)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("base targets must be numeric") from exc
    if not all(math.isfinite(value) for value in parsed):
        raise argparse.ArgumentTypeError("base targets must be finite")
    return parsed  # type: ignore[return-value]


def write_text(port, command: str) -> None:
    port.write((command.strip() + "\n").encode("ascii"))
    port.flush()


def build_joint_target_frame(j00_j03: list[float] | tuple[float, ...]) -> bytes:
    """Match the upper computer's 21-slot angle frame; only J00-J03 are controlled."""
    if len(j00_j03) != 4:
        raise ValueError("need exactly four J00-J03 targets")
    targets = [0.0] * ENCODER_COUNT
    targets[:4] = [float(value) for value in j00_j03]
    return build_upper_angle_cmd(targets)


def build_fieldnames() -> list[str]:
    fields = ["t_wall", "t_rel", "line"]
    for joint in range(4):
        fields += [f"j{joint}_target", f"j{joint}_actual", f"j{joint}_error"]
    for motor in range(5):
        fields += [
            f"m{motor}_len0", f"m{motor}_len1", f"m{motor}_len2",
            f"m{motor}_map", f"m{motor}_solver", f"m{motor}_cmd",
            f"m{motor}_now", f"m{motor}_load", f"m{motor}_cur", f"m{motor}_bias",
        ]
    return fields


def parse_ctrl_line(line: str, t0: float) -> dict | None:
    if CTRL_RE.search(line) is None:
        return None
    now = time.time()
    row: dict[str, object] = {
        "t_wall": f"{now:.6f}", "t_rel": f"{now - t0:.6f}", "line": line.strip()
    }
    for match in JOINT_RE.finditer(line):
        joint = int(match.group(1))
        if joint < 4:
            target = float(match.group(2))
            actual = float(match.group(3))
            row[f"j{joint}_target"] = target
            row[f"j{joint}_actual"] = actual
            row[f"j{joint}_error"] = target - actual
    for match in MOTOR_RE.finditer(line):
        motor = int(match.group(1))
        if motor < 5:
            row[f"m{motor}_len0"] = float(match.group(2))
            row[f"m{motor}_len1"] = float(match.group(3))
            row[f"m{motor}_len2"] = float(match.group(4))
            row[f"m{motor}_map"] = float(match.group(5))
            row[f"m{motor}_solver"] = int(match.group(6))
            row[f"m{motor}_cmd"] = int(match.group(7))
            row[f"m{motor}_now"] = int(match.group(8))
            row[f"m{motor}_load"] = int(match.group(9))
            row[f"m{motor}_cur"] = int(match.group(10))
            row[f"m{motor}_bias"] = int(match.group(11))
    return row


def read_available_lines(port, buffer: bytearray) -> list[str]:
    chunk = port.read(4096)
    if chunk:
        buffer.extend(chunk)
    lines: list[str] = []
    while b"\n" in buffer:
        raw, _, remainder = buffer.partition(b"\n")
        buffer[:] = remainder
        lines.append(raw.decode("utf-8", errors="replace").strip())
    return lines


class ForceReader(threading.Thread):
    def __init__(self, port_name: str, baud: int, interval_s: float = 0.1) -> None:
        super().__init__(daemon=True)
        self.port_name = port_name
        self.baud = baud
        self.interval_s = interval_s
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_raw: dict[int, int] = {}
        self.latest_ts: dict[int, float] = {}
        self.error: str = ""

    def run(self) -> None:
        requests = {
            channel: build_read_holding_registers(1, weight_register_for_channel(channel), 2)
            for channel in DISPLAY_CHANNELS
        }
        try:
            port = open_serial_port(self.port_name, self.baud, timeout_s=0.08)
            try:
                while not self.stop_event.is_set():
                    cycle_started = time.monotonic()
                    for channel, request in requests.items():
                        if self.stop_event.is_set():
                            break
                        port.reset_input_buffer()
                        port.write(request)
                        port.flush()
                        response = read_exact(port, 9, timeout_s=0.08)
                        raw, _low, _high = decode_live_weight_response(response, 1)
                        with self.lock:
                            self.latest_raw[channel] = raw
                            self.latest_ts[channel] = time.time()
                    remaining = self.interval_s - (time.monotonic() - cycle_started)
                    if remaining > 0:
                        self.stop_event.wait(remaining)
            finally:
                port.close()
        except Exception as exc:  # surfaced in the guarded main loop
            self.error = str(exc)

    def stop(self) -> None:
        self.stop_event.set()

    def snapshot(self, baselines: dict[int, int]) -> tuple[dict[int, float], dict[int, int], float]:
        with self.lock:
            raw = dict(self.latest_raw)
            timestamps = dict(self.latest_ts)
        if set(raw) != set(DISPLAY_CHANNELS):
            raise RuntimeError("five-channel force snapshot is incomplete")
        newest_age = max(time.time() - timestamps[channel] for channel in DISPLAY_CHANNELS)
        forces = {
            channel: (raw[channel] - baselines[channel]) / 10.0
            for channel in DISPLAY_CHANNELS
        }
        return forces, raw, newest_age


def output_fields() -> list[str]:
    fields = build_fieldnames()
    fields[3:3] = [
        "phase",
        "command_j00_target",
        "command_j01_target",
        "command_j02_target",
        "command_j03_target",
        "send_sequence",
        "send_lag_ms",
        "force_snapshot_age_s",
        "peak_tension_n",
        "nullspace_alpha_counts",
        "nullspace_alpha_measured_n",
        "nullspace_alpha_desired_n",
        "nullspace_alpha_min_n",
        "nullspace_alpha_max_n",
        "nullspace_feasible",
    ]
    for channel in DISPLAY_CHANNELS:
        fields += [f"force_ch{channel}_raw", f"force_ch{channel}_n"]
    for motor in range(5):
        fields += [f"force_m{motor:02d}_n", f"nullspace_bias_m{motor:02d}_counts"]
    return fields


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("sine", "step"), default="sine")
    parser.add_argument("--base-targets", type=parse_four_targets, default=INITIAL_TARGETS)
    parser.add_argument("--servo-port", default="COM6")
    parser.add_argument("--force-port", default="COM7")
    parser.add_argument("--servo-baud", type=int, default=921600)
    parser.add_argument("--force-baud", type=int, default=115200)
    parser.add_argument("--baseline-s", type=float, default=15.0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--return-s", type=float, default=12.0)
    parser.add_argument("--interval-s", type=float, default=0.1)
    parser.add_argument("--frequency-hz", type=float, default=0.1)
    parser.add_argument("--amplitude-deg", type=float, default=10.0)
    parser.add_argument("--mean-deg", type=float, default=20.0)
    parser.add_argument("--sine-joint", type=int, choices=range(4), default=3)
    parser.add_argument("--step-amplitude-deg", type=float, default=5.0)
    parser.add_argument(
        "--step-joint",
        type=int,
        choices=(-1, 0, 1, 2, 3),
        default=-1,
        help="Joint to excite; -1 applies the same step to all four joints.",
    )
    parser.add_argument("--step-hold-s", type=float, default=6.0)
    parser.add_argument("--center-hold-s", type=float, default=4.0)
    parser.add_argument("--kp", type=float, default=0.70)
    parser.add_argument("--ki", type=float, default=0.02)
    parser.add_argument("--r-blend", type=float, default=0.0)
    parser.add_argument("--p-blend", type=float, default=0.25)
    parser.add_argument("--abort-tension-n", type=float, default=80.0)
    parser.add_argument("--min-tension-n", type=float, default=5.0)
    parser.add_argument("--startup-grace-s", type=float, default=1.5)
    parser.add_argument(
        "--skip-angle-guard",
        action="store_true",
        help="Disable measured-angle aborts; force and feedback-freshness guards remain active.",
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.output is None:
        args.output = default_output_path(args.mode)

    if args.interval_s < 0.05:
        parser.error("interval must be at least 50 ms")
    extrema = [list(args.base_targets)]
    if args.mode == "step":
        for sign in (-1.0, 1.0):
            target = list(args.base_targets)
            joints = range(4) if args.step_joint < 0 else (args.step_joint,)
            for joint in joints:
                target[joint] += sign * args.step_amplitude_deg
            extrema.append(target)
    else:
        for value in (args.mean_deg - args.amplitude_deg, args.mean_deg + args.amplitude_deg):
            target = list(args.base_targets)
            target[args.sine_joint] = value
            extrema.append(target)
    for target in extrema:
        for joint, (lower, upper) in enumerate(JOINT_LIMITS_DEG):
            if not lower <= target[joint] <= upper:
                parser.error(
                    f"J{joint:02d} target {target[joint]:.2f} outside [{lower:.1f},{upper:.1f}]"
                )
        if target[1] <= 0.0:
            parser.error("J01 target must remain strictly greater than 0 deg")
    baselines = load_force_zero_baselines()
    if set(baselines) != set(DISPLAY_CHANNELS):
        parser.error("saved five-channel force zero baselines are required")
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    raw_path = os.path.splitext(args.output)[0] + ".raw.txt"

    force_reader = ForceReader(args.force_port, args.force_baud, args.interval_s)
    force_reader.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if force_reader.error:
            raise RuntimeError(f"force serial failed: {force_reader.error}")
        try:
            force_reader.snapshot(baselines)
            break
        except RuntimeError:
            time.sleep(0.05)
    else:
        raise RuntimeError("force reader did not produce five channels")

    t0 = time.time()
    aborted = False
    abort_reason = ""
    phase = "baseline"
    base_targets = tuple(float(value) for value in args.base_targets)
    target_vector = list(base_targets)
    send_sequence = 0
    send_lag_ms = 0.0
    alpha_counts = 0.0
    alpha_measured_n = 0.0
    alpha_desired_n = 0.0
    alpha_min_n = 0.0
    alpha_max_n = 0.0
    nullspace_feasible = True
    biases = {motor: 0 for motor in range(5)}
    latest_actual = [math.nan] * 4
    latest_actual_ts = 0.0
    last_biases = dict(biases)
    loose_since = {motor: None for motor in range(5)}

    try:
        servo = open_serial_port(args.servo_port, args.servo_baud, timeout_s=0.01)
        try:
            with open(args.output, "w", newline="", encoding="utf-8-sig") as fp, open(
                raw_path, "w", encoding="utf-8"
            ) as raw_fp:
                writer = csv.DictWriter(fp, fieldnames=output_fields(), extrasaction="ignore", restval="")
                writer.writeheader()

                for command in (
                    "text",
                    f"kp {args.kp:.4f}",
                    f"ki {args.ki:.6f}",
                    f"rblend {args.r_blend:.4f}",
                    f"pblend {args.p_blend:.4f}",
                    "degree",
                    "tension on",
                ):
                    write_text(servo, command)
                    time.sleep(0.03)
                for motor in range(5):
                    write_text(servo, f"tensionbias m{motor} 0")
                servo.write(build_joint_target_frame(target_vector))
                servo.flush()
                write_text(servo, "start")
                write_text(servo, "status")

                schedule_start = time.monotonic()
                excitation_start = schedule_start + args.baseline_s
                if args.mode == "sine":
                    excitation_end = excitation_start + args.duration_s
                    return_end = excitation_end + args.return_s
                else:
                    step_plus_end = excitation_start + args.step_hold_s
                    step_center_end = step_plus_end + args.center_hold_s
                    step_minus_end = step_center_end + args.step_hold_s
                    return_end = step_minus_end + args.return_s
                next_send = schedule_start
                next_nullspace = schedule_start
                text_buffer = bytearray()

                while time.monotonic() < return_end:
                    now_mono = time.monotonic()
                    if force_reader.error:
                        raise RuntimeError(f"force serial failed: {force_reader.error}")
                    forces_by_channel, raw_forces, force_age = force_reader.snapshot(baselines)
                    force_by_motor = {
                        FORCE_CHANNEL_TO_MOTOR[channel]: forces_by_channel[channel]
                        for channel in DISPLAY_CHANNELS
                    }
                    peak_tension = max(-value for value in force_by_motor.values())
                    if force_age > 0.5:
                        raise RuntimeError(f"force data stale: {force_age:.3f}s")
                    if peak_tension > args.abort_tension_n:
                        raise RuntimeError(
                            f"tension guard: {peak_tension:.1f}N > {args.abort_tension_n:.1f}N"
                        )
                    # Positive sensor force or a value above -min_tension means
                    # that the corresponding tendon is slack or its signal is
                    # invalid.  Require persistence to reject a single noisy
                    # sample, but never run a long test without all five valid
                    # tension channels.
                    for motor, force_n in force_by_motor.items():
                        tension_n = -force_n
                        if tension_n < args.min_tension_n:
                            if loose_since[motor] is None:
                                loose_since[motor] = now_mono
                            elif (
                                now_mono - schedule_start >= args.startup_grace_s
                                and now_mono - loose_since[motor] >= 0.5
                            ):
                                raise RuntimeError(
                                    f"loose/invalid tension M{motor:02d}: "
                                    f"sensor={force_n:.1f}N, tension={tension_n:.1f}N "
                                    f"< {args.min_tension_n:.1f}N"
                                )
                        else:
                            loose_since[motor] = None
                    if latest_actual_ts and time.time() - latest_actual_ts > 0.8:
                        raise RuntimeError("joint feedback stale for more than 0.8s")
                    if not latest_actual_ts and now_mono - schedule_start > 1.0:
                        raise RuntimeError("joint feedback was not received within 1.0s")
                    # The serial buffer can still contain one diagnostic line
                    # from the previously active target.  Give the freshly
                    # sent binary target 1.5 s to take ownership before the
                    # experiment-range guard is enforced.
                    if not args.skip_angle_guard and now_mono - schedule_start >= args.startup_grace_s:
                        limits = ACTUAL_GUARD_LIMITS_DEG
                        for joint, (lower, upper) in enumerate(limits):
                            if math.isfinite(latest_actual[joint]) and not lower <= latest_actual[joint] <= upper:
                                raise RuntimeError(
                                    f"J{joint:02d}={latest_actual[joint]:.2f}deg outside [{lower},{upper}]"
                                )

                    if now_mono >= next_nullspace:
                        allocation = allocate_nullspace_internal_tension(force_by_motor)
                        alpha_measured_n = allocation.alpha_measured_n
                        alpha_min_n = allocation.alpha_min_n
                        alpha_max_n = allocation.alpha_max_n
                        nullspace_feasible = allocation.feasible
                        # An empty alpha interval is a task/allocation conflict,
                        # not evidence of an unsafe measured tension.  During a
                        # guarded validation run, keep the existing bias instead
                        # of replacing the real force guards with this numerical
                        # feasibility test.  The conflict remains visible in CSV.
                        alpha_desired_n = (
                            allocation.alpha_desired_n
                            if allocation.feasible
                            else allocation.alpha_measured_n
                        )
                        alpha_counts = max(
                            0.0,
                            min(
                                NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS,
                                alpha_counts
                                + nullspace_alpha_count_step(alpha_desired_n - alpha_measured_n),
                            ),
                        )
                        biases = nullspace_pretension_bias_counts(alpha_counts)
                        if biases != last_biases:
                            for motor in range(5):
                                write_text(servo, f"tensionbias m{motor} {biases[motor]}")
                            last_biases = dict(biases)
                        while next_nullspace <= now_mono:
                            next_nullspace += args.interval_s

                    if now_mono >= next_send:
                        scheduled = next_send
                        if now_mono < excitation_start:
                            phase = "baseline"
                            target_vector = list(base_targets)
                        elif args.mode == "sine" and now_mono < excitation_end:
                            phase = "sine"
                            elapsed = now_mono - excitation_start
                            target_vector = list(base_targets)
                            target_vector[args.sine_joint] = args.mean_deg - args.amplitude_deg * math.cos(
                                2.0 * math.pi * args.frequency_hz * elapsed
                            )
                        elif args.mode == "step" and now_mono < step_plus_end:
                            phase = "step_plus"
                            target_vector = list(base_targets)
                            if args.step_joint < 0:
                                target_vector = [value + args.step_amplitude_deg for value in base_targets]
                            else:
                                target_vector[args.step_joint] += args.step_amplitude_deg
                        elif args.mode == "step" and now_mono < step_center_end:
                            phase = "center"
                            target_vector = list(base_targets)
                        elif args.mode == "step" and now_mono < step_minus_end:
                            phase = "step_minus"
                            target_vector = list(base_targets)
                            if args.step_joint < 0:
                                target_vector = [value - args.step_amplitude_deg for value in base_targets]
                            else:
                                target_vector[args.step_joint] -= args.step_amplitude_deg
                        else:
                            phase = "return"
                            target_vector = list(base_targets)
                        servo.write(build_joint_target_frame(target_vector))
                        servo.flush()
                        send_sequence += 1
                        send_lag_ms = max(0.0, (now_mono - scheduled) * 1000.0)
                        next_send += args.interval_s

                    for line in read_available_lines(servo, text_buffer):
                        raw_fp.write(line + "\n")
                        row = parse_ctrl_line(line, t0)
                        if row:
                            latest_actual_ts = time.time()
                            for joint in range(4):
                                value = row.get(f"j{joint}_actual")
                                if value not in (None, ""):
                                    latest_actual[joint] = float(value)
                            row.update(
                                {
                                    "phase": phase,
                                    "command_j00_target": f"{target_vector[0]:.4f}",
                                    "command_j01_target": f"{target_vector[1]:.4f}",
                                    "command_j02_target": f"{target_vector[2]:.4f}",
                                    "command_j03_target": f"{target_vector[3]:.4f}",
                                    "send_sequence": send_sequence,
                                    "send_lag_ms": f"{send_lag_ms:.3f}",
                                    "force_snapshot_age_s": f"{force_age:.4f}",
                                    "peak_tension_n": f"{peak_tension:.3f}",
                                    "nullspace_alpha_counts": f"{alpha_counts:.3f}",
                                    "nullspace_alpha_measured_n": f"{alpha_measured_n:.4f}",
                                    "nullspace_alpha_desired_n": f"{alpha_desired_n:.4f}",
                                    "nullspace_alpha_min_n": f"{alpha_min_n:.4f}",
                                    "nullspace_alpha_max_n": f"{alpha_max_n:.4f}",
                                    "nullspace_feasible": int(nullspace_feasible),
                                }
                            )
                            for channel in DISPLAY_CHANNELS:
                                row[f"force_ch{channel}_raw"] = raw_forces[channel]
                                row[f"force_ch{channel}_n"] = f"{forces_by_channel[channel]:.3f}"
                            for motor in range(5):
                                row[f"force_m{motor:02d}_n"] = f"{force_by_motor[motor]:.3f}"
                                row[f"nullspace_bias_m{motor:02d}_counts"] = biases[motor]
                            writer.writerow(row)
                            fp.flush()

                servo.write(build_joint_target_frame(base_targets))
                servo.flush()
                write_text(servo, "kp 0.7000")
                write_text(servo, "ki 0.020000")
                write_text(servo, "status")
        finally:
            servo.close()
    except Exception as exc:
        aborted = True
        abort_reason = str(exc)
        try:
            servo = open_serial_port(args.servo_port, args.servo_baud, timeout_s=0.05)
            try:
                write_text(servo, "stop")
                write_text(servo, "tension off")
                for motor in range(5):
                    write_text(servo, f"tensionbias m{motor} 0")
            finally:
                servo.close()
        except Exception:
            pass
    finally:
        force_reader.stop()
        force_reader.join(timeout=1.0)

    print(f"CSV={args.output}")
    print(f"RAW={raw_path}")
    print(f"ABORTED={int(aborted)}")
    if abort_reason:
        print(f"ABORT_REASON={abort_reason}")
    return 2 if aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
