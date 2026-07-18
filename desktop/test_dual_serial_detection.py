from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from force_sensor_monitor import (  # noqa: E402
    DEFAULT_BAUDRATE,
    DISPLAY_CHANNELS,
    ENCODER_COUNT,
    ENCODER_RAW_TO_DEG,
    ENCODER_SERVO_DEFAULT_BAUDRATE,
    PACKET_TYPE_JOINT_DEBUG,
    PACKET_TYPE_SENSOR,
    PACKET_TYPE_SERVO_ANGLE,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SERVO_TELEM,
    SERVO_COUNT,
    build_read_holding_registers,
    build_upper_stream_mode_cmd,
    decode_live_weight_response,
    format_force_n,
    format_hex,
    force_motor_label,
    list_serial_ports,
    open_serial_port,
    parse_encoder_sensor_payload,
    parse_joint_debug_payload,
    parse_servo_angle_payload,
    parse_servo_raw_payload,
    parse_servo_telem_payload,
    parse_upper_frame_bytes,
    read_exact,
    text_key_values,
    weight_register_for_channel,
)


@dataclass
class ForceReadResult:
    port: str
    opened: bool = False
    samples: int = 0
    values: dict[int, int] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.opened and self.samples > 0 and bool(self.values)


@dataclass
class EncoderServoReadResult:
    port: str
    opened: bool = False
    encoder_frames: int = 0
    servo_angle_frames: int = 0
    servo_raw_frames: int = 0
    servo_telem_frames: int = 0
    text_lines: int = 0
    encoder_deg: list[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    encoder_valid: list[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    servo_abs: list[int] = field(default_factory=lambda: [0] * SERVO_COUNT)
    servo_online: list[bool] = field(default_factory=lambda: [False] * SERVO_COUNT)
    last_line: str = "--"
    errors: list[str] = field(default_factory=list)

    @property
    def encoder_ok(self) -> bool:
        return self.opened and (self.encoder_frames > 0 or any(self.encoder_valid))

    @property
    def servo_ok(self) -> bool:
        return self.opened and (
            self.servo_angle_frames > 0
            or self.servo_raw_frames > 0
            or self.servo_telem_frames > 0
            or any(self.servo_online)
        )

    @property
    def score(self) -> int:
        score = self.encoder_frames + self.servo_angle_frames + self.servo_raw_frames + self.servo_telem_frames
        score += sum(1 for valid in self.encoder_valid if valid)
        score += sum(1 for online in self.servo_online if online)
        return score


def write_port(port, data: bytes) -> None:
    port.write(data)
    if hasattr(port, "flush"):
        port.flush()


def write_text(port, text: str) -> None:
    write_port(port, text.strip().encode("ascii", errors="ignore") + b"\n")


def update_encoder_servo_from_frame(result: EncoderServoReadResult, pkt_type: int, payload: bytes) -> None:
    if pkt_type == PACKET_TYPE_SENSOR:
        _raw, mapped, valid = parse_encoder_sensor_payload(payload)
        result.encoder_frames += 1
        result.encoder_valid = list(valid)
        result.encoder_deg = [mapped[index] * ENCODER_RAW_TO_DEG for index in range(ENCODER_COUNT)]
        return

    if pkt_type == PACKET_TYPE_SERVO_ANGLE:
        parsed = parse_servo_angle_payload(payload)
        if parsed is None:
            return
        angles, _offsets, online = parsed
        result.servo_angle_frames += 1
        result.servo_abs = list(angles)
        result.servo_online = [result.servo_online[index] or online[index] for index in range(SERVO_COUNT)]
        return

    if pkt_type == PACKET_TYPE_SERVO_RAW:
        parsed = parse_servo_raw_payload(payload)
        if parsed is None:
            return
        _raw, online = parsed
        result.servo_raw_frames += 1
        result.servo_online = [result.servo_online[index] or online[index] for index in range(SERVO_COUNT)]
        return

    if pkt_type == PACKET_TYPE_SERVO_TELEM:
        parsed = parse_servo_telem_payload(payload)
        if parsed is None:
            return
        _speed, _load, _current, _voltage, _temperature, online = parsed
        result.servo_telem_frames += 1
        result.servo_online = [result.servo_online[index] or online[index] for index in range(SERVO_COUNT)]
        return

    if pkt_type == PACKET_TYPE_JOINT_DEBUG:
        parsed = parse_joint_debug_payload(payload)
        if parsed is None:
            return
        joint_index, valid, _target_deg, actual_deg, _cmd_target_pos = parsed
        if 0 <= joint_index < ENCODER_COUNT:
            result.encoder_valid[joint_index] = valid
            result.encoder_deg[joint_index] = actual_deg


def update_encoder_servo_from_line(result: EncoderServoReadResult, line: str) -> None:
    result.text_lines += 1
    result.last_line = line
    values = text_key_values(line)

    match = re.search(r"\bENC\s+J(\d+)", line)
    if match:
        index = int(match.group(1))
        if 0 <= index < ENCODER_COUNT:
            if "deg" in values:
                result.encoder_deg[index] = float(values["deg"])
            elif "mapped" in values:
                result.encoder_deg[index] = int(float(values["mapped"])) * ENCODER_RAW_TO_DEG
            valid = values.get("mapped_valid", values.get("raw_valid", "1"))
            result.encoder_valid[index] = valid not in ("0", "false", "False")

    match = re.search(r"\bSERVO\s+M(\d+)", line)
    if match:
        index = int(match.group(1))
        if 0 <= index < SERVO_COUNT:
            if "motor_abs" in values:
                result.servo_abs[index] = int(float(values["motor_abs"]))
            if "online" in values:
                result.servo_online[index] = values["online"] not in ("0", "false", "False")

    match = re.search(r"\bLOAD\s+M(\d+)", line)
    if match:
        index = int(match.group(1))
        if 0 <= index < SERVO_COUNT and "online" in values:
            result.servo_online[index] = values["online"] not in ("0", "false", "False")


def read_encoder_servo_port(port_name: str, baudrate: int, duration_s: float) -> EncoderServoReadResult:
    result = EncoderServoReadResult(port=port_name)
    port = None
    try:
        port = open_serial_port(port_name, baudrate, timeout_s=0.05)
        result.opened = True
        write_port(port, build_upper_stream_mode_cmd())
        write_text(port, "binary")
        write_text(port, "encoder")
        write_text(port, "servo")
        write_text(port, "status")

        buffer = bytearray()
        deadline = time.monotonic() + duration_s
        next_snapshot = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_snapshot:
                write_text(port, "encoder")
                write_text(port, "servo")
                next_snapshot = time.monotonic() + 1.0

            chunk = port.read(512)
            if not chunk:
                time.sleep(0.01)
                continue
            buffer.extend(chunk)
            frames, lines = parse_upper_frame_bytes(buffer)
            for pkt_type, payload in frames:
                update_encoder_servo_from_frame(result, pkt_type, payload)
            for line in lines:
                update_encoder_servo_from_line(result, line)
    except Exception as exc:
        result.errors.append(str(exc))
    finally:
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
    return result


def read_force_sensor_loop(
    result: ForceReadResult,
    stop_event: threading.Event,
    slave_id: int,
    baudrate: int,
) -> None:
    port = None
    requests = {
        channel: build_read_holding_registers(slave_id, weight_register_for_channel(channel), 2)
        for channel in DISPLAY_CHANNELS
    }
    try:
        port = open_serial_port(result.port, baudrate, timeout_s=0.25)
        result.opened = True
        while not stop_event.is_set():
            for channel, request in requests.items():
                if stop_event.is_set():
                    break
                try:
                    if hasattr(port, "reset_input_buffer"):
                        port.reset_input_buffer()
                    write_port(port, request)
                    response = read_exact(port, 9, timeout_s=0.35)
                    value, _low, _high = decode_live_weight_response(response, slave_id)
                    result.values[channel] = value
                    result.samples += 1
                except Exception as exc:
                    if len(result.errors) < 5:
                        result.errors.append(f"CH{channel}: {exc}")
            stop_event.wait(0.05)
    except Exception as exc:
        result.errors.append(str(exc))
    finally:
        if port is not None:
            try:
                port.close()
            except Exception:
                pass


def read_encoder_servo_loop(
    result_holder: dict[str, EncoderServoReadResult],
    port_name: str,
    baudrate: int,
    stop_event: threading.Event,
) -> None:
    duration_s = 0.1
    start = time.monotonic()
    while not stop_event.is_set():
        duration_s = max(0.1, time.monotonic() - start + 0.5)
        break
    result_holder["result"] = read_encoder_servo_port(port_name, baudrate, duration_s)


def detect_encoder_servo_port(
    candidates: list[str],
    baudrate: int,
    probe_seconds: float,
) -> Optional[EncoderServoReadResult]:
    best: Optional[EncoderServoReadResult] = None
    for port_name in candidates:
        print(f"[probe] trying encoder/servo port {port_name} @ {baudrate}...")
        result = read_encoder_servo_port(port_name, baudrate, probe_seconds)
        print(
            f"[probe] {port_name}: opened={result.opened} "
            f"encoder_frames={result.encoder_frames} "
            f"servo_frames={result.servo_angle_frames + result.servo_raw_frames + result.servo_telem_frames} "
            f"text_lines={result.text_lines} score={result.score}"
        )
        if best is None or result.score > best.score:
            best = result
        if result.encoder_ok or result.servo_ok:
            return result
    return best if best and best.score > 0 else None


def print_force_summary(result: ForceReadResult) -> None:
    print("\n[force]")
    print(f"port={result.port} opened={result.opened} samples={result.samples} ok={result.ok}")
    if result.values:
        for channel in DISPLAY_CHANNELS:
            if channel in result.values:
                value = result.values[channel]
                print(f"  CH{channel} / {force_motor_label(channel)}: raw={value} force={format_force_n(value)}")
    if result.errors:
        print("  errors:")
        for error in result.errors[:5]:
            print(f"    {error}")


def print_encoder_servo_summary(result: Optional[EncoderServoReadResult]) -> None:
    print("\n[encoder_servo]")
    if result is None:
        print("port=-- opened=False ok=False")
        return
    valid_count = sum(1 for valid in result.encoder_valid if valid)
    online_count = sum(1 for online in result.servo_online if online)
    print(
        f"port={result.port} opened={result.opened} "
        f"encoder_ok={result.encoder_ok} servo_ok={result.servo_ok}"
    )
    print(
        f"  encoder_frames={result.encoder_frames} valid_channels={valid_count}/{ENCODER_COUNT}"
    )
    print(
        "  servo_frames="
        f"{result.servo_angle_frames + result.servo_raw_frames + result.servo_telem_frames} "
        f"online_channels={online_count}/{SERVO_COUNT}"
    )
    encoder_preview = [
        f"J{index}={result.encoder_deg[index]:.2f}"
        for index, valid in enumerate(result.encoder_valid)
        if valid
    ][:8]
    servo_preview = [
        f"M{index}={result.servo_abs[index]}"
        for index, online in enumerate(result.servo_online)
        if online
    ][:8]
    if encoder_preview:
        print("  encoder_deg: " + ", ".join(encoder_preview))
    if servo_preview:
        print("  servo_abs: " + ", ".join(servo_preview))
    if result.last_line and result.last_line != "--":
        print(f"  last_line: {result.last_line[:180]}")
    if result.errors:
        print("  errors:")
        for error in result.errors[:5]:
            print(f"    {error}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Test force sensor and encoder/servo serial inputs at the same time.")
    parser.add_argument("--force-port", default="COM7")
    parser.add_argument("--force-baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--servo-port", default="")
    parser.add_argument("--servo-baudrate", type=int, default=ENCODER_SERVO_DEFAULT_BAUDRATE)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--probe-seconds", type=float, default=2.0)
    args = parser.parse_args()

    ports = list_serial_ports()
    print("[ports] " + ", ".join(ports))

    servo_result: Optional[EncoderServoReadResult] = None
    servo_port = args.servo_port.strip()
    if not servo_port:
        candidates = [port for port in ports if port.upper() != args.force_port.upper()]
        servo_result = detect_encoder_servo_port(candidates, args.servo_baudrate, args.probe_seconds)
        if servo_result is not None:
            servo_port = servo_result.port
    if not servo_port:
        print("[detect] no encoder/servo port detected.")

    stop_event = threading.Event()
    force_result = ForceReadResult(port=args.force_port)
    force_thread = threading.Thread(
        target=read_force_sensor_loop,
        args=(force_result, stop_event, args.slave_id, args.force_baudrate),
        daemon=True,
    )

    encoder_holder: dict[str, EncoderServoReadResult] = {}
    encoder_thread: Optional[threading.Thread] = None
    if servo_port:
        encoder_thread = threading.Thread(
            target=lambda: encoder_holder.setdefault(
                "result",
                read_encoder_servo_port(servo_port, args.servo_baudrate, args.seconds),
            ),
            daemon=True,
        )

    print(f"[run] simultaneous read for {args.seconds:.1f}s")
    force_thread.start()
    if encoder_thread is not None:
        encoder_thread.start()
    time.sleep(max(0.1, args.seconds))
    stop_event.set()
    force_thread.join(timeout=2.0)
    if encoder_thread is not None:
        encoder_thread.join(timeout=2.0)
        servo_result = encoder_holder.get("result", servo_result)

    print_force_summary(force_result)
    print_encoder_servo_summary(servo_result)

    all_ok = force_result.ok and servo_result is not None and servo_result.encoder_ok and servo_result.servo_ok
    partial_ok = force_result.ok and servo_result is not None and (servo_result.encoder_ok or servo_result.servo_ok)
    print("\n[result]")
    if all_ok:
        print("OK: force + encoder + servo detected.")
        return 0
    if partial_ok:
        print("PARTIAL: force detected, encoder/servo only partially detected.")
        return 2
    print("FAIL: simultaneous detection did not get all required data.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
