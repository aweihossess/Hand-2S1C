#!/usr/bin/env python3
"""Move one Feetech ST3215 servo and print current feedback.

This script talks directly to a Feetech TTL serial servo adapter. It uses the
same STS/SMS register map as the firmware:
  56..70 = position, speed, load, voltage, temperature, moving, current.
"""

import argparse
import csv
import sys
import time

try:
    import serial
except ImportError:
    serial = None


INST_READ = 0x02
INST_WRITE = 0x03

STS_MODE = 33
STS_TORQUE_ENABLE = 40
STS_ACC = 41
STS_PRESENT_POSITION_L = 56
STS_PRESENT_CURRENT_H = 70

CURRENT_MA_PER_COUNT = 6.5


def checksum(values):
    return (~(sum(values) & 0xFF)) & 0xFF


def encode_signed_word(value, sign_bit=15):
    value = int(value)
    if value < 0:
        value = (-value) | (1 << sign_bit)
    return value & 0xFFFF


def decode_signed_word(lo, hi, sign_bit=15):
    raw = ((hi & 0xFF) << 8) | (lo & 0xFF)
    mask = 1 << sign_bit
    return -(raw & ~mask) if raw & mask else raw


def write_packet(port, servo_id, instruction, params):
    length = len(params) + 2
    body = [servo_id, length, instruction] + list(params)
    packet = bytes([0xFF, 0xFF] + body + [checksum(body)])
    port.reset_input_buffer()
    port.write(packet)
    port.flush()


def read_status(port, servo_id, payload_len, timeout_s):
    deadline = time.time() + timeout_s
    state = 0
    while time.time() < deadline:
        b = port.read(1)
        if not b:
            continue
        v = b[0]
        if state == 0:
            state = 1 if v == 0xFF else 0
        elif state == 1:
            state = 2 if v == 0xFF else 0
            if v == 0xFF:
                break
    else:
        raise TimeoutError("no status header")

    header = port.read(3)
    if len(header) != 3:
        raise TimeoutError("short status header")
    rx_id, length, error = header
    if rx_id != servo_id:
        raise RuntimeError(f"unexpected servo id {rx_id}, expected {servo_id}")
    if length != payload_len + 2:
        raise RuntimeError(f"unexpected payload length {length}, expected {payload_len + 2}")
    payload = port.read(payload_len)
    tail = port.read(1)
    if len(payload) != payload_len or len(tail) != 1:
        raise TimeoutError("short status payload")
    calc = checksum([rx_id, length, error] + list(payload))
    if tail[0] != calc:
        raise RuntimeError(f"checksum mismatch rx=0x{tail[0]:02X} calc=0x{calc:02X}")
    if error:
        raise RuntimeError(f"servo returned error status 0x{error:02X}")
    return bytes(payload)


def write_registers(port, servo_id, address, data, timeout_s):
    write_packet(port, servo_id, INST_WRITE, [address] + list(data))
    try:
        read_status(port, servo_id, 0, timeout_s)
    except TimeoutError:
        # Some adapters/servos are configured with no write ACK. The write may still succeed.
        pass


def read_registers(port, servo_id, address, length, timeout_s):
    write_packet(port, servo_id, INST_READ, [address, length])
    return read_status(port, servo_id, length, timeout_s)


def set_u8(port, servo_id, address, value, timeout_s):
    write_registers(port, servo_id, address, [value & 0xFF], timeout_s)


def set_wheel_speed(port, servo_id, speed, acc, timeout_s):
    speed_word = encode_signed_word(speed, 15)
    data = [
        acc & 0xFF,
        0, 0,              # position
        0, 0,              # time
        speed_word & 0xFF,
        (speed_word >> 8) & 0xFF,
    ]
    write_registers(port, servo_id, STS_ACC, data, timeout_s)


def set_position(port, servo_id, position, speed, acc, timeout_s):
    position_word = encode_signed_word(position, 15)
    data = [
        acc & 0xFF,
        position_word & 0xFF,
        (position_word >> 8) & 0xFF,
        0, 0,              # time
        speed & 0xFF,
        (speed >> 8) & 0xFF,
    ]
    write_registers(port, servo_id, STS_ACC, data, timeout_s)


def parse_feedback(payload):
    pos = decode_signed_word(payload[0], payload[1], 15)
    speed = decode_signed_word(payload[2], payload[3], 15)
    load = decode_signed_word(payload[4], payload[5], 10)
    voltage = payload[6]
    temperature = payload[7]
    moving = payload[10] if len(payload) > 10 else 0
    current = decode_signed_word(payload[13], payload[14], 15) if len(payload) >= 15 else 0
    return {
        "pos": pos,
        "speed": speed,
        "load": load,
        "voltage": voltage,
        "temperature": temperature,
        "moving": moving,
        "current_raw": current,
        "current_ma": current * CURRENT_MA_PER_COUNT,
    }


def print_feedback(feedback, prefix=""):
    print(
        f"{prefix}pos={feedback['pos']:6d} speed={feedback['speed']:5d} "
        f"load={feedback['load']:5d} current={feedback['current_raw']:5d} "
        f"current_mA={feedback['current_ma']:8.1f} "
        f"voltage={feedback['voltage']:3d} temp={feedback['temperature']:3d} "
        f"moving={feedback['moving']}"
    )


def main():
    parser = argparse.ArgumentParser(description="Feetech ST3215 current feedback test")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Servo baud rate")
    parser.add_argument("--id", type=int, default=1, help="Servo ID")
    parser.add_argument(
        "--mode",
        choices=["read", "wheel", "position", "hold"],
        default="wheel",
        help="read: feedback only; wheel: spin; position: alternate position targets; hold: hold one target",
    )
    parser.add_argument("--speed", type=int, default=120, help="Wheel speed or position-mode goal speed")
    parser.add_argument("--acc", type=int, default=20, help="Acceleration command")
    parser.add_argument("--period", type=float, default=0.05, help="Read period in seconds")
    parser.add_argument("--duration", type=float, default=10.0, help="Total test duration in seconds; <=0 runs until Ctrl+C")
    parser.add_argument("--reverse-every", type=float, default=2.0, help="Reverse speed every N seconds")
    parser.add_argument("--center", type=int, help="Position-mode center target; default is current position")
    parser.add_argument("--amplitude", type=int, default=200, help="Position-mode +/- target amplitude in counts")
    parser.add_argument("--target", type=int, help="Hold-mode absolute target; default is current position")
    parser.add_argument("--read-only", action="store_true", help="Alias for --mode read")
    parser.add_argument("--keep-torque-on-exit", action="store_true", help="Leave torque enabled when the script exits")
    parser.add_argument("--csv", help="Optional CSV output path")
    args = parser.parse_args()

    if serial is None:
        print("Missing dependency: pyserial. Install with: python -m pip install pyserial", file=sys.stderr)
        return 1

    if args.read_only:
        args.mode = "read"

    read_len = STS_PRESENT_CURRENT_H - STS_PRESENT_POSITION_L + 1
    csv_file = None
    writer = None

    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        if args.csv:
            csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    "time_s", "command_speed", "command_target", "pos", "speed", "load",
                    "current_raw", "current_ma", "voltage", "temperature", "moving",
                ],
            )
            writer.writeheader()

        try:
            initial_payload = read_registers(port, args.id, STS_PRESENT_POSITION_L, read_len, 0.08)
            initial_feedback = parse_feedback(initial_payload)
            print_feedback(initial_feedback, prefix="initial ")

            if args.mode == "wheel":
                set_u8(port, args.id, STS_MODE, 1, 0.05)
                set_u8(port, args.id, STS_TORQUE_ENABLE, 1, 0.05)
                command_speed = args.speed
                command_target = None
                set_wheel_speed(port, args.id, command_speed, args.acc, 0.05)
            elif args.mode == "position":
                set_u8(port, args.id, STS_MODE, 0, 0.05)
                set_u8(port, args.id, STS_TORQUE_ENABLE, 1, 0.05)
                center = initial_feedback["pos"] if args.center is None else args.center
                command_speed = 0
                command_target = center + args.amplitude
                set_position(port, args.id, command_target, abs(args.speed), args.acc, 0.05)
            elif args.mode == "hold":
                set_u8(port, args.id, STS_MODE, 0, 0.05)
                set_u8(port, args.id, STS_TORQUE_ENABLE, 1, 0.05)
                command_speed = 0
                command_target = initial_feedback["pos"] if args.target is None else args.target
                set_position(port, args.id, command_target, abs(args.speed), args.acc, 0.05)
            else:
                command_speed = 0
                command_target = None

            start = time.time()
            next_reverse = start + args.reverse_every
            position_sign = -1

            while args.duration <= 0 or time.time() - start < args.duration:
                now = time.time()
                if args.mode == "wheel" and now >= next_reverse:
                    command_speed = -command_speed
                    next_reverse += args.reverse_every
                    set_wheel_speed(port, args.id, command_speed, args.acc, 0.05)
                elif args.mode == "position" and now >= next_reverse:
                    center = initial_feedback["pos"] if args.center is None else args.center
                    command_target = center + position_sign * args.amplitude
                    position_sign = -position_sign
                    next_reverse += args.reverse_every
                    set_position(port, args.id, command_target, abs(args.speed), args.acc, 0.05)

                payload = read_registers(port, args.id, STS_PRESENT_POSITION_L, read_len, 0.08)
                feedback = parse_feedback(payload)
                elapsed = now - start
                if command_target is None:
                    prefix = f"t={elapsed:6.2f}s mode={args.mode:8s} cmd={command_speed:5d} "
                else:
                    prefix = f"t={elapsed:6.2f}s mode={args.mode:8s} target={command_target:6d} "
                print_feedback(feedback, prefix=prefix)

                if writer:
                    row = {"time_s": elapsed, "command_speed": command_speed}
                    row.update(feedback)
                    row["command_target"] = command_target
                    writer.writerow(row)

                time.sleep(args.period)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            if args.mode == "wheel":
                try:
                    set_wheel_speed(port, args.id, 0, args.acc, 0.05)
                except Exception:
                    pass
            if args.mode != "read" and not args.keep_torque_on_exit:
                try:
                    set_u8(port, args.id, STS_TORQUE_ENABLE, 0, 0.05)
                except Exception:
                    pass
            if csv_file:
                csv_file.close()


if __name__ == "__main__":
    sys.exit(main())
