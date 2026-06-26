#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import sys
import time
from datetime import datetime

import serial


CTRL_RE = re.compile(r"\[MCP5 CTRL\]\s+(.*)")
JOINT_RE = re.compile(r"J(\d\d) target=([-0-9.]+) actual=([-0-9.]+)")
MOTOR_RE = re.compile(
    r"M(\d\d) len=([-0-9.]+)/([-0-9.]+)/([-0-9.]+) "
    r"map=([-0-9.]+) solver=([-0-9]+) cmd=([-0-9]+) now=([-0-9]+) "
    r"load=([-0-9]+) cur=([-0-9]+) bias=([-0-9]+)"
)


def default_output_path() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("run_data", f"mcp_control_{stamp}.csv")


def build_fieldnames() -> list[str]:
    fields = ["t_wall", "t_rel", "line"]
    for j in range(4):
        fields += [f"j{j}_target", f"j{j}_actual", f"j{j}_error"]
    for m in range(5):
        fields += [
            f"m{m}_len0",
            f"m{m}_len1",
            f"m{m}_len2",
            f"m{m}_map",
            f"m{m}_solver",
            f"m{m}_cmd",
            f"m{m}_now",
            f"m{m}_load",
            f"m{m}_cur",
            f"m{m}_bias",
        ]
    return fields


def parse_ctrl_line(line: str, t0: float) -> dict | None:
    match = CTRL_RE.search(line)
    if not match:
        return None

    now = time.time()
    row = {"t_wall": f"{now:.6f}", "t_rel": f"{now - t0:.6f}", "line": line.strip()}

    for joint_match in JOINT_RE.finditer(line):
        idx = int(joint_match.group(1))
        if idx >= 4:
            continue
        target = float(joint_match.group(2))
        actual = float(joint_match.group(3))
        row[f"j{idx}_target"] = target
        row[f"j{idx}_actual"] = actual
        row[f"j{idx}_error"] = target - actual

    for motor_match in MOTOR_RE.finditer(line):
        idx = int(motor_match.group(1))
        if idx >= 5:
            continue
        row[f"m{idx}_len0"] = float(motor_match.group(2))
        row[f"m{idx}_len1"] = float(motor_match.group(3))
        row[f"m{idx}_len2"] = float(motor_match.group(4))
        row[f"m{idx}_map"] = float(motor_match.group(5))
        row[f"m{idx}_solver"] = int(motor_match.group(6))
        row[f"m{idx}_cmd"] = int(motor_match.group(7))
        row[f"m{idx}_now"] = int(motor_match.group(8))
        row[f"m{idx}_load"] = int(motor_match.group(9))
        row[f"m{idx}_cur"] = int(motor_match.group(10))
        row[f"m{idx}_bias"] = int(motor_match.group(11))

    return row


def write_command(ser: serial.Serial, command: str) -> None:
    if not command:
        return
    ser.write((command.strip() + "\n").encode("ascii"))
    ser.flush()
    print(f">>> {command.strip()}")


def write_command_list(ser: serial.Serial, commands: str) -> None:
    for part in commands.split(";"):
        command = part.strip()
        if not command:
            continue
        write_command(ser, command)
        lower_command = command.lower()
        if lower_command == "zero":
            time.sleep(0.8)
        elif lower_command in ("start", "enable", "run"):
            time.sleep(0.3)
        else:
            time.sleep(0.03)


def send_step_sequence(ser: serial.Serial, joint: int, amplitude: float, hold_s: float) -> None:
    for target in (0.0, amplitude, 0.0, -amplitude, 0.0):
        write_command(ser, f"j{joint} {target:.3f}")
        time.sleep(hold_s)


def send_sine_sequence(
    ser: serial.Serial,
    joint: int,
    amplitude: float,
    frequency: float,
    duration_s: float,
    interval_s: float,
) -> None:
    start = time.time()
    next_send = start
    while True:
        now = time.time()
        elapsed = now - start
        if elapsed > duration_s:
            break
        if now >= next_send:
            target = amplitude * math.sin(2.0 * math.pi * frequency * elapsed)
            write_command(ser, f"j{joint} {target:.3f}")
            next_send += interval_s
        time.sleep(0.005)
    write_command(ser, f"j{joint} 0")


class TargetScheduler:
    def __init__(self, args):
        self.args = args
        self.started = False
        self.done = False
        self.start_time = 0.0
        self.next_send = 0.0
        self.step_state = "idle"
        self.step_settle_start = 0.0
        self.step_baseline_until = 0.0
        self.step_reached_until = 0.0
        self.latest_actual = [math.nan, math.nan, math.nan, math.nan]

    def begin_at(self, start_time: float) -> None:
        self.start_time = start_time
        self.next_send = start_time

    def update_latest(self, row: dict) -> None:
        for joint in range(4):
            value = row.get(f"j{joint}_actual")
            if value not in (None, ""):
                self.latest_actual[joint] = float(value)

    def _joints_near_targets(self, targets: list[float]) -> bool:
        for actual, target in zip(self.latest_actual, targets):
            if not math.isfinite(actual):
                return False
            if abs(actual - target) > self.args.step_settle_tolerance:
                return False
        return True

    def _format_actuals(self) -> str:
        return (
            f"J0={self.latest_actual[0]:.2f} J1={self.latest_actual[1]:.2f} "
            f"J2={self.latest_actual[2]:.2f} J3={self.latest_actual[3]:.2f}"
        )

    def update(self, ser: serial.Serial, now: float) -> bool:
        if self.done or now < self.start_time:
            return False
        if self.args.step_joint is not None:
            return self._update_step(ser, now)
        if self.args.sine_joint is not None:
            return self._update_sine(ser, now)
        self.done = True
        return True

    def _update_step(self, ser: serial.Serial, now: float) -> bool:
        joint = self.args.step_joint
        if self.step_state == "idle":
            print(
                "Settling J0/J1/J2/J3 to 0 deg; waiting for all "
                f"abs(actual) <= {self.args.step_settle_tolerance:.2f} deg"
            )
            for zero_joint in range(4):
                write_command(ser, f"j{zero_joint} 0.000")
                time.sleep(0.03)
            self.step_settle_start = now
            self.next_send = now + 1.0
            self.step_state = "settle"
            return False
        if self.step_state == "settle":
            if self._joints_near_targets([0.0, 0.0, 0.0, 0.0]):
                print(
                    f"Settled all joints: {self._format_actuals()}; "
                    f"holding baseline {self.args.step_baseline_hold:.1f}s"
                )
                self.step_baseline_until = now + self.args.step_baseline_hold
                self.step_state = "baseline"
            elif now - self.step_settle_start >= self.args.step_settle_timeout:
                print(
                    "Step test aborted: not all joints settled within "
                    f"+/-{self.args.step_settle_tolerance:.2f} deg in "
                    f"{self.args.step_settle_timeout:.1f}s. Last actual: {self._format_actuals()}"
                )
                self.done = True
                return True
            elif now >= self.next_send:
                for zero_joint in range(4):
                    write_command(ser, f"j{zero_joint} 0.000")
                    time.sleep(0.03)
                self.next_send = now + 1.0
            return False
        if self.step_state == "baseline":
            if now >= self.step_baseline_until:
                print(f"Step j{joint}: 0 -> {self.args.step_amp:.3f} deg; other joints stay at 0 deg")
                write_command(ser, f"j{joint} {self.args.step_amp:.3f}")
                self.next_send = now + self.args.step_hold
                self.step_state = "hold"
            return False
        if self.step_state == "hold":
            targets = [0.0, 0.0, 0.0, 0.0]
            targets[joint] = self.args.step_amp
            if self._joints_near_targets(targets):
                if self.step_reached_until <= 0.0:
                    self.step_reached_until = now + self.args.step_after_reached_hold
                    print(
                        f"Reached step targets: {self._format_actuals()}; "
                        f"recording {self.args.step_after_reached_hold:.1f}s more"
                    )
                elif now >= self.step_reached_until:
                    self.done = True
                    return True
            else:
                self.step_reached_until = 0.0
            if now >= self.next_send:
                print(f"Step hold timeout reached before all targets settled. Last actual: {self._format_actuals()}")
                self.done = True
                return True
        return False

    def _update_sine(self, ser: serial.Serial, now: float) -> bool:
        elapsed = now - self.start_time
        if elapsed > self.args.sine_duration:
            write_command(ser, f"j{self.args.sine_joint} 0")
            self.done = True
            return True
        if now >= self.next_send:
            target = self.args.sine_amp * math.sin(2.0 * math.pi * self.args.sine_freq * elapsed)
            write_command(ser, f"j{self.args.sine_joint} {target:.3f}")
            self.next_send += self.args.sine_interval
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Record MCP5 control text logs to CSV.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM4.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", default=default_output_path())
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to record. 0 means until Ctrl+C.")
    parser.add_argument("--init", default="text; zero; start; degree; j0 0; j1 0; j2 0; j3 0")
    parser.add_argument("--step-joint", type=int, choices=range(4))
    parser.add_argument("--step-amp", type=float, default=5.0)
    parser.add_argument("--step-hold", type=float, default=5.0)
    parser.add_argument("--step-settle-tolerance", type=float, default=1.0)
    parser.add_argument("--step-settle-timeout", type=float, default=30.0)
    parser.add_argument("--step-baseline-hold", type=float, default=1.0)
    parser.add_argument("--step-after-reached-hold", type=float, default=5.0)
    parser.add_argument("--sine-joint", type=int, choices=range(4))
    parser.add_argument("--sine-amp", type=float, default=5.0)
    parser.add_argument("--sine-freq", type=float, default=0.05)
    parser.add_argument("--sine-duration", type=float, default=60.0)
    parser.add_argument("--sine-interval", type=float, default=0.1)
    parser.add_argument("--stop-on-exit", action="store_true", help="Send stop before closing the serial port.")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    fieldnames = build_fieldnames()
    t0 = time.time()
    rows = 0
    raw_output = os.path.splitext(args.output)[0] + ".raw.txt"

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser, open(
        args.output, "w", newline="", encoding="utf-8"
    ) as csv_file, open(raw_output, "w", encoding="utf-8") as raw_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        print(f"Recording {args.port} at {args.baud} -> {args.output}")
        print(f"Raw text -> {raw_output}")

        if args.init:
            write_command_list(ser, args.init)

        scheduled_start_time = time.time() + 1.0
        scheduler = TargetScheduler(args)
        scheduler.begin_at(scheduled_start_time)
        deadline = None if args.duration <= 0 else time.time() + args.duration

        try:
            while True:
                now = time.time()
                if deadline is not None and now >= deadline:
                    break

                if not scheduler.done:
                    scheduler.update(ser, now)

                raw = ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    raw_file.write(line + "\n")
                    row = parse_ctrl_line(line, t0)
                    if row:
                        scheduler.update_latest(row)
                        writer.writerow(row)
                        rows += 1
                        if rows % 20 == 0:
                            csv_file.flush()
                            print(f"rows={rows} t={float(row['t_rel']):.1f}s")

                if scheduler.done and deadline is None:
                    deadline = time.time() + 2.0
        except KeyboardInterrupt:
            print("Interrupted.")
        finally:
            if args.stop_on_exit:
                try:
                    write_command(ser, "stop")
                except Exception as exc:
                    print(f"Failed to send stop on exit: {exc}")
            csv_file.flush()

    print(f"Saved {rows} rows to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
