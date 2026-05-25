#!/usr/bin/env python3
"""Release MCP tendons by commanding M00/M01/M02 in the loosening direction."""

from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

from low_tension_takeup import (
    MOTOR_COUNT,
    MotorState,
    JointState,
    TextFirmwareClient,
    build_motor_pos_abs_cmd,
    parse_lines,
)


BAUD = 921600
MOTORS = {
    0: -1,  # M00 tighten_dir=+1, release is -
    1: -1,  # M01 tighten_dir=+1, release is -
    2: -1,  # M02 tighten_dir=+1, release is -
}
TOTAL_RELEASE_COUNTS = 4096 * 2
STEP_COUNTS = 160
PERIOD_S = 0.04


def refresh(client: TextFirmwareClient, motor_states: dict[int, MotorState]) -> None:
    lines: list[str] = []
    lines.extend(client.command_and_collect("servo", settle_s=0.025))
    lines.extend(client.command_and_collect("load", settle_s=0.025))
    lines.extend(client.read_available_lines(wait_s=0.005))
    parse_lines(lines, motor_states, {})


def build_targets(motor_states: dict[int, MotorState], selected_targets: dict[int, int]) -> list[int]:
    targets = [0] * MOTOR_COUNT
    for ch in range(MOTOR_COUNT):
        state = motor_states.setdefault(ch, MotorState())
        if ch in selected_targets:
            targets[ch] = selected_targets[ch]
        elif state.motor_abs is not None:
            targets[ch] = int(state.motor_abs)
    return targets


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: release_mcp_two_turns.py COM10 [out.csv]", file=sys.stderr)
        return 2
    port = sys.argv[1]
    out_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path("run_data/mcp_release_two_turns.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    motor_states: dict[int, MotorState] = {ch: MotorState() for ch in range(MOTOR_COUNT)}
    client = TextFirmwareClient(port, BAUD)
    output_enabled = False
    try:
        client.read_available_lines(wait_s=0.1)
        client.command_and_collect("text", settle_s=0.05)
        refresh(client, motor_states)
        start_abs: dict[int, int] = {}
        for ch in MOTORS:
            state = motor_states.setdefault(ch, MotorState())
            if state.motor_abs is None or not state.online:
                raise RuntimeError(f"M{ch:02d} is not online or has no motor_abs")
            start_abs[ch] = int(state.motor_abs)

        print("MCP release two turns")
        for ch in MOTORS:
            end_abs = start_abs[ch] + MOTORS[ch] * TOTAL_RELEASE_COUNTS
            print(f"  M{ch:02d}: {start_abs[ch]} -> {end_abs}")
        print("Type YES and press Enter to start, or Ctrl+C to cancel.")
        if input("> ").strip() != "YES":
            print("Cancelled; motors were not enabled.")
            return 2

        client.command_and_collect("start", settle_s=0.05)
        client.command_and_collect("direct", settle_s=0.05)
        output_enabled = True

        start_time = time.time()
        fields = ["time_s", "phase"]
        for ch in MOTORS:
            fields.extend([
                f"m{ch:02d}_abs",
                f"m{ch:02d}_target",
                f"m{ch:02d}_load",
                f"m{ch:02d}_current",
            ])

        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for travel in range(STEP_COUNTS, TOTAL_RELEASE_COUNTS + STEP_COUNTS, STEP_COUNTS):
                travel = min(travel, TOTAL_RELEASE_COUNTS)
                refresh(client, motor_states)
                selected_targets = {
                    ch: start_abs[ch] + MOTORS[ch] * travel
                    for ch in MOTORS
                }
                client.write_binary_frame(build_motor_pos_abs_cmd(build_targets(motor_states, selected_targets)))
                row: dict[str, object] = {"time_s": time.time() - start_time, "phase": "release"}
                for ch in MOTORS:
                    state = motor_states.setdefault(ch, MotorState())
                    row[f"m{ch:02d}_abs"] = "" if state.motor_abs is None else state.motor_abs
                    row[f"m{ch:02d}_target"] = selected_targets[ch]
                    row[f"m{ch:02d}_load"] = "" if state.load is None else state.load
                    row[f"m{ch:02d}_current"] = "" if state.current is None else state.current
                writer.writerow(row)
                if travel % 640 == 0 or travel == TOTAL_RELEASE_COUNTS:
                    print(
                        f"travel={travel}/{TOTAL_RELEASE_COUNTS} "
                        + " ".join(
                            f"M{ch:02d} abs={motor_states[ch].motor_abs} tgt={selected_targets[ch]} "
                            f"load={motor_states[ch].load} cur={motor_states[ch].current}"
                            for ch in MOTORS
                        )
                    )
                time.sleep(PERIOD_S)

        print(f"Done. CSV: {out_path}")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by Ctrl+C.")
        return 130
    finally:
        if output_enabled:
            print("Stopping output and leaving firmware in STOP state.")
            try:
                client.command_and_collect("stop", settle_s=0.05)
            except Exception as exc:
                print(f"WARNING: failed to send stop: {exc}", file=sys.stderr)
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
