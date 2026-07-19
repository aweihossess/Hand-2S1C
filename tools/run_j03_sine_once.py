"""Run one guarded J03 sine-tracking experiment through the open desktop UI.

The experiment starts and ends at [10, 10, 20, 10] degrees.  J03 follows
20 - 10*cos(2*pi*0.1*t), so the sine phase begins continuously at 10 degrees.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

from pywinauto import keyboard, mouse

from live_pi_autotune import (
    LiveGui,
    SafetyAbort,
    as_float,
    find_encoder_window,
    read_rows,
    wait_for_new_record,
)


RECORD_DIR = Path(__file__).resolve().parents[1] / "run_data" / "continuous_records"
INITIAL = (10.0, 10.0, 20.0, 10.0)
TEST_KP = 1.0
TEST_KI = 0.3
RESTORE_KP = 0.5
RESTORE_KI = 0.02
SINE_MEAN_DEG = 20.0
SINE_AMPLITUDE_DEG = 10.0
SINE_FREQUENCY_HZ = 0.1
SINE_DURATION_S = 30.0
UPDATE_PERIOD_S = 0.4
MAX_TENSION_N = 40.0


def check_latest(path: Path) -> float:
    rows = read_rows(path)
    if not rows:
        return 0.0
    row = rows[-1]
    if row.get("force_complete") not in {"1", "1.0", "True", "true"}:
        raise SafetyAbort("five-channel force data are incomplete")

    peak_tension = 0.0
    for motor in range(5):
        force = as_float(row, f"tension_m{motor:02d}_n")
        if force is None:
            raise SafetyAbort(f"M{motor:02d} tension is invalid")
        peak_tension = max(peak_tension, -force)
        if row.get(f"servo_m{motor:02d}_online") not in {"1", "1.0", "True", "true"}:
            raise SafetyAbort(f"M{motor:02d} is offline")
    if peak_tension >= MAX_TENSION_N:
        raise SafetyAbort(
            f"tension reached guard limit: {peak_tension:.1f} N >= {MAX_TENSION_N:.1f} N"
        )

    limits = ((-5.0, 30.0), (0.5, 30.0), (-5.0, 30.0), (-5.0, 35.0))
    for joint, (lower, upper) in enumerate(limits):
        if row.get(f"joint_j{joint:02d}_valid") not in {"1", "1.0", "True", "true"}:
            raise SafetyAbort(f"J{joint:02d} encoder is invalid")
        angle = as_float(row, f"joint_j{joint:02d}_deg")
        if angle is None or not lower <= angle <= upper:
            raise SafetyAbort(
                f"J{joint:02d}={angle} deg is outside the guarded range [{lower}, {upper}]"
            )
    return peak_tension


def guarded_hold(path: Path, duration_s: float) -> float:
    deadline = time.monotonic() + duration_s
    peak = 0.0
    while time.monotonic() < deadline:
        time.sleep(0.25)
        peak = max(peak, check_latest(path))
    return peak


def send_targets_fast(gui: LiveGui, targets: tuple[float, float, float, float]) -> None:
    """Update an already-focused Tk window without re-querying UI Automation."""
    text = ";".join(f"{target:.3f}" for target in targets)
    mouse.click(button="left", coords=gui.TARGET_ENTRY)
    keyboard.send_keys("^a", pause=0.005)
    keyboard.send_keys(text, pause=0.001, with_spaces=True)
    mouse.click(button="left", coords=gui.TARGET_ALL)
    time.sleep(0.03)


def main() -> int:
    gui = LiveGui(find_encoder_window())
    started_at = time.time()
    record_path: Path | None = None
    recording = False
    safe_to_return = True
    peak_tension = 0.0
    target_updates = 0
    sine_started = 0.0

    try:
        gui.set_gains(TEST_KP, TEST_KI)
        gui.toggle_record()
        recording = True
        record_path = wait_for_new_record(RECORD_DIR, started_at)

        gui.send_joint_targets(INITIAL)
        peak_tension = max(peak_tension, guarded_hold(record_path, 15.0))

        sine_started = time.monotonic()
        gui.window.set_focus()
        next_update = sine_started
        while True:
            now = time.monotonic()
            elapsed = now - sine_started
            if elapsed >= SINE_DURATION_S:
                break
            if now < next_update:
                time.sleep(min(0.03, next_update - now))
                continue
            j03 = SINE_MEAN_DEG - SINE_AMPLITUDE_DEG * math.cos(
                2.0 * math.pi * SINE_FREQUENCY_HZ * elapsed
            )
            send_targets_fast(gui, (INITIAL[0], INITIAL[1], INITIAL[2], j03))
            target_updates += 1
            peak_tension = max(peak_tension, check_latest(record_path))
            next_update += UPDATE_PERIOD_S

        gui.send_joint_targets(INITIAL)
        peak_tension = max(peak_tension, guarded_hold(record_path, 12.0))
    except SafetyAbort as exc:
        safe_to_return = False
        gui.stop_control()
        print(f"SAFETY_ABORT={exc}")
    finally:
        if recording:
            gui.toggle_record()
        gui.set_gains(RESTORE_KP, RESTORE_KI)
        if safe_to_return:
            gui.send_joint_targets(INITIAL)

    if record_path is None:
        raise RuntimeError("recording CSV was not created")
    print(f"CSV={record_path}")
    print(f"SINE_UPDATES={target_updates}")
    print(f"PEAK_TENSION_N={peak_tension:.3f}")
    return 0 if safe_to_return else 2


if __name__ == "__main__":
    raise SystemExit(main())
