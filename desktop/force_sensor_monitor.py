"""Force sensor Modbus-RTU monitor.

Based on the supplied manual:
- Default serial settings: 9600 baud, 8 data bits, no parity, 1 stop bit.
- Default slave address: 1.
- Live weight/force value: holding registers 0x0000 and 0x0001.
- The 32-bit signed value uses the lower 16-bit word first.

The UI uses Tkinter so it can run without Qt. On Windows it can talk to COM
ports through pyserial when installed, or through the native Win32 serial API.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import os
import queue
import random
import re
import statistics
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


def configure_tcl_tk_env() -> None:
    python_dir = Path(sys.executable).resolve().parent
    tcl_root = python_dir / "tcl"
    tcl_dir = tcl_root / "tcl8.6"
    tk_dir = tcl_root / "tk8.6"
    if (tcl_dir / "init.tcl").exists():
        os.environ.setdefault("TCL_LIBRARY", tcl_dir.as_posix())
    if (tk_dir / "tk.tcl").exists():
        os.environ.setdefault("TK_LIBRARY", tk_dir.as_posix())
    if tcl_root.exists():
        os.environ.setdefault("TCLLIBPATH", tcl_root.as_posix())


configure_tcl_tk_env()

from tkinter import filedialog, messagebox, ttk
import tkinter as tk

try:
    from .linearity_analysis import LinearityAnalysisError, analyze_linearity_csv
except ImportError:
    from linearity_analysis import LinearityAnalysisError, analyze_linearity_csv

try:
    from .multi_input_analysis import (
        MultiInputAnalysisError,
        analyze_multi_input_csv,
        build_multi_input_sequence,
        input_independence_metrics,
        parse_number_vector,
        parse_split_ratios,
        split_labels,
    )
except ImportError:
    from multi_input_analysis import (
        MultiInputAnalysisError,
        analyze_multi_input_csv,
        build_multi_input_sequence,
        input_independence_metrics,
        parse_number_vector,
        parse_split_ratios,
        split_labels,
    )

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # The Win32 backend below is enough on this Windows setup.
    serial = None
    list_ports = None

if sys.platform == "win32":
    import winreg
    from ctypes import wintypes


BAUD_RATES = [2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200, 256000, 500000]
DEFAULT_BAUDRATE = 115200
DEFAULT_CHANNEL = 5
DEFAULT_FORCE_INTERVAL_MS = 100
REFERENCE_CHANNEL = 1
DISPLAY_CHANNELS = [1, 2, 3, 4, 5]
FORCE_CHANNEL_TO_MOTOR = {
    2: 0,
    1: 1,
    3: 2,
    4: 3,
    5: 4,
}
CHANNEL_COLORS = {
    1: "#dc2626",
    2: "#f97316",
    3: "#84cc16",
    4: "#06b6d4",
    5: "#2563eb",
}
MAX_SAMPLES = 5000
MAX_TABLE_ROWS = 300
RELATIVE_UNITS_PER_NEWTON = 10.0
FORCE_ZERO_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "force_sensor_zero.json"
FORCE_ZERO_CAPTURE_SECONDS = 2.0
FORCE_ZERO_MIN_SAMPLES_PER_CHANNEL = 8
FORCE_ZERO_MAX_SPAN_UNITS = 20
ENCODER_ZERO_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "encoder_zero.json"
TRAINING_DEFAULT_JOINT_RANGES = "J0:-15:15,J1:0:30,J2:0:60,J3:-45:45"
TRAINING_DEFAULT_SAMPLE_INTERVAL_MS = 200
TRAINING_DEFAULT_TARGET_PERIOD_S = 5.0
TRAINING_DEFAULT_LOAD_LIMIT = ""
TRAINING_DEFAULT_TARGET_MODE = "coverage"
MANUAL_RECORD_STALE_LIMIT_S = 1.5
MANUAL_RECORD_JOINTS = tuple(range(4))
CONTINUOUS_RECORD_DEFAULT_INTERVAL_MS = 100
STEP_TEST_DEFAULT_POSES = (
    "-5;11;10;-35|-5;15;10;0|-5;20;10;35|"
    "-5;15;30;-35|-5;20;30;0|-5;11;30;35|"
    "-5;20;50;-35|-5;11;50;0|-5;15;50;35|"
    "0;15;10;-35|0;20;10;0|0;11;10;35|"
    "0;20;30;-35|0;11;30;0|0;15;30;35|"
    "0;11;50;-35|0;15;50;0|0;20;50;35|"
    "5;20;10;-35|5;11;10;0|5;15;10;35|"
    "5;11;30;-35|5;15;30;0|5;20;30;35|"
    "5;15;50;-35|5;20;50;0|5;11;50;35"
)
STEP_TEST_REQUIRED_POSE_COUNT = 27
STEP_TEST_DEFAULT_AMPLITUDES = "10"
STEP_TEST_DEFAULT_REPEATS = 3
STEP_TEST_DEFAULT_BASELINE_S = 2.0
STEP_TEST_DEFAULT_SETTLE_BAND_DEG = 1.0
STEP_TEST_DEFAULT_SETTLE_HOLD_S = 2.0
STEP_TEST_DEFAULT_TIMEOUT_S = 25.0
STEP_TEST_TICK_MS = 100
STEP_TEST_J01_MIN_DEG = 0.0
STEP_TEST_JOINT_LIMITS = {
    0: (-15.0, 15.0),
    1: (0.0, 30.0),
    2: (0.0, 60.0),
    3: (-45.0, 45.0),
}
SINE_TEST_DEFAULT_POSES = (
    "-10;7;5;-40|-10;15;5;0|-10;23;5;40|"
    "-10;15;30;-40|-10;23;30;0|-10;7;30;40|"
    "-10;23;55;-40|-10;7;55;0|-10;15;55;40|"
    "0;15;5;-40|0;23;5;0|0;7;5;40|"
    "0;23;30;-40|0;7;30;0|0;15;30;40|"
    "0;7;55;-40|0;15;55;0|0;23;55;40|"
    "10;23;5;-40|10;7;5;0|10;15;5;40|"
    "10;7;30;-40|10;15;30;0|10;23;30;40|"
    "10;15;55;-40|10;23;55;0|10;7;55;40"
)
SINE_TEST_REQUIRED_POSE_COUNT = 27
SINE_TEST_DEFAULT_AMPLITUDES = "5"
SINE_TEST_DEFAULT_FREQUENCIES_HZ = "0.05;0.10;0.20"
SINE_TEST_DEFAULT_CYCLES = 3
SINE_TEST_DEFAULT_REPEATS = 2
SINE_TEST_DEFAULT_BASELINE_S = 2.0
SINE_TEST_DEFAULT_SETTLE_BAND_DEG = 1.0
SINE_TEST_DEFAULT_SETTLE_HOLD_S = 2.0
SINE_TEST_DEFAULT_POSE_TIMEOUT_S = 25.0
SINE_TEST_TICK_MS = 100
LINEARITY_DEFAULT_STATE_MOTOR_POSITIONS = ""
LINEARITY_DEFAULT_MOTORS = "M00,M01,M02,M03,M04"
LINEARITY_DEFAULT_DELTAS = "0,50,100,50,0,-50,-100,-50,0"
LINEARITY_DEFAULT_REPEATS = 3
LINEARITY_DEFAULT_STABLE_HOLD_S = 1.0
LINEARITY_POST_RECORD_HOLD_S = 1.0
LINEARITY_DEFAULT_TIMEOUT_S = 40.0
LINEARITY_TRACE_SAMPLE_INTERVAL_S = 0.1
LINEARITY_SERVO_DEVIATION_ABORT_COUNTS = 300
LINEARITY_SERVO_TARGET_TOLERANCE_COUNTS = 10
LINEARITY_SERVO_DEVIATION_GRACE_S = 0.8
LINEARITY_SERVO_LIMIT_MARGIN_COUNTS = 500
FORCE_HARD_ABORT_N = -90.0
# When the hard threshold is crossed, all five motors are driven in the
# experimentally verified loosening direction.  They are stopped only after
# every tendon is less tight than this hysteresis threshold.
FORCE_HARD_RELAX_STOP_N = -70.0
FORCE_HARD_RELAX_STEP_COUNTS = 200
FORCE_HARD_RELAX_INTERVAL_MS = 100
LINEARITY_TIGHT_ABORT_N = FORCE_HARD_ABORT_N
MULTI_INPUT_DEFAULT_MODE = "prbs"
MULTI_INPUT_DEFAULT_AMPLITUDES = "200;200;200;200;200"
MULTI_INPUT_DEFAULT_SAMPLE_COUNT = 120
MULTI_INPUT_DEFAULT_SEED = 20260716
MULTI_INPUT_DEFAULT_SPLIT = "0.60;0.20;0.20"
MULTI_INPUT_DEFAULT_RIDGE_LAMBDA = "1.0"
MULTI_INPUT_DEFAULT_FREQUENCIES = "0.037;0.053;0.071;0.089;0.113"
MULTI_INPUT_DEFAULT_RECORD_INTERVAL_MS = 100
MULTI_INPUT_MAX_CORRELATION = 0.90
TRAINING_TARGET_MODES = ("coverage", "single", "combo", "roundtrip", "step")
TRAINING_STEP_TARGETS = {
    0: 10.0,
    1: 20.0,
    2: 20.0,
    3: 35.0,
}
TRAINING_TARGET_COVERAGE_FRACTIONS = (0.08, 0.92, 0.25, 0.75, 0.50)
TRAINING_TARGET_JITTER_FRACTION = 0.04
TRAINING_COMBO_TARGET_PATTERNS = (
    {2: 0.92, 3: 0.92},
    {1: 0.92, 2: 0.50, 3: 0.50},
    {0: 0.08, 2: 0.75, 3: 0.25},
    {0: 0.92, 2: 0.25, 3: 0.75},
    {1: 0.25, 2: 0.92, 3: 0.08},
    {1: 0.75, 2: 0.08, 3: 0.92},
)
TRAINING_ROUNDTRIP_FRACTIONS = (0.08, 0.92, 0.08, 0.50)
PRETENSION_DEFAULT_TARGET_N = 0.0
PRETENSION_DEFAULT_TIGHT_LIMIT_N = -80.0
PRETENSION_RETURN_MOTOR = 4
PRETENSION_RETURN_TARGET_N = -15.0
PRETENSION_RETURN_TIGHT_LIMIT_N = -20.0
PRETENSION_ALWAYS_TENSION_MOTORS = {2, PRETENSION_RETURN_MOTOR}
PRETENSION_DEFAULT_STEP_TEXT = "M00:100,M01:100,M02:100,M03:100,M04:100"
PRETENSION_DEFAULT_TIMEOUT_S = 20.0
PRETENSION_STEP_INTERVAL_S = 0.2
TENSION_BIAS_ABS_LIMIT_COUNTS = 7200
TENSION_RETURN_BIAS_ABS_LIMIT_COUNTS = 7200
DEGREE_FORCE_STALE_LIMIT_S = 1.5
DEGREE_FORCE_STEP_INTERVAL_S = PRETENSION_STEP_INTERVAL_S
DEGREE_FORCE_ANGLE_READY_DEG = 1.0
DEGREE_FORCE_ANGLE_HOLD_S = 1.0
SAFE_RELAX_TARGET_ABS = -6000
SAFE_RELAX_START_DELAY_MS = 300
SAFE_RELAX_COMPLETE_DELAY_MS = 1200
FORCE_TARGET_DEFAULT_INTERVAL_MS = 100
FORCE_TARGET_DEFAULT_STEP_COUNTS = 20
FORCE_TARGET_DEFAULT_DEADBAND_N = 0.0
FORCE_TARGET_DEFAULT_N_BY_CHANNEL = {5: -10.0}
FORCE_TARGET_NEAR_ERROR_N = 5.0
FORCE_TARGET_NEAR_STEP_COUNTS = 1
FORCE_TARGET_NEAR_STEP_INTERVAL_S = 0.2
FORCE_TARGET_SETTLE_ERROR_N = 1.0
FORCE_TARGET_SETTLE_HOLD_S = 2.0
FORCE_TARGET_MIN_ABS = -30719
FORCE_TARGET_MAX_ABS = 30719
FEEDBACK_PRETENSION_LOOSE_LIMIT_N = -10.0
FEEDBACK_PRETENSION_TIGHT_LIMIT_N = -15.0
FEEDBACK_PRETENSION_TARGET_N = (
    FEEDBACK_PRETENSION_LOOSE_LIMIT_N + FEEDBACK_PRETENSION_TIGHT_LIMIT_N
) / 2.0
FEEDBACK_PRETENSION_DEADBAND_N = (
    FEEDBACK_PRETENSION_LOOSE_LIMIT_N - FEEDBACK_PRETENSION_TIGHT_LIMIT_N
) / 2.0
FEEDBACK_PRETENSION_HOLD_S = 1.0
FEEDBACK_PRETENSION_INTERVAL_MS = 100
FEEDBACK_PRETENSION_STEP_COUNTS = 20
FEEDBACK_PRETENSION_ABORT_N = FORCE_HARD_ABORT_N
FEEDBACK_PRETENSION_TIMEOUT_S = 30.0

# One-dimensional null-space pretension for the identified 4x5 tendon
# Jacobian.  The vector is L2-normalized and satisfies J @ n ~= 0.  A scalar
# alpha_counts is converted to the five firmware tensionbias values with
# round(alpha_counts * n[i]).  Positive alpha is the experimentally verified
# tightening direction for M00..M04.
NULLSPACE_PRETENSION_VECTOR = (
    0.40395201,
    0.39558015,
    0.61216442,
    0.37580693,
    0.40541705,
)
# The firmware now applies both the angle controller and tension biases to one
# absolute requested target before the finite-difference limiter.  It is
# therefore safe to attach the staged tension controller to every degree run.
NULLSPACE_PRETENSION_AUTOSTART_ENABLED = True
# Negative sensor values mean tension.  When enabled, normal null-space
# regulation keeps the loosest tendon at or below -5 N.  Reaching -90 N no
# longer ramps alpha down: it immediately clears the bias and hands control to
# the active all-motor release.  The independent -90 N guard is a second path.
# Robonaut-style bounds use positive tension magnitudes T=-F_sensor.  Thus
# T_min=5 N and T_max=90 N correspond to sensor readings -5 N and -90 N.
NULLSPACE_TENSION_MIN_N = 5.0
NULLSPACE_TENSION_MAX_N = 90.0
NULLSPACE_PRETENSION_F_MIN_N = -NULLSPACE_TENSION_MIN_N
NULLSPACE_PRETENSION_RELEASE_N = -NULLSPACE_TENSION_MAX_N
# alpha_d is solved in the positive-tension force domain.  This bounded inner
# regulator converts its force error to the existing motor-position alpha.
NULLSPACE_ALPHA_FORCE_KP_COUNTS_PER_N = 20.0
NULLSPACE_ALPHA_FORCE_DEADBAND_N = 0.25
NULLSPACE_PRETENSION_ALPHA_STEP_COUNTS = 50.0
NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS = 7200.0
NULLSPACE_PRETENSION_INTERVAL_MS = 100
DIRECT_ABS_UI_MOTOR_COUNT = 5
DIRECT_ABS_CONFIRM_DELTA_COUNTS = 4096
DIRECT_ABS_START_DELAY_MS = 300
DIRECT_ABS_VERIFY_INTERVAL_MS = 250
DIRECT_ABS_VERIFY_TIMEOUT_S = 8.0
DIRECT_ABS_TARGET_TOLERANCE_COUNTS = 20
DIRECT_ABS_MOTION_THRESHOLD_COUNTS = 5
ENCODER_COUNT = 21
SERVO_COUNT = 22
ENCODER_RAW_TO_DEG = 360.0 / 16384.0
ENCODER_SERVO_DEFAULT_PORT = "COM6"
ENCODER_SERVO_DEFAULT_BAUDRATE = 921600
ENCODER_SERVO_TEXT_POLL_INTERVAL_S = 0.2
ENCODER_SERVO_MAX_HISTORY = 1200
UPPER_PROTOCOL_HEADER = 0xFE
UPPER_PROTOCOL_TAIL = 0xFF
PACKET_TYPE_SENSOR = 0x01
PACKET_TYPE_CALIB_ACK = 0x02
PACKET_TYPE_SERVO_ANGLE = 0x03
PACKET_TYPE_JOINT_DEBUG = 0x04
PACKET_TYPE_SERVO_TELEM = 0x05
PACKET_TYPE_PROTO_ACK = 0x06
PACKET_TYPE_FAULT_STATUS = 0x07
PACKET_TYPE_RELEASE_FAULT = 0x08
PACKET_TYPE_SERVO_RAW = 0x09
PACKET_TYPE_CONTROL_STATUS = 0x0B
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE
CMD_ANGLE_CTRL = 0xCB
CMD_SENSOR_STREAM_MODE = 0xD1
CMD_MOTOR_POS_ABS = 0xD3
SENSOR_STREAM_MODE_SIGNED_I16 = 1
DISCONNECT_SENTINEL = 0x7FFF
ENCODER_SERVO_COLORS = [
    "#dc2626",
    "#ea580c",
    "#ca8a04",
    "#65a30d",
    "#16a34a",
    "#059669",
    "#0891b2",
    "#0284c7",
    "#2563eb",
    "#4f46e5",
    "#7c3aed",
    "#9333ea",
    "#c026d3",
    "#db2777",
    "#e11d48",
    "#475569",
]
ENCODER_PLOT_JOINTS = (0, 1, 2, 3)
DEGREE_TARGET_RE = re.compile(r"\bdegree\s*;\s*j(\d+)\s+([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)
DIRECT_TARGET_RE = re.compile(r"\b(?:direct\s*;\s*)?m(\d+)\s+([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)


@dataclass(frozen=True)
class ForceSample:
    timestamp: float
    channel: int
    value: int
    low_word: int
    high_word: int
    request_hex: str
    response_hex: str


@dataclass(frozen=True)
class NullspaceTensionAllocation:
    tension_by_motor: tuple[float, ...]
    task_tension_by_motor: tuple[float, ...]
    alpha_measured_n: float
    alpha_min_n: float
    alpha_max_n: float
    alpha_desired_n: float
    feasible: bool


@dataclass(frozen=True)
class JointDebugSnapshot:
    timestamp: float
    joint_index: int
    valid: bool
    target_deg: float
    actual_deg: float
    loop1_output: float
    loop2_actual: float
    loop2_output: float
    target_length: float
    actual_length: float
    mapped_motor_target: float
    motor_zero_abs: int
    solver_output_pos: int
    motor_targets_valid: bool
    zero_homing: bool
    cmd_valid: bool
    cmd_target_pos: int
    firmware_time_ms: int
    q_ref_deg: float
    q_fb_filtered_deg: float
    q_fb_velocity_deg_s: float
    angle_error_deg: float
    angle_integral_deg_s: float
    feedforward_counts: float
    angle_p_counts: float
    angle_i_counts: float
    angle_d_counts: float
    angle_feedback_counts: float
    tension_bias_counts: int
    pre_limit_target_pos: int
    tension_bias_enabled: bool
    command_limit_flags: int


@dataclass(frozen=True)
class EncoderServoSnapshot:
    timestamp: float
    encoder_raw: list[int]
    encoder_mapped: list[int]
    encoder_deg: list[float]
    encoder_valid: list[bool]
    servo_abs: list[int]
    servo_hardware_abs: list[int]
    servo_zero_offset: list[int]
    servo_online: list[bool]
    servo_raw: list[int]
    servo_raw_online: list[bool]
    servo_speed: list[int]
    servo_load: list[int]
    servo_current: list[int]
    servo_voltage: list[int]
    servo_temperature: list[int]
    joint_debug: dict[int, JointDebugSnapshot]
    last_line: str


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def format_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def format_force_n(relative_value: int) -> str:
    force_n = relative_value / RELATIVE_UNITS_PER_NEWTON
    if force_n == 0:
        return "0 N"
    return f"{force_n:.1f} N"


def unique_continuous_record_paths(directory: Path) -> tuple[Path, Path]:
    """Return per-run data/event paths with microsecond-resolution names."""
    now_ns = time.time_ns()
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now_ns / 1_000_000_000))
    microseconds = (now_ns // 1_000) % 1_000_000
    stem = f"continuous_record_{timestamp}_{microseconds:06d}"
    data_path = directory / f"{stem}.csv"
    return data_path, directory / f"{stem}_events.csv"


def force_zero_baseline_from_samples(values: list[int]) -> tuple[int, int]:
    """Return a robust no-load baseline and the raw peak-to-peak span."""
    if len(values) < FORCE_ZERO_MIN_SAMPLES_PER_CHANNEL:
        raise ValueError(
            f"need at least {FORCE_ZERO_MIN_SAMPLES_PER_CHANNEL} samples, got {len(values)}"
        )
    ordered = sorted(int(value) for value in values)
    trim = max(1, len(ordered) // 10) if len(ordered) >= 10 else 0
    core = ordered[trim:-trim] if trim else ordered
    baseline = int(round(sum(core) / len(core)))
    return baseline, ordered[-1] - ordered[0]


def force_motor_label(channel: int) -> str:
    motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
    return f"M{motor:02d}" if motor is not None else "--"


def force_channel_label(channel: int) -> str:
    return f"CH{channel} / {force_motor_label(channel)}"


def force_target_step_delta_from_error(
    error_n: float,
    far_step_counts: int,
    deadband_n: float = 0.0,
) -> int:
    abs_error_n = abs(error_n)
    if abs_error_n <= deadband_n:
        return 0
    if abs_error_n <= FORCE_TARGET_NEAR_ERROR_N:
        step_counts = FORCE_TARGET_NEAR_STEP_COUNTS
    else:
        step_counts = far_step_counts
    return -step_counts if error_n > 0 else step_counts


def nullspace_pretension_bias_counts(alpha_counts: float) -> dict[int, int]:
    """Map one non-negative null-space coordinate to five motor biases."""
    alpha = max(0.0, min(NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS, float(alpha_counts)))
    return {
        motor: int(round(alpha * component))
        for motor, component in enumerate(NULLSPACE_PRETENSION_VECTOR)
    }


def allocate_nullspace_internal_tension(
    force_by_motor: dict[int, float],
    *,
    tension_min_n: float = NULLSPACE_TENSION_MIN_N,
    tension_max_n: float = NULLSPACE_TENSION_MAX_N,
) -> NullspaceTensionAllocation:
    """Project measured tension and solve the Robonaut-style alpha interval.

    Force sensors report tension as negative values, while this allocator uses
    positive tension magnitudes.  With an L2-normalized positive null vector,
    ``alpha_measured = n.T @ tension`` and the range-space estimate is
    ``f_task = tension - alpha_measured * n``.
    """
    if not 0.0 <= tension_min_n < tension_max_n:
        raise ValueError("tension bounds must satisfy 0 <= f_min < f_max")
    forces = [float(force_by_motor[motor]) for motor in range(5)]
    if any(not math.isfinite(value) for value in forces):
        raise ValueError("all five allocator force values must be finite")
    null_norm_sq = sum(component * component for component in NULLSPACE_PRETENSION_VECTOR)
    if not math.isfinite(null_norm_sq) or null_norm_sq <= 1.0e-9:
        raise ValueError("null-space vector norm is invalid")

    tensions = tuple(-value for value in forces)
    alpha_measured = sum(
        NULLSPACE_PRETENSION_VECTOR[motor] * tensions[motor]
        for motor in range(5)
    ) / null_norm_sq
    task_tensions = tuple(
        tensions[motor] - alpha_measured * NULLSPACE_PRETENSION_VECTOR[motor]
        for motor in range(5)
    )
    alpha_min = max(
        (tension_min_n - task_tensions[motor]) / NULLSPACE_PRETENSION_VECTOR[motor]
        for motor in range(5)
    )
    alpha_max = min(
        (tension_max_n - task_tensions[motor]) / NULLSPACE_PRETENSION_VECTOR[motor]
        for motor in range(5)
    )
    alpha_desired = max(0.0, alpha_min)
    feasible = alpha_desired <= alpha_max + 1.0e-9
    return NullspaceTensionAllocation(
        tension_by_motor=tensions,
        task_tension_by_motor=task_tensions,
        alpha_measured_n=alpha_measured,
        alpha_min_n=alpha_min,
        alpha_max_n=alpha_max,
        alpha_desired_n=alpha_desired,
        feasible=feasible,
    )


def nullspace_alpha_count_step(alpha_error_n: float) -> int:
    """Convert force-domain internal-tension error to a bounded count step."""
    error_n = float(alpha_error_n)
    if not math.isfinite(error_n):
        raise ValueError("alpha force error must be finite")
    if abs(error_n) <= NULLSPACE_ALPHA_FORCE_DEADBAND_N:
        return 0
    raw_step = int(round(NULLSPACE_ALPHA_FORCE_KP_COUNTS_PER_N * error_n))
    if raw_step == 0:
        raw_step = 1 if error_n > 0.0 else -1
    limit = int(round(NULLSPACE_PRETENSION_ALPHA_STEP_COUNTS))
    return max(-limit, min(limit, raw_step))


def nullspace_pretension_action(force_by_motor: dict[int, float]) -> tuple[str, float, float]:
    """Choose the safety-prioritized scalar null-space action.

    Forces are negative in tension.  ``loosest`` is therefore the largest value
    and ``tightest`` the most negative value.
    """
    values = [float(force_by_motor[motor]) for motor in range(5)]
    if any(not math.isfinite(value) for value in values):
        raise ValueError("all five null-space force values must be finite")
    loosest = max(values)
    tightest = min(values)
    if tightest <= NULLSPACE_PRETENSION_RELEASE_N:
        return "release", loosest, tightest
    if loosest > NULLSPACE_PRETENSION_F_MIN_N:
        return "tighten", loosest, tightest
    return "hold", loosest, tightest


def parse_training_joint_ranges(text: str) -> list[tuple[int, float, float]]:
    ranges: list[tuple[int, float, float]] = []
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    for token in tokens:
        parts = token.replace("=", ":").split(":")
        if len(parts) != 3:
            raise ValueError(f"关节范围格式错误: {token}")
        joint_text = parts[0].strip().upper()
        if joint_text.startswith("J"):
            joint_text = joint_text[1:]
        try:
            joint = int(joint_text)
            low = float(parts[1])
            high = float(parts[2])
        except ValueError as exc:
            raise ValueError(f"关节范围不是数字: {token}") from exc
        if not 0 <= joint < ENCODER_COUNT:
            raise ValueError(f"关节索引越界: J{joint}")
        if low > high:
            raise ValueError(f"关节范围下限大于上限: {token}")
        ranges.append((joint, low, high))
    if not ranges:
        raise ValueError("至少需要一个关节范围，例如 J0:-10:10")
    return ranges


def build_training_coverage_target(
    valid_ranges: list[tuple[int, float, float]],
    target_index: int,
    jitter_fraction: float = TRAINING_TARGET_JITTER_FRACTION,
) -> tuple[int, float]:
    if not valid_ranges:
        raise ValueError("no valid training joint ranges")
    joint, low, high = valid_ranges[target_index % len(valid_ranges)]
    span = high - low
    if span <= 0:
        return joint, low

    sweep_index = target_index // len(valid_ranges)
    fraction = TRAINING_TARGET_COVERAGE_FRACTIONS[
        sweep_index % len(TRAINING_TARGET_COVERAGE_FRACTIONS)
    ]
    if jitter_fraction > 0:
        fraction += random.uniform(-jitter_fraction, jitter_fraction)
    fraction = min(0.98, max(0.02, fraction))
    return joint, low + span * fraction


def training_zero_target(low: float, high: float) -> float:
    if low <= 0.0 <= high:
        return 0.0
    return low if abs(low) <= abs(high) else high


def training_fraction_target(low: float, high: float, fraction: float) -> float:
    fraction = min(0.98, max(0.02, fraction))
    return low + (high - low) * fraction


def build_training_target_batch(
    valid_ranges: list[tuple[int, float, float]],
    target_index: int,
    mode: str,
) -> tuple[str, Optional[int], list[tuple[int, float]]]:
    if not valid_ranges:
        raise ValueError("no valid training joint ranges")
    if mode not in TRAINING_TARGET_MODES:
        mode = TRAINING_DEFAULT_TARGET_MODE

    if mode == "coverage":
        joint, target = build_training_coverage_target(valid_ranges, target_index)
        return "coverage", joint, [(joint, target)]

    if mode == "single":
        joint, target = build_training_coverage_target(valid_ranges, target_index)
        targets = [
            (range_joint, target if range_joint == joint else training_zero_target(low, high))
            for range_joint, low, high in valid_ranges
        ]
        return f"single_j{joint:02d}", joint, targets

    if mode == "combo":
        pattern_index = target_index % len(TRAINING_COMBO_TARGET_PATTERNS)
        pattern = TRAINING_COMBO_TARGET_PATTERNS[pattern_index]
        targets = []
        for joint, low, high in valid_ranges:
            if joint in pattern:
                target = training_fraction_target(low, high, pattern[joint])
            else:
                target = training_zero_target(low, high)
            targets.append((joint, target))
        return f"combo_{pattern_index + 1:02d}", None, targets

    if mode == "step":
        targets = []
        for joint, low, high in valid_ranges:
            if joint in TRAINING_STEP_TARGETS:
                target = min(high, max(low, TRAINING_STEP_TARGETS[joint]))
            else:
                target = training_zero_target(low, high)
            targets.append((joint, target))
        return "step_10_20_20_35", None, targets

    fraction_index = target_index % len(TRAINING_ROUNDTRIP_FRACTIONS)
    joint_index = (target_index // len(TRAINING_ROUNDTRIP_FRACTIONS)) % len(valid_ranges)
    active_joint, active_low, active_high = valid_ranges[joint_index]
    active_target = training_fraction_target(
        active_low,
        active_high,
        TRAINING_ROUNDTRIP_FRACTIONS[fraction_index],
    )
    targets = [
        (
            joint,
            active_target if joint == active_joint else training_zero_target(low, high),
        )
        for joint, low, high in valid_ranges
    ]
    return f"roundtrip_j{active_joint:02d}_{fraction_index + 1}", active_joint, targets


def parse_pretension_steps(text: str) -> dict[int, int]:
    steps: dict[int, int] = {}
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    for token in tokens:
        parts = token.replace("=", ":").split(":")
        if len(parts) != 2:
            raise ValueError(f"预紧步进格式错误: {token}")
        motor_text = parts[0].strip().upper()
        if motor_text.startswith("M"):
            motor_text = motor_text[1:]
        try:
            motor = int(motor_text)
            step = int(round(float(parts[1])))
        except ValueError as exc:
            raise ValueError(f"预紧步进不是数字: {token}") from exc
        if not 0 <= motor < SERVO_COUNT:
            raise ValueError(f"电机索引越界: M{motor:02d}")
        if step == 0:
            raise ValueError(f"预紧步进不能为 0: M{motor:02d}")
        steps[motor] = step

    required = training_target_motor_indices()
    missing = [motor for motor in required if motor not in steps]
    if missing:
        raise ValueError("缺少预紧步进: " + ", ".join(f"M{motor:02d}" for motor in missing))
    return steps


def parse_j00_j03_target_values(text: str) -> list[float]:
    """Parse one broadcast angle or four J00-J03 absolute target angles."""
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    if len(tokens) not in (1, 4):
        raise ValueError("请输入 1 个角度，或按 J00;J01;J02;J03 输入 4 个角度。")
    try:
        values = [float(token) for token in tokens]
    except ValueError as exc:
        raise ValueError("J00-J03 目标角度必须全部是数字。") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("J00-J03 目标角度必须是有限数字。")
    if len(values) == 1:
        values *= 4
    return values


def parse_step_test_poses(
    text: str,
    *,
    required_count: int = STEP_TEST_REQUIRED_POSE_COUNT,
    experiment_label: str = "自动阶跃",
) -> list[tuple[float, float, float, float]]:
    chunks = [chunk.strip() for chunk in re.split(r"[|\n\r]+", text.strip()) if chunk.strip()]
    if len(chunks) != required_count:
        raise ValueError(
            f"{experiment_label}必须配置{required_count}个姿态，"
            "姿态之间用 | 分隔。"
        )
    poses: list[tuple[float, float, float, float]] = []
    for pose_index, chunk in enumerate(chunks, start=1):
        values = parse_j00_j03_target_values(chunk)
        if len(values) != 4:
            raise ValueError(f"姿态S{pose_index:02d}必须包含J00-J03四个角度。")
        pose = tuple(float(value) for value in values)
        for joint, value in enumerate(pose):
            low, high = STEP_TEST_JOINT_LIMITS[joint]
            if not low <= value <= high:
                raise ValueError(
                    f"姿态S{pose_index:02d} J{joint:02d}={value:g}°超出"
                    f"{low:g}～{high:g}°。"
                )
        if pose[1] <= STEP_TEST_J01_MIN_DEG:
            raise ValueError(
                f"姿态S{pose_index:02d}的J01={pose[1]:g}°，必须严格大于0°。"
            )
        poses.append(pose)
    return poses


def parse_sine_test_poses(text: str) -> list[tuple[float, float, float, float]]:
    return parse_step_test_poses(
        text,
        required_count=SINE_TEST_REQUIRED_POSE_COUNT,
        experiment_label="自动正弦",
    )


def parse_step_test_amplitudes(text: str) -> list[float]:
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    if not tokens:
        raise ValueError("至少需要一个阶跃幅值，例如10。")
    amplitudes: list[float] = []
    for token in tokens:
        try:
            amplitude = abs(float(token))
        except ValueError as exc:
            raise ValueError(f"阶跃幅值不是数字: {token}") from exc
        if not math.isfinite(amplitude) or amplitude <= 0.0:
            raise ValueError("阶跃幅值必须是大于0的有限数字。")
        if amplitude not in amplitudes:
            amplitudes.append(amplitude)
    return amplitudes


def build_step_test_trials(
    poses: list[tuple[float, float, float, float]],
    amplitudes: list[float],
    repeats: int,
) -> list[tuple[int, int, float, int]]:
    if repeats <= 0:
        raise ValueError("自动阶跃重复次数必须大于0。")
    trials: list[tuple[int, int, float, int]] = []
    for pose_index in range(len(poses)):
        for repeat in range(1, repeats + 1):
            for joint in MANUAL_RECORD_JOINTS:
                for amplitude in amplitudes:
                    trials.append((pose_index, joint, float(amplitude), repeat))
                    trials.append((pose_index, joint, -float(amplitude), repeat))
    return trials


def validate_step_test_targets(
    poses: list[tuple[float, float, float, float]],
    amplitudes: list[float],
) -> None:
    for pose_index, pose in enumerate(poses, start=1):
        for joint in MANUAL_RECORD_JOINTS:
            low, high = STEP_TEST_JOINT_LIMITS[joint]
            for signed_amplitude in (*amplitudes, *(-value for value in amplitudes)):
                target = pose[joint] + signed_amplitude
                if not low <= target <= high:
                    raise ValueError(
                        f"S{pose_index:02d} J{joint:02d}的{signed_amplitude:+g}°阶跃"
                        f"得到{target:g}°，超出{low:g}～{high:g}°。"
                    )
                if joint == 1 and target <= STEP_TEST_J01_MIN_DEG:
                    raise ValueError(
                        f"S{pose_index:02d}的J01阶跃目标{target:g}°不满足严格大于0°。"
                    )


def step_test_summary_fieldnames() -> list[str]:
    fields = [
        "experiment_id",
        "raw_csv",
        "pose_index",
        "pose_j00_deg",
        "pose_j01_deg",
        "pose_j02_deg",
        "pose_j03_deg",
        "kp",
        "ki_per_s",
        "settle_band_deg",
        "settle_hold_s",
        "repeat",
        "commanded_joint",
        "step_deg",
        "target_deg",
        "completion",
        "response_duration_s",
        "initial_deg",
        "final_mean_deg",
        "final_std_deg",
        "steady_error_deg",
        "minimum_deg",
        "maximum_deg",
        "overshoot_deg",
        "overshoot_percent",
        "rise_time_10_90_s",
        "settling_time_commanded_s",
        "settling_time_all_joints_s",
        "max_cross_coupling_deg",
        "minimum_j01_deg",
    ]
    for joint in MANUAL_RECORD_JOINTS:
        fields.extend(
            [
                f"initial_j{joint:02d}_deg",
                f"final_mean_j{joint:02d}_deg",
                f"minimum_j{joint:02d}_deg",
                f"maximum_j{joint:02d}_deg",
            ]
        )
    fields.extend(f"minimum_force_m{motor:02d}_n" for motor in training_target_motor_indices())
    return fields


def parse_sine_test_frequencies(text: str) -> list[float]:
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    if not tokens:
        raise ValueError("至少需要一个正弦频率，例如0.05;0.10;0.20 Hz。")
    frequencies: list[float] = []
    for token in tokens:
        try:
            frequency = float(token)
        except ValueError as exc:
            raise ValueError(f"正弦频率不是数字: {token}") from exc
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError("正弦频率必须是大于0的有限数字。")
        if frequency > 1.0:
            raise ValueError("自动正弦频率上限为1 Hz；100 ms采样下更高频率无法可靠辨识。")
        if frequency not in frequencies:
            frequencies.append(frequency)
    return frequencies


def build_sine_test_trials(
    poses: list[tuple[float, float, float, float]],
    amplitudes: list[float],
    frequencies_hz: list[float],
    repeats: int,
) -> list[tuple[int, int, float, float, int]]:
    if repeats <= 0:
        raise ValueError("自动正弦重复次数必须大于0。")
    trials: list[tuple[int, int, float, float, int]] = []
    for pose_index in range(len(poses)):
        for repeat in range(1, repeats + 1):
            for joint in MANUAL_RECORD_JOINTS:
                for amplitude in amplitudes:
                    for frequency_hz in frequencies_hz:
                        trials.append(
                            (pose_index, joint, float(amplitude), float(frequency_hz), repeat)
                        )
    return trials


def validate_sine_test_targets(
    poses: list[tuple[float, float, float, float]],
    amplitudes: list[float],
) -> None:
    for pose_index, pose in enumerate(poses, start=1):
        for joint in MANUAL_RECORD_JOINTS:
            low, high = STEP_TEST_JOINT_LIMITS[joint]
            for target in (pose[joint] - max(amplitudes), pose[joint] + max(amplitudes)):
                if not low <= target <= high:
                    raise ValueError(
                        f"S{pose_index:02d} J{joint:02d}正弦范围达到{target:g}°，"
                        f"超出{low:g}~{high:g}°。"
                    )
                if joint == 1 and target <= STEP_TEST_J01_MIN_DEG:
                    raise ValueError(
                        f"S{pose_index:02d}的J01正弦最低目标{target:g}°不满足严格大于0°。"
                    )


def _solve_three_by_three(matrix: list[list[float]], vector: list[float]) -> Optional[tuple[float, float, float]]:
    augmented = [list(row) + [float(value)] for row, value in zip(matrix, vector)]
    for pivot_col in range(3):
        pivot_row = max(range(pivot_col, 3), key=lambda row: abs(augmented[row][pivot_col]))
        if abs(augmented[pivot_row][pivot_col]) <= 1e-12:
            return None
        augmented[pivot_col], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_col]
        pivot = augmented[pivot_col][pivot_col]
        augmented[pivot_col] = [value / pivot for value in augmented[pivot_col]]
        for row in range(3):
            if row == pivot_col:
                continue
            factor = augmented[row][pivot_col]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[pivot_col])
            ]
    return tuple(augmented[row][3] for row in range(3))  # type: ignore[return-value]


def fit_sine_signal(
    samples: list[tuple[float, float]], frequency_hz: float
) -> Optional[tuple[float, float, float, float]]:
    """Fit y=a*sin(wt)+b*cos(wt)+bias; return amplitude, phase_deg, bias and RMSE."""
    if len(samples) < 6 or frequency_hz <= 0.0:
        return None
    omega = 2.0 * math.pi * frequency_hz
    rows = [(math.sin(omega * t), math.cos(omega * t), 1.0, value) for t, value in samples]
    normal = [[sum(row[i] * row[j] for row in rows) for j in range(3)] for i in range(3)]
    rhs = [sum(row[i] * row[3] for row in rows) for i in range(3)]
    solved = _solve_three_by_three(normal, rhs)
    if solved is None:
        return None
    sine_coefficient, cosine_coefficient, bias = solved
    amplitude = math.hypot(sine_coefficient, cosine_coefficient)
    phase_deg = math.degrees(math.atan2(cosine_coefficient, sine_coefficient))
    residuals = [
        value
        - (
            sine_coefficient * math.sin(omega * elapsed)
            + cosine_coefficient * math.cos(omega * elapsed)
            + bias
        )
        for elapsed, value in samples
    ]
    rmse = math.sqrt(statistics.fmean(residual * residual for residual in residuals))
    return amplitude, phase_deg, bias, rmse


def sine_test_summary_fieldnames() -> list[str]:
    fields = [
        "experiment_id",
        "raw_csv",
        "pose_index",
        "pose_j00_deg",
        "pose_j01_deg",
        "pose_j02_deg",
        "pose_j03_deg",
        "kp",
        "ki_per_s",
        "repeat",
        "commanded_joint",
        "command_amplitude_deg",
        "frequency_hz",
        "cycles",
        "completion",
        "duration_s",
        "sample_count",
        "tracking_rmse_deg",
        "tracking_bias_deg",
        "measured_amplitude_deg",
        "amplitude_gain",
        "phase_lag_deg",
        "sine_fit_rmse_deg",
        "max_cross_coupling_amplitude_deg",
        "minimum_j01_deg",
    ]
    for joint in MANUAL_RECORD_JOINTS:
        fields.extend(
            [
                f"measured_amplitude_j{joint:02d}_deg",
                f"phase_j{joint:02d}_deg",
                f"bias_j{joint:02d}_deg",
                f"fit_rmse_j{joint:02d}_deg",
            ]
        )
    fields.extend(f"minimum_force_m{motor:02d}_n" for motor in training_target_motor_indices())
    return fields


def training_target_motor_indices() -> list[int]:
    return sorted(set(FORCE_CHANNEL_TO_MOTOR.values()))


def manual_record_csv_fieldnames() -> list[str]:
    fields = [
        "timestamp",
        "local_time",
        "sample_index",
        "force_complete",
        "encoder_snapshot_ts",
        "encoder_age_s",
        "control_mode",
        "last_host_command",
        "degree_force_active",
        "degree_force_phase",
        "degree_force_tension_window_ok",
        "tension_window_target_n",
        "tension_window_tight_limit_n",
    ]
    for motor in training_target_motor_indices():
        prefix = f"tension_m{motor:02d}"
        fields.extend(
            [
                f"{prefix}_channel",
                f"{prefix}_raw",
                f"{prefix}_relative",
                f"{prefix}_n",
                f"{prefix}_age_s",
                f"{prefix}_target_n",
                f"{prefix}_tight_limit_n",
                f"{prefix}_error_to_target_n",
                f"{prefix}_window_action",
                f"{prefix}_host_bias_counts",
            ]
        )
    for joint in MANUAL_RECORD_JOINTS:
        fields.append(f"target_j{joint:02d}_deg")
    for joint in MANUAL_RECORD_JOINTS:
        prefix = f"joint_j{joint:02d}"
        fields.extend(
            [
                f"{prefix}_raw",
                f"{prefix}_mapped",
                f"{prefix}_deg",
                f"{prefix}_valid",
            ]
        )
    for motor in training_target_motor_indices():
        prefix = f"servo_m{motor:02d}"
        fields.extend(
            [
                f"{prefix}_abs",
                f"{prefix}_hardware_abs",
                f"{prefix}_zero_offset",
                f"{prefix}_raw",
                f"{prefix}_speed",
                f"{prefix}_load",
                f"{prefix}_current",
                f"{prefix}_voltage",
                f"{prefix}_temperature",
                f"{prefix}_online",
                f"{prefix}_raw_online",
            ]
        )
    for motor in training_target_motor_indices():
        prefix = f"mcp_m{motor:02d}"
        fields.extend(
            [
                f"{prefix}_target_deg",
                f"{prefix}_actual_deg",
                f"{prefix}_mapped_target",
                f"{prefix}_solver_target",
                f"{prefix}_cmd_target",
                f"{prefix}_cmd_valid",
                f"{prefix}_debug_age_s",
                f"{prefix}_firmware_time_ms",
                f"{prefix}_q_ref_deg",
                f"{prefix}_q_fb_filtered_deg",
                f"{prefix}_q_fb_velocity_deg_s",
                f"{prefix}_angle_error_deg",
                f"{prefix}_angle_integral_deg_s",
                f"{prefix}_feedforward_counts",
                f"{prefix}_angle_p_counts",
                f"{prefix}_angle_i_counts",
                f"{prefix}_angle_d_counts",
                f"{prefix}_angle_feedback_counts",
                f"{prefix}_tension_bias_counts",
                f"{prefix}_pre_limit_target",
                f"{prefix}_tension_bias_enabled",
                f"{prefix}_command_limit_flags",
                f"{prefix}_command_rate_limited",
                f"{prefix}_command_abs_limited",
            ]
        )
    return fields


def continuous_record_csv_fieldnames() -> list[str]:
    fields = manual_record_csv_fieldnames()
    return fields[:2] + ["elapsed_s"] + fields[2:]


def parse_linearity_state_motor_positions(text: str) -> list[dict[int, int]]:
    states: list[dict[int, int]] = []
    chunks = [chunk.strip() for chunk in re.split(r"[|\n\r]+", text.strip()) if chunk.strip()]
    expected_motors = set(training_target_motor_indices())
    for chunk in chunks:
        tokens = [token.strip() for token in re.split(r"[;；]+", chunk) if token.strip()]
        if len(tokens) != DIRECT_ABS_UI_MOTOR_COUNT:
            raise ValueError(
                "每组初始位置必须正好包含 M00-M04 的 5 个 ABS 位置，"
                "电机间用分号分隔"
            )

        has_labels = [("=" in token or ":" in token) for token in tokens]
        if any(has_labels) and not all(has_labels):
            raise ValueError("同一组初始位置不能混用“纯数字”和“Mxx=数字”格式")

        targets: dict[int, int] = {}
        for index, token in enumerate(tokens):
            motor = index
            value_text = token
            if all(has_labels):
                parts = token.replace(":", "=", 1).split("=", 1)
                motor_text = parts[0].strip().upper()
                if motor_text.startswith("M"):
                    motor_text = motor_text[1:]
                try:
                    motor = int(motor_text)
                except ValueError as exc:
                    raise ValueError(f"电机索引不是数字: {token}") from exc
                value_text = parts[1].strip()
            try:
                target = int(value_text, 10)
            except ValueError as exc:
                raise ValueError(f"初始 ABS 位置必须是整数: {token}") from exc
            if motor not in expected_motors:
                raise ValueError(f"初始位置只支持 M00-M04: M{motor:02d}")
            if motor in targets:
                raise ValueError(f"初始位置中电机重复: M{motor:02d}")
            if not FORCE_TARGET_MIN_ABS <= target <= FORCE_TARGET_MAX_ABS:
                raise ValueError(
                    f"M{motor:02d} ABS 位置 {target} 超出 "
                    f"{FORCE_TARGET_MIN_ABS}~{FORCE_TARGET_MAX_ABS}"
                )
            targets[motor] = target
        if set(targets) != expected_motors:
            raise ValueError("每组初始位置必须完整指定 M00-M04")
        states.append(targets)
    if not states:
        raise ValueError("至少需要一组初始位置，例如 3500;4500;4000;3000;2000")
    return states


def parse_linearity_motors(text: str) -> list[int]:
    motors: list[int] = []
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    for token in tokens:
        motor_text = token.strip().upper()
        if motor_text.startswith("M"):
            motor_text = motor_text[1:]
        try:
            motor = int(motor_text)
        except ValueError as exc:
            raise ValueError(f"电机索引不是数字: {token}") from exc
        if motor not in training_target_motor_indices():
            raise ValueError(f"线性实验当前只支持 M00-M04: M{motor:02d}")
        if motor not in motors:
            motors.append(motor)
    if not motors:
        raise ValueError("至少需要一个扰动电机，例如 M00,M01")
    return motors


def parse_linearity_deltas(text: str) -> list[int]:
    deltas: list[int] = []
    tokens = [token for token in re.split(r"[,;，；\s]+", text.strip()) if token]
    for token in tokens:
        try:
            delta = int(round(float(token)))
        except ValueError as exc:
            raise ValueError(f"扰动 counts 不是数字: {token}") from exc
        deltas.append(delta)
    if not deltas:
        raise ValueError("至少需要一个扰动 counts，例如 0,50,100,50,0,-50,-100,-50,0")
    return deltas


def format_linearity_state_motor_positions(targets: dict[int, int]) -> str:
    return ";".join(f"M{motor:02d}={targets[motor]}" for motor in sorted(targets))


def linearity_experiment_csv_fieldnames() -> list[str]:
    fields = [
        "experiment_id",
        "experiment_sample_index",
        "phase",
        "state_i",
        "state_label",
        "state_target",
        "motor_j",
        "delta_counts",
        "actual_delta_counts",
        "repeat_id",
        "trial_index",
        "sequence_step",
        "is_baseline",
        "baseline_sample_index",
        "baseline_label",
        "settle_elapsed_s",
        "stable_hold_s",
        "max_joint_delta_deg",
        "max_force_delta_n",
        "max_motor_speed",
        "command",
    ]
    fields.extend(manual_record_csv_fieldnames())
    for joint in MANUAL_RECORD_JOINTS:
        fields.append(f"delta_joint_j{joint:02d}_deg")
    for motor in training_target_motor_indices():
        fields.append(f"delta_tension_m{motor:02d}_n")
    for motor in training_target_motor_indices():
        fields.append(f"delta_servo_m{motor:02d}_abs")
    for motor in training_target_motor_indices():
        fields.append(f"target_servo_m{motor:02d}_abs")
    for motor in training_target_motor_indices():
        fields.append(f"target_error_m{motor:02d}_counts")
    return fields


def multi_input_experiment_csv_fieldnames() -> list[str]:
    fields = [
        "experiment_id",
        "experiment_sample_index",
        "phase",
        "dataset_split",
        "sequence_index",
        "sequence_count",
        "excitation_mode",
        "random_seed",
        "ridge_lambda",
        "split_train",
        "split_validation",
        "split_test",
        "state_target",
        "is_baseline",
        "baseline_sample_index",
        "settle_elapsed_s",
        "stable_hold_s",
        "max_joint_delta_deg",
        "max_force_delta_n",
        "max_motor_speed",
        "command",
    ]
    for motor in training_target_motor_indices():
        fields.append(f"command_delta_m{motor:02d}_counts")
    fields.extend(manual_record_csv_fieldnames())
    for joint in MANUAL_RECORD_JOINTS:
        fields.append(f"delta_joint_j{joint:02d}_deg")
    for motor in training_target_motor_indices():
        fields.append(f"delta_tension_m{motor:02d}_n")
    for motor in training_target_motor_indices():
        fields.append(f"delta_servo_m{motor:02d}_abs")
    for motor in training_target_motor_indices():
        fields.append(f"target_servo_m{motor:02d}_abs")
    for motor in training_target_motor_indices():
        fields.append(f"target_error_m{motor:02d}_counts")
    return fields


def training_csv_fieldnames() -> list[str]:
    fields = [
        "timestamp",
        "local_time",
        "elapsed_s",
        "target_mode",
        "target_phase",
        "target_joint",
        "target_relative_deg",
        "target_device_deg",
        "target_command",
        "tension_window_ok",
        "tension_action",
        "tension_action_count",
        "tension_bias_m00_counts",
        "tension_bias_m01_counts",
        "tension_bias_m02_counts",
        "tension_bias_m03_counts",
        "tension_bias_m04_counts",
        "target_vector_complete",
        "force_complete",
        "encoder_snapshot_ts",
        "encoder_age_s",
    ]
    for joint in range(ENCODER_COUNT):
        prefix = f"target_j{joint:02d}"
        fields.extend(
            [
                f"{prefix}_relative_deg",
                f"{prefix}_device_deg",
                f"{prefix}_known",
            ]
        )
    for channel in DISPLAY_CHANNELS:
        prefix = f"force_ch{channel}"
        fields.extend(
            [
                f"{prefix}_motor",
                f"{prefix}_raw",
                f"{prefix}_relative",
                f"{prefix}_n",
                f"{prefix}_age_s",
            ]
        )
    for motor in training_target_motor_indices():
        prefix = f"tension_m{motor:02d}"
        fields.extend(
            [
                f"{prefix}_channel",
                f"{prefix}_relative",
                f"{prefix}_n",
            ]
        )
    for joint in range(ENCODER_COUNT):
        prefix = f"joint_j{joint:02d}"
        fields.extend(
            [
                f"{prefix}_raw",
                f"{prefix}_mapped",
                f"{prefix}_deg_abs",
                f"{prefix}_deg_rel",
                f"{prefix}_valid",
            ]
        )
    for motor in range(SERVO_COUNT):
        prefix = f"servo_m{motor:02d}"
        fields.extend(
            [
                f"{prefix}_abs",
                f"{prefix}_hardware_abs",
                f"{prefix}_zero_offset",
                f"{prefix}_raw",
                f"{prefix}_speed",
                f"{prefix}_load",
                f"{prefix}_current",
                f"{prefix}_voltage",
                f"{prefix}_temperature",
                f"{prefix}_online",
                f"{prefix}_raw_online",
            ]
        )
    return fields


def load_force_zero_baselines() -> dict[int, int]:
    try:
        with open(FORCE_ZERO_CONFIG_PATH, "r", encoding="utf-8") as fp:
            payload = json.load(fp)
    except FileNotFoundError:
        return {}
    except Exception:
        return {}

    baselines: dict[int, int] = {}
    raw = payload.get("baselines", payload) if isinstance(payload, dict) else {}
    if not isinstance(raw, dict):
        return {}
    for key, value in raw.items():
        try:
            channel = int(key)
            if channel in DISPLAY_CHANNELS:
                baselines[channel] = int(value)
        except (TypeError, ValueError):
            continue
    return baselines


def save_force_zero_baselines(baselines: dict[int, int]) -> None:
    FORCE_ZERO_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "Force sensor no-tension zero baseline. Relative force N = (raw - baseline) / 10.",
        "channels": DISPLAY_CHANNELS,
        "baselines": {str(channel): int(baselines[channel]) for channel in DISPLAY_CHANNELS},
    }
    with open(FORCE_ZERO_CONFIG_PATH, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
        fp.write("\n")


def delete_force_zero_baselines() -> None:
    try:
        FORCE_ZERO_CONFIG_PATH.unlink()
    except FileNotFoundError:
        pass


def delete_encoder_zero_offsets() -> None:
    try:
        ENCODER_ZERO_CONFIG_PATH.unlink()
    except FileNotFoundError:
        pass


def build_upper_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    frame_len = 1 + len(payload) + 1
    return (
        bytes((UPPER_PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF))
        + payload
        + bytes((UPPER_PROTOCOL_TAIL,))
    )


def build_upper_stream_mode_cmd(mode: int = SENSOR_STREAM_MODE_SIGNED_I16) -> bytes:
    return build_upper_command_frame(CMD_SENSOR_STREAM_MODE, bytes((mode & 0xFF,)))


def build_upper_angle_cmd(angles: list[float]) -> bytes:
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    payload = struct.pack(f"<{ENCODER_COUNT}f", *(float(value) for value in angles))
    return build_upper_command_frame(CMD_ANGLE_CTRL, payload)


def build_upper_motor_pos_abs_frame(motor_abs: list[int]) -> bytes:
    if not motor_abs:
        raise ValueError("need at least one motor absolute value")
    payload = bytearray()
    for value in motor_abs:
        target = max(FORCE_TARGET_MIN_ABS, min(FORCE_TARGET_MAX_ABS, int(value)))
        payload.extend(struct.pack(">h", target))
    return build_upper_command_frame(CMD_MOTOR_POS_ABS, bytes(payload))


def build_upper_motor_pos_abs_cmd(motor_abs: list[int]) -> bytes:
    if len(motor_abs) != SERVO_COUNT:
        raise ValueError(f"need exactly {SERVO_COUNT} motor absolute values")
    return build_upper_motor_pos_abs_frame(motor_abs)


def parse_proto_ack_payload(payload: bytes) -> Optional[tuple[int, int, int]]:
    if len(payload) < 3:
        return None
    return payload[0], payload[1], payload[2]


def build_host_degree_target_vector(
    sent_targets: dict[int, float],
    feedback_deg: list[float],
    feedback_valid: list[bool],
) -> list[float]:
    targets = [0.0] * ENCODER_COUNT
    for joint in range(ENCODER_COUNT):
        if joint in sent_targets:
            targets[joint] = float(sent_targets[joint])
        elif joint < len(feedback_deg) and joint < len(feedback_valid) and feedback_valid[joint]:
            targets[joint] = float(feedback_deg[joint])
    return targets


def build_host_direct_target_vector(
    sent_targets: dict[int, int],
    feedback_abs: list[int],
) -> list[int]:
    targets = [0] * SERVO_COUNT
    for motor in range(SERVO_COUNT):
        if motor in sent_targets:
            targets[motor] = int(sent_targets[motor])
        elif motor < len(feedback_abs):
            targets[motor] = int(feedback_abs[motor])
    return targets


def parse_upper_frame_bytes(buffer: bytearray) -> tuple[list[tuple[int, bytes]], list[str]]:
    frames: list[tuple[int, bytes]] = []
    lines: list[str] = []

    while buffer:
        if buffer[0] == UPPER_PROTOCOL_HEADER:
            if len(buffer) < 4:
                break
            wire_len = buffer[1]
            frame_len = wire_len + 2
            if wire_len < 2:
                del buffer[0]
                continue
            if len(buffer) < frame_len:
                break
            frame = bytes(buffer[:frame_len])
            del buffer[:frame_len]
            if frame[-1] != UPPER_PROTOCOL_TAIL:
                continue
            frames.append((frame[2], frame[3:-1]))
            continue

        next_header = buffer.find(bytes((UPPER_PROTOCOL_HEADER,)))
        next_newline_candidates = [idx for idx in (buffer.find(b"\n"), buffer.find(b"\r")) if idx >= 0]
        next_newline = min(next_newline_candidates) if next_newline_candidates else -1

        if next_newline >= 0 and (next_header < 0 or next_newline < next_header):
            raw_line = bytes(buffer[:next_newline])
            del buffer[: next_newline + 1]
            while buffer[:1] in (b"\n", b"\r"):
                del buffer[0]
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
            continue

        if next_header > 0:
            raw_line = bytes(buffer[:next_header]).strip()
            del buffer[:next_header]
            if raw_line:
                lines.append(raw_line.decode("utf-8", errors="replace"))
            continue

        if len(buffer) > 512:
            raw_line = bytes(buffer).strip()
            buffer.clear()
            if raw_line:
                lines.append(raw_line.decode("utf-8", errors="replace"))
        break

    return frames, lines


def parse_encoder_sensor_payload(payload: bytes) -> tuple[list[int], list[int], list[bool]]:
    raw_counts = [0] * ENCODER_COUNT
    mapped_counts = [0] * ENCODER_COUNT
    valid = [False] * ENCODER_COUNT
    legacy_len = ENCODER_COUNT * 2
    dual_len = ENCODER_COUNT * 4
    if len(payload) >= dual_len:
        for index in range(ENCODER_COUNT):
            raw_offset = index * 2
            mapped_offset = legacy_len + index * 2
            raw_u16 = (payload[raw_offset] << 8) | payload[raw_offset + 1]
            mapped_u16 = (payload[mapped_offset] << 8) | payload[mapped_offset + 1]
            is_valid = raw_u16 != DISCONNECT_SENTINEL and mapped_u16 != DISCONNECT_SENTINEL
            raw_counts[index] = int(raw_u16 if is_valid else 0)
            mapped_counts[index] = struct.unpack(">h", payload[mapped_offset : mapped_offset + 2])[0] if is_valid else 0
            valid[index] = is_valid
    elif len(payload) >= legacy_len:
        for index in range(ENCODER_COUNT):
            offset = index * 2
            mapped_u16 = (payload[offset] << 8) | payload[offset + 1]
            is_valid = mapped_u16 != DISCONNECT_SENTINEL
            mapped = struct.unpack(">h", payload[offset : offset + 2])[0] if is_valid else 0
            raw_counts[index] = mapped
            mapped_counts[index] = mapped
            valid[index] = is_valid
    return raw_counts, mapped_counts, valid


def parse_servo_angle_payload(payload: bytes) -> Optional[tuple[list[int], list[int], list[bool]]]:
    if not payload:
        return None
    has_offsets = (len(payload) % 9) == 0
    if has_offsets:
        count = len(payload) // 9
    elif (len(payload) % 5) == 0:
        count = len(payload) // 5
    else:
        return None
    if count <= 0:
        return None

    count = min(count, SERVO_COUNT)
    angles = [0] * SERVO_COUNT
    offsets = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        start = index * 4
        angles[index] = struct.unpack(">i", payload[start : start + 4])[0]
    base = count * 4
    if has_offsets:
        for index in range(count):
            start = base + index * 4
            offsets[index] = struct.unpack(">i", payload[start : start + 4])[0]
        base += count * 4
    for index in range(count):
        online[index] = payload[base + index] == 1
    return angles, offsets, online


def parse_servo_raw_payload(payload: bytes) -> Optional[tuple[list[int], list[bool]]]:
    if not payload or (len(payload) % 3) != 0:
        return None
    count = min(len(payload) // 3, SERVO_COUNT)
    raw_positions = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        start = index * 2
        raw_positions[index] = struct.unpack(">h", payload[start : start + 2])[0]
    base = count * 2
    for index in range(count):
        online[index] = payload[base + index] == 1
    return raw_positions, online


def parse_servo_telem_payload(payload: bytes) -> Optional[tuple[list[int], list[int], list[int], list[int], list[int], list[bool]]]:
    if not payload:
        return None
    if (len(payload) % 9) == 0:
        stride = 9
        has_current = True
    elif (len(payload) % 7) == 0:
        stride = 7
        has_current = False
    else:
        return None
    count = min(len(payload) // stride, SERVO_COUNT)
    speeds = [0] * SERVO_COUNT
    loads = [0] * SERVO_COUNT
    currents = [0] * SERVO_COUNT
    volts = [0] * SERVO_COUNT
    temps = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        base = index * stride
        speeds[index] = struct.unpack(">h", payload[base : base + 2])[0]
        loads[index] = struct.unpack(">h", payload[base + 2 : base + 4])[0]
        if has_current:
            currents[index] = struct.unpack(">h", payload[base + 4 : base + 6])[0]
            volts[index] = payload[base + 6]
            temps[index] = payload[base + 7]
            online[index] = payload[base + 8] == 1
        else:
            volts[index] = payload[base + 4]
            temps[index] = payload[base + 5]
            online[index] = payload[base + 6] == 1
    return speeds, loads, currents, volts, temps, online


def parse_joint_debug_payload(payload: bytes) -> Optional[JointDebugSnapshot]:
    legacy_len = 2 + 5 * 4
    cmd_len = legacy_len + 1 + 2
    cmd_ts_len = cmd_len + 4
    extended_len = 2 + 8 * 4 + 4 + 2 + 1 + 1 + 2
    extended_v2_len = extended_len + 4 + 10 * 4 + 2 + 2 + 1 + 1
    if len(payload) not in (legacy_len, cmd_len, cmd_ts_len, extended_len, extended_v2_len):
        return None
    timestamp = time.time()
    joint_index = payload[0]
    valid = payload[1] == 1
    floats = struct.unpack(">5f", payload[2:legacy_len])
    target_deg = float(floats[0])
    actual_deg = float(floats[1])
    loop1_output = float(floats[2])
    loop2_actual = float(floats[3])
    loop2_output = float(floats[4])
    target_length = 0.0
    actual_length = 0.0
    mapped_motor_target = 0.0
    motor_zero_abs = 0
    solver_output_pos = 0
    motor_targets_valid = False
    zero_homing = False
    cmd_valid = False
    cmd_target_pos = 0
    firmware_time_ms = 0
    q_ref_deg = 0.0
    q_fb_filtered_deg = 0.0
    q_fb_velocity_deg_s = 0.0
    angle_error_deg = 0.0
    angle_integral_deg_s = 0.0
    feedforward_counts = 0.0
    angle_p_counts = 0.0
    angle_i_counts = 0.0
    angle_d_counts = 0.0
    angle_feedback_counts = 0.0
    tension_bias_counts = 0
    pre_limit_target_pos = 0
    tension_bias_enabled = False
    command_limit_flags = 0
    if len(payload) in (extended_len, extended_v2_len):
        extra = struct.unpack(">3fihBBh", payload[legacy_len:extended_len])
        target_length = float(extra[0])
        actual_length = float(extra[1])
        mapped_motor_target = float(extra[2])
        motor_zero_abs = int(extra[3])
        solver_output_pos = int(extra[4])
        motor_targets_valid = True
        zero_homing = bool(extra[5])
        cmd_valid = bool(extra[6])
        cmd_target_pos = int(extra[7])
        if len(payload) == extended_v2_len:
            diagnostic = struct.unpack(">I10fhhBB", payload[extended_len:extended_v2_len])
            firmware_time_ms = int(diagnostic[0])
            q_ref_deg = float(diagnostic[1])
            q_fb_filtered_deg = float(diagnostic[2])
            q_fb_velocity_deg_s = float(diagnostic[3])
            angle_error_deg = float(diagnostic[4])
            angle_integral_deg_s = float(diagnostic[5])
            feedforward_counts = float(diagnostic[6])
            angle_p_counts = float(diagnostic[7])
            angle_i_counts = float(diagnostic[8])
            angle_d_counts = float(diagnostic[9])
            angle_feedback_counts = float(diagnostic[10])
            tension_bias_counts = int(diagnostic[11])
            pre_limit_target_pos = int(diagnostic[12])
            tension_bias_enabled = bool(diagnostic[13])
            command_limit_flags = int(diagnostic[14])
    elif len(payload) >= cmd_len:
        zero_homing = bool(payload[legacy_len])
        cmd_valid = True
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1 : legacy_len + 3])[0]
    return JointDebugSnapshot(
        timestamp=timestamp,
        joint_index=joint_index,
        valid=valid,
        target_deg=target_deg,
        actual_deg=actual_deg,
        loop1_output=loop1_output,
        loop2_actual=loop2_actual,
        loop2_output=loop2_output,
        target_length=target_length,
        actual_length=actual_length,
        mapped_motor_target=mapped_motor_target,
        motor_zero_abs=motor_zero_abs,
        solver_output_pos=solver_output_pos,
        motor_targets_valid=motor_targets_valid,
        zero_homing=zero_homing,
        cmd_valid=cmd_valid,
        cmd_target_pos=int(cmd_target_pos),
        firmware_time_ms=firmware_time_ms,
        q_ref_deg=q_ref_deg,
        q_fb_filtered_deg=q_fb_filtered_deg,
        q_fb_velocity_deg_s=q_fb_velocity_deg_s,
        angle_error_deg=angle_error_deg,
        angle_integral_deg_s=angle_integral_deg_s,
        feedforward_counts=feedforward_counts,
        angle_p_counts=angle_p_counts,
        angle_i_counts=angle_i_counts,
        angle_d_counts=angle_d_counts,
        angle_feedback_counts=angle_feedback_counts,
        tension_bias_counts=tension_bias_counts,
        pre_limit_target_pos=pre_limit_target_pos,
        tension_bias_enabled=tension_bias_enabled,
        command_limit_flags=command_limit_flags,
    )


def text_key_values(line: str) -> dict[str, str]:
    return dict(re.findall(r"([A-Za-z_]+)=([^\s>]+)", line))


def build_read_holding_registers(slave_id: int, start_register: int, count: int) -> bytes:
    payload = bytes(
        (
            slave_id & 0xFF,
            0x03,
            (start_register >> 8) & 0xFF,
            start_register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        )
    )
    return append_crc(payload)


def weight_register_for_channel(channel: int) -> int:
    if channel < 1:
        raise ValueError("channel must be >= 1")
    return (channel - 1) * 2


def build_write_single_register(slave_id: int, register: int, value: int) -> bytes:
    payload = bytes(
        (
            slave_id & 0xFF,
            0x06,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        )
    )
    return append_crc(payload)


def validate_crc(frame: bytes) -> None:
    if len(frame) < 4:
        raise ValueError("response too short")
    expected = frame[-2] | (frame[-1] << 8)
    actual = crc16_modbus(frame[:-2])
    if expected != actual:
        raise ValueError(f"CRC mismatch: got 0x{expected:04X}, expected 0x{actual:04X}")


def decode_live_weight_response(frame: bytes, slave_id: int) -> tuple[int, int, int]:
    validate_crc(frame)
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x03:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    if frame[2] < 4 or len(frame) < 9:
        raise ValueError("response does not contain two registers")

    low_word = (frame[3] << 8) | frame[4]
    high_word = (frame[5] << 8) | frame[6]
    raw = (high_word << 16) | low_word
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw, low_word, high_word


def decode_holding_registers_response(frame: bytes, slave_id: int) -> list[int]:
    validate_crc(frame)
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x03:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    byte_count = frame[2]
    if byte_count % 2:
        raise ValueError("register byte count should be even")
    if len(frame) != byte_count + 5:
        raise ValueError(f"response length mismatch: got {len(frame)}, expected {byte_count + 5}")
    registers = []
    for index in range(byte_count // 2):
        offset = 3 + index * 2
        registers.append((frame[offset] << 8) | frame[offset + 1])
    return registers


def words_to_u32_low_first(low_word: int, high_word: int) -> int:
    return ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)


def words_to_i32_low_first(low_word: int, high_word: int) -> int:
    raw = words_to_u32_low_first(low_word, high_word)
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw


def decode_write_single_register_response(frame: bytes, request: bytes, slave_id: int) -> None:
    validate_crc(frame)
    if len(frame) != 8:
        raise ValueError("write response should be 8 bytes")
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x06:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    if frame[:6] != request[:6]:
        raise ValueError("write response does not echo the request")


class Win32Serial:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    FILE_ATTRIBUTE_NORMAL = 0x80
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
    PURGE_RXCLEAR = 0x0008
    PURGE_TXCLEAR = 0x0004

    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("fFlags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    def __init__(self, port: str, baudrate: int, timeout_s: float = 0.25) -> None:
        if sys.platform != "win32":
            raise RuntimeError("native serial backend is only available on Windows")
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._bind_kernel32_functions()
        name = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        self.handle = self.kernel32.CreateFileW(
            name,
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            self.FILE_ATTRIBUTE_NORMAL,
            None,
        )
        if self.handle == self.INVALID_HANDLE_VALUE:
            self._raise_last_error(f"open {port}")

        try:
            self._configure(baudrate, timeout_s)
        except Exception:
            self.close()
            raise

    def _bind_kernel32_functions(self) -> None:
        self.kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.LPVOID,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.HANDLE,
        ]
        self.kernel32.CreateFileW.restype = wintypes.HANDLE
        self.kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        self.kernel32.CloseHandle.restype = wintypes.BOOL
        self.kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self.DCB)]
        self.kernel32.GetCommState.restype = wintypes.BOOL
        self.kernel32.BuildCommDCBW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(self.DCB)]
        self.kernel32.BuildCommDCBW.restype = wintypes.BOOL
        self.kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self.DCB)]
        self.kernel32.SetCommState.restype = wintypes.BOOL
        self.kernel32.SetCommTimeouts.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(self.COMMTIMEOUTS),
        ]
        self.kernel32.SetCommTimeouts.restype = wintypes.BOOL
        self.kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self.kernel32.PurgeComm.restype = wintypes.BOOL
        self.kernel32.WriteFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPCVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.WriteFile.restype = wintypes.BOOL
        self.kernel32.ReadFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.ReadFile.restype = wintypes.BOOL
        self.kernel32.FlushFileBuffers.argtypes = [wintypes.HANDLE]
        self.kernel32.FlushFileBuffers.restype = wintypes.BOOL

    def _raise_last_error(self, action: str) -> None:
        code = ctypes.get_last_error()
        raise OSError(code, f"Win32 serial {action} failed: {ctypes.FormatError(code)}")

    def _configure(self, baudrate: int, timeout_s: float) -> None:
        dcb = self.DCB()
        dcb.DCBlength = ctypes.sizeof(self.DCB)
        if not self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)):
            self._raise_last_error("GetCommState")

        config = f"baud={baudrate} parity=N data=8 stop=1"
        if not self.kernel32.BuildCommDCBW(config, ctypes.byref(dcb)):
            self._raise_last_error("BuildCommDCB")
        dcb.BaudRate = baudrate
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.fFlags |= 0x00000001  # fBinary
        dcb.fFlags &= ~0x00000002  # fParity
        if not self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
            self._raise_last_error("SetCommState")

        timeout_ms = max(1, int(timeout_s * 1000))
        timeouts = self.COMMTIMEOUTS()
        timeouts.ReadIntervalTimeout = 20
        timeouts.ReadTotalTimeoutMultiplier = 0
        timeouts.ReadTotalTimeoutConstant = timeout_ms
        timeouts.WriteTotalTimeoutMultiplier = 0
        timeouts.WriteTotalTimeoutConstant = timeout_ms
        if not self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)):
            self._raise_last_error("SetCommTimeouts")

    def reset_input_buffer(self) -> None:
        self.kernel32.PurgeComm(self.handle, self.PURGE_RXCLEAR | self.PURGE_TXCLEAR)

    def write(self, data: bytes) -> int:
        written = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(data)
        if not self.kernel32.WriteFile(
            self.handle, buffer, len(data), ctypes.byref(written), None
        ):
            self._raise_last_error("WriteFile")
        return int(written.value)

    def read(self, size: int) -> bytes:
        read_count = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(size)
        if not self.kernel32.ReadFile(
            self.handle, buffer, size, ctypes.byref(read_count), None
        ):
            self._raise_last_error("ReadFile")
        return buffer.raw[: read_count.value]

    def flush(self) -> None:
        self.kernel32.FlushFileBuffers(self.handle)

    def close(self) -> None:
        if getattr(self, "handle", None):
            self.kernel32.CloseHandle(self.handle)
            self.handle = None


def list_serial_ports() -> list[str]:
    ports: list[str] = []
    if list_ports is not None:
        ports.extend(port.device for port in list_ports.comports())
    elif sys.platform == "win32":
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            index = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, index)
                    ports.append(str(value))
                    index += 1
                except OSError:
                    break
        except OSError:
            pass
    if "COM7" not in ports:
        ports.insert(0, "COM7")
    return sorted(dict.fromkeys(ports))


def open_serial_port(port: str, baudrate: int, timeout_s: float = 0.25):
    if serial is not None:
        return serial.Serial(
            port,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )
    return Win32Serial(port, baudrate, timeout_s)


def read_exact(port, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < size and time.monotonic() < deadline:
        chunk = port.read(size - len(data))
        if chunk:
            data.extend(chunk)
        else:
            time.sleep(0.005)
    return bytes(data)


class SensorPoller(threading.Thread):
    def __init__(
        self,
        outbox: "queue.Queue[tuple[str, object]]",
        command_queue: "queue.Queue[tuple[str, bytes, int]]",
        stop_event: threading.Event,
        port_name: str,
        baudrate: int,
        slave_id: int,
        channel: int,
        interval_ms: int,
    ) -> None:
        super().__init__(daemon=True)
        self.outbox = outbox
        self.command_queue = command_queue
        self.stop_event = stop_event
        self.port_name = port_name
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.channel = channel
        self.channels = list(DISPLAY_CHANNELS)
        self.interval_ms = interval_ms

    def run(self) -> None:
        requests = {
            channel: build_read_holding_registers(
                self.slave_id, weight_register_for_channel(channel), 2
            )
            for channel in self.channels
        }
        port = None
        last_error: Optional[str] = None
        try:
            self.outbox.put(("status", f"opening {self.port_name} @ {self.baudrate} 8N1"))
            port = open_serial_port(self.port_name, self.baudrate, timeout_s=0.25)
            self.outbox.put(("connected", True))
            self.outbox.put(("status", "connected"))
            while not self.stop_event.is_set():
                started = time.monotonic()
                try:
                    self._process_commands(port)
                    for channel, request in requests.items():
                        if hasattr(port, "reset_input_buffer"):
                            port.reset_input_buffer()
                        port.write(request)
                        if hasattr(port, "flush"):
                            port.flush()
                        response = read_exact(port, 9, timeout_s=0.3)
                        value, low_word, high_word = decode_live_weight_response(
                            response, self.slave_id
                        )
                        self.outbox.put(
                            (
                                "sample",
                                ForceSample(
                                    timestamp=time.time(),
                                    channel=channel,
                                    value=value,
                                    low_word=low_word,
                                    high_word=high_word,
                                    request_hex=format_hex(request),
                                    response_hex=format_hex(response),
                                ),
                            )
                        )
                except Exception as exc:
                    last_error = str(exc)
                    self.outbox.put(("error", last_error))

                elapsed = time.monotonic() - started
                wait_s = max(0.01, self.interval_ms / 1000.0 - elapsed)
                self.stop_event.wait(wait_s)
        except Exception as exc:
            last_error = str(exc)
            self.outbox.put(("error", last_error))
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
            self.outbox.put(("connected", False))
            if self.stop_event.is_set():
                self.outbox.put(("status", "disconnected"))
            elif last_error:
                self.outbox.put(("status", f"disconnected: {last_error}"))
            else:
                self.outbox.put(("status", "disconnected"))

    def _process_commands(self, port) -> None:
        while not self.stop_event.is_set():
            try:
                label, request, response_len = self.command_queue.get_nowait()
            except queue.Empty:
                return
            if hasattr(port, "reset_input_buffer"):
                port.reset_input_buffer()
            port.write(request)
            if hasattr(port, "flush"):
                port.flush()
            response = read_exact(port, response_len, timeout_s=0.3)
            if response_len == 8:
                decode_write_single_register_response(response, request, self.slave_id)
                if (
                    len(request) >= 6
                    and request[1] == 0x06
                    and int.from_bytes(request[2:4], "big") == 0x0015
                ):
                    self.outbox.put(
                        ("device_tare", int.from_bytes(request[4:6], "big"))
                    )
            else:
                registers = decode_holding_registers_response(response, self.slave_id)
                if label.startswith("诊断读取") and len(registers) >= 2:
                    weight = words_to_i32_low_first(registers[0], registers[1])
                    self.outbox.put(
                        (
                            "diagnostic",
                            {
                                "label": label,
                                "weight": weight,
                                "low": registers[0],
                                "high": registers[1],
                            },
                        )
                    )
            self.outbox.put(("frame", (label, format_hex(request), format_hex(response))))
            self.outbox.put(("status", f"{label} ok"))


class EncoderServoPoller(threading.Thread):
    def __init__(
        self,
        outbox: "queue.Queue[tuple[str, object]]",
        command_queue: "queue.Queue[tuple[str, object]]",
        stop_event: threading.Event,
        port_name: str,
        baudrate: int,
    ) -> None:
        super().__init__(daemon=True)
        self.outbox = outbox
        self.command_queue = command_queue
        self.stop_event = stop_event
        self.port_name = port_name
        self.baudrate = baudrate
        self.buffer = bytearray()
        self.encoder_raw = [0] * ENCODER_COUNT
        self.encoder_mapped = [0] * ENCODER_COUNT
        self.encoder_deg = [0.0] * ENCODER_COUNT
        self.encoder_valid = [False] * ENCODER_COUNT
        self.servo_abs = [0] * SERVO_COUNT
        self.servo_hardware_abs = [0] * SERVO_COUNT
        self.servo_zero_offset = [0] * SERVO_COUNT
        self.servo_online = [False] * SERVO_COUNT
        self.servo_raw = [0] * SERVO_COUNT
        self.servo_raw_online = [False] * SERVO_COUNT
        self.servo_speed = [0] * SERVO_COUNT
        self.servo_load = [0] * SERVO_COUNT
        self.servo_current = [0] * SERVO_COUNT
        self.servo_voltage = [0] * SERVO_COUNT
        self.servo_temperature = [0] * SERVO_COUNT
        self.joint_debug: dict[int, JointDebugSnapshot] = {}
        self.last_line = "--"
        self.last_emit = 0.0

    def run(self) -> None:
        port = None
        last_error: Optional[str] = None
        try:
            self.outbox.put(("status", f"opening {self.port_name} @ {self.baudrate} 8N1"))
            port = open_serial_port(self.port_name, self.baudrate, timeout_s=0.05)
            self.outbox.put(("connected", True))
            self.outbox.put(("status", "connected"))
            self._write_bytes(port, build_upper_stream_mode_cmd())
            self._write_text(port, "binary")
            self._write_text(port, "encoder")
            self._write_text(port, "servo")
            self._write_text(port, "load")
            next_snapshot = time.monotonic() + ENCODER_SERVO_TEXT_POLL_INTERVAL_S
            while not self.stop_event.is_set():
                self._process_commands(port)
                now = time.monotonic()
                if now >= next_snapshot:
                    self._write_text(port, "encoder")
                    self._write_text(port, "servo")
                    self._write_text(port, "load")
                    next_snapshot = now + ENCODER_SERVO_TEXT_POLL_INTERVAL_S
                chunk = port.read(512)
                if chunk:
                    self.buffer.extend(chunk)
                    frames, lines = parse_upper_frame_bytes(self.buffer)
                    updated = False
                    for pkt_type, payload in frames:
                        updated = self._handle_frame(pkt_type, payload) or updated
                    for line in lines:
                        updated = self._handle_text_line(line) or updated
                    if updated:
                        self._emit_snapshot()
                else:
                    self.stop_event.wait(0.01)
        except Exception as exc:
            last_error = str(exc)
            self.outbox.put(("error", last_error))
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
            self.outbox.put(("connected", False))
            if self.stop_event.is_set():
                self.outbox.put(("status", "disconnected"))
            elif last_error:
                self.outbox.put(("status", f"disconnected: {last_error}"))
            else:
                self.outbox.put(("status", "disconnected"))

    def _write_bytes(self, port, data: bytes) -> None:
        port.write(data)
        if hasattr(port, "flush"):
            port.flush()

    def _write_text(self, port, text: str) -> None:
        payload = text.strip().encode("ascii", errors="ignore") + b"\n"
        self._write_bytes(port, payload)

    def _process_commands(self, port) -> None:
        while not self.stop_event.is_set():
            try:
                kind, payload = self.command_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "text":
                text = str(payload).strip()
                if text:
                    self._write_text(port, text)
                    self.outbox.put(("tx", text))
            elif kind == "bytes":
                data = bytes(payload)
                if data:
                    self._write_bytes(port, data)
                    self.outbox.put(("tx", format_hex(data)))

    def _handle_frame(self, pkt_type: int, payload: bytes) -> bool:
        if pkt_type == PACKET_TYPE_SENSOR:
            raw, mapped, valid = parse_encoder_sensor_payload(payload)
            self.encoder_raw = raw
            self.encoder_mapped = mapped
            self.encoder_valid = valid
            self.encoder_deg = [mapped[index] * ENCODER_RAW_TO_DEG for index in range(ENCODER_COUNT)]
            return True
        if pkt_type == PACKET_TYPE_SERVO_ANGLE:
            parsed = parse_servo_angle_payload(payload)
            if parsed is None:
                return False
            angles, offsets, online = parsed
            self.servo_abs = angles
            self.servo_zero_offset = offsets
            self.servo_online = online
            self.servo_hardware_abs = [
                self.servo_abs[index] + self.servo_zero_offset[index] for index in range(SERVO_COUNT)
            ]
            return True
        if pkt_type == PACKET_TYPE_SERVO_RAW:
            parsed = parse_servo_raw_payload(payload)
            if parsed is None:
                return False
            self.servo_raw, self.servo_raw_online = parsed
            return True
        if pkt_type == PACKET_TYPE_SERVO_TELEM:
            parsed = parse_servo_telem_payload(payload)
            if parsed is None:
                return False
            speed, load, current, voltage, temperature, online = parsed
            self.servo_speed = speed
            self.servo_load = load
            self.servo_current = current
            self.servo_voltage = voltage
            self.servo_temperature = temperature
            self.servo_online = [
                self.servo_online[index] or online[index] for index in range(SERVO_COUNT)
            ]
            return True
        if pkt_type == PACKET_TYPE_JOINT_DEBUG:
            parsed = parse_joint_debug_payload(payload)
            if parsed is None:
                return False
            joint_index = parsed.joint_index
            if joint_index in ENCODER_PLOT_JOINTS:
                self.encoder_deg[joint_index] = parsed.actual_deg
                self.encoder_valid[joint_index] = parsed.valid
                self.joint_debug[joint_index] = parsed
            elif joint_index in training_target_motor_indices():
                self.joint_debug[joint_index] = parsed
            return True
        if pkt_type == PACKET_TYPE_PROTO_ACK:
            ack = parse_proto_ack_payload(payload)
            if ack is not None:
                self.outbox.put(("proto_ack", ack))
            self.last_line = f"RX packet 0x{pkt_type:02X}: {format_hex(payload)}"
            self.outbox.put(("line", self.last_line))
            return False
        if pkt_type in (PACKET_TYPE_CALIB_ACK, PACKET_TYPE_FAULT_STATUS, PACKET_TYPE_RELEASE_FAULT, PACKET_TYPE_CONTROL_STATUS):
            self.last_line = f"RX packet 0x{pkt_type:02X}: {format_hex(payload)}"
            self.outbox.put(("line", self.last_line))
            return False
        return False

    def _handle_text_line(self, line: str) -> bool:
        self.last_line = line
        self.outbox.put(("line", line))
        values = text_key_values(line)
        updated = False

        match = re.search(r"\bENC\s+J(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < ENCODER_COUNT:
                if "raw" in values:
                    self.encoder_raw[index] = int(float(values["raw"]))
                if "mapped" in values:
                    self.encoder_mapped[index] = int(float(values["mapped"]))
                if "deg" in values:
                    self.encoder_deg[index] = float(values["deg"])
                elif "mapped" in values:
                    self.encoder_deg[index] = self.encoder_mapped[index] * ENCODER_RAW_TO_DEG
                valid = values.get("mapped_valid", values.get("raw_valid"))
                if valid is not None:
                    self.encoder_valid[index] = valid not in ("0", "false", "False")
                updated = True

        match = re.search(r"\bSERVO\s+M(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < SERVO_COUNT:
                if "motor_abs" in values:
                    self.servo_abs[index] = int(float(values["motor_abs"]))
                if "hardware_abs" in values:
                    self.servo_hardware_abs[index] = int(float(values["hardware_abs"]))
                if "sw_zero_ofs" in values:
                    self.servo_zero_offset[index] = int(float(values["sw_zero_ofs"]))
                if "online" in values:
                    self.servo_online[index] = values["online"] not in ("0", "false", "False")
                updated = True

        match = re.search(r"\bLOAD\s+M(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < SERVO_COUNT:
                if "load" in values:
                    self.servo_load[index] = int(float(values["load"]))
                if "current" in values:
                    self.servo_current[index] = int(float(values["current"]))
                if "speed" in values:
                    self.servo_speed[index] = int(float(values["speed"]))
                if "voltage" in values:
                    self.servo_voltage[index] = int(float(values["voltage"]))
                if "temperature" in values:
                    self.servo_temperature[index] = int(float(values["temperature"]))
                if "online" in values:
                    self.servo_online[index] = values["online"] not in ("0", "false", "False")
                updated = True

        return updated

    def _emit_snapshot(self) -> None:
        now = time.time()
        if now - self.last_emit < 0.03:
            return
        self.last_emit = now
        self.outbox.put(
            (
                "snapshot",
                EncoderServoSnapshot(
                    timestamp=now,
                    encoder_raw=list(self.encoder_raw),
                    encoder_mapped=list(self.encoder_mapped),
                    encoder_deg=list(self.encoder_deg),
                    encoder_valid=list(self.encoder_valid),
                    servo_abs=list(self.servo_abs),
                    servo_hardware_abs=list(self.servo_hardware_abs),
                    servo_zero_offset=list(self.servo_zero_offset),
                    servo_online=list(self.servo_online),
                    servo_raw=list(self.servo_raw),
                    servo_raw_online=list(self.servo_raw_online),
                    servo_speed=list(self.servo_speed),
                    servo_load=list(self.servo_load),
                    servo_current=list(self.servo_current),
                    servo_voltage=list(self.servo_voltage),
                    servo_temperature=list(self.servo_temperature),
                    joint_debug=dict(self.joint_debug),
                    last_line=self.last_line,
                ),
            )
        )


class EncoderServoWindow(tk.Toplevel):
    def __init__(self, parent: tk.Misc, default_port: str, default_baudrate: int) -> None:
        super().__init__(parent)
        self.host = parent
        self.title("Encoder / Servo 监视与目标输出")
        self.geometry("1700x1000")
        self.minsize(1100, 760)

        self.outbox: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.command_queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.poller: Optional[EncoderServoPoller] = None
        self.connected = False
        self.history: list[EncoderServoSnapshot] = []
        self.latest_snapshot: Optional[EncoderServoSnapshot] = None
        self.motor_target_rejected_monotonic = 0.0

        self.port_var = tk.StringVar(value=default_port)
        self.baud_var = tk.StringVar(value=str(default_baudrate))
        self.status_var = tk.StringVar(value="未连接")
        self.last_line_var = tk.StringVar(value="RX: --")
        self.encoder_zero_status_var = tk.StringVar(value=self._encoder_zero_status_text())
        self.mode_var = tk.StringVar(value="degree")
        self.target_index_var = tk.IntVar(value=0)
        self.target_value_var = tk.StringVar(value="0")
        self.angle_kp_scale_var = tk.StringVar(value="0.70")
        self.angle_ki_scale_var = tk.StringVar(value="0.02")
        host_record_path_var = getattr(self.master, "continuous_record_path_var", None)
        self.feedback_record_path_var = (
            host_record_path_var
            if host_record_path_var is not None
            else tk.StringVar(master=self, value="--")
        )
        self.direct_abs_target_vars = [
            tk.StringVar(value="") for _ in range(DIRECT_ABS_UI_MOTOR_COUNT)
        ]
        self.direct_abs_targets_initialized = False
        self.direct_abs_apply_after_id: Optional[str] = None
        self.direct_abs_verify_after_id: Optional[str] = None
        self.direct_abs_sequence = 0
        self.direct_abs_pending_targets: Optional[list[int]] = None
        self.direct_abs_initial_abs: Optional[list[int]] = None
        self.direct_abs_started_monotonic = 0.0
        self.direct_abs_ack_received = False
        self.sent_degree_targets: dict[int, float] = {}
        self.sent_direct_targets: dict[int, int] = {}
        self.sent_last_target_command = "--"
        self.sent_target_var = tk.StringVar(value=self._sent_target_status_text())
        self.direct_abs_status_var = tk.StringVar(value="五路 ABS: idle")
        self.step_test_poses_var = tk.StringVar(value=STEP_TEST_DEFAULT_POSES)
        self.step_test_amplitudes_var = tk.StringVar(value=STEP_TEST_DEFAULT_AMPLITUDES)
        self.step_test_repeats_var = tk.IntVar(value=STEP_TEST_DEFAULT_REPEATS)
        self.step_test_baseline_var = tk.StringVar(value=f"{STEP_TEST_DEFAULT_BASELINE_S:.1f}")
        self.step_test_band_var = tk.StringVar(value=f"{STEP_TEST_DEFAULT_SETTLE_BAND_DEG:.1f}")
        self.step_test_hold_var = tk.StringVar(value=f"{STEP_TEST_DEFAULT_SETTLE_HOLD_S:.1f}")
        self.step_test_timeout_var = tk.StringVar(value=f"{STEP_TEST_DEFAULT_TIMEOUT_S:.1f}")
        self.step_test_status_var = tk.StringVar(value="自动阶跃: 未启动（J01全程必须>0°）")
        self.step_test_summary_path_var = tk.StringVar(value="--")
        self.step_test_running = False
        self.step_test_after_id: Optional[str] = None
        self.step_test_phase = "idle"
        self.step_test_phase_started_monotonic = 0.0
        self.step_test_stable_since_monotonic = 0.0
        self.step_test_poses: list[tuple[float, float, float, float]] = []
        self.step_test_trials: list[tuple[int, int, float, int]] = []
        self.step_test_trial_index = 0
        self.step_test_current_target: Optional[tuple[float, float, float, float]] = None
        self.step_test_baseline_s = STEP_TEST_DEFAULT_BASELINE_S
        self.step_test_band_deg = STEP_TEST_DEFAULT_SETTLE_BAND_DEG
        self.step_test_hold_s = STEP_TEST_DEFAULT_SETTLE_HOLD_S
        self.step_test_timeout_s = STEP_TEST_DEFAULT_TIMEOUT_S
        self.step_test_baseline_samples: list[tuple[float, tuple[float, float, float, float]]] = []
        self.step_test_response_samples: list[
            tuple[float, tuple[float, float, float, float], tuple[float, float, float, float, float]]
        ] = []
        self.step_test_last_snapshot_ts = 0.0
        self.step_test_response_started_monotonic = 0.0
        self.step_test_settle_candidate_monotonic = 0.0
        self.step_test_summary_file: Optional[object] = None
        self.step_test_summary_writer: Optional[csv.DictWriter] = None
        self.step_test_summary_path: Optional[Path] = None
        self.step_test_experiment_id = ""
        self.step_test_started_continuous_record = False
        self.sine_test_poses_var = tk.StringVar(value=SINE_TEST_DEFAULT_POSES)
        self.sine_test_amplitudes_var = tk.StringVar(value=SINE_TEST_DEFAULT_AMPLITUDES)
        self.sine_test_frequencies_var = tk.StringVar(value=SINE_TEST_DEFAULT_FREQUENCIES_HZ)
        self.sine_test_cycles_var = tk.IntVar(value=SINE_TEST_DEFAULT_CYCLES)
        self.sine_test_repeats_var = tk.IntVar(value=SINE_TEST_DEFAULT_REPEATS)
        self.sine_test_baseline_var = tk.StringVar(value=f"{SINE_TEST_DEFAULT_BASELINE_S:.1f}")
        self.sine_test_band_var = tk.StringVar(value=f"{SINE_TEST_DEFAULT_SETTLE_BAND_DEG:.1f}")
        self.sine_test_hold_var = tk.StringVar(value=f"{SINE_TEST_DEFAULT_SETTLE_HOLD_S:.1f}")
        self.sine_test_timeout_var = tk.StringVar(value=f"{SINE_TEST_DEFAULT_POSE_TIMEOUT_S:.1f}")
        self.sine_test_status_var = tk.StringVar(value="自动正弦: 未启动（J01目标及实测全程>0°）")
        self.sine_test_summary_path_var = tk.StringVar(value="--")
        self.sine_test_running = False
        self.sine_test_after_id: Optional[str] = None
        self.sine_test_phase = "idle"
        self.sine_test_phase_started_monotonic = 0.0
        self.sine_test_stable_since_monotonic = 0.0
        self.sine_test_poses: list[tuple[float, float, float, float]] = []
        self.sine_test_trials: list[tuple[int, int, float, float, int]] = []
        self.sine_test_trial_index = 0
        self.sine_test_current_target: Optional[tuple[float, float, float, float]] = None
        self.sine_test_cycles = SINE_TEST_DEFAULT_CYCLES
        self.sine_test_baseline_s = SINE_TEST_DEFAULT_BASELINE_S
        self.sine_test_band_deg = SINE_TEST_DEFAULT_SETTLE_BAND_DEG
        self.sine_test_hold_s = SINE_TEST_DEFAULT_SETTLE_HOLD_S
        self.sine_test_timeout_s = SINE_TEST_DEFAULT_POSE_TIMEOUT_S
        self.sine_test_last_snapshot_ts = 0.0
        self.sine_test_response_started_monotonic = 0.0
        self.sine_test_baseline_samples: list[tuple[float, tuple[float, float, float, float]]] = []
        self.sine_test_response_samples: list[
            tuple[
                float,
                tuple[float, float, float, float],
                tuple[float, float, float, float],
                tuple[float, float, float, float, float],
            ]
        ] = []
        self.sine_test_summary_file: Optional[object] = None
        self.sine_test_summary_writer: Optional[csv.DictWriter] = None
        self.sine_test_summary_path: Optional[Path] = None
        self.sine_test_experiment_id = ""
        self.sine_test_started_continuous_record = False

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_outbox)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=2)
        self.rowconfigure(3, weight=3)

        connection = ttk.LabelFrame(self, text="Encoder / Servo 串口")
        connection.grid(row=0, column=0, padx=10, pady=(10, 6), sticky="ew")
        connection.columnconfigure(12, weight=1)

        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=12)
        self.baud_combo = ttk.Combobox(
            connection,
            textvariable=self.baud_var,
            values=[str(v) for v in BAUD_RATES + [ENCODER_SERVO_DEFAULT_BAUDRATE]],
            width=10,
        )
        self.refresh_button = ttk.Button(connection, text="刷新串口", command=self._refresh_ports)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(8, 4), pady=8)
        self.port_combo.grid(row=0, column=1, padx=(0, 10), pady=8)
        ttk.Label(connection, text="波特率").grid(row=0, column=2, padx=(8, 4), pady=8)
        self.baud_combo.grid(row=0, column=3, padx=(0, 10), pady=8)
        self.refresh_button.grid(row=0, column=4, padx=4, pady=8)
        self.connect_button.grid(row=0, column=5, padx=4, pady=8)
        ttk.Label(connection, text="状态").grid(row=0, column=6, padx=(20, 4), pady=8)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=7, padx=(0, 10), pady=8, sticky="w")

        commands = ttk.LabelFrame(self, text="目标输出与快照命令")
        commands.grid(row=1, column=0, padx=10, pady=6, sticky="ew")
        for col in range(21):
            commands.columnconfigure(col, weight=0)
        commands.columnconfigure(20, weight=1)

        quick = [
            ("START", "start"),
            ("STOP", "stop"),
            ("ZERO", "zero"),
            ("ENCODER", "encoder"),
            ("SERVO", "servo"),
            ("STATUS", "status"),
            ("BINARY", "binary"),
        ]
        for col, (label, command) in enumerate(quick):
            ttk.Button(commands, text=label, command=lambda c=command: self._send_text(c)).grid(
                row=0, column=col, padx=3, pady=8
            )

        target_col = len(quick) + 1
        ttk.Label(commands, text="模式").grid(row=0, column=target_col, padx=(20, 4), pady=8)
        self.mode_combo = ttk.Combobox(
            commands,
            textvariable=self.mode_var,
            values=["degree", "direct"],
            width=8,
            state="readonly",
        )
        self.mode_combo.grid(row=0, column=target_col + 1, padx=(0, 8), pady=8)
        ttk.Label(commands, text="索引").grid(row=0, column=target_col + 2, padx=(4, 4), pady=8)
        self.target_index_spin = tk.Spinbox(commands, from_=0, to=SERVO_COUNT - 1, textvariable=self.target_index_var, width=5)
        self.target_index_spin.grid(row=0, column=target_col + 3, padx=(0, 8), pady=8)
        ttk.Label(commands, text="目标").grid(row=0, column=target_col + 4, padx=(4, 4), pady=8)
        self.target_entry = ttk.Entry(commands, textvariable=self.target_value_var, width=28)
        self.target_entry.grid(row=0, column=target_col + 5, padx=(0, 8), pady=8)
        ttk.Button(commands, text="发送目标", command=self._send_target).grid(
            row=0, column=target_col + 6, padx=4, pady=8
        )
        ttk.Button(commands, text="发送J00-J03目标", command=self._send_j00_j03_target).grid(
            row=0, column=target_col + 7, padx=4, pady=8
        )
        self.feedback_record_button = ttk.Button(
            commands,
            text=(
                "停止反馈连续记录"
                if getattr(self.master, "continuous_record_running", False)
                else "开始反馈连续记录"
            ),
            command=self._toggle_feedback_continuous_record,
        )
        self.feedback_record_button.grid(row=0, column=target_col + 8, padx=(8, 4), pady=8)
        ttk.Label(commands, text="CSV:").grid(
            row=0, column=target_col + 9, padx=(8, 4), pady=8, sticky="e"
        )
        ttk.Label(
            commands,
            textvariable=self.feedback_record_path_var,
            anchor="w",
        ).grid(
            row=0,
            column=target_col + 10,
            columnspan=3,
            padx=(0, 8),
            pady=8,
            sticky="ew",
        )
        ttk.Button(commands, text="固件机械零位", command=self._set_encoder_zero).grid(
            row=1, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Button(commands, text="删除旧主机零点", command=self._clear_encoder_zero).grid(
            row=1, column=2, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, text="零点状态").grid(row=1, column=4, padx=(16, 4), pady=(0, 8), sticky="w")
        ttk.Label(commands, textvariable=self.encoder_zero_status_var).grid(
            row=1, column=5, columnspan=12, padx=(0, 8), pady=(0, 8), sticky="w"
        )

        ttk.Label(commands, text="五电机 ABS").grid(row=2, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w")
        abs_col = 2
        for motor, target_var in enumerate(self.direct_abs_target_vars):
            ttk.Label(commands, text=f"M{motor:02d}").grid(
                row=2, column=abs_col, padx=(4, 2), pady=(0, 8), sticky="e"
            )
            ttk.Entry(commands, textvariable=target_var, width=8).grid(
                row=2, column=abs_col + 1, padx=(0, 4), pady=(0, 8), sticky="w"
            )
            abs_col += 2
        ttk.Button(commands, text="读取当前 ABS", command=self._load_current_direct_abs_targets).grid(
            row=2, column=abs_col, padx=(8, 4), pady=(0, 8), sticky="w"
        )
        ttk.Button(commands, text="移动到五路 ABS", command=self._send_direct_abs_targets).grid(
            row=2, column=abs_col + 1, padx=4, pady=(0, 8), sticky="w"
        )

        ttk.Label(commands, text="下发目标").grid(row=3, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w")
        ttk.Label(commands, textvariable=self.sent_target_var).grid(
            row=3, column=2, columnspan=14, padx=(0, 8), pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, text="五路 ABS 状态").grid(
            row=4, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, textvariable=self.direct_abs_status_var).grid(
            row=4, column=2, columnspan=14, padx=(0, 8), pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, text="P 增益 Kp").grid(
            row=5, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Entry(commands, textvariable=self.angle_kp_scale_var, width=8).grid(
            row=5, column=2, padx=(0, 4), pady=(0, 8), sticky="w"
        )
        ttk.Button(commands, text="应用 Kp", command=self._apply_angle_kp).grid(
            row=5, column=3, columnspan=2, padx=4, pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, text="I 增益 Ki(1/s)").grid(
            row=5, column=5, padx=(8, 3), pady=(0, 8), sticky="w"
        )
        ttk.Entry(commands, textvariable=self.angle_ki_scale_var, width=8).grid(
            row=5, column=6, padx=(0, 4), pady=(0, 8), sticky="w"
        )
        ttk.Button(commands, text="应用 Ki", command=self._apply_angle_ki).grid(
            row=5, column=7, columnspan=2, padx=4, pady=(0, 8), sticky="w"
        )
        ttk.Label(
            commands,
            text=(
                "R 前馈：ΔL_ff=200×R×(θref−θentry)；"
                "PI 反馈：ΔL_fb=200×P×[Kp×e+Ki×∫e dt]；"
                "任务投影：ΔL_task=(R×R⁺)×(ΔL_ff+ΔL_fb)；"
                "最终再加零空间αn；积分死区±0.2°，无独立I项限幅"
            ),
        ).grid(row=6, column=0, columnspan=17, padx=(3, 4), pady=(0, 8), sticky="w")

        ttk.Label(commands, text="阶跃27姿态(J00;J01;J02;J03，|分隔)").grid(
            row=7, column=0, columnspan=3, padx=3, pady=(0, 6), sticky="w"
        )
        self.step_test_poses_entry = ttk.Entry(
            commands, textvariable=self.step_test_poses_var, width=104
        )
        self.step_test_poses_entry.grid(
            row=7, column=3, columnspan=13, padx=(0, 8), pady=(0, 6), sticky="ew"
        )
        self.step_test_start_button = ttk.Button(
            commands, text="开始自动阶跃", command=self._start_step_test
        )
        self.step_test_start_button.grid(row=7, column=16, padx=4, pady=(0, 6), sticky="w")
        self.step_test_stop_button = ttk.Button(
            commands,
            text="停止自动阶跃",
            command=lambda: self._stop_step_test("手动停止", send_stop=True),
            state="disabled",
        )
        self.step_test_stop_button.grid(row=7, column=17, padx=4, pady=(0, 6), sticky="w")

        ttk.Label(commands, text="阶跃幅值°").grid(row=8, column=0, padx=3, pady=(0, 6), sticky="w")
        self.step_test_amplitudes_entry = ttk.Entry(
            commands, textvariable=self.step_test_amplitudes_var, width=8
        )
        self.step_test_amplitudes_entry.grid(row=8, column=1, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="重复").grid(row=8, column=2, padx=3, pady=(0, 6), sticky="w")
        self.step_test_repeats_spin = tk.Spinbox(
            commands, from_=1, to=20, increment=1, textvariable=self.step_test_repeats_var, width=4
        )
        self.step_test_repeats_spin.grid(row=8, column=3, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="基线s").grid(row=8, column=4, padx=3, pady=(0, 6), sticky="w")
        self.step_test_baseline_entry = ttk.Entry(
            commands, textvariable=self.step_test_baseline_var, width=5
        )
        self.step_test_baseline_entry.grid(row=8, column=5, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="稳定±°").grid(row=8, column=6, padx=3, pady=(0, 6), sticky="w")
        self.step_test_band_entry = ttk.Entry(commands, textvariable=self.step_test_band_var, width=5)
        self.step_test_band_entry.grid(row=8, column=7, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="连续稳定s").grid(row=8, column=8, padx=3, pady=(0, 6), sticky="w")
        self.step_test_hold_entry = ttk.Entry(commands, textvariable=self.step_test_hold_var, width=5)
        self.step_test_hold_entry.grid(row=8, column=9, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="单阶跃超时s").grid(row=8, column=10, padx=3, pady=(0, 6), sticky="w")
        self.step_test_timeout_entry = ttk.Entry(
            commands, textvariable=self.step_test_timeout_var, width=5
        )
        self.step_test_timeout_entry.grid(row=8, column=11, padx=(0, 8), pady=(0, 6), sticky="w")
        ttk.Label(commands, textvariable=self.step_test_status_var).grid(
            row=8, column=12, columnspan=9, padx=(4, 8), pady=(0, 6), sticky="w"
        )
        ttk.Label(commands, text="汇总CSV").grid(row=9, column=0, padx=3, pady=(0, 6), sticky="w")
        ttk.Label(commands, textvariable=self.step_test_summary_path_var).grid(
            row=9, column=1, columnspan=20, padx=(0, 8), pady=(0, 6), sticky="w"
        )

        ttk.Label(commands, text="正弦27姿态(J00;J01;J02;J03，|分隔)").grid(
            row=10, column=0, columnspan=3, padx=3, pady=(0, 6), sticky="w"
        )
        self.sine_test_poses_entry = ttk.Entry(
            commands, textvariable=self.sine_test_poses_var, width=104
        )
        self.sine_test_poses_entry.grid(
            row=10, column=3, columnspan=13, padx=(0, 8), pady=(0, 6), sticky="ew"
        )
        self.sine_test_start_button = ttk.Button(
            commands, text="开始自动正弦", command=self._start_sine_test
        )
        self.sine_test_start_button.grid(row=10, column=16, padx=4, pady=(0, 6), sticky="w")
        self.sine_test_stop_button = ttk.Button(
            commands,
            text="停止自动正弦",
            command=lambda: self._stop_sine_test("手动停止", send_stop=True),
            state="disabled",
        )
        self.sine_test_stop_button.grid(row=10, column=17, padx=4, pady=(0, 6), sticky="w")

        ttk.Label(commands, text="幅值°").grid(row=11, column=0, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_amplitudes_entry = ttk.Entry(
            commands, textvariable=self.sine_test_amplitudes_var, width=7
        )
        self.sine_test_amplitudes_entry.grid(row=11, column=1, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="频率Hz").grid(row=11, column=2, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_frequencies_entry = ttk.Entry(
            commands, textvariable=self.sine_test_frequencies_var, width=18
        )
        self.sine_test_frequencies_entry.grid(row=11, column=3, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="周期").grid(row=11, column=4, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_cycles_spin = tk.Spinbox(
            commands, from_=1, to=20, increment=1, textvariable=self.sine_test_cycles_var, width=4
        )
        self.sine_test_cycles_spin.grid(row=11, column=5, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="重复").grid(row=11, column=6, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_repeats_spin = tk.Spinbox(
            commands, from_=1, to=20, increment=1, textvariable=self.sine_test_repeats_var, width=4
        )
        self.sine_test_repeats_spin.grid(row=11, column=7, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="基线s").grid(row=11, column=8, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_baseline_entry = ttk.Entry(
            commands, textvariable=self.sine_test_baseline_var, width=5
        )
        self.sine_test_baseline_entry.grid(row=11, column=9, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="到位±°/保持s").grid(row=11, column=10, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_band_entry = ttk.Entry(commands, textvariable=self.sine_test_band_var, width=5)
        self.sine_test_band_entry.grid(row=11, column=11, padx=(0, 3), pady=(0, 6), sticky="w")
        self.sine_test_hold_entry = ttk.Entry(commands, textvariable=self.sine_test_hold_var, width=5)
        self.sine_test_hold_entry.grid(row=11, column=12, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, text="姿态超时s").grid(row=11, column=13, padx=3, pady=(0, 6), sticky="w")
        self.sine_test_timeout_entry = ttk.Entry(commands, textvariable=self.sine_test_timeout_var, width=5)
        self.sine_test_timeout_entry.grid(row=11, column=14, padx=(0, 5), pady=(0, 6), sticky="w")
        ttk.Label(commands, textvariable=self.sine_test_status_var).grid(
            row=11, column=15, columnspan=6, padx=(4, 8), pady=(0, 6), sticky="w"
        )
        ttk.Label(commands, text="正弦汇总CSV").grid(row=12, column=0, padx=3, pady=(0, 6), sticky="w")
        ttk.Label(commands, textvariable=self.sine_test_summary_path_var).grid(
            row=12, column=1, columnspan=20, padx=(0, 8), pady=(0, 6), sticky="w"
        )

        self.encoder_plot = tk.Canvas(
            self,
            height=230,
            bg="white",
            highlightthickness=1,
            highlightbackground="#d0d7de",
        )
        self.encoder_plot.grid(row=2, column=0, padx=10, pady=6, sticky="nsew")
        self.encoder_plot.bind("<Configure>", lambda _event: self._draw_encoder_plot())

        data = ttk.Frame(self)
        data.grid(row=3, column=0, padx=10, pady=6, sticky="nsew")
        data.columnconfigure(0, weight=1)
        data.columnconfigure(1, weight=2)
        data.rowconfigure(0, weight=1)

        encoder_frame = ttk.LabelFrame(data, text="Encoder 角度")
        encoder_frame.grid(row=0, column=0, padx=(0, 6), sticky="nsew")
        encoder_frame.columnconfigure(0, weight=1)
        encoder_frame.rowconfigure(0, weight=1)
        self.encoder_table = ttk.Treeview(
            encoder_frame,
            columns=("joint", "raw", "mapped", "deg", "valid"),
            show="headings",
            height=12,
        )
        for key, text, width, anchor in (
            ("joint", "Joint", 70, "center"),
            ("raw", "Raw", 86, "e"),
            ("mapped", "Mapped", 86, "e"),
            ("deg", "Deg", 86, "e"),
            ("valid", "Valid", 64, "center"),
        ):
            self.encoder_table.heading(key, text=text)
            self.encoder_table.column(key, width=width, anchor=anchor, stretch=(key == "deg"))
        enc_scroll = ttk.Scrollbar(encoder_frame, orient="vertical", command=self.encoder_table.yview)
        self.encoder_table.configure(yscrollcommand=enc_scroll.set)
        self.encoder_table.grid(row=0, column=0, sticky="nsew")
        enc_scroll.grid(row=0, column=1, sticky="ns")
        for index in range(ENCODER_COUNT):
            self.encoder_table.insert("", "end", iid=f"J{index}", values=(f"J{index:02d}", "--", "--", "--", "--"))

        servo_frame = ttk.LabelFrame(data, text="Servo 数字读数")
        servo_frame.grid(row=0, column=1, padx=(6, 0), sticky="nsew")
        servo_frame.columnconfigure(0, weight=1)
        servo_frame.rowconfigure(0, weight=1)
        self.servo_table = ttk.Treeview(
            servo_frame,
            columns=("motor", "abs", "hardware", "zero", "raw", "speed", "load", "current", "volt", "temp", "online"),
            show="headings",
            height=12,
        )
        servo_columns = (
            ("motor", "Motor", 64, "center"),
            ("abs", "Abs", 86, "e"),
            ("hardware", "HwAbs", 86, "e"),
            ("zero", "ZeroOfs", 86, "e"),
            ("raw", "Raw", 72, "e"),
            ("speed", "Speed", 72, "e"),
            ("load", "Load", 72, "e"),
            ("current", "Current", 78, "e"),
            ("volt", "V", 52, "e"),
            ("temp", "Temp", 58, "e"),
            ("online", "Online", 64, "center"),
        )
        for key, text, width, anchor in servo_columns:
            self.servo_table.heading(key, text=text)
            self.servo_table.column(key, width=width, anchor=anchor, stretch=(key in ("abs", "hardware")))
        servo_scroll = ttk.Scrollbar(servo_frame, orient="vertical", command=self.servo_table.yview)
        self.servo_table.configure(yscrollcommand=servo_scroll.set)
        self.servo_table.grid(row=0, column=0, sticky="nsew")
        servo_scroll.grid(row=0, column=1, sticky="ns")
        for index in range(SERVO_COUNT):
            self.servo_table.insert(
                "",
                "end",
                iid=f"M{index}",
                values=(f"M{index:02d}", "--", "--", "--", "--", "--", "--", "--", "--", "--"),
            )

        line_frame = ttk.LabelFrame(self, text="通讯状态")
        line_frame.grid(row=4, column=0, padx=10, pady=(6, 10), sticky="ew")
        line_frame.columnconfigure(0, weight=1)
        ttk.Label(line_frame, textvariable=self.last_line_var).grid(row=0, column=0, padx=8, pady=6, sticky="w")

    def _refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if not self.port_var.get() and ports:
            candidates = [port for port in ports if port.upper() != "COM7"]
            self.port_var.set(candidates[0] if candidates else ports[0])

    def _toggle_connection(self) -> None:
        if self.poller is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        try:
            port = self.port_var.get().strip()
            baudrate = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "请检查 Encoder / Servo 波特率。", parent=self)
            return
        if not port:
            messagebox.showwarning("缺少串口", "请选择 Encoder / Servo 串口。", parent=self)
            return

        self.stop_event.clear()
        self.poller = EncoderServoPoller(self.outbox, self.command_queue, self.stop_event, port, baudrate)
        self.poller.start()
        self._set_controls_connected(True)

    def _disconnect(self) -> None:
        if self.step_test_running:
            self._stop_step_test("Encoder / Servo串口断开", send_stop=True)
        if self.sine_test_running:
            self._stop_sine_test("Encoder / Servo串口断开", send_stop=True)
        self._cancel_direct_abs_sequence("串口已断开")
        host = self.master
        if getattr(host, "linearity_running", False):
            host._stop_linearity_experiment("Encoder / Servo disconnected", send_cleanup=False)
        if getattr(host, "multi_input_running", False):
            host._stop_multi_input_experiment("Encoder / Servo disconnected", send_cleanup=False)
        if getattr(host, "feedback_pretension_active", False):
            self.queue_emergency_stop()
            host._stop_nullspace_pretension(
                "Encoder / Servo disconnected", hold_bias=False
            )
        if getattr(host, "degree_force_active", False):
            host._stop_degree_force_control("Encoder / Servo disconnected")
        self.stop_event.set()
        poller = self.poller
        self.poller = None
        if poller is not None:
            poller.join(timeout=1.5)
        self._set_controls_connected(False)

    def _set_controls_connected(self, connected: bool) -> None:
        self.connected = connected
        self.connect_button.configure(text="断开" if connected else "连接")
        state = "disabled" if connected else "normal"
        self.port_combo.configure(state=state)
        self.baud_combo.configure(state=state)
        self.refresh_button.configure(state=state)

    def _process_outbox(self) -> None:
        try:
            while True:
                kind, payload = self.outbox.get_nowait()
                if kind == "snapshot":
                    self._handle_snapshot(payload)
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self.status_var.set(str(payload))
                elif kind == "connected":
                    self._set_controls_connected(bool(payload))
                    if not payload:
                        self._cancel_direct_abs_sequence("串口已断开")
                        self.poller = None
                elif kind == "proto_ack":
                    cmd, applied_mode, ack_status = payload
                    if cmd == CMD_MOTOR_POS_ABS and self.direct_abs_pending_targets is not None:
                        if ack_status == 0:
                            self.direct_abs_ack_received = True
                            self.direct_abs_status_var.set(
                                f"五路 ABS: 下位机已确认目标帧（mode={applied_mode}），等待位置反馈"
                            )
                        else:
                            self.direct_abs_status_var.set(
                                f"五路 ABS: 下位机拒绝目标帧（status={ack_status}, mode={applied_mode}）"
                            )
                elif kind == "line":
                    line = str(payload)
                    if "MOTOR_TARGET rejected" in line or "MOTOR_TARGET pending" in line:
                        self.motor_target_rejected_monotonic = time.monotonic()
                    if self.direct_abs_pending_targets is not None and "<<<STATUS " in line:
                        values = text_key_values(line)
                        ready = (
                            values.get("mode") == "1"
                            and values.get("enabled") == "1"
                            and values.get("owner") == "1"
                            and values.get("state") == "2"
                        )
                        if not ready:
                            self.direct_abs_status_var.set(
                                "五路 ABS: 控制状态未就绪 "
                                f"(mode={values.get('mode', '?')}, enabled={values.get('enabled', '?')}, "
                                f"owner={values.get('owner', '?')}, state={values.get('state', '?')})"
                            )
                    self.last_line_var.set(f"RX: {line}")
                elif kind == "tx":
                    command = str(payload)
                    self.last_line_var.set(f"TX: {command}")
                    self.host._record_continuous_command_event(command)
        except queue.Empty:
            pass
        if self.winfo_exists():
            self.after(50, self._process_outbox)

    def _encoder_display_deg(self, snapshot: EncoderServoSnapshot, joint: int) -> float:
        return snapshot.encoder_deg[joint]

    def _encoder_zero_status_text(self) -> str:
        return "使用固件标定的机械零位；上位机不保存、不叠加角度零点。"

    def _refresh_encoder_table_and_plot(self) -> None:
        snapshot = self.latest_snapshot
        if snapshot is None:
            self._draw_encoder_plot()
            return
        for index in range(ENCODER_COUNT):
            self.encoder_table.item(
                f"J{index}",
                values=(
                    f"J{index:02d}",
                    snapshot.encoder_raw[index],
                    snapshot.encoder_mapped[index],
                    f"{self._encoder_display_deg(snapshot, index):.2f}",
                    "OK" if snapshot.encoder_valid[index] else "--",
                ),
            )
        self._draw_encoder_plot()

    def _set_encoder_zero(self) -> None:
        try:
            delete_encoder_zero_offsets()
        except Exception as exc:
            messagebox.showerror("删除失败", f"旧主机角度零点删除失败: {exc}", parent=self)
            return
        self.encoder_zero_status_var.set(self._encoder_zero_status_text())
        self.last_line_var.set("使用固件机械零位；degree 目标将直接发送，不加上位机偏置。")
        self._refresh_encoder_table_and_plot()

    def _clear_encoder_zero(self) -> None:
        try:
            delete_encoder_zero_offsets()
        except Exception as exc:
            messagebox.showerror("删除失败", f"旧主机角度零点配置删除失败: {exc}", parent=self)
            return
        self.encoder_zero_status_var.set(self._encoder_zero_status_text())
        self.last_line_var.set("旧主机角度零点已删除；当前只使用固件机械零位。")
        self._refresh_encoder_table_and_plot()

    def _handle_snapshot(self, snapshot: EncoderServoSnapshot) -> None:
        self.latest_snapshot = snapshot
        self.history.append(snapshot)
        if len(self.history) > ENCODER_SERVO_MAX_HISTORY:
            del self.history[:300]
        if not self.direct_abs_targets_initialized:
            first_five_online = all(
                snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
                for motor in range(DIRECT_ABS_UI_MOTOR_COUNT)
            )
            if first_five_online and all(not var.get().strip() for var in self.direct_abs_target_vars):
                self._set_direct_abs_target_entries(snapshot.servo_abs[:DIRECT_ABS_UI_MOTOR_COUNT])
            self.direct_abs_targets_initialized = True
        for index in range(ENCODER_COUNT):
            self.encoder_table.item(
                f"J{index}",
                values=(
                    f"J{index:02d}",
                    snapshot.encoder_raw[index],
                    snapshot.encoder_mapped[index],
                    f"{self._encoder_display_deg(snapshot, index):.2f}",
                    "OK" if snapshot.encoder_valid[index] else "--",
                ),
            )
        for index in range(SERVO_COUNT):
            online = snapshot.servo_online[index] or snapshot.servo_raw_online[index]
            self.servo_table.item(
                f"M{index}",
                values=(
                    f"M{index:02d}",
                    snapshot.servo_abs[index],
                    snapshot.servo_hardware_abs[index],
                    snapshot.servo_zero_offset[index],
                    snapshot.servo_raw[index],
                    snapshot.servo_speed[index],
                    snapshot.servo_load[index],
                    snapshot.servo_current[index],
                    snapshot.servo_voltage[index],
                    snapshot.servo_temperature[index],
                    "OK" if online else "--",
                ),
            )
        self.last_line_var.set(f"RX: {snapshot.last_line}")
        self._draw_encoder_plot()

    def _draw_encoder_plot(self) -> None:
        canvas = self.encoder_plot
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 56, 18, 18, 34
        x0, y0 = pad_l, height - pad_b
        x1, y1 = width - pad_r, pad_t
        canvas.create_line(x0, y0, x1, y0, fill="#889096")
        canvas.create_line(x0, y0, x0, y1, fill="#889096")
        canvas.create_text(x0 + 8, y1 + 8, text="Encoder J00-J03 angles from firmware zero (deg)", anchor="w", fill="#334155")

        visible = self.history[-300:]
        if not visible:
            canvas.create_text(width / 2, height / 2, text="等待 Encoder / Servo 数据...", fill="#667085")
            return

        values: list[float] = []
        for snapshot in visible:
            for joint in ENCODER_PLOT_JOINTS:
                if snapshot.encoder_valid[joint]:
                    values.append(self._encoder_display_deg(snapshot, joint))
        if not values:
            values = [
                self._encoder_display_deg(snapshot, joint)
                for snapshot in visible
                for joint in ENCODER_PLOT_JOINTS
            ]
        v_min, v_max = min(values), max(values)
        if v_min == v_max:
            v_min -= 1.0
            v_max += 1.0
        t0 = visible[0].timestamp
        t1 = visible[-1].timestamp
        if t0 == t1:
            t1 = t0 + 1.0

        for i in range(5):
            y = y0 - (y0 - y1) * i / 4
            value = v_min + (v_max - v_min) * i / 4
            canvas.create_line(x0, y, x1, y, fill="#edf2f7")
            canvas.create_text(6, y, text=f"{value:.1f}", anchor="w", fill="#667085")

        for joint in ENCODER_PLOT_JOINTS:
            points: list[float] = []
            color = ENCODER_SERVO_COLORS[joint % len(ENCODER_SERVO_COLORS)]
            for snapshot in visible:
                if not snapshot.encoder_valid[joint]:
                    continue
                value = self._encoder_display_deg(snapshot, joint)
                x = x0 + (x1 - x0) * (snapshot.timestamp - t0) / (t1 - t0)
                y = y0 - (y0 - y1) * (value - v_min) / (v_max - v_min)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=1)

        legend_x = x0 + 8
        legend_y = height - 14
        for joint in ENCODER_PLOT_JOINTS:
            color = ENCODER_SERVO_COLORS[joint % len(ENCODER_SERVO_COLORS)]
            canvas.create_text(legend_x, legend_y, text=f"J{joint:02d}", anchor="w", fill=color)
            legend_x += 48
        canvas.create_text((x0 + x1) / 2, height - 10, text="时间", fill="#667085")

    def _sent_target_status_text(self) -> str:
        joint_parts = []
        for joint in ENCODER_PLOT_JOINTS:
            target = self.sent_degree_targets.get(joint)
            value = "--" if target is None else f"{target:.2f} deg"
            joint_parts.append(f"J{joint:02d}: {value}")
        return " | ".join(joint_parts) + f"    last: {self.sent_last_target_command}"

    def _record_sent_target_command(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        target_seen = False
        for match in DEGREE_TARGET_RE.finditer(text):
            joint = int(match.group(1))
            if 0 <= joint < ENCODER_COUNT:
                self.sent_degree_targets[joint] = float(match.group(2))
                target_seen = True
        direct_target_command = re.match(
            r"^(?:(?:direct|derict|motor_mode)\s*;\s*)?m\d+\b",
            text,
            re.IGNORECASE,
        )
        if direct_target_command is not None:
            for match in DIRECT_TARGET_RE.finditer(text):
                motor = int(match.group(1))
                if 0 <= motor < SERVO_COUNT:
                    self.sent_direct_targets[motor] = int(round(float(match.group(2))))
                    target_seen = True
        if target_seen:
            self.sent_last_target_command = text
            self.sent_target_var.set(self._sent_target_status_text())

    def _note_sent_mode_command(self, text: str) -> None:
        mode = text.strip().split(";", 1)[0].strip().lower()
        if mode in ("direct", "derict", "motor_mode"):
            self.mode_var.set("direct")
        elif mode in ("degree", "deg", "joint"):
            self.mode_var.set("degree")

    def _current_host_degree_targets(self) -> list[float]:
        snapshot = self.latest_snapshot
        feedback_deg = snapshot.encoder_deg if snapshot is not None else []
        feedback_valid = snapshot.encoder_valid if snapshot is not None else []
        return build_host_degree_target_vector(
            self.sent_degree_targets,
            feedback_deg,
            feedback_valid,
        )

    def _current_host_direct_targets(self) -> list[int]:
        snapshot = self.latest_snapshot
        feedback_abs = snapshot.servo_abs if snapshot is not None else []
        return build_host_direct_target_vector(self.sent_direct_targets, feedback_abs)

    def _queue_start_with_host_state(self) -> None:
        mode = self.mode_var.get().strip().lower()
        host = self.master
        if getattr(host, "force_emergency_relax_active", False):
            messagebox.showwarning(
                "全电机正在安全放松",
                f"拉力恢复到五路均 > {FORCE_HARD_RELAX_STOP_N:.0f} N "
                "并自动停止前，不能启动其他运动。",
                parent=self,
            )
            return

        if mode == "degree":
            kp_scale = self._read_angle_kp_scale(show_error=True)
            if kp_scale is None:
                return
            ki_scale = self._read_angle_ki_scale(show_error=True)
            if ki_scale is None:
                return
            if hasattr(host, "_start_or_refresh_nullspace_tension_control"):
                host._start_or_refresh_nullspace_tension_control(
                    self,
                    self.latest_snapshot,
                )
            if hasattr(host, "_clear_tension_bias_for_angle_start"):
                host._clear_tension_bias_for_angle_start(self)
            targets = self._current_host_degree_targets()
            if hasattr(host, "_start_or_refresh_degree_force_control"):
                degree_force_ready = host._start_or_refresh_degree_force_control(
                    self,
                    self.latest_snapshot,
                )
                if bool(host.degree_force_enabled_var.get()) and not degree_force_ready:
                    messagebox.showwarning(
                        "degree 力反馈未启动",
                        host.degree_force_status_var.get(),
                        parent=self,
                    )
                    return
            self.command_queue.put(("text", f"kp {kp_scale:.4f}"))
            self.command_queue.put(("text", f"ki {ki_scale:.6f}"))
            self.command_queue.put(("text", "degree"))
            self.command_queue.put(("bytes", build_upper_angle_cmd(targets)))
            target_text = ", ".join(
                f"J{joint:02d}={targets[joint]:.2f}"
                for joint in ENCODER_PLOT_JOINTS
            )
            label = f"start degree [{target_text}]"
        else:
            if getattr(host, "feedback_pretension_active", False):
                host._stop_nullspace_pretension("direct START requested", hold_bias=False)
            if getattr(host, "degree_force_active", False):
                host._stop_degree_force_control("direct START requested", send_stop=False)
            targets = self._current_host_direct_targets()
            self.command_queue.put(("text", "tension off"))
            self.command_queue.put(("text", "direct"))
            self.command_queue.put(("bytes", build_upper_motor_pos_abs_cmd(targets)))
            known = sorted(self.sent_direct_targets)
            target_text = ", ".join(f"M{motor:02d}={targets[motor]}" for motor in known[:5])
            label = f"start direct [{target_text or 'feedback hold'}]"

        # Configure the stopped controller first, then enable output. This avoids
        # one control tick using stale lower-controller targets.
        self.command_queue.put(("text", "start"))
        self.command_queue.put(("text", "status"))
        self.sent_last_target_command = label
        self.sent_target_var.set(self._sent_target_status_text())

    def _read_angle_kp_scale(self, show_error: bool = False) -> Optional[float]:
        try:
            value = float(self.angle_kp_scale_var.get().strip())
        except ValueError:
            value = float("nan")
        if not math.isfinite(value) or value < 0.0 or value > 5.0:
            if show_error:
                messagebox.showwarning(
                    "Kp 参数无效",
                    "P 增益 Kp 必须在 0～5 之间。当前启动默认值为 0.70。",
                    parent=self,
                )
            return None
        return value

    def _apply_angle_kp(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        value = self._read_angle_kp_scale(show_error=True)
        if value is None:
            return
        self.command_queue.put(("text", f"kp {value:.4f}"))
        self.last_line_var.set(f"TX: P gain Kp={value:.4f}")

    def _read_angle_ki_scale(self, show_error: bool = False) -> Optional[float]:
        try:
            value = float(self.angle_ki_scale_var.get().strip())
        except ValueError:
            value = float("nan")
        if not math.isfinite(value) or value < 0.0 or value > 1.0:
            if show_error:
                messagebox.showwarning(
                    "Ki 参数无效",
                    "I 增益 Ki 必须在 0～1 (1/s) 之间。当前启动默认值为 0.02。",
                    parent=self,
                )
            return None
        return value

    def _apply_angle_ki(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        ki_scale = self._read_angle_ki_scale(show_error=True)
        if ki_scale is None:
            return
        self.command_queue.put(("text", f"ki {ki_scale:.6f}"))
        self.last_line_var.set(f"TX: I gain Ki={ki_scale:.6f}/s (no separate I limit)")

    def _queue_zero_reset(self) -> None:
        host = self.master
        if hasattr(host, "_reset_host_control_state_for_zero"):
            host._reset_host_control_state_for_zero()
        self.clear_pending_commands()
        self.sent_degree_targets.clear()
        self.sent_direct_targets.clear()
        self.target_index_var.set(0)
        self.target_value_var.set("0")
        for target_var in self.direct_abs_target_vars:
            target_var.set("")
        # ZERO changes the software ABS coordinate. Require an explicit refresh
        # so an in-flight pre-ZERO snapshot cannot refill these fields.
        self.direct_abs_targets_initialized = True
        self.sent_last_target_command = "zero"
        self.sent_target_var.set(self._sent_target_status_text())
        self.command_queue.put(("text", "zero"))
        self.command_queue.put(("text", "status"))

    def _send_text(self, text: str) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        command = text.strip().split(";", 1)[0].strip().lower()
        host = self.master
        if self.step_test_running:
            if command == "stop":
                self._stop_step_test("STOP command requested", send_stop=True)
                return
            if command in ("start", "enable", "run", "zero", "setzero", "servozero", "direct", "derict", "motor_mode"):
                messagebox.showinfo(
                    "自动阶跃运行中",
                    "请先停止自动阶跃，再发送会改变控制状态的手动命令。",
                    parent=self,
                )
                return
        if self.sine_test_running:
            if command == "stop":
                self._stop_sine_test("STOP command requested", send_stop=True)
                return
            if command in ("start", "enable", "run", "zero", "setzero", "servozero", "direct", "derict", "motor_mode"):
                messagebox.showinfo(
                    "自动正弦运行中",
                    "请先停止自动正弦，再发送会改变控制状态的手动命令。",
                    parent=self,
                )
                return
        if command in ("start", "enable", "run"):
            self._cancel_direct_abs_sequence("已由手动 START 接管")
            self._queue_start_with_host_state()
            return
        if command in ("zero", "setzero", "servozero"):
            self._cancel_direct_abs_sequence("ZERO 已取消五路 ABS")
            self._queue_zero_reset()
            return
        if command == "stop":
            self._cancel_direct_abs_sequence("STOP 已取消五路 ABS")
            if getattr(host, "feedback_pretension_active", False):
                host._stop_nullspace_pretension("STOP command requested", hold_bias=False)
            if getattr(host, "linearity_running", False):
                host._stop_linearity_experiment("STOP command requested", send_cleanup=False)
            if getattr(host, "multi_input_running", False):
                host._stop_multi_input_experiment("STOP command requested", send_cleanup=False)
            if getattr(host, "force_target_running", False):
                host._stop_force_target_control("STOP command requested")
            if getattr(host, "safe_relax_active", False):
                host._stop_safe_relax("STOP command requested", send_stop=False)
            if getattr(host, "training_running", False):
                host._stop_training_collection("STOP command requested")
            if getattr(host, "degree_force_active", False):
                host._stop_degree_force_control("STOP command requested", send_stop=False)
            self.queue_emergency_stop()
            return
        if command in ("direct", "derict", "motor_mode") and getattr(
            host, "feedback_pretension_active", False
        ):
            host._stop_nullspace_pretension(f"{command} command requested", hold_bias=False)
        if command in ("stop", "direct", "derict", "motor_mode") and getattr(host, "degree_force_active", False):
            host._stop_degree_force_control(f"{command} command requested")
        self.command_queue.put(("text", text))
        self._note_sent_mode_command(text)

    def clear_pending_commands(self) -> int:
        cleared = 0
        while True:
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                return cleared
            cleared += 1

    def queue_emergency_stop(self) -> bool:
        if self.poller is None:
            return False
        self._cancel_direct_abs_sequence("STOP 已取消五路 ABS")
        self.clear_pending_commands()
        commands = ["stop", "status"]
        for command in commands:
            self.command_queue.put(("text", command))
        self.sent_last_target_command = "stop"
        self.sent_target_var.set(self._sent_target_status_text())
        return True

    def queue_bytes_command(self, data: bytes, label: str = "") -> bool:
        if self.poller is None:
            return False
        self.command_queue.put(("bytes", bytes(data)))
        if label:
            self.sent_last_target_command = label
            self.sent_target_var.set(self._sent_target_status_text())
        return True

    def queue_text_command(self, text: str) -> bool:
        if self.poller is None:
            return False
        self.command_queue.put(("text", text))
        self._note_sent_mode_command(text)
        self._record_sent_target_command(text)
        return True

    def _cancel_direct_abs_sequence(self, reason: str = "") -> None:
        was_active = (
            self.direct_abs_pending_targets is not None
            or self.direct_abs_apply_after_id is not None
            or self.direct_abs_verify_after_id is not None
        )
        for attr in ("direct_abs_apply_after_id", "direct_abs_verify_after_id"):
            after_id = getattr(self, attr)
            if after_id is not None:
                try:
                    self.after_cancel(after_id)
                except tk.TclError:
                    pass
                setattr(self, attr, None)
        self.direct_abs_sequence += 1
        self.direct_abs_pending_targets = None
        self.direct_abs_initial_abs = None
        self.direct_abs_started_monotonic = 0.0
        self.direct_abs_ack_received = False
        if was_active and reason:
            self.direct_abs_status_var.set(f"五路 ABS: 已取消（{reason}）")

    def _apply_direct_abs_after_start(self, sequence: int) -> None:
        self.direct_abs_apply_after_id = None
        if sequence != self.direct_abs_sequence or self.direct_abs_pending_targets is None:
            return
        if self.poller is None or self.latest_snapshot is None:
            self._cancel_direct_abs_sequence("串口或反馈已断开")
            return

        targets = [int(value) for value in self.latest_snapshot.servo_abs]
        targets[:DIRECT_ABS_UI_MOTOR_COUNT] = self.direct_abs_pending_targets
        self.command_queue.put(("text", "direct"))
        self.command_queue.put(("bytes", build_upper_motor_pos_abs_cmd(targets)))
        self.command_queue.put(("text", "status"))
        self.command_queue.put(("text", "servo"))
        self.direct_abs_status_var.set("五路 ABS: START 后目标已再次下发，等待下位机确认")
        self.direct_abs_verify_after_id = self.after(
            DIRECT_ABS_VERIFY_INTERVAL_MS,
            lambda token=sequence: self._verify_direct_abs_motion(token),
        )

    def _verify_direct_abs_motion(self, sequence: int) -> None:
        self.direct_abs_verify_after_id = None
        targets = self.direct_abs_pending_targets
        snapshot = self.latest_snapshot
        if sequence != self.direct_abs_sequence or targets is None:
            return
        if self.poller is None or snapshot is None:
            self._cancel_direct_abs_sequence("串口或反馈已断开")
            return

        actual = [int(value) for value in snapshot.servo_abs[:DIRECT_ABS_UI_MOTOR_COUNT]]
        errors = [target - value for target, value in zip(targets, actual)]
        max_error = max(abs(error) for error in errors)
        if max_error <= DIRECT_ABS_TARGET_TOLERANCE_COUNTS:
            target_text = ", ".join(
                f"M{motor:02d}={value}" for motor, value in enumerate(actual)
            )
            self.direct_abs_status_var.set(f"五路 ABS: 已到位（±{max_error} counts）；{target_text}")
            self.direct_abs_pending_targets = None
            self.direct_abs_initial_abs = None
            return

        elapsed = time.monotonic() - self.direct_abs_started_monotonic
        initial = self.direct_abs_initial_abs or actual
        moved = max(abs(value - start) for value, start in zip(actual, initial))
        moving = moved >= DIRECT_ABS_MOTION_THRESHOLD_COUNTS or any(
            abs(int(speed)) > 0 for speed in snapshot.servo_speed[:DIRECT_ABS_UI_MOTOR_COUNT]
        )
        ack_text = "已确认" if self.direct_abs_ack_received else "未收到D3 ACK"
        if elapsed >= DIRECT_ABS_VERIFY_TIMEOUT_S:
            details = ", ".join(
                f"M{motor:02d} {actual[motor]}->{targets[motor]} (e={errors[motor]:+d})"
                for motor in range(DIRECT_ABS_UI_MOTOR_COUNT)
            )
            self.direct_abs_status_var.set(
                f"五路 ABS: 超时未到位，{ack_text}，最大误差 {max_error} counts；{details}"
            )
            self.direct_abs_pending_targets = None
            self.direct_abs_initial_abs = None
            messagebox.showwarning(
                "五路 ABS 未到位",
                "8 秒内未到达目标。\n"
                f"下位机目标帧：{ack_text}\n"
                f"反馈：{details}\n\n"
                "请检查下方五路 ABS 状态和 RX STATUS；若始终未收到 D3 ACK，需更新下位机固件。",
                parent=self,
            )
            return

        phase = "运动中" if moving else "等待电机响应"
        self.direct_abs_status_var.set(
            f"五路 ABS: {phase}，{ack_text}，最大误差 {max_error} counts，已移动 {moved} counts"
        )
        self.direct_abs_verify_after_id = self.after(
            DIRECT_ABS_VERIFY_INTERVAL_MS,
            lambda token=sequence: self._verify_direct_abs_motion(token),
        )

    def _set_direct_abs_target_entries(self, targets: list[int]) -> None:
        for motor, target_var in enumerate(self.direct_abs_target_vars):
            if motor < len(targets):
                target_var.set(str(int(targets[motor])))

    def _load_current_direct_abs_targets(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        snapshot = self.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待数据", "请等待第一帧 Servo ABS 数据。", parent=self)
            return
        offline = [
            motor
            for motor in range(DIRECT_ABS_UI_MOTOR_COUNT)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            messagebox.showwarning(
                "电机未在线",
                "无法读取当前 ABS: " + ", ".join(f"M{motor:02d}" for motor in offline),
                parent=self,
            )
            return
        self._set_direct_abs_target_entries(snapshot.servo_abs[:DIRECT_ABS_UI_MOTOR_COUNT])
        self.direct_abs_targets_initialized = True
        self.last_line_var.set("已将 M00-M04 当前 Servo ABS 填入五电机目标。")

    def _parse_direct_abs_targets(self) -> list[int]:
        targets: list[int] = []
        for motor, target_var in enumerate(self.direct_abs_target_vars):
            text = target_var.get().strip()
            if not text:
                raise ValueError(f"M{motor:02d} ABS 目标不能为空。")
            try:
                target = int(text, 10)
            except ValueError as exc:
                raise ValueError(f"M{motor:02d} ABS 目标必须是整数。") from exc
            if not FORCE_TARGET_MIN_ABS <= target <= FORCE_TARGET_MAX_ABS:
                raise ValueError(
                    f"M{motor:02d} ABS 目标超出范围 "
                    f"{FORCE_TARGET_MIN_ABS}~{FORCE_TARGET_MAX_ABS}。"
                )
            targets.append(target)
        return targets

    def _send_direct_abs_targets(self) -> None:
        if self.step_test_running or self.sine_test_running:
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试。", parent=self)
            return
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        snapshot = self.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待数据", "请等待第一帧 Servo ABS 数据。", parent=self)
            return
        try:
            first_five_targets = self._parse_direct_abs_targets()
        except ValueError as exc:
            messagebox.showwarning("ABS 目标错误", str(exc), parent=self)
            return

        offline = [
            motor
            for motor in range(DIRECT_ABS_UI_MOTOR_COUNT)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            messagebox.showwarning(
                "电机未在线",
                "五电机 ABS 控制需要 M00-M04 全部在线，当前未在线: "
                + ", ".join(f"M{motor:02d}" for motor in offline),
                parent=self,
            )
            return

        host = self.master
        if getattr(host, "linearity_running", False):
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再发送五电机 ABS。", parent=self)
            return
        if getattr(host, "multi_input_running", False):
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再发送五电机 ABS。", parent=self)
            return
        if getattr(host, "training_running", False):
            messagebox.showinfo("训练采集中", "请先停止训练采集，再发送五电机 ABS。", parent=self)
            return

        deltas = [
            target - int(snapshot.servo_abs[motor])
            for motor, target in enumerate(first_five_targets)
        ]
        if max(abs(delta) for delta in deltas) >= DIRECT_ABS_CONFIRM_DELTA_COUNTS:
            summary = "\n".join(
                f"M{motor:02d}: {snapshot.servo_abs[motor]} -> {target}  (Δ{delta:+d})"
                for motor, (target, delta) in enumerate(zip(first_five_targets, deltas))
            )
            if not messagebox.askyesno(
                "确认大幅移动",
                "至少一路目标变化达到一圈（4096 counts），确认继续吗？\n\n" + summary,
                parent=self,
            ):
                return

        if getattr(host, "force_target_running", False):
            host._stop_force_target_control("manual five-motor ABS requested")
        if getattr(host, "safe_relax_active", False):
            host._stop_safe_relax("manual five-motor ABS requested", send_stop=False)
        if getattr(host, "degree_force_active", False):
            host._stop_degree_force_control("manual five-motor ABS requested", send_stop=False)

        targets = [int(value) for value in snapshot.servo_abs]
        targets[:DIRECT_ABS_UI_MOTOR_COUNT] = first_five_targets
        self._cancel_direct_abs_sequence()
        sequence = self.direct_abs_sequence
        self.direct_abs_pending_targets = list(first_five_targets)
        self.direct_abs_initial_abs = [
            int(value) for value in snapshot.servo_abs[:DIRECT_ABS_UI_MOTOR_COUNT]
        ]
        self.direct_abs_started_monotonic = time.monotonic()
        self.direct_abs_ack_received = False
        self.clear_pending_commands()
        self.sent_direct_targets.clear()
        self.sent_direct_targets.update(enumerate(first_five_targets))
        self.mode_var.set("direct")
        self.command_queue.put(("text", "tension off"))
        self.command_queue.put(("text", "direct"))
        self.command_queue.put(("bytes", build_upper_motor_pos_abs_cmd(targets)))
        self.command_queue.put(("text", "start"))
        target_text = ", ".join(
            f"M{motor:02d}={target}" for motor, target in enumerate(first_five_targets)
        )
        self.sent_last_target_command = f"direct ABS batch [{target_text}]"
        self.sent_target_var.set(self._sent_target_status_text())
        self.direct_abs_status_var.set(
            f"五路 ABS: 目标已预写，等待 START 生效（{DIRECT_ABS_START_DELAY_MS} ms）"
        )
        self.last_line_var.set(f"五电机 ABS 正在启动: {target_text}")
        self.direct_abs_apply_after_id = self.after(
            DIRECT_ABS_START_DELAY_MS,
            lambda token=sequence: self._apply_direct_abs_after_start(token),
        )

    def _build_j00_j03_angle_targets(self, target_values: list[float]) -> list[float]:
        targets = self._current_host_degree_targets()
        for joint, target_value in zip(ENCODER_PLOT_JOINTS, target_values):
            targets[joint] = float(target_value)
        return targets

    def _set_feedback_record_button_state(self, running: bool) -> None:
        if hasattr(self, "feedback_record_button"):
            self.feedback_record_button.configure(
                text="停止反馈连续记录" if running else "开始反馈连续记录"
            )

    def _toggle_feedback_continuous_record(self) -> None:
        if self.step_test_running or self.sine_test_running:
            messagebox.showinfo(
                "自动角度测试运行中",
                "原始连续记录由自动测试管理；请使用对应的停止按钮。",
                parent=self,
            )
            return
        host = self.master
        toggle = getattr(host, "_toggle_continuous_record", None)
        if toggle is None:
            messagebox.showerror("连续记录不可用", "当前上位机没有连续记录功能。", parent=self)
            return
        toggle()
        self._set_feedback_record_button_state(
            bool(getattr(host, "continuous_record_running", False))
        )

    def _parse_step_test_settings(
        self,
    ) -> tuple[
        list[tuple[float, float, float, float]],
        list[float],
        int,
        float,
        float,
        float,
        float,
    ]:
        poses = parse_step_test_poses(self.step_test_poses_var.get())
        amplitudes = parse_step_test_amplitudes(self.step_test_amplitudes_var.get())
        try:
            repeats = int(self.step_test_repeats_var.get())
            baseline_s = float(self.step_test_baseline_var.get().strip())
            band_deg = float(self.step_test_band_var.get().strip())
            hold_s = float(self.step_test_hold_var.get().strip())
            timeout_s = float(self.step_test_timeout_var.get().strip())
        except (tk.TclError, ValueError) as exc:
            raise ValueError("重复、基线、稳定带、连续稳定时间和超时必须是数字。") from exc
        if repeats <= 0:
            raise ValueError("重复次数必须大于0。")
        if baseline_s <= 0.0:
            raise ValueError("基线记录时间必须大于0秒。")
        if band_deg <= 0.0:
            raise ValueError("稳定角度带必须大于0°。")
        if hold_s <= 0.0:
            raise ValueError("连续稳定时间必须大于0秒。")
        if timeout_s <= hold_s:
            raise ValueError("单阶跃超时必须大于连续稳定时间。")
        validate_step_test_targets(poses, amplitudes)
        return poses, amplitudes, repeats, baseline_s, band_deg, hold_s, timeout_s

    def _set_step_test_controls_running(self, running: bool) -> None:
        state = "disabled" if running else "normal"
        for attr in (
            "step_test_poses_entry",
            "step_test_amplitudes_entry",
            "step_test_repeats_spin",
            "step_test_baseline_entry",
            "step_test_band_entry",
            "step_test_hold_entry",
            "step_test_timeout_entry",
        ):
            widget = getattr(self, attr, None)
            if widget is not None:
                widget.configure(state=state)
        if hasattr(self, "step_test_start_button"):
            self.step_test_start_button.configure(state="disabled" if running else "normal")
        if hasattr(self, "step_test_stop_button"):
            self.step_test_stop_button.configure(state="normal" if running else "disabled")
        if hasattr(self, "sine_test_start_button"):
            self.sine_test_start_button.configure(state="disabled" if running else "normal")

    def _step_test_current_angles(
        self, snapshot: EncoderServoSnapshot
    ) -> Optional[tuple[float, float, float, float]]:
        if not all(snapshot.encoder_valid[joint] for joint in MANUAL_RECORD_JOINTS):
            return None
        return tuple(float(snapshot.encoder_deg[joint]) for joint in MANUAL_RECORD_JOINTS)

    def _step_test_force_vector(self) -> tuple[float, float, float, float, float]:
        host = self.master
        channel_by_motor = {motor: channel for channel, motor in FORCE_CHANNEL_TO_MOTOR.items()}
        values: list[float] = []
        for motor in training_target_motor_indices():
            channel = channel_by_motor[motor]
            value = host._force_target_current_n(channel)
            values.append(float(value) if value is not None else float("nan"))
        return tuple(values)  # type: ignore[return-value]

    def _step_test_record_event(self, text: str) -> None:
        host = self.master
        recorder = getattr(host, "_record_continuous_command_event", None)
        if recorder is not None:
            recorder(f"angle step_test {text}")

    def _queue_step_test_target(
        self,
        targets: tuple[float, float, float, float],
        label: str,
    ) -> bool:
        if self.poller is None:
            return False
        if targets[1] <= STEP_TEST_J01_MIN_DEG:
            self._stop_step_test(
                f"拒绝目标：J01={targets[1]:.3f}°，必须严格大于0°",
                send_stop=True,
            )
            return False
        for joint, target in enumerate(targets):
            self.sent_degree_targets[joint] = float(target)
        self.mode_var.set("degree")
        self.target_value_var.set(";".join(f"{target:g}" for target in targets))
        self._queue_start_with_host_state()
        self.sent_last_target_command = f"angle step_test {label}"
        self.sent_target_var.set(self._sent_target_status_text())
        self._step_test_record_event(label)
        return True

    def _start_step_test(self) -> None:
        if self.step_test_running:
            return
        if self.sine_test_running:
            messagebox.showinfo("自动正弦运行中", "请先停止自动正弦测试。", parent=self)
            return
        host = self.master
        if self.poller is None or self.latest_snapshot is None:
            messagebox.showinfo("等待数据", "请先连接Encoder / Servo并等待有效角度数据。", parent=self)
            return
        if getattr(host, "poller", None) is None:
            messagebox.showinfo("力传感器未连接", "自动阶跃必须同时记录并保护五路张力。", parent=self)
            return
        conflicts = (
            ("linearity_running", "局部线性辨识"),
            ("multi_input_running", "多输入联合辨识"),
            ("training_running", "训练采集"),
            ("force_target_running", "力目标控制"),
            ("safe_relax_active", "安全放松"),
            ("force_emergency_relax_active", "全电机紧急放松"),
        )
        for attr, label in conflicts:
            if bool(getattr(host, attr, False)):
                messagebox.showinfo("当前不能开始", f"请先停止{label}。", parent=self)
                return
        if bool(getattr(host, "continuous_record_running", False)):
            messagebox.showinfo(
                "连续记录已在运行",
                "请先停止当前连续记录；自动阶跃会为本次实验创建独立CSV。",
                parent=self,
            )
            return
        try:
            (
                poses,
                amplitudes,
                repeats,
                baseline_s,
                band_deg,
                hold_s,
                timeout_s,
            ) = self._parse_step_test_settings()
        except ValueError as exc:
            messagebox.showwarning("自动阶跃参数错误", str(exc), parent=self)
            return
        snapshot = self.latest_snapshot
        angles = self._step_test_current_angles(snapshot)
        if angles is None:
            messagebox.showwarning("角度无效", "J00-J03必须全部有效才能开始自动阶跃。", parent=self)
            return
        if angles[1] <= STEP_TEST_J01_MIN_DEG:
            messagebox.showwarning(
                "J01不满足约束",
                f"当前J01={angles[1]:.3f}°，必须严格大于0°。",
                parent=self,
            )
            return
        snapshot_ok, issues = host._record_snapshot_issues()
        if snapshot_ok is None:
            messagebox.showinfo("等待完整数据", ", ".join(issues), parent=self)
            return
        trials = build_step_test_trials(poses, amplitudes, repeats)
        estimated_minutes = len(trials) * (baseline_s + timeout_s + hold_s) / 60.0
        if not messagebox.askyesno(
            "确认自动阶跃实验",
            f"将执行{len(trials)}个阶跃（{len(poses)}姿态，J01目标及实测均须>0°）。\n"
            f"按超时上限估算约{estimated_minutes:.0f}分钟。\n\n"
            "实验会自动控制关节并创建独立原始CSV和汇总CSV，确认开始吗？",
            parent=self,
        ):
            return

        host.continuous_record_interval_var.set(CONTINUOUS_RECORD_DEFAULT_INTERVAL_MS)
        host._start_continuous_record()
        if not bool(getattr(host, "continuous_record_running", False)):
            return
        self.step_test_started_continuous_record = True
        summary_file: Optional[object] = None
        try:
            directory = Path.cwd() / "run_data" / "step_tests"
            directory.mkdir(parents=True, exist_ok=True)
            now = time.time()
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
            experiment_id = f"step_test_{stamp}_{int((now % 1.0) * 1_000_000):06d}"
            summary_path = directory / f"{experiment_id}_summary.csv"
            summary_file = open(summary_path, "x", newline="", encoding="utf-8-sig")
            summary_writer = csv.DictWriter(
                summary_file,
                fieldnames=step_test_summary_fieldnames(),
                extrasaction="ignore",
                restval="",
            )
            summary_writer.writeheader()
            summary_file.flush()
        except Exception as exc:
            if summary_file is not None:
                summary_file.close()
            host._stop_continuous_record("自动阶跃汇总文件创建失败")
            self.step_test_started_continuous_record = False
            messagebox.showerror("自动阶跃启动失败", f"无法创建汇总CSV: {exc}", parent=self)
            return

        self.step_test_poses = poses
        self.step_test_trials = trials
        self.step_test_trial_index = 0
        self.step_test_baseline_s = baseline_s
        self.step_test_band_deg = band_deg
        self.step_test_hold_s = hold_s
        self.step_test_timeout_s = timeout_s
        self.step_test_summary_file = summary_file
        self.step_test_summary_writer = summary_writer
        self.step_test_summary_path = summary_path
        self.step_test_summary_path_var.set(str(summary_path))
        self.step_test_experiment_id = experiment_id
        self.step_test_running = True
        self._set_step_test_controls_running(True)
        self._step_test_record_event(
            f"begin id={experiment_id} trials={len(trials)} j01_gt_0=1"
        )
        self._step_test_begin_trial()

    def _step_test_begin_trial(self) -> None:
        if not self.step_test_running:
            return
        if self.step_test_trial_index >= len(self.step_test_trials):
            self._finish_step_test()
            return
        pose_index, joint, signed_step, repeat = self.step_test_trials[self.step_test_trial_index]
        pose = self.step_test_poses[pose_index]
        self.step_test_baseline_samples = []
        self.step_test_response_samples = []
        self.step_test_last_snapshot_ts = 0.0
        self.step_test_stable_since_monotonic = 0.0
        self.step_test_settle_candidate_monotonic = 0.0
        self.step_test_phase = "move_pose"
        self.step_test_phase_started_monotonic = time.monotonic()
        self.step_test_current_target = pose
        label = (
            f"move_pose pose={pose_index + 1} repeat={repeat} "
            f"joint=J{joint:02d} step={signed_step:+g}"
        )
        if not self._queue_step_test_target(pose, label):
            return
        self.step_test_status_var.set(
            f"自动阶跃 {self.step_test_trial_index + 1}/{len(self.step_test_trials)}: "
            f"到S{pose_index + 1:02d}，准备J{joint:02d}{signed_step:+g}°"
        )
        self._schedule_step_test_tick()

    def _schedule_step_test_tick(self) -> None:
        if self.step_test_running and self.step_test_after_id is None:
            self.step_test_after_id = self.after(STEP_TEST_TICK_MS, self._step_test_tick)

    def _step_test_enter_baseline(self) -> None:
        self.step_test_phase = "baseline"
        self.step_test_phase_started_monotonic = time.monotonic()
        self.step_test_baseline_samples = []
        self.step_test_last_snapshot_ts = 0.0
        self._step_test_record_event(
            f"baseline trial={self.step_test_trial_index + 1} duration={self.step_test_baseline_s:g}"
        )
        self.step_test_status_var.set(
            f"自动阶跃 {self.step_test_trial_index + 1}/{len(self.step_test_trials)}: "
            f"记录{self.step_test_baseline_s:g}s初始基线"
        )

    def _step_test_enter_response(self) -> None:
        pose_index, joint, signed_step, repeat = self.step_test_trials[self.step_test_trial_index]
        pose = self.step_test_poses[pose_index]
        target_list = list(pose)
        target_list[joint] += signed_step
        target = tuple(target_list)
        self.step_test_phase = "response"
        self.step_test_phase_started_monotonic = time.monotonic()
        self.step_test_response_started_monotonic = self.step_test_phase_started_monotonic
        self.step_test_settle_candidate_monotonic = 0.0
        self.step_test_response_samples = []
        self.step_test_last_snapshot_ts = 0.0
        self.step_test_current_target = target
        label = (
            f"step trial={self.step_test_trial_index + 1} pose={pose_index + 1} "
            f"repeat={repeat} joint=J{joint:02d} delta={signed_step:+g} "
            + "target=" + ";".join(f"{value:g}" for value in target)
        )
        self._queue_step_test_target(target, label)
        self.step_test_status_var.set(
            f"自动阶跃 {self.step_test_trial_index + 1}/{len(self.step_test_trials)}: "
            f"J{joint:02d}{signed_step:+g}°响应中，等待±{self.step_test_band_deg:g}°"
        )

    def _step_test_enter_return(self) -> None:
        pose_index, joint, signed_step, repeat = self.step_test_trials[self.step_test_trial_index]
        pose = self.step_test_poses[pose_index]
        self.step_test_phase = "return_pose"
        self.step_test_phase_started_monotonic = time.monotonic()
        self.step_test_stable_since_monotonic = 0.0
        self.step_test_current_target = pose
        self._queue_step_test_target(
            pose,
            f"return trial={self.step_test_trial_index + 1} pose={pose_index + 1} "
            f"joint=J{joint:02d} step={signed_step:+g} repeat={repeat}",
        )
        self.step_test_status_var.set(
            f"自动阶跃 {self.step_test_trial_index + 1}/{len(self.step_test_trials)}: 返回S{pose_index + 1:02d}"
        )

    @staticmethod
    def _step_test_settling_time(
        samples: list[tuple[float, tuple[float, float, float, float], tuple[float, float, float, float, float]]],
        targets: tuple[float, float, float, float],
        band_deg: float,
        joints: tuple[int, ...],
    ) -> Optional[float]:
        if not samples:
            return None
        outside_indices = [
            index
            for index, (_elapsed, angles, _forces) in enumerate(samples)
            if any(abs(angles[joint] - targets[joint]) > band_deg for joint in joints)
        ]
        index = outside_indices[-1] + 1 if outside_indices else 0
        if index >= len(samples):
            return None
        return float(samples[index][0])

    def _write_step_test_summary(self, completion: str) -> None:
        writer = self.step_test_summary_writer
        fp = self.step_test_summary_file
        if writer is None or fp is None or not self.step_test_response_samples:
            return
        pose_index, joint, signed_step, repeat = self.step_test_trials[self.step_test_trial_index]
        pose = self.step_test_poses[pose_index]
        target_list = list(pose)
        target_list[joint] += signed_step
        targets = tuple(target_list)
        baseline_angles = [sample[1] for sample in self.step_test_baseline_samples]
        if baseline_angles:
            initial_by_joint = tuple(
                float(statistics.median(values)) for values in zip(*baseline_angles)
            )
        else:
            initial_by_joint = pose
        samples = self.step_test_response_samples
        elapsed_values = [sample[0] for sample in samples]
        commanded_values = [sample[1][joint] for sample in samples]
        final_start = max(0.0, elapsed_values[-1] - min(2.0, elapsed_values[-1]))
        final_samples = [sample for sample in samples if sample[0] >= final_start] or samples[-1:]
        final_values = [sample[1][joint] for sample in final_samples]
        final_mean = statistics.fmean(final_values)
        final_std = statistics.pstdev(final_values) if len(final_values) > 1 else 0.0
        initial = initial_by_joint[joint]
        target = targets[joint]
        movement = target - initial
        direction = 1.0 if movement >= 0.0 else -1.0
        peak = max(commanded_values) if direction > 0.0 else min(commanded_values)
        overshoot = max(0.0, direction * (peak - target))
        overshoot_percent = 100.0 * overshoot / abs(movement) if abs(movement) > 1e-9 else 0.0
        progress = [direction * (value - initial) for value in commanded_values]

        def first_cross(fraction: float) -> Optional[float]:
            threshold = fraction * abs(movement)
            for elapsed, value in zip(elapsed_values, progress):
                if value >= threshold:
                    return float(elapsed)
            return None

        t10 = first_cross(0.1)
        t90 = first_cross(0.9)
        rise_time = t90 - t10 if t10 is not None and t90 is not None else None
        settle_commanded = self._step_test_settling_time(
            samples, targets, self.step_test_band_deg, (joint,)
        )
        settle_all = self._step_test_settling_time(
            samples, targets, self.step_test_band_deg, tuple(MANUAL_RECORD_JOINTS)
        )
        cross_coupling = max(
            (
                abs(sample[1][other] - initial_by_joint[other])
                for sample in samples
                for other in MANUAL_RECORD_JOINTS
                if other != joint
            ),
            default=0.0,
        )
        row: dict[str, object] = {
            "experiment_id": self.step_test_experiment_id,
            "raw_csv": str(getattr(self.master, "continuous_record_path", "") or ""),
            "pose_index": pose_index + 1,
            **{f"pose_j{index:02d}_deg": f"{value:.6f}" for index, value in enumerate(pose)},
            "kp": self.angle_kp_scale_var.get().strip(),
            "ki_per_s": self.angle_ki_scale_var.get().strip(),
            "settle_band_deg": f"{self.step_test_band_deg:.6f}",
            "settle_hold_s": f"{self.step_test_hold_s:.6f}",
            "repeat": repeat,
            "commanded_joint": f"J{joint:02d}",
            "step_deg": f"{signed_step:.6f}",
            "target_deg": f"{target:.6f}",
            "completion": completion,
            "response_duration_s": f"{elapsed_values[-1]:.6f}",
            "initial_deg": f"{initial:.6f}",
            "final_mean_deg": f"{final_mean:.6f}",
            "final_std_deg": f"{final_std:.6f}",
            "steady_error_deg": f"{final_mean - target:.6f}",
            "minimum_deg": f"{min(commanded_values):.6f}",
            "maximum_deg": f"{max(commanded_values):.6f}",
            "overshoot_deg": f"{overshoot:.6f}",
            "overshoot_percent": f"{overshoot_percent:.6f}",
            "rise_time_10_90_s": "" if rise_time is None else f"{rise_time:.6f}",
            "settling_time_commanded_s": "" if settle_commanded is None else f"{settle_commanded:.6f}",
            "settling_time_all_joints_s": "" if settle_all is None else f"{settle_all:.6f}",
            "max_cross_coupling_deg": f"{cross_coupling:.6f}",
            "minimum_j01_deg": f"{min(sample[1][1] for sample in samples):.6f}",
        }
        for other in MANUAL_RECORD_JOINTS:
            all_values = [sample[1][other] for sample in samples]
            final_joint_values = [sample[1][other] for sample in final_samples]
            row[f"initial_j{other:02d}_deg"] = f"{initial_by_joint[other]:.6f}"
            row[f"final_mean_j{other:02d}_deg"] = f"{statistics.fmean(final_joint_values):.6f}"
            row[f"minimum_j{other:02d}_deg"] = f"{min(all_values):.6f}"
            row[f"maximum_j{other:02d}_deg"] = f"{max(all_values):.6f}"
        for motor in training_target_motor_indices():
            finite_forces = [
                sample[2][motor] for sample in samples if math.isfinite(sample[2][motor])
            ]
            row[f"minimum_force_m{motor:02d}_n"] = (
                f"{min(finite_forces):.6f}" if finite_forces else ""
            )
        writer.writerow(row)
        fp.flush()

    def _step_test_complete_response(self, completion: str) -> None:
        self._write_step_test_summary(completion)
        self._step_test_record_event(
            f"response_end trial={self.step_test_trial_index + 1} completion={completion}"
        )
        self._step_test_enter_return()

    def _step_test_tick(self) -> None:
        self.step_test_after_id = None
        if not self.step_test_running:
            return
        host = self.master
        if self.poller is None or self.latest_snapshot is None:
            self._stop_step_test("Encoder / Servo连接或数据丢失", send_stop=True)
            return
        if not bool(getattr(host, "continuous_record_running", False)):
            self._stop_step_test("原始连续记录意外停止", send_stop=True)
            return
        if bool(getattr(host, "force_emergency_relax_active", False)):
            self._stop_step_test("已触发全电机张力保护", send_stop=False)
            return
        snapshot = self.latest_snapshot
        angles = self._step_test_current_angles(snapshot)
        if angles is None:
            self._stop_step_test("J00-J03出现无效角度，无法验证J01>0°", send_stop=True)
            return
        if angles[1] <= STEP_TEST_J01_MIN_DEG:
            self._stop_step_test(
                f"实测J01={angles[1]:.3f}°≤0°，违反实验约束",
                send_stop=True,
            )
            return
        now = time.monotonic()
        target = self.step_test_current_target
        if target is None:
            self._stop_step_test("内部目标状态缺失", send_stop=True)
            return
        if snapshot.timestamp > self.step_test_last_snapshot_ts:
            self.step_test_last_snapshot_ts = snapshot.timestamp
            if self.step_test_phase == "baseline":
                self.step_test_baseline_samples.append((now, angles))
            elif self.step_test_phase == "response":
                self.step_test_response_samples.append(
                    (now - self.step_test_response_started_monotonic, angles, self._step_test_force_vector())
                )

        errors = [abs(angles[joint] - target[joint]) for joint in MANUAL_RECORD_JOINTS]
        all_close = max(errors) <= self.step_test_band_deg
        phase_elapsed = now - self.step_test_phase_started_monotonic
        if self.step_test_phase in ("move_pose", "return_pose"):
            if all_close:
                if self.step_test_stable_since_monotonic <= 0.0:
                    self.step_test_stable_since_monotonic = now
                elif now - self.step_test_stable_since_monotonic >= self.step_test_hold_s:
                    if self.step_test_phase == "move_pose":
                        self._step_test_enter_baseline()
                        self._schedule_step_test_tick()
                        return
                    else:
                        self.step_test_trial_index += 1
                        self._step_test_begin_trial()
                        return
            else:
                self.step_test_stable_since_monotonic = 0.0
            if phase_elapsed >= self.step_test_timeout_s:
                self._stop_step_test(
                    f"{self.step_test_phase}阶段{self.step_test_timeout_s:g}s未到位",
                    send_stop=True,
                )
                return
        elif self.step_test_phase == "baseline":
            if not all_close:
                self.step_test_phase = "move_pose"
                self.step_test_phase_started_monotonic = now
                self.step_test_stable_since_monotonic = 0.0
                self.step_test_baseline_samples = []
                self.step_test_status_var.set("基线期间离开稳定带，重新等待初始姿态稳定")
                self._schedule_step_test_tick()
                return
            if phase_elapsed >= self.step_test_baseline_s:
                self._step_test_enter_response()
        elif self.step_test_phase == "response":
            if all_close:
                if self.step_test_settle_candidate_monotonic <= 0.0:
                    self.step_test_settle_candidate_monotonic = now
                elif now - self.step_test_settle_candidate_monotonic >= self.step_test_hold_s:
                    self._step_test_complete_response("settled")
                    self._schedule_step_test_tick()
                    return
            else:
                self.step_test_settle_candidate_monotonic = 0.0
            if phase_elapsed >= self.step_test_timeout_s:
                self._step_test_complete_response("timeout")
                self._schedule_step_test_tick()
                return
        if self.step_test_running:
            self._schedule_step_test_tick()

    def _stop_step_test(self, reason: str, *, send_stop: bool) -> None:
        was_running = self.step_test_running
        if was_running and self.step_test_phase == "response" and self.step_test_response_samples:
            self._write_step_test_summary("aborted")
        self.step_test_running = False
        if self.step_test_after_id is not None:
            try:
                self.after_cancel(self.step_test_after_id)
            except tk.TclError:
                pass
            self.step_test_after_id = None
        self.step_test_phase = "idle"
        self.step_test_current_target = None
        self._set_step_test_controls_running(False)
        fp = self.step_test_summary_file
        self.step_test_summary_file = None
        self.step_test_summary_writer = None
        if fp is not None:
            try:
                fp.close()
            except Exception:
                pass
        host = self.master
        if self.step_test_started_continuous_record and bool(
            getattr(host, "continuous_record_running", False)
        ):
            host._stop_continuous_record(f"自动阶跃停止: {reason}")
        self.step_test_started_continuous_record = False
        if send_stop and self.poller is not None:
            self.queue_emergency_stop()
        if was_running:
            self.step_test_status_var.set(
                f"自动阶跃已停止: {reason}; 完成{self.step_test_trial_index}/{len(self.step_test_trials)}"
            )

    def _finish_step_test(self) -> None:
        if not self.step_test_running:
            return
        total = len(self.step_test_trials)
        summary_path = self.step_test_summary_path
        self._step_test_record_event(f"complete trials={total}")
        self._stop_step_test("全部完成", send_stop=False)
        self.step_test_status_var.set(f"自动阶跃完成: {total}/{total}；控制器保持最后初始姿态")
        if summary_path is not None:
            self.last_line_var.set(f"自动阶跃汇总: {summary_path}")

    def _parse_sine_test_settings(
        self,
    ) -> tuple[
        list[tuple[float, float, float, float]],
        list[float],
        list[float],
        int,
        int,
        float,
        float,
        float,
        float,
    ]:
        poses = parse_sine_test_poses(self.sine_test_poses_var.get())
        amplitudes = parse_step_test_amplitudes(self.sine_test_amplitudes_var.get())
        frequencies_hz = parse_sine_test_frequencies(self.sine_test_frequencies_var.get())
        try:
            cycles = int(self.sine_test_cycles_var.get())
            repeats = int(self.sine_test_repeats_var.get())
            baseline_s = float(self.sine_test_baseline_var.get().strip())
            band_deg = float(self.sine_test_band_var.get().strip())
            hold_s = float(self.sine_test_hold_var.get().strip())
            timeout_s = float(self.sine_test_timeout_var.get().strip())
        except (tk.TclError, ValueError) as exc:
            raise ValueError("周期、重复、基线、到位稳定带、保持时间和姿态超时必须是数字。") from exc
        if cycles <= 0 or repeats <= 0:
            raise ValueError("周期数和重复次数必须大于0。")
        if baseline_s <= 0.0 or band_deg <= 0.0 or hold_s <= 0.0:
            raise ValueError("基线、到位稳定带和保持时间必须大于0。")
        if timeout_s <= hold_s:
            raise ValueError("姿态超时必须大于到位保持时间。")
        validate_sine_test_targets(poses, amplitudes)
        return (
            poses,
            amplitudes,
            frequencies_hz,
            cycles,
            repeats,
            baseline_s,
            band_deg,
            hold_s,
            timeout_s,
        )

    def _set_sine_test_controls_running(self, running: bool) -> None:
        state = "disabled" if running else "normal"
        for attr in (
            "sine_test_poses_entry",
            "sine_test_amplitudes_entry",
            "sine_test_frequencies_entry",
            "sine_test_cycles_spin",
            "sine_test_repeats_spin",
            "sine_test_baseline_entry",
            "sine_test_band_entry",
            "sine_test_hold_entry",
            "sine_test_timeout_entry",
        ):
            widget = getattr(self, attr, None)
            if widget is not None:
                widget.configure(state=state)
        if hasattr(self, "sine_test_start_button"):
            self.sine_test_start_button.configure(state="disabled" if running else "normal")
        if hasattr(self, "sine_test_stop_button"):
            self.sine_test_stop_button.configure(state="normal" if running else "disabled")
        if hasattr(self, "step_test_start_button"):
            self.step_test_start_button.configure(state="disabled" if running else "normal")

    def _sine_test_record_event(self, text: str) -> None:
        recorder = getattr(self.master, "_record_continuous_command_event", None)
        if recorder is not None:
            recorder(f"angle sine_test {text}")

    def _queue_sine_test_pose_target(
        self, targets: tuple[float, float, float, float], label: str
    ) -> bool:
        if self.poller is None:
            return False
        if targets[1] <= STEP_TEST_J01_MIN_DEG:
            self._stop_sine_test(
                f"拒绝目标：J01={targets[1]:.3f}°，必须严格大于0°", send_stop=True
            )
            return False
        for joint, target in enumerate(targets):
            self.sent_degree_targets[joint] = float(target)
        self.mode_var.set("degree")
        self.target_value_var.set(";".join(f"{target:g}" for target in targets))
        self._queue_start_with_host_state()
        self.sent_last_target_command = f"angle sine_test {label}"
        self.sent_target_var.set(self._sent_target_status_text())
        self._sine_test_record_event(label)
        return True

    def _queue_sine_stream_target(
        self, targets: tuple[float, float, float, float]
    ) -> bool:
        if self.poller is None or targets[1] <= STEP_TEST_J01_MIN_DEG:
            return False
        for joint, target in enumerate(targets):
            self.sent_degree_targets[joint] = float(target)
        self.command_queue.put(("bytes", build_upper_angle_cmd(self._current_host_degree_targets())))
        self.sent_last_target_command = "angle sine_test streaming"
        self.sent_target_var.set(self._sent_target_status_text())
        return True

    def _start_sine_test(self) -> None:
        if self.sine_test_running:
            return
        if self.step_test_running:
            messagebox.showinfo("自动阶跃运行中", "请先停止自动阶跃。", parent=self)
            return
        host = self.master
        if self.poller is None or self.latest_snapshot is None:
            messagebox.showinfo("等待数据", "请先连接Encoder / Servo并等待有效角度数据。", parent=self)
            return
        if getattr(host, "poller", None) is None:
            messagebox.showinfo("力传感器未连接", "自动正弦必须同时记录并保护五路张力。", parent=self)
            return
        conflicts = (
            ("linearity_running", "局部线性辨识"),
            ("multi_input_running", "多输入联合辨识"),
            ("training_running", "训练采集"),
            ("force_target_running", "力目标控制"),
            ("safe_relax_active", "安全放松"),
            ("force_emergency_relax_active", "全电机紧急放松"),
        )
        for attr, label in conflicts:
            if bool(getattr(host, attr, False)):
                messagebox.showinfo("当前不能开始", f"请先停止{label}。", parent=self)
                return
        if bool(getattr(host, "continuous_record_running", False)):
            messagebox.showinfo(
                "连续记录已在运行",
                "请先停止当前连续记录；自动正弦会为本次实验创建独立CSV。",
                parent=self,
            )
            return
        try:
            (
                poses,
                amplitudes,
                frequencies_hz,
                cycles,
                repeats,
                baseline_s,
                band_deg,
                hold_s,
                timeout_s,
            ) = self._parse_sine_test_settings()
        except ValueError as exc:
            messagebox.showwarning("自动正弦参数错误", str(exc), parent=self)
            return
        angles = self._step_test_current_angles(self.latest_snapshot)
        if angles is None:
            messagebox.showwarning("角度无效", "J00-J03必须全部有效才能开始自动正弦。", parent=self)
            return
        if angles[1] <= STEP_TEST_J01_MIN_DEG:
            messagebox.showwarning(
                "J01不满足约束", f"当前J01={angles[1]:.3f}°，必须严格大于0°。", parent=self
            )
            return
        snapshot_ok, issues = host._record_snapshot_issues()
        if snapshot_ok is None:
            messagebox.showinfo("等待完整数据", ", ".join(issues), parent=self)
            return
        trials = build_sine_test_trials(poses, amplitudes, frequencies_hz, repeats)
        excitation_seconds = sum(cycles / trial[3] for trial in trials)
        estimated_minutes = (
            excitation_seconds + len(trials) * (baseline_s + 2.0 * hold_s)
        ) / 60.0
        if not messagebox.askyesno(
            "确认自动正弦实验",
            f"将执行{len(trials)}组正弦（{len(poses)}姿态，每次仅激励一个关节）。\n"
            f"J01目标和实测全程必须>0°，预计至少{estimated_minutes:.0f}分钟。\n\n"
            "实验会创建独立原始CSV与增益/相位/RMSE汇总CSV，确认开始吗？",
            parent=self,
        ):
            return
        host.continuous_record_interval_var.set(CONTINUOUS_RECORD_DEFAULT_INTERVAL_MS)
        host._start_continuous_record()
        if not bool(getattr(host, "continuous_record_running", False)):
            return
        self.sine_test_started_continuous_record = True
        summary_file: Optional[object] = None
        try:
            directory = Path.cwd() / "run_data" / "sine_tests"
            directory.mkdir(parents=True, exist_ok=True)
            now = time.time()
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
            experiment_id = f"sine_test_{stamp}_{int((now % 1.0) * 1_000_000):06d}"
            summary_path = directory / f"{experiment_id}_summary.csv"
            summary_file = open(summary_path, "x", newline="", encoding="utf-8-sig")
            summary_writer = csv.DictWriter(
                summary_file,
                fieldnames=sine_test_summary_fieldnames(),
                extrasaction="ignore",
                restval="",
            )
            summary_writer.writeheader()
            summary_file.flush()
        except Exception as exc:
            if summary_file is not None:
                summary_file.close()
            host._stop_continuous_record("自动正弦汇总文件创建失败")
            self.sine_test_started_continuous_record = False
            messagebox.showerror("自动正弦启动失败", f"无法创建汇总CSV: {exc}", parent=self)
            return
        self.sine_test_poses = poses
        self.sine_test_trials = trials
        self.sine_test_trial_index = 0
        self.sine_test_cycles = cycles
        self.sine_test_baseline_s = baseline_s
        self.sine_test_band_deg = band_deg
        self.sine_test_hold_s = hold_s
        self.sine_test_timeout_s = timeout_s
        self.sine_test_summary_file = summary_file
        self.sine_test_summary_writer = summary_writer
        self.sine_test_summary_path = summary_path
        self.sine_test_summary_path_var.set(str(summary_path))
        self.sine_test_experiment_id = experiment_id
        self.sine_test_running = True
        self._set_sine_test_controls_running(True)
        self._sine_test_record_event(
            f"begin id={experiment_id} trials={len(trials)} cycles={cycles} j01_gt_0=1"
        )
        self._sine_test_begin_trial()

    def _sine_test_begin_trial(self) -> None:
        if not self.sine_test_running:
            return
        if self.sine_test_trial_index >= len(self.sine_test_trials):
            self._finish_sine_test()
            return
        pose_index, joint, amplitude, frequency_hz, repeat = self.sine_test_trials[
            self.sine_test_trial_index
        ]
        pose = self.sine_test_poses[pose_index]
        self.sine_test_baseline_samples = []
        self.sine_test_response_samples = []
        self.sine_test_last_snapshot_ts = 0.0
        self.sine_test_stable_since_monotonic = 0.0
        self.sine_test_phase = "move_pose"
        self.sine_test_phase_started_monotonic = time.monotonic()
        self.sine_test_current_target = pose
        label = (
            f"move_pose trial={self.sine_test_trial_index + 1} pose={pose_index + 1} "
            f"joint={joint} amplitude={amplitude:g} frequency={frequency_hz:g} repeat={repeat}"
        )
        if not self._queue_sine_test_pose_target(pose, label):
            return
        self.sine_test_status_var.set(
            f"自动正弦 {self.sine_test_trial_index + 1}/{len(self.sine_test_trials)}: "
            f"到S{pose_index + 1:02d}，J{joint:02d} ±{amplitude:g}° @ {frequency_hz:g}Hz"
        )
        self._schedule_sine_test_tick()

    def _schedule_sine_test_tick(self) -> None:
        if self.sine_test_running and self.sine_test_after_id is None:
            self.sine_test_after_id = self.after(SINE_TEST_TICK_MS, self._sine_test_tick)

    def _sine_test_enter_baseline(self) -> None:
        self.sine_test_phase = "baseline"
        self.sine_test_phase_started_monotonic = time.monotonic()
        self.sine_test_baseline_samples = []
        self.sine_test_last_snapshot_ts = 0.0
        self._sine_test_record_event(
            f"baseline trial={self.sine_test_trial_index + 1} duration={self.sine_test_baseline_s:g}"
        )
        self.sine_test_status_var.set(
            f"自动正弦 {self.sine_test_trial_index + 1}/{len(self.sine_test_trials)}: "
            f"记录{self.sine_test_baseline_s:g}s基线"
        )

    def _sine_test_enter_response(self) -> None:
        pose_index, joint, amplitude, frequency_hz, repeat = self.sine_test_trials[
            self.sine_test_trial_index
        ]
        self.sine_test_phase = "response"
        self.sine_test_phase_started_monotonic = time.monotonic()
        self.sine_test_response_started_monotonic = self.sine_test_phase_started_monotonic
        self.sine_test_response_samples = []
        self.sine_test_last_snapshot_ts = 0.0
        self.sine_test_current_target = self.sine_test_poses[pose_index]
        self._sine_test_record_event(
            f"sine trial={self.sine_test_trial_index + 1} pose={pose_index + 1} joint={joint} "
            f"amplitude={amplitude:g} frequency={frequency_hz:g} cycles={self.sine_test_cycles} "
            f"repeat={repeat}"
        )
        self.sine_test_status_var.set(
            f"自动正弦 {self.sine_test_trial_index + 1}/{len(self.sine_test_trials)}: "
            f"J{joint:02d} ±{amplitude:g}° @ {frequency_hz:g}Hz，{self.sine_test_cycles}周期"
        )

    def _sine_test_enter_return(self) -> None:
        pose_index, joint, amplitude, frequency_hz, repeat = self.sine_test_trials[
            self.sine_test_trial_index
        ]
        pose = self.sine_test_poses[pose_index]
        self.sine_test_phase = "return_pose"
        self.sine_test_phase_started_monotonic = time.monotonic()
        self.sine_test_stable_since_monotonic = 0.0
        self.sine_test_current_target = pose
        self._queue_sine_test_pose_target(
            pose,
            f"return trial={self.sine_test_trial_index + 1} pose={pose_index + 1} "
            f"joint={joint} amplitude={amplitude:g} frequency={frequency_hz:g} repeat={repeat}",
        )
        self.sine_test_status_var.set(
            f"自动正弦 {self.sine_test_trial_index + 1}/{len(self.sine_test_trials)}: 返回S{pose_index + 1:02d}"
        )

    @staticmethod
    def _wrap_phase_degrees(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def _write_sine_test_summary(self, completion: str) -> None:
        writer = self.sine_test_summary_writer
        fp = self.sine_test_summary_file
        samples = self.sine_test_response_samples
        if writer is None or fp is None or not samples:
            return
        pose_index, joint, amplitude, frequency_hz, repeat = self.sine_test_trials[
            self.sine_test_trial_index
        ]
        pose = self.sine_test_poses[pose_index]
        fits: dict[int, tuple[float, float, float, float]] = {}
        for other in MANUAL_RECORD_JOINTS:
            fitted = fit_sine_signal(
                [(sample[0], sample[1][other]) for sample in samples], frequency_hz
            )
            if fitted is not None:
                fits[other] = fitted
        commanded_fit = fits.get(joint)
        tracking_errors = [sample[1][joint] - sample[2][joint] for sample in samples]
        tracking_rmse = math.sqrt(statistics.fmean(error * error for error in tracking_errors))
        tracking_bias = statistics.fmean(tracking_errors)
        cross_amplitudes = [fits[other][0] for other in MANUAL_RECORD_JOINTS if other != joint and other in fits]
        row: dict[str, object] = {
            "experiment_id": self.sine_test_experiment_id,
            "raw_csv": str(getattr(self.master, "continuous_record_path", "") or ""),
            "pose_index": pose_index + 1,
            **{f"pose_j{index:02d}_deg": f"{value:.6f}" for index, value in enumerate(pose)},
            "kp": self.angle_kp_scale_var.get().strip(),
            "ki_per_s": self.angle_ki_scale_var.get().strip(),
            "repeat": repeat,
            "commanded_joint": f"J{joint:02d}",
            "command_amplitude_deg": f"{amplitude:.6f}",
            "frequency_hz": f"{frequency_hz:.6f}",
            "cycles": self.sine_test_cycles,
            "completion": completion,
            "duration_s": f"{samples[-1][0]:.6f}",
            "sample_count": len(samples),
            "tracking_rmse_deg": f"{tracking_rmse:.6f}",
            "tracking_bias_deg": f"{tracking_bias:.6f}",
            "max_cross_coupling_amplitude_deg": f"{max(cross_amplitudes, default=0.0):.6f}",
            "minimum_j01_deg": f"{min(sample[1][1] for sample in samples):.6f}",
        }
        if commanded_fit is not None:
            measured_amplitude, phase_deg, _bias, fit_rmse = commanded_fit
            row.update(
                {
                    "measured_amplitude_deg": f"{measured_amplitude:.6f}",
                    "amplitude_gain": f"{measured_amplitude / amplitude:.6f}",
                    "phase_lag_deg": f"{self._wrap_phase_degrees(-phase_deg):.6f}",
                    "sine_fit_rmse_deg": f"{fit_rmse:.6f}",
                }
            )
        for other, fitted in fits.items():
            measured_amplitude, phase_deg, bias, fit_rmse = fitted
            row[f"measured_amplitude_j{other:02d}_deg"] = f"{measured_amplitude:.6f}"
            row[f"phase_j{other:02d}_deg"] = f"{phase_deg:.6f}"
            row[f"bias_j{other:02d}_deg"] = f"{bias:.6f}"
            row[f"fit_rmse_j{other:02d}_deg"] = f"{fit_rmse:.6f}"
        for motor in training_target_motor_indices():
            finite_forces = [sample[3][motor] for sample in samples if math.isfinite(sample[3][motor])]
            row[f"minimum_force_m{motor:02d}_n"] = (
                f"{min(finite_forces):.6f}" if finite_forces else ""
            )
        writer.writerow(row)
        fp.flush()

    def _sine_test_tick(self) -> None:
        self.sine_test_after_id = None
        if not self.sine_test_running:
            return
        host = self.master
        if self.poller is None or self.latest_snapshot is None:
            self._stop_sine_test("Encoder / Servo连接或数据丢失", send_stop=True)
            return
        if not bool(getattr(host, "continuous_record_running", False)):
            self._stop_sine_test("原始连续记录意外停止", send_stop=True)
            return
        if bool(getattr(host, "force_emergency_relax_active", False)):
            self._stop_sine_test("已触发全电机张力保护", send_stop=False)
            return
        snapshot = self.latest_snapshot
        angles = self._step_test_current_angles(snapshot)
        if angles is None:
            self._stop_sine_test("J00-J03出现无效角度，无法验证J01>0°", send_stop=True)
            return
        if angles[1] <= STEP_TEST_J01_MIN_DEG:
            self._stop_sine_test(f"实测J01={angles[1]:.3f}°≤0°，违反实验约束", send_stop=True)
            return
        now = time.monotonic()
        phase_elapsed = now - self.sine_test_phase_started_monotonic
        if self.sine_test_phase == "response":
            pose_index, joint, amplitude, frequency_hz, _repeat = self.sine_test_trials[
                self.sine_test_trial_index
            ]
            pose = self.sine_test_poses[pose_index]
            target_list = list(pose)
            target_list[joint] = pose[joint] + amplitude * math.sin(
                2.0 * math.pi * frequency_hz * phase_elapsed
            )
            target = tuple(target_list)
            if target[1] <= STEP_TEST_J01_MIN_DEG or not self._queue_sine_stream_target(target):
                self._stop_sine_test("正弦流目标无效或串口断开", send_stop=True)
                return
            self.sine_test_current_target = target
        target = self.sine_test_current_target
        if target is None:
            self._stop_sine_test("内部目标状态缺失", send_stop=True)
            return
        if snapshot.timestamp > self.sine_test_last_snapshot_ts:
            self.sine_test_last_snapshot_ts = snapshot.timestamp
            if self.sine_test_phase == "baseline":
                self.sine_test_baseline_samples.append((now, angles))
            elif self.sine_test_phase == "response":
                self.sine_test_response_samples.append(
                    (
                        phase_elapsed,
                        angles,
                        target,
                        self._step_test_force_vector(),
                    )
                )
        errors = [abs(angles[joint] - target[joint]) for joint in MANUAL_RECORD_JOINTS]
        all_close = max(errors) <= self.sine_test_band_deg
        if self.sine_test_phase in ("move_pose", "return_pose"):
            if all_close:
                if self.sine_test_stable_since_monotonic <= 0.0:
                    self.sine_test_stable_since_monotonic = now
                elif now - self.sine_test_stable_since_monotonic >= self.sine_test_hold_s:
                    if self.sine_test_phase == "move_pose":
                        self._sine_test_enter_baseline()
                    else:
                        self.sine_test_trial_index += 1
                        self._sine_test_begin_trial()
                        return
            else:
                self.sine_test_stable_since_monotonic = 0.0
            if phase_elapsed >= self.sine_test_timeout_s:
                self._stop_sine_test(
                    f"{self.sine_test_phase}阶段{self.sine_test_timeout_s:g}s未到位", send_stop=True
                )
                return
        elif self.sine_test_phase == "baseline":
            if not all_close:
                self.sine_test_phase = "move_pose"
                self.sine_test_phase_started_monotonic = now
                self.sine_test_stable_since_monotonic = 0.0
                self.sine_test_baseline_samples = []
                self.sine_test_status_var.set("基线期间离开稳定带，重新等待初始姿态稳定")
            elif phase_elapsed >= self.sine_test_baseline_s:
                self._sine_test_enter_response()
        elif self.sine_test_phase == "response":
            pose_index, _joint, _amplitude, frequency_hz, _repeat = self.sine_test_trials[
                self.sine_test_trial_index
            ]
            duration_s = self.sine_test_cycles / frequency_hz
            if phase_elapsed >= duration_s:
                self._write_sine_test_summary("complete")
                self._sine_test_record_event(
                    f"response_end trial={self.sine_test_trial_index + 1} completion=complete"
                )
                self._sine_test_enter_return()
        if self.sine_test_running:
            self._schedule_sine_test_tick()

    def _stop_sine_test(self, reason: str, *, send_stop: bool) -> None:
        was_running = self.sine_test_running
        if was_running and self.sine_test_phase == "response" and self.sine_test_response_samples:
            self._write_sine_test_summary("aborted")
        self.sine_test_running = False
        if self.sine_test_after_id is not None:
            try:
                self.after_cancel(self.sine_test_after_id)
            except tk.TclError:
                pass
            self.sine_test_after_id = None
        self.sine_test_phase = "idle"
        self.sine_test_current_target = None
        self._set_sine_test_controls_running(False)
        fp = self.sine_test_summary_file
        self.sine_test_summary_file = None
        self.sine_test_summary_writer = None
        if fp is not None:
            try:
                fp.close()
            except Exception:
                pass
        host = self.master
        if self.sine_test_started_continuous_record and bool(
            getattr(host, "continuous_record_running", False)
        ):
            host._stop_continuous_record(f"自动正弦停止: {reason}")
        self.sine_test_started_continuous_record = False
        if send_stop and self.poller is not None:
            self.queue_emergency_stop()
        if was_running:
            self.sine_test_status_var.set(
                f"自动正弦已停止: {reason}; 完成{self.sine_test_trial_index}/{len(self.sine_test_trials)}"
            )

    def _finish_sine_test(self) -> None:
        if not self.sine_test_running:
            return
        total = len(self.sine_test_trials)
        summary_path = self.sine_test_summary_path
        self._sine_test_record_event(f"complete trials={total}")
        self._stop_sine_test("全部完成", send_stop=False)
        self.sine_test_status_var.set(f"自动正弦完成: {total}/{total}；控制器保持最后初始姿态")
        if summary_path is not None:
            self.last_line_var.set(f"自动正弦汇总: {summary_path}")

    def _send_j00_j03_target(self) -> None:
        if self.step_test_running or self.sine_test_running:
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试。", parent=self)
            return
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        try:
            target_text = self.target_value_var.get().strip()
            target_values = parse_j00_j03_target_values(target_text)
        except ValueError as exc:
            messagebox.showwarning("参数错误", str(exc), parent=self)
            return

        host = self.master
        if getattr(host, "linearity_running", False):
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再手动发送目标。", parent=self)
            return
        if getattr(host, "multi_input_running", False):
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再手动发送目标。", parent=self)
            return
        if getattr(host, "force_target_running", False):
            host._stop_force_target_control("J00-J03 degree target requested")
        if getattr(host, "safe_relax_active", False):
            host._stop_safe_relax("J00-J03 degree target requested")

        self._cancel_direct_abs_sequence("已由 J00-J03 角度目标接管")
        for joint, target_value in zip(ENCODER_PLOT_JOINTS, target_values):
            self.sent_degree_targets[joint] = float(target_value)
        self.mode_var.set("degree")
        target_label = ", ".join(
            f"J{joint:02d}={target_value:.3f}deg"
            for joint, target_value in zip(ENCODER_PLOT_JOINTS, target_values)
        )
        self.sent_last_target_command = f"angle batch [{target_label}]"
        self.sent_target_var.set(self._sent_target_status_text())
        self._queue_start_with_host_state()
        if hasattr(host, "_record_continuous_command_event"):
            host._record_continuous_command_event(self.sent_last_target_command)

    def _send_target(self) -> None:
        if self.step_test_running or self.sine_test_running:
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试。", parent=self)
            return
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        try:
            index = int(self.target_index_var.get())
            target_text = self.target_value_var.get().strip()
            target_value = float(target_text)
        except ValueError:
            messagebox.showwarning("参数错误", "目标索引和目标值需要是数字。", parent=self)
            return

        mode = self.mode_var.get().strip().lower()
        host = self.master
        if getattr(host, "force_emergency_relax_active", False):
            messagebox.showwarning(
                "全电机正在安全放松",
                f"拉力恢复到五路均 > {FORCE_HARD_RELAX_STOP_N:.0f} N "
                "并自动停止前，不能发送新目标。",
                parent=self,
            )
            return
        if getattr(host, "linearity_running", False):
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再手动发送目标。", parent=self)
            return
        if getattr(host, "multi_input_running", False):
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再手动发送目标。", parent=self)
            return
        if mode == "degree":
            if not 0 <= index < ENCODER_COUNT:
                messagebox.showwarning("索引越界", f"degree 模式索引范围为 0-{ENCODER_COUNT - 1}。", parent=self)
                return
            if getattr(host, "force_target_running", False):
                host._stop_force_target_control("degree target requested")
            if getattr(host, "safe_relax_active", False):
                host._stop_safe_relax("degree target requested")
            if hasattr(host, "_start_or_refresh_nullspace_tension_control"):
                host._start_or_refresh_nullspace_tension_control(self, self.latest_snapshot)
            if hasattr(host, "_clear_tension_bias_for_angle_start"):
                host._clear_tension_bias_for_angle_start(self)
            if hasattr(host, "_start_or_refresh_degree_force_control"):
                degree_force_ready = host._start_or_refresh_degree_force_control(self, self.latest_snapshot)
                if bool(host.degree_force_enabled_var.get()) and not degree_force_ready:
                    messagebox.showwarning(
                        "degree 力反馈未启动",
                        host.degree_force_status_var.get(),
                        parent=self,
                    )
                    return
            device_target = target_value
            command = f"degree; j{index} {device_target:.3f}"
        else:
            if not 0 <= index < SERVO_COUNT:
                messagebox.showwarning("索引越界", f"direct 模式索引范围为 0-{SERVO_COUNT - 1}。", parent=self)
                return
            if getattr(host, "degree_force_active", False):
                host._stop_degree_force_control("direct target requested")
            if getattr(host, "feedback_pretension_active", False):
                host._stop_nullspace_pretension("direct target requested", hold_bias=False)
            command = f"direct; m{index} {int(round(target_value))}"
        self._cancel_direct_abs_sequence("已由单路目标接管")
        self.command_queue.put(("text", command))
        self._note_sent_mode_command(command)
        self._record_sent_target_command(command)

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


class ForceSensorApp(tk.Tk):
    def __init__(self, default_port: str, default_servo_port: str, default_servo_baudrate: int) -> None:
        super().__init__()
        self.title("力传感器数据监视器")
        self.geometry("1900x920")
        self.minsize(1200, 700)

        self.samples: list[ForceSample] = []
        self.baselines: dict[int, int] = load_force_zero_baselines()
        self.latest_by_channel: dict[int, ForceSample] = {}
        self.force_hard_stop_latched = False
        self.force_emergency_relax_active = False
        self.force_emergency_relax_targets: dict[int, int] = {}
        self.force_emergency_relax_move_count = 0
        self.force_emergency_relax_trigger_limit_n = FORCE_HARD_ABORT_N
        self.force_zero_capture_active = False
        self.force_zero_capture_samples: dict[int, list[int]] = {
            channel: [] for channel in DISPLAY_CHANNELS
        }
        self.force_zero_capture_after_id: Optional[str] = None
        self.outbox: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.command_queue: "queue.Queue[tuple[str, bytes, int]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.poller: Optional[SensorPoller] = None
        self.connected = False
        self.encoder_servo_window: Optional[EncoderServoWindow] = None
        self.default_servo_port = default_servo_port
        self.default_servo_baudrate = default_servo_baudrate

        self.port_var = tk.StringVar(value=default_port)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.slave_var = tk.IntVar(value=1)
        self.channel_var = tk.IntVar(value=DEFAULT_CHANNEL)
        self.interval_var = tk.IntVar(value=DEFAULT_FORCE_INTERVAL_MS)
        self.value_var = tk.StringVar(value="--")
        self.relative_var = tk.StringVar(value="--")
        self.max_tension_var = tk.StringVar(value="--")
        self.max_tension_relative_by_channel: dict[int, Optional[int]] = {
            channel: None for channel in DISPLAY_CHANNELS
        }
        self.channel_value_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.channel_relative_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.channel_max_tension_vars = {
            channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS
        }
        self.status_var = tk.StringVar(value="未连接")
        self.count_var = tk.StringVar(value="0")
        self.low_word_var = tk.StringVar(value="--")
        self.high_word_var = tk.StringVar(value="--")
        self.request_var = tk.StringVar(value="--")
        self.response_var = tk.StringVar(value="--")
        self.diagnostic_var = tk.StringVar(value="诊断: --")
        self.zero_status_var = tk.StringVar(value=self._zero_status_text())
        self.training_status_var = tk.StringVar(value="未采集")
        self.training_path_var = tk.StringVar(value="--")
        self.manual_record_status_var = tk.StringVar(value="线性记录: 0点")
        self.manual_record_path_var = tk.StringVar(value="--")
        self.continuous_record_status_var = tk.StringVar(value="连续记录: 未启动")
        self.continuous_record_path_var = tk.StringVar(value="--")
        self.continuous_record_interval_var = tk.IntVar(value=CONTINUOUS_RECORD_DEFAULT_INTERVAL_MS)
        self.linearity_status_var = tk.StringVar(value="线性判别: 未启动")
        self.linearity_path_var = tk.StringVar(value="--")
        self.linearity_analysis_csv_var = tk.StringVar(value="--")
        self.linearity_analysis_status_var = tk.StringVar(value="CSV分析: 未导入")
        self.safe_relax_status_var = tk.StringVar(value="安全放松: 未启动")
        self.training_interval_var = tk.IntVar(value=TRAINING_DEFAULT_SAMPLE_INTERVAL_MS)
        self.training_target_period_var = tk.StringVar(value=f"{TRAINING_DEFAULT_TARGET_PERIOD_S:.1f}")
        self.training_load_limit_var = tk.StringVar(value=TRAINING_DEFAULT_LOAD_LIMIT)
        self.training_ranges_var = tk.StringVar(value=TRAINING_DEFAULT_JOINT_RANGES)
        self.training_target_mode_var = tk.StringVar(value=TRAINING_DEFAULT_TARGET_MODE)
        self.training_send_targets_var = tk.BooleanVar(value=True)
        self.pretension_enabled_var = tk.BooleanVar(value=True)
        self.pretension_target_var = tk.StringVar(value=f"{PRETENSION_DEFAULT_TARGET_N:.1f}")
        self.pretension_tight_limit_var = tk.StringVar(value=f"{PRETENSION_DEFAULT_TIGHT_LIMIT_N:.1f}")
        self.pretension_step_var = tk.StringVar(value=PRETENSION_DEFAULT_STEP_TEXT)
        self.pretension_timeout_var = tk.StringVar(value=f"{PRETENSION_DEFAULT_TIMEOUT_S:.1f}")
        # Angle control is R feedforward + P feedback only by default.  The
        # host-side force-window controller remains available as an explicit
        # opt-in, but must never start automatically in a fresh session.
        self.degree_force_enabled_var = tk.BooleanVar(value=False)
        self.degree_force_status_var = tk.StringVar(value="Degree force: idle")
        self.linearity_states_var = tk.StringVar(value=LINEARITY_DEFAULT_STATE_MOTOR_POSITIONS)
        self.linearity_motors_var = tk.StringVar(value=LINEARITY_DEFAULT_MOTORS)
        self.linearity_deltas_var = tk.StringVar(value=LINEARITY_DEFAULT_DELTAS)
        self.linearity_repeats_var = tk.IntVar(value=LINEARITY_DEFAULT_REPEATS)
        self.linearity_hold_var = tk.StringVar(value=f"{LINEARITY_DEFAULT_STABLE_HOLD_S:.1f}")
        self.linearity_timeout_var = tk.StringVar(value=f"{LINEARITY_DEFAULT_TIMEOUT_S:.1f}")
        self.multi_input_status_var = tk.StringVar(value="联合辨识: 未启动")
        self.multi_input_path_var = tk.StringVar(value="--")
        self.multi_input_analysis_status_var = tk.StringVar(value="结果: 尚未分析")
        self.multi_input_state_var = tk.StringVar(value=LINEARITY_DEFAULT_STATE_MOTOR_POSITIONS)
        self.multi_input_mode_var = tk.StringVar(value=MULTI_INPUT_DEFAULT_MODE)
        self.multi_input_amplitudes_var = tk.StringVar(value=MULTI_INPUT_DEFAULT_AMPLITUDES)
        self.multi_input_count_var = tk.IntVar(value=MULTI_INPUT_DEFAULT_SAMPLE_COUNT)
        self.multi_input_seed_var = tk.IntVar(value=MULTI_INPUT_DEFAULT_SEED)
        self.multi_input_hold_var = tk.StringVar(value=f"{LINEARITY_DEFAULT_STABLE_HOLD_S:.1f}")
        self.multi_input_post_hold_var = tk.StringVar(value=f"{LINEARITY_POST_RECORD_HOLD_S:.1f}")
        self.multi_input_timeout_var = tk.StringVar(value=f"{LINEARITY_DEFAULT_TIMEOUT_S:.1f}")
        self.multi_input_record_interval_var = tk.IntVar(value=MULTI_INPUT_DEFAULT_RECORD_INTERVAL_MS)
        self.multi_input_split_var = tk.StringVar(value=MULTI_INPUT_DEFAULT_SPLIT)
        self.multi_input_lambda_var = tk.StringVar(value=MULTI_INPUT_DEFAULT_RIDGE_LAMBDA)
        self.multi_input_frequencies_var = tk.StringVar(value=MULTI_INPUT_DEFAULT_FREQUENCIES)
        self.training_running = False
        self.training_phase = "idle"
        self.training_file: Optional[object] = None
        self.training_writer: Optional[csv.DictWriter] = None
        self.training_path: Optional[Path] = None
        self.manual_record_path: Optional[Path] = None
        self.continuous_record_running = False
        self.continuous_record_file: Optional[object] = None
        self.continuous_record_writer: Optional[csv.DictWriter] = None
        self.continuous_record_path: Optional[Path] = None
        self.continuous_event_file: Optional[object] = None
        self.continuous_event_writer: Optional[csv.DictWriter] = None
        self.continuous_event_path: Optional[Path] = None
        self.continuous_record_rows = 0
        self.continuous_record_started_monotonic = 0.0
        self.continuous_record_next_sample_monotonic = 0.0
        self.continuous_record_interval_ms = CONTINUOUS_RECORD_DEFAULT_INTERVAL_MS
        self.linearity_path: Optional[Path] = None
        self.linearity_analysis_csv_path: Optional[Path] = None
        self.linearity_analysis_markdown = ""
        self.linearity_analysis_output_dir: Optional[Path] = None
        self.manual_record_rows = 0
        self.linearity_rows = 0
        self.training_rows = 0
        self.training_started_monotonic = 0.0
        self.training_next_sample_monotonic = 0.0
        self.training_next_target_monotonic = 0.0
        self.training_next_tension_check_monotonic = 0.0
        self.training_sample_interval_ms = TRAINING_DEFAULT_SAMPLE_INTERVAL_MS
        self.training_target_period_s = TRAINING_DEFAULT_TARGET_PERIOD_S
        self.training_load_limit: Optional[float] = None
        self.training_ranges = parse_training_joint_ranges(TRAINING_DEFAULT_JOINT_RANGES)
        self.training_target_mode = TRAINING_DEFAULT_TARGET_MODE
        self.training_last_target_joint: Optional[int] = None
        self.training_last_target_relative: Optional[float] = None
        self.training_last_target_device: Optional[float] = None
        self.training_last_target_command = ""
        self.training_last_target_mode = TRAINING_DEFAULT_TARGET_MODE
        self.training_last_target_phase = ""
        self.training_target_index = 0
        self.training_target_relative_by_joint: dict[int, float] = {}
        self.training_target_device_by_joint: dict[int, float] = {}
        self.pretension_target_n = PRETENSION_DEFAULT_TARGET_N
        self.pretension_tight_limit_n = PRETENSION_DEFAULT_TIGHT_LIMIT_N
        self.pretension_steps = parse_pretension_steps(PRETENSION_DEFAULT_STEP_TEXT)
        self.pretension_timeout_s = PRETENSION_DEFAULT_TIMEOUT_S
        self.pretension_started_monotonic = 0.0
        self.pretension_next_step_monotonic = 0.0
        self.pretension_move_count = 0
        self.pretension_last_targets: dict[int, int] = {}
        self.training_tension_action = ""
        self.training_tension_action_count = 0
        self.training_tension_window_ok = 1
        self.training_tension_bias_by_motor: dict[int, int] = {}
        self.degree_force_active = False
        self.degree_force_target_n = PRETENSION_DEFAULT_TARGET_N
        self.degree_force_tight_limit_n = PRETENSION_DEFAULT_TIGHT_LIMIT_N
        self.degree_force_steps = parse_pretension_steps(PRETENSION_DEFAULT_STEP_TEXT)
        self.degree_force_next_step_monotonic = 0.0
        self.degree_force_move_count = 0
        self.degree_force_bias_by_motor: dict[int, int] = {}
        self.degree_force_phase = "idle"
        self.degree_force_angle_ready_since_monotonic = 0.0
        self.degree_force_tension_window_ok = False
        self.linearity_running = False
        self.linearity_experiment_id = ""
        self.linearity_started_monotonic = 0.0
        self.linearity_phase = "idle"
        self.linearity_phase_started_monotonic = 0.0
        self.linearity_next_trace_sample_monotonic = 0.0
        self.linearity_phase_command_sent = False
        self.linearity_prev_degree_force_enabled = True
        self.linearity_states: list[dict[int, int]] = []
        self.linearity_motors: list[int] = []
        self.linearity_deltas: list[int] = []
        self.linearity_repeats = LINEARITY_DEFAULT_REPEATS
        self.linearity_hold_s = LINEARITY_DEFAULT_STABLE_HOLD_S
        self.linearity_timeout_s = LINEARITY_DEFAULT_TIMEOUT_S
        self.linearity_state_index = 0
        self.linearity_trials: list[tuple[int, int, int]] = []
        self.linearity_trial_index = 0
        self.linearity_current_trial: Optional[tuple[int, int, int]] = None
        self.linearity_baseline_sample_index = 0
        self.linearity_baseline_label = ""
        self.linearity_baseline_values: dict[str, dict[int, float]] = {}
        self.linearity_baseline_abs_by_motor: dict[int, int] = {}
        self.linearity_expected_abs_by_motor: dict[int, int] = {}
        self.linearity_stability_anchor_joints: Optional[dict[int, float]] = None
        self.linearity_stability_anchor_forces: Optional[dict[int, float]] = None
        self.linearity_stability_since_monotonic = 0.0
        self.linearity_last_stable_hold_s = 0.0
        self.linearity_last_max_joint_delta_deg = 0.0
        self.linearity_last_max_force_delta_n = 0.0
        self.linearity_last_max_motor_speed = 0
        self.linearity_last_command = ""
        self.linearity_last_actual_delta_counts = 0
        self.multi_input_running = False
        self.multi_input_experiment_id = ""
        self.multi_input_path: Optional[Path] = None
        self.multi_input_rows = 0
        self.multi_input_phase = "idle"
        self.multi_input_phase_started_monotonic = 0.0
        self.multi_input_next_trace_sample_monotonic = 0.0
        self.multi_input_state: dict[int, int] = {}
        self.multi_input_mode = MULTI_INPUT_DEFAULT_MODE
        self.multi_input_amplitudes = [200.0] * 5
        self.multi_input_count = MULTI_INPUT_DEFAULT_SAMPLE_COUNT
        self.multi_input_seed = MULTI_INPUT_DEFAULT_SEED
        self.multi_input_hold_s = LINEARITY_DEFAULT_STABLE_HOLD_S
        self.multi_input_post_hold_s = LINEARITY_POST_RECORD_HOLD_S
        self.multi_input_timeout_s = LINEARITY_DEFAULT_TIMEOUT_S
        self.multi_input_record_interval_s = MULTI_INPUT_DEFAULT_RECORD_INTERVAL_MS / 1000.0
        self.multi_input_split_ratios = (0.6, 0.2, 0.2)
        self.multi_input_lambda = float(MULTI_INPUT_DEFAULT_RIDGE_LAMBDA)
        self.multi_input_frequencies = [0.037, 0.053, 0.071, 0.089, 0.113]
        self.multi_input_sequence: list[list[int]] = []
        self.multi_input_split_labels: list[str] = []
        self.multi_input_sequence_index = 0
        self.multi_input_current_delta: list[int] = [0] * 5
        self.multi_input_baseline_sample_index = 0
        self.multi_input_baseline_values: dict[str, dict[int, float]] = {}
        self.multi_input_baseline_abs_by_motor: dict[int, int] = {}
        self.multi_input_expected_abs_by_motor: dict[int, int] = {}
        self.multi_input_stability_anchor_joints: Optional[dict[int, float]] = None
        self.multi_input_stability_anchor_forces: Optional[dict[int, float]] = None
        self.multi_input_stability_since_monotonic = 0.0
        self.multi_input_last_stable_hold_s = 0.0
        self.multi_input_last_max_joint_delta_deg = 0.0
        self.multi_input_last_max_force_delta_n = 0.0
        self.multi_input_last_max_motor_speed = 0
        self.multi_input_last_command = ""
        self.multi_input_prev_degree_force_enabled = True
        self.multi_input_analysis_markdown = ""
        self.multi_input_analysis_output_dir: Optional[Path] = None
        self.safe_relax_active = False
        self.safe_relax_ok_since = 0.0
        self.safe_relax_next_step_monotonic = 0.0
        self.safe_relax_move_count = 0
        self.safe_relax_last_targets: dict[int, int] = {}
        self.safe_relax_after_id: Optional[str] = None
        self.force_target_enabled_vars = {channel: tk.BooleanVar(value=True) for channel in DISPLAY_CHANNELS}
        self.force_target_vars = {
            channel: tk.StringVar(value=f"{FORCE_TARGET_DEFAULT_N_BY_CHANNEL.get(channel, 0.0):.1f}")
            for channel in DISPLAY_CHANNELS
        }
        self.force_target_current_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.force_target_motor_target_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.force_target_interval_var = tk.IntVar(value=FORCE_TARGET_DEFAULT_INTERVAL_MS)
        self.force_target_step_var = tk.IntVar(value=FORCE_TARGET_DEFAULT_STEP_COUNTS)
        self.force_target_deadband_var = tk.StringVar(value=f"{FORCE_TARGET_DEFAULT_DEADBAND_N:.1f}")
        self.force_target_status_var = tk.StringVar(value="Force target: idle")
        self.force_target_running = False
        self.feedback_pretension_active = False
        self.feedback_pretension_started_monotonic = 0.0
        self.nullspace_pretension_alpha_counts = 0.0
        self.nullspace_pretension_bias_by_motor = {motor: 0 for motor in range(5)}
        self.nullspace_pretension_next_step_monotonic = 0.0
        self.nullspace_pretension_move_count = 0
        self.force_target_next_step_monotonic = 0.0
        self.force_target_settle_since_monotonic = 0.0
        self.force_target_move_count = 0
        self.force_target_last_targets: dict[int, int] = {}
        self.force_target_next_channel_step_monotonic: dict[int, float] = {}
        self.force_target_active_channels: set[int] = set()

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_outbox)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(7, weight=1)

        connection = ttk.LabelFrame(self, text="连接")
        connection.grid(row=0, column=0, padx=10, pady=(10, 6), sticky="ew")
        for col in range(14):
            connection.columnconfigure(col, weight=0)
        connection.columnconfigure(13, weight=1)

        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=12)
        self.baud_combo = ttk.Combobox(
            connection, textvariable=self.baud_var, values=[str(v) for v in BAUD_RATES], width=10
        )
        self.slave_spin = tk.Spinbox(connection, from_=1, to=255, textvariable=self.slave_var, width=6)
        self.channel_spin = tk.Spinbox(connection, from_=1, to=8, textvariable=self.channel_var, width=5)
        self.interval_spin = tk.Spinbox(
            connection, from_=20, to=5000, increment=20, textvariable=self.interval_var, width=8
        )
        self.refresh_button = ttk.Button(connection, text="刷新串口", command=self._refresh_ports)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        self.encoder_servo_button = ttk.Button(
            connection,
            text="Encoder / Servo",
            command=self._open_encoder_servo_window,
        )
        self.safe_relax_top_button = ttk.Button(
            connection,
            text="安全放松",
            command=self._toggle_safe_relax,
        )

        widgets = [
            ("串口", self.port_combo),
            ("波特率", self.baud_combo),
            ("从站地址", self.slave_spin),
            ("通道", self.channel_spin),
            ("轮询间隔(ms)", self.interval_spin),
        ]
        col = 0
        for label, widget in widgets:
            ttk.Label(connection, text=label).grid(row=0, column=col, padx=(8, 4), pady=8)
            widget.grid(row=0, column=col + 1, padx=(0, 10), pady=8)
            col += 2
        self.refresh_button.grid(row=0, column=col, padx=4, pady=8)
        self.connect_button.grid(row=0, column=col + 1, padx=4, pady=8)
        self.encoder_servo_button.grid(row=0, column=col + 2, padx=(12, 8), pady=8)
        self.safe_relax_top_button.grid(row=0, column=col + 3, padx=(4, 8), pady=8)

        readout = ttk.LabelFrame(self, text="实时数据")
        readout.grid(row=1, column=0, padx=10, pady=6, sticky="ew")
        readout.columnconfigure(1, weight=1)
        readout.columnconfigure(3, weight=1)
        readout.columnconfigure(5, weight=1)

        ttk.Label(readout, text="主通道当前值").grid(row=0, column=0, padx=10, pady=8, sticky="w")
        tk.Label(readout, textvariable=self.value_var, font=("Segoe UI", 30, "bold")).grid(
            row=0, column=1, padx=6, pady=8, sticky="w"
        )
        ttk.Label(readout, text="主通道相对力(N)").grid(row=0, column=2, padx=10, pady=8, sticky="w")
        tk.Label(readout, textvariable=self.relative_var, font=("Segoe UI", 24, "bold")).grid(
            row=0, column=3, padx=6, pady=8, sticky="w"
        )
        ttk.Label(readout, text="主通道最大拉力(N)").grid(
            row=0, column=4, padx=10, pady=8, sticky="w"
        )
        tk.Label(
            readout,
            textvariable=self.max_tension_var,
            font=("Segoe UI", 24, "bold"),
            fg="#7f1d1d",
        ).grid(row=0, column=5, padx=6, pady=8, sticky="w")

        for index, channel in enumerate(DISPLAY_CHANNELS, start=1):
            color = CHANNEL_COLORS[channel]
            ttk.Label(readout, text=force_channel_label(channel)).grid(
                row=index, column=0, padx=10, pady=3, sticky="w"
            )
            tk.Label(
                readout,
                textvariable=self.channel_value_vars[channel],
                font=("Segoe UI", 16, "bold"),
                fg=color,
            ).grid(row=index, column=1, padx=6, pady=3, sticky="w")
            ttk.Label(readout, text=f"{force_channel_label(channel)} 相对力(N)").grid(
                row=index, column=2, padx=10, pady=3, sticky="w"
            )
            tk.Label(
                readout,
                textvariable=self.channel_relative_vars[channel],
                font=("Segoe UI", 16, "bold"),
                fg=color,
            ).grid(row=index, column=3, padx=6, pady=3, sticky="w")
            ttk.Label(readout, text=f"{force_channel_label(channel)} 最大拉力(N)").grid(
                row=index, column=4, padx=10, pady=3, sticky="w"
            )
            tk.Label(
                readout,
                textvariable=self.channel_max_tension_vars[channel],
                font=("Segoe UI", 16, "bold"),
                fg=color,
            ).grid(row=index, column=5, padx=6, pady=3, sticky="w")

        info_row = 1 + len(DISPLAY_CHANNELS)
        ttk.Label(readout, text="状态").grid(row=info_row, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.status_var).grid(row=info_row, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(readout, text="样本数").grid(row=info_row, column=2, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.count_var).grid(row=info_row, column=3, padx=6, pady=4, sticky="w")

        word_row = info_row + 1
        ttk.Label(readout, text="低字").grid(row=word_row, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.low_word_var).grid(row=word_row, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(readout, text="高字").grid(row=word_row, column=2, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.high_word_var).grid(row=word_row, column=3, padx=6, pady=4, sticky="w")

        ttk.Button(readout, text="无载校零（2秒）", command=self._set_baseline).grid(
            row=word_row + 1, column=0, padx=10, pady=8, sticky="w"
        )
        ttk.Button(readout, text="清除零位", command=self._clear_baseline).grid(
            row=word_row + 1, column=1, padx=6, pady=8, sticky="w"
        )
        ttk.Button(readout, text="设备去皮", command=lambda: self._queue_tare_command(1)).grid(
            row=word_row + 1, column=2, padx=6, pady=8, sticky="w"
        )
        ttk.Button(readout, text="取消去皮", command=lambda: self._queue_tare_command(2)).grid(
            row=word_row + 1, column=3, padx=6, pady=8, sticky="w"
        )
        ttk.Label(readout, text="零位状态").grid(row=word_row + 2, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.zero_status_var).grid(
            row=word_row + 2, column=1, columnspan=3, padx=6, pady=4, sticky="w"
        )
        ttk.Button(readout, text="诊断读取", command=self._queue_diagnostic_read).grid(
            row=word_row + 3, column=0, padx=10, pady=4, sticky="w"
        )
        ttk.Label(readout, textvariable=self.diagnostic_var).grid(
            row=word_row + 3, column=1, columnspan=3, padx=6, pady=4, sticky="w"
        )

        self.plot_canvas = tk.Canvas(self, height=220, bg="white", highlightthickness=1, highlightbackground="#d0d7de")
        self.plot_canvas.grid(row=2, column=0, padx=10, pady=6, sticky="nsew")
        self.plot_canvas.bind("<Configure>", lambda _event: self._draw_plot())

        frames = ttk.LabelFrame(self, text="Modbus 帧")
        frames.grid(row=3, column=0, padx=10, pady=6, sticky="ew")
        frames.columnconfigure(1, weight=1)
        ttk.Label(frames, text="请求").grid(row=0, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(frames, textvariable=self.request_var).grid(row=0, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(frames, text="响应").grid(row=1, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(frames, textvariable=self.response_var).grid(row=1, column=1, padx=6, pady=4, sticky="w")

        force_target = ttk.LabelFrame(self, text="Force Target Control (Direct Motor Mode)")
        force_target.grid(row=4, column=0, padx=10, pady=6, sticky="ew")
        for col in range(12):
            force_target.columnconfigure(col, weight=0)
        force_target.columnconfigure(11, weight=1)

        ttk.Label(force_target, text="Enable").grid(row=0, column=0, padx=(10, 4), pady=4)
        ttk.Label(force_target, text="Sensor / Motor").grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(force_target, text="Current N").grid(row=0, column=2, padx=4, pady=4)
        ttk.Label(force_target, text="Target N").grid(row=0, column=3, padx=4, pady=4)
        ttk.Label(force_target, text="Motor target").grid(row=0, column=4, padx=4, pady=4)
        ttk.Label(force_target, text="Output").grid(row=0, column=5, padx=4, pady=4)

        for row, channel in enumerate(DISPLAY_CHANNELS, start=1):
            ttk.Checkbutton(
                force_target,
                variable=self.force_target_enabled_vars[channel],
            ).grid(row=row, column=0, padx=(10, 4), pady=3)
            ttk.Label(force_target, text=force_channel_label(channel)).grid(row=row, column=1, padx=4, pady=3, sticky="w")
            ttk.Label(force_target, textvariable=self.force_target_current_vars[channel]).grid(
                row=row, column=2, padx=4, pady=3, sticky="e"
            )
            ttk.Entry(force_target, textvariable=self.force_target_vars[channel], width=8).grid(
                row=row, column=3, padx=4, pady=3
            )
            ttk.Label(force_target, textvariable=self.force_target_motor_target_vars[channel]).grid(
                row=row, column=4, padx=4, pady=3, sticky="w"
            )
            ttk.Button(
                force_target,
                text="Output",
                command=lambda c=channel: self._start_force_target_control([c]),
            ).grid(row=row, column=5, padx=4, pady=3, sticky="ew")

        ttk.Label(force_target, text="Interval ms").grid(row=1, column=6, padx=(18, 4), pady=3, sticky="e")
        tk.Spinbox(
            force_target,
            from_=20,
            to=5000,
            increment=10,
            textvariable=self.force_target_interval_var,
            width=7,
        ).grid(row=1, column=7, padx=(0, 8), pady=3, sticky="w")
        ttk.Label(force_target, text="Far step").grid(row=2, column=6, padx=(18, 4), pady=3, sticky="e")
        tk.Spinbox(
            force_target,
            from_=1,
            to=2000,
            increment=1,
            textvariable=self.force_target_step_var,
            width=7,
        ).grid(row=2, column=7, padx=(0, 8), pady=3, sticky="w")
        ttk.Label(force_target, text="Deadband N").grid(row=3, column=6, padx=(18, 4), pady=3, sticky="e")
        ttk.Entry(force_target, textvariable=self.force_target_deadband_var, width=8).grid(
            row=3, column=7, padx=(0, 8), pady=3, sticky="w"
        )
        self.force_target_start_button = ttk.Button(
            force_target,
            text="Start Force Loop",
            command=self._start_force_target_control,
        )
        self.force_target_start_button.grid(row=4, column=6, padx=(18, 4), pady=5, sticky="ew")
        self.force_target_stop_button = ttk.Button(
            force_target,
            text="Stop Force Loop",
            command=lambda: self._stop_force_target_control("manual stop"),
            state="disabled",
        )
        self.force_target_stop_button.grid(row=4, column=7, padx=(0, 8), pady=5, sticky="ew")
        self.feedback_pretension_button = ttk.Button(
            force_target,
            text="反馈前预紧（-15~-10 N）",
            command=self._toggle_nullspace_pretension,
        )
        self.feedback_pretension_button.grid(
            row=5,
            column=6,
            columnspan=2,
            padx=(18, 8),
            pady=5,
            sticky="ew",
        )
        self.feedback_pretension_button.configure(
            text="零空间张力分配"
        )
        self.feedback_pretension_button.grid_remove()
        ttk.Label(
            force_target,
            text=(
                "Step mode: <=5N uses 1 count per 200ms; >5N uses Far step. "
                "反馈前预紧固定每100ms走20 counts，五路进入 -15~-10N 并保持1s后自动完成。"
            ),
        ).grid(row=6, column=0, columnspan=7, padx=10, pady=(2, 5), sticky="w")
        ttk.Label(
            force_target,
            text=(
                "degree反馈只使用零空间张力分配：按"
                "拉力幅值f_min=5 N、f_max=90 N（传感器读数-5~-90 N）"
                "解析分配统一的零空间alpha，不再单独收紧某一路；"
                "-90 N触发全电机放松至 >-70 N。"
            ),
        ).grid(row=6, column=0, columnspan=7, padx=10, pady=(2, 5), sticky="w")
        ttk.Label(force_target, textvariable=self.force_target_status_var).grid(
            row=1, column=10, rowspan=4, columnspan=2, padx=(10, 8), pady=3, sticky="nw"
        )

        training = ttk.LabelFrame(self, text="训练数据采集")
        training.grid(row=5, column=0, padx=10, pady=6, sticky="ew")
        for col in range(14):
            training.columnconfigure(col, weight=0)
        training.columnconfigure(13, weight=1)

        ttk.Label(training, text="采样(ms)").grid(row=0, column=0, padx=(10, 4), pady=5, sticky="w")
        self.training_interval_spin = tk.Spinbox(
            training,
            from_=20,
            to=5000,
            increment=20,
            textvariable=self.training_interval_var,
            width=7,
        )
        self.training_interval_spin.grid(row=0, column=1, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(training, text="目标周期(s)").grid(row=0, column=2, padx=(4, 4), pady=5, sticky="w")
        self.training_target_period_entry = ttk.Entry(training, textvariable=self.training_target_period_var, width=7)
        self.training_target_period_entry.grid(row=0, column=3, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(training, text="Load保护(abs)").grid(row=0, column=4, padx=(4, 4), pady=5, sticky="w")
        self.training_load_limit_entry = ttk.Entry(training, textvariable=self.training_load_limit_var, width=8)
        self.training_load_limit_entry.grid(row=0, column=5, padx=(0, 10), pady=5, sticky="w")
        self.training_send_targets_check = ttk.Checkbutton(
            training,
            text="覆盖degree目标",
            variable=self.training_send_targets_var,
        )
        self.training_send_targets_check.grid(row=0, column=6, padx=(4, 10), pady=5, sticky="w")
        self.training_start_button = ttk.Button(training, text="开始采集", command=self._start_training_collection)
        self.training_start_button.grid(row=0, column=7, padx=4, pady=5, sticky="w")
        self.training_stop_button = ttk.Button(
            training,
            text="停止采集",
            command=lambda: self._stop_training_collection("手动停止"),
            state="disabled",
        )
        self.training_stop_button.grid(row=0, column=8, padx=4, pady=5, sticky="w")
        ttk.Label(training, textvariable=self.training_status_var).grid(
            row=0, column=9, columnspan=5, padx=(10, 8), pady=5, sticky="w"
        )

        ttk.Label(training, text="关节范围").grid(row=1, column=0, padx=(10, 4), pady=5, sticky="w")
        self.training_ranges_entry = ttk.Entry(training, textvariable=self.training_ranges_var, width=42)
        self.training_ranges_entry.grid(row=1, column=1, columnspan=5, padx=(0, 10), pady=5, sticky="ew")
        ttk.Label(training, text="mode").grid(row=1, column=6, padx=(4, 4), pady=5, sticky="w")
        self.training_target_mode_combo = ttk.Combobox(
            training,
            textvariable=self.training_target_mode_var,
            values=list(TRAINING_TARGET_MODES),
            width=10,
            state="readonly",
        )
        self.training_target_mode_combo.grid(row=1, column=7, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(training, text="文件").grid(row=1, column=8, padx=(4, 4), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.training_path_var).grid(
            row=1, column=9, columnspan=5, padx=(0, 8), pady=5, sticky="w"
        )
        self.pretension_enabled_check = ttk.Checkbutton(
            training,
            text="采集前自动预紧",
            variable=self.pretension_enabled_var,
        )
        self.pretension_enabled_check.grid(row=2, column=0, columnspan=2, padx=(10, 10), pady=5, sticky="w")
        ttk.Label(training, text="预紧目标(N)").grid(row=2, column=2, padx=(4, 4), pady=5, sticky="w")
        self.pretension_target_entry = ttk.Entry(training, textvariable=self.pretension_target_var, width=8)
        self.pretension_target_entry.grid(row=2, column=3, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(training, text="预紧步进").grid(row=2, column=4, padx=(4, 4), pady=5, sticky="w")
        self.pretension_step_entry = ttk.Entry(training, textvariable=self.pretension_step_var, width=36)
        self.pretension_step_entry.grid(row=2, column=5, columnspan=3, padx=(0, 10), pady=5, sticky="ew")
        ttk.Label(training, text="超时(s)").grid(row=2, column=8, padx=(4, 4), pady=5, sticky="w")
        self.pretension_timeout_entry = ttk.Entry(training, textvariable=self.pretension_timeout_var, width=8)
        self.pretension_timeout_entry.grid(row=2, column=9, padx=(0, 8), pady=5, sticky="w")
        ttk.Label(training, text="M00-M03下限(N)").grid(row=2, column=10, padx=(4, 4), pady=5, sticky="w")
        self.pretension_tight_limit_entry = ttk.Entry(
            training,
            textvariable=self.pretension_tight_limit_var,
            width=8,
        )
        self.pretension_tight_limit_entry.grid(row=2, column=11, padx=(0, 8), pady=5, sticky="w")
        self.safe_relax_button = ttk.Button(training, text="安全放松", command=self._toggle_safe_relax)
        self.safe_relax_button.grid(row=3, column=0, padx=(10, 10), pady=5, sticky="w")
        ttk.Label(training, text="目标").grid(row=3, column=1, padx=(4, 4), pady=5, sticky="e")
        ttk.Label(training, text=f"M00-M04 = {SAFE_RELAX_TARGET_ABS} counts").grid(
            row=3, column=2, columnspan=3, padx=(0, 10), pady=5, sticky="w"
        )
        ttk.Label(training, textvariable=self.safe_relax_status_var).grid(
            row=3, column=5, columnspan=9, padx=(0, 8), pady=5, sticky="w"
        )
        self.manual_record_button = ttk.Button(
            training,
            text="记录当前点",
            command=self._record_linearity_sample,
        )
        self.manual_record_button.grid(row=4, column=0, padx=(10, 10), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.manual_record_status_var).grid(
            row=4, column=1, columnspan=5, padx=(0, 10), pady=5, sticky="w"
        )
        ttk.Label(training, text="线性记录文件").grid(row=4, column=6, padx=(4, 4), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.manual_record_path_var).grid(
            row=4, column=7, columnspan=7, padx=(0, 8), pady=5, sticky="w"
        )
        self.continuous_record_button = ttk.Button(
            training,
            text="开始连续记录",
            command=self._toggle_continuous_record,
        )
        self.continuous_record_button.grid(row=5, column=0, padx=(10, 10), pady=5, sticky="w")
        ttk.Label(training, text="间隔(ms)").grid(row=5, column=1, padx=(0, 4), pady=5, sticky="e")
        self.continuous_record_interval_spin = tk.Spinbox(
            training,
            from_=20,
            to=5000,
            increment=20,
            textvariable=self.continuous_record_interval_var,
            width=7,
        )
        self.continuous_record_interval_spin.grid(row=5, column=2, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.continuous_record_status_var).grid(
            row=5, column=3, columnspan=3, padx=(0, 10), pady=5, sticky="w"
        )
        ttk.Label(training, text="连续记录文件").grid(row=5, column=6, padx=(4, 4), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.continuous_record_path_var).grid(
            row=5, column=7, columnspan=7, padx=(0, 8), pady=5, sticky="w"
        )
        self.degree_force_enabled_check = ttk.Checkbutton(
            training,
            text="degree 力反馈",
            variable=self.degree_force_enabled_var,
        )
        self.degree_force_enabled_check.grid(row=6, column=0, columnspan=2, padx=(10, 10), pady=5, sticky="w")
        self.degree_force_stop_button = ttk.Button(
            training,
            text="停止 degree 力反馈",
            command=lambda: self._stop_degree_force_control("manual stop"),
            state="disabled",
        )
        self.degree_force_stop_button.grid(row=6, column=2, columnspan=2, padx=(4, 10), pady=5, sticky="w")
        ttk.Label(training, textvariable=self.degree_force_status_var).grid(
            row=6, column=4, columnspan=10, padx=(0, 8), pady=5, sticky="w"
        )

        identification = ttk.Frame(self)
        identification.grid(row=6, column=0, padx=10, pady=6, sticky="ew")
        identification.columnconfigure(0, weight=1)
        identification.columnconfigure(1, weight=1)

        linearity = ttk.LabelFrame(identification, text="局部线性辨识实验")
        linearity.grid(row=0, column=0, padx=(0, 5), sticky="nsew")
        for col in range(14):
            linearity.columnconfigure(col, weight=0)
        linearity.columnconfigure(13, weight=1)

        ttk.Label(linearity, text="初始ABS(M00;M01;M02;M03;M04)").grid(
            row=0, column=0, padx=(10, 4), pady=5, sticky="w"
        )
        self.linearity_states_entry = ttk.Entry(linearity, textvariable=self.linearity_states_var, width=36)
        self.linearity_states_entry.grid(row=0, column=1, columnspan=7, padx=(0, 10), pady=5, sticky="ew")
        ttk.Label(linearity, text="电机").grid(row=0, column=8, padx=(4, 4), pady=5, sticky="w")
        self.linearity_motors_entry = ttk.Entry(linearity, textvariable=self.linearity_motors_var, width=16)
        self.linearity_motors_entry.grid(row=0, column=9, columnspan=2, padx=(0, 10), pady=5, sticky="w")
        self.linearity_start_button = ttk.Button(
            linearity,
            text="步骤1 到位并记录",
            command=self._start_linearity_experiment,
        )
        self.linearity_start_button.grid(row=0, column=11, padx=4, pady=5, sticky="w")
        self.linearity_scan_button = ttk.Button(
            linearity,
            text="步骤2 执行九点扫描",
            command=self._start_linearity_scan,
            state="disabled",
        )
        self.linearity_scan_button.grid(row=0, column=12, padx=4, pady=5, sticky="w")
        self.linearity_stop_button = ttk.Button(
            linearity,
            text="停止",
            command=lambda: self._stop_linearity_experiment("手动停止"),
            state="disabled",
        )
        self.linearity_stop_button.grid(row=0, column=13, padx=4, pady=5, sticky="w")

        ttk.Label(linearity, text="扰动counts").grid(row=1, column=0, padx=(10, 4), pady=5, sticky="w")
        self.linearity_deltas_entry = ttk.Entry(linearity, textvariable=self.linearity_deltas_var, width=22)
        self.linearity_deltas_entry.grid(row=1, column=1, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(linearity, text="重复").grid(row=1, column=2, padx=(4, 4), pady=5, sticky="w")
        self.linearity_repeats_spin = tk.Spinbox(
            linearity,
            from_=1,
            to=20,
            increment=1,
            textvariable=self.linearity_repeats_var,
            width=5,
        )
        self.linearity_repeats_spin.grid(row=1, column=3, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(linearity, text="到位停留s").grid(row=1, column=4, padx=(4, 4), pady=5, sticky="w")
        self.linearity_hold_entry = ttk.Entry(linearity, textvariable=self.linearity_hold_var, width=7)
        self.linearity_hold_entry.grid(row=1, column=5, padx=(0, 10), pady=5, sticky="w")
        ttk.Label(linearity, text="超时s").grid(row=1, column=6, padx=(4, 4), pady=5, sticky="w")
        self.linearity_timeout_entry = ttk.Entry(linearity, textvariable=self.linearity_timeout_var, width=7)
        self.linearity_timeout_entry.grid(row=1, column=7, padx=(0, 10), pady=5, sticky="w")

        ttk.Label(linearity, textvariable=self.linearity_status_var).grid(
            row=2, column=0, columnspan=8, padx=(10, 10), pady=5, sticky="w"
        )
        ttk.Label(linearity, text="文件").grid(row=2, column=8, padx=(4, 4), pady=5, sticky="w")
        ttk.Label(linearity, textvariable=self.linearity_path_var).grid(
            row=2, column=9, columnspan=5, padx=(0, 8), pady=5, sticky="w"
        )

        ttk.Label(linearity, text="CSV后处理").grid(row=3, column=0, padx=(10, 4), pady=5, sticky="w")
        ttk.Button(linearity, text="导入实验CSV", command=self._select_linearity_analysis_csv).grid(
            row=3, column=1, padx=4, pady=5, sticky="w"
        )
        ttk.Label(linearity, textvariable=self.linearity_analysis_csv_var).grid(
            row=3, column=2, columnspan=5, padx=(0, 8), pady=5, sticky="w"
        )
        ttk.Button(
            linearity,
            text="步骤3 影响系数",
            command=lambda: self._run_linearity_csv_analysis("impact"),
        ).grid(row=3, column=8, padx=4, pady=5, sticky="w")
        ttk.Button(
            linearity,
            text="步骤4 回差分析",
            command=lambda: self._run_linearity_csv_analysis("hysteresis"),
        ).grid(row=3, column=9, padx=4, pady=5, sticky="w")
        ttk.Button(
            linearity,
            text="步骤5 线性判断",
            command=lambda: self._run_linearity_csv_analysis("linearity"),
        ).grid(row=3, column=10, padx=4, pady=5, sticky="w")
        ttk.Button(linearity, text="复制Markdown", command=self._copy_linearity_analysis_markdown).grid(
            row=3, column=11, padx=4, pady=5, sticky="w"
        )
        ttk.Button(linearity, text="打开结果目录", command=self._open_linearity_analysis_output).grid(
            row=3, column=12, padx=4, pady=5, sticky="w"
        )
        ttk.Label(linearity, textvariable=self.linearity_analysis_status_var, wraplength=1200).grid(
            row=4, column=0, columnspan=14, padx=(10, 8), pady=(0, 5), sticky="w"
        )

        multi_input = ttk.LabelFrame(identification, text="多输入联合辨识实验")
        multi_input.grid(row=0, column=1, padx=(5, 0), sticky="nsew")
        for col in range(10):
            multi_input.columnconfigure(col, weight=0)
        multi_input.columnconfigure(9, weight=1)

        ttk.Label(multi_input, text="初始ABS(M00;M01;M02;M03;M04)").grid(
            row=0, column=0, padx=(10, 4), pady=4, sticky="w"
        )
        self.multi_input_state_entry = ttk.Entry(
            multi_input, textvariable=self.multi_input_state_var, width=32
        )
        self.multi_input_state_entry.grid(row=0, column=1, columnspan=4, padx=(0, 8), pady=4, sticky="ew")
        self.multi_input_start_button = ttk.Button(
            multi_input, text="步骤1 到位并记录", command=self._start_multi_input_experiment
        )
        self.multi_input_start_button.grid(row=0, column=5, padx=3, pady=4, sticky="w")
        self.multi_input_scan_button = ttk.Button(
            multi_input,
            text="步骤2 执行联合激励",
            command=self._start_multi_input_scan,
            state="disabled",
        )
        self.multi_input_scan_button.grid(row=0, column=6, columnspan=2, padx=3, pady=4, sticky="w")
        self.multi_input_stop_button = ttk.Button(
            multi_input,
            text="停止",
            command=lambda: self._stop_multi_input_experiment("手动停止"),
            state="disabled",
        )
        self.multi_input_stop_button.grid(row=0, column=8, padx=3, pady=4, sticky="w")

        ttk.Label(multi_input, text="模式").grid(row=1, column=0, padx=(10, 4), pady=4, sticky="w")
        self.multi_input_mode_combo = ttk.Combobox(
            multi_input,
            textvariable=self.multi_input_mode_var,
            values=("prbs", "random", "multisine"),
            width=10,
            state="readonly",
        )
        self.multi_input_mode_combo.grid(row=1, column=1, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="五路幅值counts").grid(row=1, column=2, padx=4, pady=4, sticky="w")
        self.multi_input_amplitudes_entry = ttk.Entry(
            multi_input, textvariable=self.multi_input_amplitudes_var, width=24
        )
        self.multi_input_amplitudes_entry.grid(row=1, column=3, columnspan=2, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="N").grid(row=1, column=5, padx=4, pady=4, sticky="w")
        self.multi_input_count_spin = tk.Spinbox(
            multi_input, from_=30, to=5000, increment=10, textvariable=self.multi_input_count_var, width=6
        )
        self.multi_input_count_spin.grid(row=1, column=6, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="seed").grid(row=1, column=7, padx=4, pady=4, sticky="w")
        self.multi_input_seed_entry = ttk.Entry(multi_input, textvariable=self.multi_input_seed_var, width=10)
        self.multi_input_seed_entry.grid(row=1, column=8, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(multi_input, text="到位停留s").grid(row=2, column=0, padx=(10, 4), pady=4, sticky="w")
        self.multi_input_hold_entry = ttk.Entry(multi_input, textvariable=self.multi_input_hold_var, width=6)
        self.multi_input_hold_entry.grid(row=2, column=1, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="终点保持s").grid(row=2, column=2, padx=4, pady=4, sticky="w")
        self.multi_input_post_hold_entry = ttk.Entry(
            multi_input, textvariable=self.multi_input_post_hold_var, width=6
        )
        self.multi_input_post_hold_entry.grid(row=2, column=3, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="超时s").grid(row=2, column=4, padx=4, pady=4, sticky="w")
        self.multi_input_timeout_entry = ttk.Entry(
            multi_input, textvariable=self.multi_input_timeout_var, width=6
        )
        self.multi_input_timeout_entry.grid(row=2, column=5, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="记录ms").grid(row=2, column=6, padx=4, pady=4, sticky="w")
        self.multi_input_record_interval_spin = tk.Spinbox(
            multi_input,
            from_=50,
            to=2000,
            increment=50,
            textvariable=self.multi_input_record_interval_var,
            width=6,
        )
        self.multi_input_record_interval_spin.grid(row=2, column=7, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(multi_input, text="训练;验证;测试").grid(
            row=3, column=0, padx=(10, 4), pady=4, sticky="w"
        )
        self.multi_input_split_entry = ttk.Entry(multi_input, textvariable=self.multi_input_split_var, width=15)
        self.multi_input_split_entry.grid(row=3, column=1, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="岭回归λ").grid(row=3, column=2, padx=4, pady=4, sticky="w")
        self.multi_input_lambda_entry = ttk.Entry(multi_input, textvariable=self.multi_input_lambda_var, width=8)
        self.multi_input_lambda_entry.grid(row=3, column=3, padx=(0, 8), pady=4, sticky="w")
        ttk.Label(multi_input, text="多正弦Hz").grid(row=3, column=4, padx=4, pady=4, sticky="w")
        self.multi_input_frequencies_entry = ttk.Entry(
            multi_input, textvariable=self.multi_input_frequencies_var, width=31
        )
        self.multi_input_frequencies_entry.grid(row=3, column=5, columnspan=4, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(multi_input, textvariable=self.multi_input_status_var, wraplength=850).grid(
            row=4, column=0, columnspan=10, padx=(10, 8), pady=4, sticky="w"
        )
        ttk.Label(multi_input, text="文件").grid(row=5, column=0, padx=(10, 4), pady=4, sticky="w")
        ttk.Label(multi_input, textvariable=self.multi_input_path_var, wraplength=760).grid(
            row=5, column=1, columnspan=9, padx=(0, 8), pady=4, sticky="w"
        )
        self.multi_input_analyze_button = ttk.Button(
            multi_input, text="步骤3 岭回归与验证", command=self._run_multi_input_analysis
        )
        self.multi_input_analyze_button.grid(row=6, column=0, columnspan=2, padx=(10, 4), pady=4, sticky="w")
        ttk.Button(
            multi_input, text="复制Markdown", command=self._copy_multi_input_analysis_markdown
        ).grid(row=6, column=2, columnspan=2, padx=4, pady=4, sticky="w")
        ttk.Button(
            multi_input, text="打开结果目录", command=self._open_multi_input_analysis_output
        ).grid(row=6, column=4, columnspan=2, padx=4, pady=4, sticky="w")
        ttk.Label(multi_input, textvariable=self.multi_input_analysis_status_var, wraplength=850).grid(
            row=7, column=0, columnspan=10, padx=(10, 8), pady=(0, 5), sticky="w"
        )

        log = ttk.LabelFrame(self, text="记录")
        log.grid(row=7, column=0, padx=10, pady=(6, 10), sticky="nsew")
        log.columnconfigure(0, weight=1)
        log.rowconfigure(1, weight=1)
        buttons = ttk.Frame(log)
        buttons.grid(row=0, column=0, sticky="ew")
        ttk.Button(buttons, text="清空", command=self._clear_samples).pack(side="right", padx=4, pady=6)
        ttk.Button(buttons, text="导出 CSV", command=self._export_csv).pack(side="right", padx=4, pady=6)

        self.table = ttk.Treeview(
            log, columns=("time", "channel", "motor", "value", "relative", "response"), show="headings"
        )
        self.table.heading("time", text="时间")
        self.table.heading("channel", text="通道")
        self.table.heading("motor", text="电机")
        self.table.heading("value", text="当前值")
        self.table.heading("relative", text="相对力(N)")
        self.table.heading("response", text="响应帧")
        self.table.column("time", width=90, anchor="center")
        self.table.column("channel", width=70, anchor="center")
        self.table.column("motor", width=70, anchor="center")
        self.table.column("value", width=120, anchor="e")
        self.table.column("relative", width=120, anchor="e")
        self.table.column("response", width=470, anchor="w")
        yscroll = ttk.Scrollbar(log, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=yscroll.set)
        self.table.grid(row=1, column=0, sticky="nsew")
        yscroll.grid(row=1, column=1, sticky="ns")

    def _refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _open_encoder_servo_window(self) -> None:
        if self.feedback_pretension_active:
            self._stop_nullspace_pretension("window closed", hold_bias=False)
        if self.encoder_servo_window is not None and self.encoder_servo_window.winfo_exists():
            self.encoder_servo_window.lift()
            self.encoder_servo_window.focus_set()
            return
        self.encoder_servo_window = EncoderServoWindow(
            self,
            self.default_servo_port,
            self.default_servo_baudrate,
        )

    def _toggle_connection(self) -> None:
        if self.poller is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            slave_id = int(self.slave_var.get())
            channel = int(self.channel_var.get())
            interval_ms = int(self.interval_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "请检查波特率、从站地址和轮询间隔。")
            return
        if not port:
            messagebox.showwarning("缺少串口", "请输入串口号，例如 COM7。")
            return

        self.stop_event.clear()
        self.poller = SensorPoller(
            self.outbox,
            self.command_queue,
            self.stop_event,
            port,
            baud,
            slave_id,
            channel,
            interval_ms,
        )
        self.poller.start()
        self._set_controls_connected(True)

    def _disconnect(self) -> None:
        self._cancel_force_zero_capture()
        window = self._active_encoder_servo_window()
        if window is not None and window.step_test_running:
            window._stop_step_test("力传感器串口断开", send_stop=True)
        if window is not None and window.sine_test_running:
            window._stop_sine_test("力传感器串口断开", send_stop=True)
        if self.force_emergency_relax_active:
            self._finish_force_emergency_relax(
                "force sensor disconnected during all-motor release",
                recovered=False,
            )
        if self.feedback_pretension_active:
            window = self._active_encoder_servo_window()
            if window is not None and window.poller is not None:
                window.queue_emergency_stop()
            self._stop_nullspace_pretension("force sensor disconnected", hold_bias=False)
        if self.force_target_running:
            self._stop_force_target_control("force sensor disconnected")
        if self.linearity_running:
            self._stop_linearity_experiment("force sensor disconnected")
        if self.multi_input_running:
            self._stop_multi_input_experiment("force sensor disconnected")
        if self.degree_force_active:
            self._stop_degree_force_control("force sensor disconnected")
        if self.safe_relax_active:
            self._stop_safe_relax("力传感器已断开")
        if self.training_running:
            self._stop_training_collection("力传感器已断开")
        if self.continuous_record_running:
            self._stop_continuous_record("力传感器已断开")
        self.stop_event.set()
        poller = self.poller
        self.poller = None
        if poller is not None:
            poller.join(timeout=1.5)
        self._set_controls_connected(False)

    def _set_controls_connected(self, connected: bool) -> None:
        self.connected = connected
        self.connect_button.configure(text="断开" if connected else "连接")
        state = "disabled" if connected else "normal"
        for widget in (
            self.port_combo,
            self.baud_combo,
            self.slave_spin,
            self.channel_spin,
            self.interval_spin,
            self.refresh_button,
        ):
            widget.configure(state=state)

    def _process_outbox(self) -> None:
        try:
            while True:
                kind, payload = self.outbox.get_nowait()
                if kind == "sample":
                    self._handle_sample(payload)
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self.status_var.set(str(payload))
                elif kind == "frame":
                    label, request_hex, response_hex = payload
                    self.request_var.set(f"{label}: {request_hex}")
                    self.response_var.set(response_hex)
                elif kind == "diagnostic":
                    info = payload
                    self.diagnostic_var.set(
                        "诊断: {label}, 重量={weight}, 低字=0x{low:04X}, 高字=0x{high:04X}".format(
                            label=info["label"],
                            weight=info["weight"],
                            low=info["low"],
                            high=info["high"],
                        )
                    )
                elif kind == "device_tare":
                    self._handle_device_tare_result(int(payload))
                elif kind == "connected":
                    self._set_controls_connected(bool(payload))
                    if not payload:
                        self._cancel_force_zero_capture()
                        self.poller = None
                        if self.safe_relax_active:
                            self._stop_safe_relax("力传感器已断开")
                        if self.degree_force_active:
                            self._stop_degree_force_control("力传感器已断开", send_stop=False)
                        if self.linearity_running:
                            self._stop_linearity_experiment("力传感器已断开", send_cleanup=False)
                        if self.multi_input_running:
                            self._stop_multi_input_experiment("力传感器已断开", send_cleanup=False)
                        if self.training_running:
                            self._stop_training_collection("力传感器已断开")
                        if self.continuous_record_running:
                            self._stop_continuous_record("力传感器已断开")
        except queue.Empty:
            pass
        self.after(50, self._process_outbox)

    def _queue_tare_command(self, value: int) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接传感器。")
            return
        block_reason = self._force_zero_block_reason()
        if block_reason:
            messagebox.showwarning(
                "当前不能去皮",
                f"{block_reason}。请停止运动/采集，并确认五路传感器完全无外力后再操作。",
                parent=self,
            )
            return
        slave_id = int(self.slave_var.get())
        request = build_write_single_register(slave_id, 0x0015, value)
        label = "设备去皮" if value == 1 else "取消去皮"
        self.command_queue.put((label, request, 8))
        self.request_var.set(f"{label}: {format_hex(request)}")
        self.response_var.set("--")

    def _force_zero_block_reason(self) -> Optional[str]:
        if self._automatic_angle_test_is_running():
            return "自动阶跃/正弦实验正在运行"
        checks = (
            ("force_target_running", "力目标控制正在运行"),
            ("degree_force_active", "角度力反馈正在运行"),
            ("safe_relax_active", "安全放松正在运行"),
            ("linearity_running", "局部线性辨识正在运行"),
            ("multi_input_running", "多输入辨识正在运行"),
            ("training_running", "训练数据采集正在运行"),
            ("continuous_record_running", "连续记录正在运行"),
        )
        for attribute, message in checks:
            if bool(getattr(self, attribute, False)):
                return message
        if self.force_zero_capture_active:
            return "无载校零已经在进行"
        return None

    def _handle_device_tare_result(self, value: int) -> None:
        if value == 1:
            # Device tare changes the raw origin. A saved host baseline from the
            # previous raw origin must never be subtracted again.
            self.baselines = {channel: 0 for channel in DISPLAY_CHANNELS}
            try:
                save_force_zero_baselines(self.baselines)
            except Exception as exc:
                self.status_var.set(f"设备去皮成功，但保存上位机零位失败: {exc}")
                return
            self.zero_status_var.set(self._zero_status_text())
            self._reset_max_tension()
            self._refresh_relative_displays()
            self.status_var.set("设备去皮成功；等待新读数后将自动做2秒残余校零")
            self.after(300, self._set_baseline)
            return

        self._cancel_force_zero_capture()
        self.baselines.clear()
        try:
            delete_force_zero_baselines()
        except Exception as exc:
            self.status_var.set(f"取消设备去皮成功，但清除上位机零位失败: {exc}")
            return
        self.zero_status_var.set(self._zero_status_text())
        self._reset_max_tension()
        self._refresh_relative_displays()
        self.status_var.set("已取消设备去皮；原零位已失效，请在无载时重新校零")

    def _queue_diagnostic_read(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接传感器。")
            return
        slave_id = int(self.slave_var.get())
        channel = int(self.channel_var.get())
        start_register = weight_register_for_channel(channel)
        request = build_read_holding_registers(slave_id, start_register, 2)
        label = f"诊断读取CH{channel}"
        self.command_queue.put((label, request, 9))
        self.request_var.set(f"{label}: {format_hex(request)}")
        self.response_var.set("--")

    def _active_encoder_servo_window(self) -> Optional[EncoderServoWindow]:
        window = self.encoder_servo_window
        if window is None:
            return None
        try:
            if not window.winfo_exists():
                return None
        except tk.TclError:
            return None
        return window

    def _step_test_is_running(self) -> bool:
        window = self._active_encoder_servo_window()
        return bool(window is not None and window.step_test_running)

    def _sine_test_is_running(self) -> bool:
        window = self._active_encoder_servo_window()
        return bool(window is not None and window.sine_test_running)

    def _automatic_angle_test_is_running(self) -> bool:
        return self._step_test_is_running() or self._sine_test_is_running()

    def _enabled_force_target_channels(self) -> list[int]:
        return [
            channel
            for channel in DISPLAY_CHANNELS
            if bool(self.force_target_enabled_vars[channel].get())
            and FORCE_CHANNEL_TO_MOTOR.get(channel) is not None
        ]

    def _force_target_current_n(self, channel: int) -> Optional[float]:
        sample = self.latest_by_channel.get(channel)
        if sample is None:
            return None
        return self._relative_value(sample.value, channel) / RELATIVE_UNITS_PER_NEWTON

    def _refresh_force_target_current_display(self, channel: int) -> None:
        force_n = self._force_target_current_n(channel)
        if force_n is None:
            self.force_target_current_vars[channel].set("--")
        else:
            self.force_target_current_vars[channel].set(f"{force_n:.1f}")

    def _parse_force_target_settings(
        self,
        channels: Optional[list[int]] = None,
    ) -> tuple[int, int, float, dict[int, float]]:
        try:
            interval_ms = int(self.force_target_interval_var.get())
            far_step_counts = int(self.force_target_step_var.get())
            deadband_n = abs(float(self.force_target_deadband_var.get().strip()))
        except (tk.TclError, ValueError) as exc:
            raise ValueError("Force loop interval, far step, and deadband must be numbers.") from exc
        if interval_ms < 20:
            raise ValueError("Force loop interval must be at least 20 ms.")
        if far_step_counts <= 0:
            raise ValueError("Force loop far step counts must be greater than 0.")

        target_channels = channels if channels is not None else self._enabled_force_target_channels()
        targets: dict[int, float] = {}
        for channel in target_channels:
            if channel not in DISPLAY_CHANNELS or FORCE_CHANNEL_TO_MOTOR.get(channel) is None:
                raise ValueError(f"CH{channel} has no motor mapping.")
            try:
                targets[channel] = float(self.force_target_vars[channel].get().strip())
            except (tk.TclError, ValueError) as exc:
                raise ValueError(f"CH{channel} target must be a number.") from exc
        if not targets:
            raise ValueError("Enable at least one force channel.")
        return interval_ms, far_step_counts, deadband_n, targets

    def _set_force_target_controls_running(self, running: bool) -> None:
        self.force_target_start_button.configure(state="disabled" if running else "normal")
        self.force_target_stop_button.configure(state="normal" if running else "disabled")
        if hasattr(self, "feedback_pretension_button"):
            if self.feedback_pretension_active:
                self.feedback_pretension_button.configure(
                    text="停止反馈前预紧",
                    state="normal",
                )
            else:
                self.feedback_pretension_button.configure(
                    text="反馈前预紧（-15~-10 N）",
                    state="disabled" if running else "normal",
                )

        if hasattr(self, "feedback_pretension_button"):
            label = (
                "停止零空间预紧"
                if self.feedback_pretension_active
                else "零空间预紧（安全整改后默认禁用）"
            )
            self.feedback_pretension_button.configure(
                text=label,
                state="disabled" if running and not self.feedback_pretension_active else "normal",
            )

    def _toggle_feedback_pretension(self) -> None:
        if self.feedback_pretension_active:
            self._stop_force_target_control("manual pretension stop")
            return
        if self.force_target_running:
            messagebox.showinfo(
                "力控制正在运行",
                "请先停止当前 Force Target Control，再启动反馈前预紧。",
            )
            return

        for channel in DISPLAY_CHANNELS:
            self.force_target_enabled_vars[channel].set(True)
            self.force_target_vars[channel].set(f"{FEEDBACK_PRETENSION_TARGET_N:.1f}")
        self.force_target_interval_var.set(FEEDBACK_PRETENSION_INTERVAL_MS)
        self.force_target_step_var.set(FEEDBACK_PRETENSION_STEP_COUNTS)
        self.force_target_deadband_var.set(f"{FEEDBACK_PRETENSION_DEADBAND_N:.1f}")

        self.feedback_pretension_active = True
        self.force_target_status_var.set(
            "反馈前预紧准备中：五路目标范围 -15~-10 N"
        )
        self._start_force_target_control(list(DISPLAY_CHANNELS))
        if not self.force_target_running:
            self.feedback_pretension_active = False
            self._set_force_target_controls_running(False)
        else:
            self.feedback_pretension_started_monotonic = time.monotonic()

    def _apply_nullspace_pretension_bias(
        self,
        window: EncoderServoWindow,
        *,
        force: bool = False,
    ) -> bool:
        biases = nullspace_pretension_bias_counts(
            self.nullspace_pretension_alpha_counts
        )
        if not force and biases == self.nullspace_pretension_bias_by_motor:
            return False
        self.nullspace_pretension_bias_by_motor = dict(biases)
        self.degree_force_bias_by_motor = dict(biases)
        self.training_tension_bias_by_motor = dict(biases)
        window.queue_text_command("tension on")
        for motor in range(5):
            window.queue_text_command(f"tensionbias m{motor} {biases[motor]}")
        return True

    def _stop_nullspace_pretension(self, reason: str = "", *, hold_bias: bool = True) -> None:
        was_active = self.feedback_pretension_active
        self.feedback_pretension_active = False
        self.feedback_pretension_started_monotonic = 0.0
        self.nullspace_pretension_next_step_monotonic = 0.0
        window = self._active_encoder_servo_window()
        if not hold_bias:
            self.nullspace_pretension_alpha_counts = 0.0
            self.nullspace_pretension_bias_by_motor = {motor: 0 for motor in range(5)}
            self.degree_force_bias_by_motor = dict(self.nullspace_pretension_bias_by_motor)
            self.training_tension_bias_by_motor = dict(self.nullspace_pretension_bias_by_motor)
            if window is not None and window.poller is not None:
                window.queue_text_command("tension off")
                for motor in range(5):
                    window.queue_text_command(f"tensionbias m{motor} 0")
        self._set_force_target_controls_running(self.force_target_running)
        suffix = f": {reason}" if reason else ""
        if was_active or reason:
            hold_text = "保持当前零空间偏置" if hold_bias else "已清零零空间偏置"
            biases = "/".join(
                str(self.nullspace_pretension_bias_by_motor[motor]) for motor in range(5)
            )
            self.force_target_status_var.set(
                f"degree零空间张力调节已停止{suffix}; {hold_text}; "
                f"alpha={self.nullspace_pretension_alpha_counts:.0f}; bias={biases}"
            )

    def _start_or_refresh_nullspace_tension_control(
        self,
        window: EncoderServoWindow,
        snapshot: Optional[EncoderServoSnapshot],
    ) -> bool:
        """Attach scalar null-space tension regulation to a degree PI run.

        This is deliberately non-blocking for ordinary angle control: if the
        five force channels are not ready, degree control still starts with
        zero tension bias.  Once active, loss of those data is safety-critical
        and the periodic tick stops the controller.
        """
        if not NULLSPACE_PRETENSION_AUTOSTART_ENABLED:
            if self.feedback_pretension_active:
                self._stop_nullspace_pretension(
                    "automatic null-space pretension disabled after over-tension event",
                    hold_bias=False,
                )
            return False
        if self.feedback_pretension_active:
            self._apply_nullspace_pretension_bias(window, force=True)
            return True
        if self.force_emergency_relax_active:
            return False
        if self.poller is None or window.poller is None or snapshot is None:
            return False
        if self.force_target_running or self.safe_relax_active:
            return False
        if self.linearity_running or self.multi_input_running or self.training_running:
            return False

        now_wall = time.time()
        for channel in DISPLAY_CHANNELS:
            sample = self.latest_by_channel.get(channel)
            if sample is None or now_wall - sample.timestamp > 1.5:
                return False
        for motor in range(5):
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]):
                return False

        # The old M02/M04 force-feedback loop is a different controller and
        # must not alter the same tension-bias inputs concurrently.
        if self.degree_force_active:
            self._stop_degree_force_control(
                "integrated null-space tension control requested",
                send_stop=False,
            )
        self.degree_force_enabled_var.set(False)
        self.feedback_pretension_active = True
        self.feedback_pretension_started_monotonic = time.monotonic()
        self.nullspace_pretension_alpha_counts = 0.0
        self.nullspace_pretension_bias_by_motor = {motor: 0 for motor in range(5)}
        self.nullspace_pretension_next_step_monotonic = 0.0
        self.nullspace_pretension_move_count = 0
        self._set_force_target_controls_running(False)
        self._apply_nullspace_pretension_bias(window, force=True)
        self.force_target_status_var.set(
            "degree反馈已启用零空间张力分配：按拉力幅值5~90N"
            "（传感器-5~-90N）解析分配统一的alpha，不含单路收紧；"
            "-90N硬保护并放松至 >-70N"
        )
        self.after(NULLSPACE_PRETENSION_INTERVAL_MS, self._nullspace_pretension_tick)
        return True

    def _toggle_nullspace_pretension(self) -> None:
        if not NULLSPACE_PRETENSION_AUTOSTART_ENABLED:
            window = self._active_encoder_servo_window()
            if self.feedback_pretension_active:
                self._stop_nullspace_pretension(
                    "null-space pretension disabled after over-tension event",
                    hold_bias=False,
                )
            elif window is not None and window.poller is not None:
                self.nullspace_pretension_alpha_counts = 0.0
                self._apply_nullspace_pretension_bias(window, force=True)
                window.queue_text_command("tension off")
            messagebox.showwarning(
                "零空间预紧已禁用",
                "当前版本默认禁止零空间预紧；请先烧录并低张力验证新的绝对目标实现。",
                parent=self,
            )
            return
        if self.feedback_pretension_active:
            self._stop_nullspace_pretension("manual stop", hold_bias=False)
            return
        if self.force_target_running:
            messagebox.showinfo("力控制正在运行", "请先停止 direct 力控制。")
            return
        if self.linearity_running or self.multi_input_running or self.training_running:
            messagebox.showinfo("实验正在运行", "请先停止当前辨识或采集实验。")
            return
        if self.safe_relax_active:
            messagebox.showinfo("安全放松正在运行", "请先停止安全放松。")
            return
        if self.degree_force_active:
            self._stop_degree_force_control("null-space pretension requested", send_stop=False)
        self.degree_force_enabled_var.set(False)
        if self.poller is None:
            messagebox.showinfo("力传感器未连接", "请先连接力传感器串口。")
            return

        now_wall = time.time()
        missing_force: list[str] = []
        for channel in DISPLAY_CHANNELS:
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                missing_force.append(f"CH{channel}: no data")
            elif now_wall - sample.timestamp > 1.5:
                missing_force.append(f"CH{channel}: stale")
        if missing_force:
            messagebox.showinfo("等待五路力数据", ", ".join(missing_force))
            return

        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("Servo 未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待 Servo 数据", "请等待第一帧 Encoder / Servo 数据。")
            return
        invalid_joints = [joint for joint in ENCODER_PLOT_JOINTS if not snapshot.encoder_valid[joint]]
        offline_motors = [
            motor
            for motor in range(5)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if invalid_joints or offline_motors:
            details: list[str] = []
            if invalid_joints:
                details.append("invalid " + ", ".join(f"J{joint:02d}" for joint in invalid_joints))
            if offline_motors:
                details.append("offline " + ", ".join(f"M{motor:02d}" for motor in offline_motors))
            messagebox.showwarning("零空间预紧条件不满足", "; ".join(details))
            return

        # Pretension around the pose that exists at the instant the button is
        # pressed.  The regular angle PI loop then rejects second-order drift.
        for joint in ENCODER_PLOT_JOINTS:
            window.sent_degree_targets[joint] = float(snapshot.encoder_deg[joint])
        window.mode_var.set("degree")
        window.sent_last_target_command = "null-space pretension hold current pose"
        window.sent_target_var.set(window._sent_target_status_text())

        self.feedback_pretension_active = True
        self.feedback_pretension_started_monotonic = time.monotonic()
        self.nullspace_pretension_alpha_counts = 0.0
        self.nullspace_pretension_bias_by_motor = {motor: 0 for motor in range(5)}
        self.nullspace_pretension_next_step_monotonic = 0.0
        self.nullspace_pretension_move_count = 0
        self._set_force_target_controls_running(False)
        window._queue_start_with_host_state()
        self._apply_nullspace_pretension_bias(window, force=True)
        window.queue_text_command("status")
        self.force_target_status_var.set(
            "零空间张力分配启动：保持当前 J00-J03；按拉力幅值5~90N"
            "（传感器-5~-90N）解析分配统一的alpha，不含单路收紧；"
            "-90N硬保护并放松至 >-70N"
        )
        self.after(NULLSPACE_PRETENSION_INTERVAL_MS, self._nullspace_pretension_tick)

    def _nullspace_pretension_tick(self) -> None:
        if not self.feedback_pretension_active:
            return
        window = self._active_encoder_servo_window()
        if self.poller is None or window is None or window.poller is None:
            if window is not None and window.poller is not None:
                window.queue_emergency_stop()
            self._stop_nullspace_pretension("sensor or servo disconnected", hold_bias=False)
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension("servo snapshot unavailable", hold_bias=False)
            return
        if window.mode_var.get().strip().lower() != "degree":
            window.queue_emergency_stop()
            self._stop_nullspace_pretension("control mode left degree", hold_bias=False)
            return
        offline_motors = [
            motor
            for motor in range(5)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline_motors:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension(
                "offline " + ", ".join(f"M{motor:02d}" for motor in offline_motors),
                hold_bias=False,
            )
            return

        now_wall = time.time()
        force_by_motor: dict[int, float] = {}
        for channel in DISPLAY_CHANNELS:
            motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
            sample = self.latest_by_channel.get(channel)
            if motor is None or sample is None or now_wall - sample.timestamp > 1.5:
                window.queue_emergency_stop()
                self._stop_nullspace_pretension(
                    f"CH{channel} force data stale", hold_bias=False
                )
                return
            force_n = self._force_target_current_n(channel)
            if force_n is None or not math.isfinite(force_n):
                window.queue_emergency_stop()
                self._stop_nullspace_pretension(
                    f"CH{channel} force invalid", hold_bias=False
                )
                return
            force_by_motor[motor] = float(force_n)

        if len(force_by_motor) != 5:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension(
                "incomplete force-to-motor mapping", hold_bias=False
            )
            return
        try:
            action, loosest, tightest = nullspace_pretension_action(force_by_motor)
        except ValueError as exc:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension(str(exc), hold_bias=False)
            return

        if action == "release":
            # Do not spend several 100 ms cycles ramping alpha down while the
            # motors are still following a tighter target.  Safety takes over:
            # clear every bias and command all motors below their current ABS.
            tightest_motor = min(force_by_motor, key=force_by_motor.get)
            tightest_channel = next(
                channel
                for channel, motor in FORCE_CHANNEL_TO_MOTOR.items()
                if motor == tightest_motor
            )
            self._start_force_emergency_relax(
                tightest_channel,
                tightest,
                trigger_limit_n=NULLSPACE_PRETENSION_RELEASE_N,
            )
            return

        now = time.monotonic()
        try:
            allocation = allocate_nullspace_internal_tension(force_by_motor)
        except ValueError as exc:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension(str(exc), hold_bias=False)
            return
        if not allocation.feasible:
            window.queue_emergency_stop()
            self._stop_nullspace_pretension(
                "Robonaut式张力分配不可行："
                f"alpha_min={allocation.alpha_min_n:.2f}N > "
                f"alpha_max={allocation.alpha_max_n:.2f}N；"
                "当前关节任务与拉力幅值f_min=5N/f_max=90N"
                "（传感器读数-5N/-90N）冲突",
                hold_bias=False,
            )
            return

        alpha_force_error_n = (
            allocation.alpha_desired_n - allocation.alpha_measured_n
        )
        alpha_step_counts = 0
        changed = False
        if now >= self.nullspace_pretension_next_step_monotonic:
            alpha_step_counts = nullspace_alpha_count_step(alpha_force_error_n)
            old_alpha = self.nullspace_pretension_alpha_counts
            self.nullspace_pretension_alpha_counts = max(
                0.0,
                min(
                    NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS,
                    old_alpha + alpha_step_counts,
                ),
            )
            changed = self.nullspace_pretension_alpha_counts != old_alpha
            if changed:
                self.nullspace_pretension_move_count += 1
                self._apply_nullspace_pretension_bias(window)
            self.nullspace_pretension_next_step_monotonic = (
                now + NULLSPACE_PRETENSION_INTERVAL_MS / 1000.0
            )

        biases = "/".join(
            str(self.nullspace_pretension_bias_by_motor[motor]) for motor in range(5)
        )
        if alpha_step_counts > 0:
            allocation_action = "增加alpha"
        elif alpha_step_counts < 0:
            allocation_action = "减小alpha"
        else:
            allocation_action = "保持alpha"
        limit_text = " count-limit" if (
            (alpha_force_error_n > NULLSPACE_ALPHA_FORCE_DEADBAND_N
             and self.nullspace_pretension_alpha_counts >= NULLSPACE_PRETENSION_ALPHA_LIMIT_COUNTS)
            or (alpha_force_error_n < -NULLSPACE_ALPHA_FORCE_DEADBAND_N
                and self.nullspace_pretension_alpha_counts <= 0.0)
        ) else ""
        self.force_target_status_var.set(
            f"degree解析张力分配 {allocation_action}{limit_text}: "
            f"alphaF={allocation.alpha_measured_n:.2f}->{allocation.alpha_desired_n:.2f}N "
            f"[{allocation.alpha_min_n:.2f},{allocation.alpha_max_n:.2f}]; "
            f"force={loosest:.1f}..{tightest:.1f}N; "
            f"alphaCounts={self.nullspace_pretension_alpha_counts:.0f}; "
            f"bias={biases}; moves={self.nullspace_pretension_move_count}"
        )
        self.after(NULLSPACE_PRETENSION_INTERVAL_MS, self._nullspace_pretension_tick)

    def _force_target_step_delta(
        self,
        error_n: float,
        *,
        far_step_counts: int,
        deadband_n: float,
    ) -> tuple[int, str]:
        if self.feedback_pretension_active:
            if abs(error_n) <= deadband_n:
                delta_counts = 0
            else:
                delta_counts = -far_step_counts if error_n > 0 else far_step_counts
        else:
            delta_counts = force_target_step_delta_from_error(error_n, far_step_counts, deadband_n)
        if delta_counts == 0:
            return 0, "hold"
        return delta_counts, f"step={abs(delta_counts)} e={error_n:.1f}"

    def _force_target_one_step(
        self,
        window: EncoderServoWindow,
        snapshot: EncoderServoSnapshot,
        channel: int,
        target_n: float,
        far_step_counts: int,
        deadband_n: float,
    ) -> tuple[bool, str]:
        motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
        if motor is None:
            return False, f"CH{channel} has no motor mapping"
        online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
        if not online:
            return False, f"M{motor:02d} offline"

        current_n = self._force_target_current_n(channel)
        if current_n is None:
            return False, f"CH{channel} has no force data"

        error_n = target_n - current_n
        current_abs = int(snapshot.servo_abs[motor])
        previous_target = int(self.force_target_last_targets.get(motor, current_abs))
        delta, step_text = self._force_target_step_delta(
            error_n,
            far_step_counts=far_step_counts,
            deadband_n=deadband_n,
        )
        if delta < 0:
            base = min(current_abs, previous_target)
        elif delta > 0:
            base = max(current_abs, previous_target)
        else:
            base = current_abs
            self.force_target_last_targets[motor] = int(base)
            self.force_target_motor_target_vars[channel].set(str(int(base)))
            return True, f"CH{channel}/M{motor:02d} hold {current_n:.1f}/{target_n:.1f}N"

        target_abs = max(FORCE_TARGET_MIN_ABS, min(FORCE_TARGET_MAX_ABS, int(base) + int(delta)))
        self.force_target_last_targets[motor] = target_abs
        self.force_target_motor_target_vars[channel].set(str(target_abs))
        window.queue_text_command(f"direct; m{motor} {target_abs}")
        self.force_target_move_count += 1
        return True, f"CH{channel}/M{motor:02d} {current_n:.1f}->{target_n:.1f}N {step_text} target={target_abs}"

    def _reset_host_control_state_for_zero(self) -> None:
        if self.feedback_pretension_active:
            self._stop_nullspace_pretension("ZERO command requested", hold_bias=False)
        if self.linearity_running:
            self._stop_linearity_experiment("ZERO command requested", send_cleanup=False)
        if self.multi_input_running:
            self._stop_multi_input_experiment("ZERO command requested", send_cleanup=False)
        if self.force_target_running:
            self._stop_force_target_control("ZERO command requested")
        if self.safe_relax_active:
            self._stop_safe_relax("ZERO command requested", send_stop=False)
        if self.training_running:
            self._stop_training_collection("ZERO command requested")
        if self.degree_force_active:
            self._stop_degree_force_control("ZERO command requested", send_stop=False)

        zero_bias = {motor: 0 for motor in training_target_motor_indices()}
        self.degree_force_bias_by_motor = dict(zero_bias)
        self.training_tension_bias_by_motor = dict(zero_bias)
        self.pretension_last_targets.clear()
        self.force_target_last_targets.clear()
        self.safe_relax_last_targets.clear()

    def _queue_direct_force_mode(self, window: EncoderServoWindow) -> None:
        for motor, target in self.force_target_last_targets.items():
            window.sent_direct_targets[motor] = int(target)
        targets = window._current_host_direct_targets()
        window.queue_text_command("tension off")
        window.queue_text_command("direct")
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(targets),
            label="direct force START snapshot",
        )
        window.queue_text_command("start")

    def _clear_tension_bias_for_angle_start(self, window: EncoderServoWindow) -> None:
        """Disable and clear stale host tension bias before a new angle run."""
        if self.degree_force_active:
            self._stop_degree_force_control("new angle control", send_stop=False)
        if self.feedback_pretension_active:
            # The null-space bias is part of angle control, not stale state.
            # Re-issue it because START/target refresh may race the periodic
            # controller, but do not reset the scalar co-contraction command.
            self._apply_nullspace_pretension_bias(window, force=True)
            return
        zero_bias = {motor: 0 for motor in training_target_motor_indices()}
        self.degree_force_bias_by_motor = dict(zero_bias)
        self.training_tension_bias_by_motor = dict(zero_bias)
        self.degree_force_tension_window_ok = False
        window.queue_text_command("tension off")
        for motor in training_target_motor_indices():
            window.queue_text_command(f"tensionbias m{motor} 0")

    def _set_degree_force_controls_active(self, active: bool) -> None:
        if hasattr(self, "degree_force_stop_button"):
            self.degree_force_stop_button.configure(state="normal" if active else "disabled")

    def _degree_force_angle_target_state(
        self,
        window: EncoderServoWindow,
        snapshot: EncoderServoSnapshot,
        tolerance_deg: float = DEGREE_FORCE_ANGLE_READY_DEG,
    ) -> tuple[bool, str]:
        targets = {
            joint: target
            for joint, target in window.sent_degree_targets.items()
            if joint in MANUAL_RECORD_JOINTS
        }
        if not targets:
            return False, "waiting for J00-J03 degree target"

        errors: list[tuple[int, float]] = []
        invalid_joints: list[int] = []
        for joint, target in sorted(targets.items()):
            if not snapshot.encoder_valid[joint]:
                invalid_joints.append(joint)
                continue
            errors.append((joint, float(snapshot.encoder_deg[joint]) - float(target)))

        if invalid_joints:
            return False, "invalid angle " + ", ".join(f"J{joint:02d}" for joint in invalid_joints)
        if not errors:
            return False, "no valid degree target angle"

        max_joint, max_error = max(errors, key=lambda item: abs(item[1]))
        close = abs(max_error) <= tolerance_deg
        details = ", ".join(f"J{joint:02d}err={error:+.2f}deg" for joint, error in errors)
        return close, f"dJmax=J{max_joint:02d} {max_error:+.2f}deg; {details}"

    def _preserved_tension_bias_by_motor(
        self,
        window: EncoderServoWindow,
        current: dict[int, int],
    ) -> dict[int, int]:
        preserved = dict(current)
        snapshot = window.latest_snapshot
        for motor in training_target_motor_indices():
            if motor in preserved:
                continue
            debug = snapshot.joint_debug.get(motor) if snapshot is not None else None
            preserved[motor] = int(debug.tension_bias_counts) if debug is not None else 0
        return preserved

    def _enter_degree_force_move_phase(self, window: EncoderServoWindow, reason: str = "") -> None:
        self.degree_force_phase = "move"
        self.degree_force_angle_ready_since_monotonic = 0.0
        self.degree_force_tension_window_ok = False
        self.degree_force_bias_by_motor = self._preserved_tension_bias_by_motor(
            window,
            self.degree_force_bias_by_motor,
        )
        self.degree_force_next_step_monotonic = time.monotonic() + DEGREE_FORCE_STEP_INTERVAL_S
        window.queue_text_command("tension on")
        window.queue_text_command("degree")
        suffix = f": {reason}" if reason else ""
        self.degree_force_status_var.set(f"Degree force move phase{suffix}; M02/M04 tension active")

    def _start_or_refresh_degree_force_control(
        self,
        window: EncoderServoWindow,
        snapshot: Optional[EncoderServoSnapshot],
    ) -> bool:
        if not bool(self.degree_force_enabled_var.get()):
            if self.degree_force_active:
                self._stop_degree_force_control("disabled")
            return False
        if self.training_running:
            self.degree_force_status_var.set("Degree force: skipped while training is running")
            return False
        if self.poller is None:
            self.degree_force_status_var.set("Degree force: force sensor disconnected")
            return False
        if snapshot is None:
            self.degree_force_status_var.set("Degree force: waiting for servo data")
            return False

        try:
            target_n, tight_limit_n, steps, _timeout_s = self._parse_pretension_settings()
        except ValueError as exc:
            self.degree_force_status_var.set(f"Degree force settings error: {exc}")
            return False

        actions = self._force_window_actions_for(target_n, tight_limit_n)
        if actions is None:
            self.degree_force_status_var.set("Degree force: waiting for fresh CH1-CH5 force data")
            return False

        missing_motors = [
            motor
            for motor in training_target_motor_indices()
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if missing_motors:
            self.degree_force_status_var.set(
                "Degree force: motor offline "
                + ", ".join(f"M{motor:02d}" for motor in missing_motors)
            )
            return False

        self.degree_force_target_n = target_n
        self.degree_force_tight_limit_n = tight_limit_n
        self.degree_force_steps = dict(steps)
        was_active = self.degree_force_active
        if not self.degree_force_active:
            self.degree_force_active = True
            self.degree_force_move_count = 0
            self._set_degree_force_controls_active(True)
            self._enter_degree_force_move_phase(window, "wait angle target")
            self.after(100, self._degree_force_tick)
        else:
            self._enter_degree_force_move_phase(window, "new degree target")
        if was_active:
            self.degree_force_status_var.set(f"Degree force move phase: new target; moves={self.degree_force_move_count}")
        return True

    def _stop_degree_force_control(self, reason: str = "", send_stop: bool = True) -> None:
        if not self.degree_force_active:
            self._set_degree_force_controls_active(False)
            self.degree_force_phase = "idle"
            self.degree_force_tension_window_ok = False
            return
        self.degree_force_active = False
        self.degree_force_phase = "idle"
        self.degree_force_angle_ready_since_monotonic = 0.0
        self.degree_force_tension_window_ok = False
        self._set_degree_force_controls_active(False)
        window = self._active_encoder_servo_window()
        if send_stop and window is not None and window.poller is not None:
            window.queue_text_command("tension off")
        suffix = f": {reason}" if reason else ""
        self.degree_force_status_var.set(f"Degree force stopped{suffix}; moves={self.degree_force_move_count}")

    def _degree_force_tick(self) -> None:
        if not self.degree_force_active:
            return
        if self.poller is None:
            self._stop_degree_force_control("force sensor disconnected", send_stop=False)
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_degree_force_control("Encoder / Servo disconnected", send_stop=False)
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(100, self._degree_force_tick)
            return

        now = time.monotonic()
        angle_close, angle_text = self._degree_force_angle_target_state(window, snapshot)

        if self.degree_force_phase != "tension":
            self.degree_force_tension_window_ok = False
            move_actions = self._force_window_actions_for(
                self.degree_force_target_n,
                self.degree_force_tight_limit_n,
                motors=PRETENSION_ALWAYS_TENSION_MOTORS,
            )
            if move_actions is None:
                self.degree_force_status_var.set(
                    f"Degree force move phase: waiting for fresh M02/M04 force data; {angle_text}"
                )
                self.after(max(50, int(DEGREE_FORCE_STEP_INTERVAL_S * 500)), self._degree_force_tick)
                return
            move_text = (
                "; ".join(
                    f"{label}:CH{channel}/M{motor:02d}={force_n:.1f}N"
                    for channel, motor, force_n, _direction, label in move_actions
                )
                if move_actions
                else "M02/M04 in negative tension windows"
            )
            if now >= self.degree_force_next_step_monotonic:
                if move_actions:
                    ok, error, action_texts = self._apply_tension_bias_actions(
                        window,
                        snapshot,
                        move_actions,
                        self.degree_force_steps,
                        self.degree_force_bias_by_motor,
                        "degree-move",
                    )
                    if not ok:
                        self._stop_degree_force_control(error)
                        return
                    self.degree_force_move_count += len(move_actions)
                    move_text = "; ".join(action_texts)
                self.degree_force_next_step_monotonic = now + DEGREE_FORCE_STEP_INTERVAL_S
            if angle_close:
                if self.degree_force_angle_ready_since_monotonic <= 0.0:
                    self.degree_force_angle_ready_since_monotonic = now
                held_s = now - self.degree_force_angle_ready_since_monotonic
                if held_s >= DEGREE_FORCE_ANGLE_HOLD_S:
                    self.degree_force_phase = "tension"
                    self.degree_force_angle_ready_since_monotonic = 0.0
                    self.degree_force_next_step_monotonic = 0.0
                    window.queue_text_command("tension on")
                    self.degree_force_status_var.set(
                        f"Degree force tension phase: angle ready {held_s:.1f}s; {angle_text}; {move_text}"
                    )
                else:
                    self.degree_force_status_var.set(
                        f"Degree force move phase: angle ready {held_s:.1f}/{DEGREE_FORCE_ANGLE_HOLD_S:.1f}s; "
                        f"{angle_text}; {move_text}"
                    )
            else:
                self.degree_force_angle_ready_since_monotonic = 0.0
                self.degree_force_status_var.set(f"Degree force move phase: {angle_text}; {move_text}")
            self.after(max(50, int(DEGREE_FORCE_STEP_INTERVAL_S * 500)), self._degree_force_tick)
            return

        if not angle_close:
            self._enter_degree_force_move_phase(window, f"angle drift; {angle_text}")
            self.after(max(50, int(DEGREE_FORCE_STEP_INTERVAL_S * 500)), self._degree_force_tick)
            return

        actions = self._force_window_actions_for(self.degree_force_target_n, self.degree_force_tight_limit_n)
        if actions is None:
            self._stop_degree_force_control("force data stale")
            return
        self.degree_force_tension_window_ok = not bool(actions)

        if now >= self.degree_force_next_step_monotonic:
            if actions:
                ok, error, action_texts = self._apply_tension_bias_actions(
                    window,
                    snapshot,
                    actions,
                    self.degree_force_steps,
                    self.degree_force_bias_by_motor,
                    "degree",
                )
                if not ok:
                    self._stop_degree_force_control(error)
                    return
                self.degree_force_move_count += len(actions)
                self.degree_force_status_var.set(
                    f"Degree force tension phase; moves={self.degree_force_move_count}; {angle_text}; "
                    + "; ".join(action_texts)
                )
            else:
                self.degree_force_status_var.set(
                    f"Degree force tension phase: in force window; moves={self.degree_force_move_count}; {angle_text}"
                )
            self.degree_force_next_step_monotonic = now + DEGREE_FORCE_STEP_INTERVAL_S

        self.after(max(50, int(DEGREE_FORCE_STEP_INTERVAL_S * 500)), self._degree_force_tick)

    def _start_force_target_control(self, channels: Optional[list[int]] = None) -> None:
        if self._automatic_angle_test_is_running():
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试，再启动力目标控制。")
            return
        if self.force_emergency_relax_active:
            self.force_target_status_var.set(
                f"全电机放松正在运行；五路均 > {FORCE_HARD_RELAX_STOP_N:.0f}N "
                "后才允许启动力控制"
            )
            return
        if self.feedback_pretension_active and not self.force_target_running:
            self._stop_nullspace_pretension("direct force target requested", hold_bias=False)
        if self.force_target_running:
            if channels is not None:
                self.force_target_active_channels = set(channels)
                window = self._active_encoder_servo_window()
                snapshot = window.latest_snapshot if window is not None else None
                if snapshot is not None:
                    for channel in channels:
                        motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
                        if motor is None:
                            continue
                        self.force_target_last_targets[motor] = int(snapshot.servo_abs[motor])
                        self.force_target_motor_target_vars[channel].set(str(snapshot.servo_abs[motor]))
                try:
                    self._parse_force_target_settings(channels)
                except ValueError:
                    pass
                self.force_target_settle_since_monotonic = 0.0
                for channel in channels:
                    self.force_target_next_channel_step_monotonic[channel] = 0.0
                self.force_target_status_var.set(
                    "Force target running: "
                    + ", ".join(f"CH{channel}" for channel in sorted(self.force_target_active_channels))
                )
            return
        if self.linearity_running:
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再启动 direct 力目标控制。")
            return
        if self.multi_input_running:
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再启动 direct 力目标控制。")
            return
        if self.training_running:
            messagebox.showinfo("Training running", "Stop training collection before starting force target control.")
            return
        if self.degree_force_active:
            self._stop_degree_force_control("direct force target requested")
        if self.safe_relax_active:
            messagebox.showinfo("Safe relax running", "Stop safe relax before starting force target control.")
            return
        if self.poller is None:
            messagebox.showinfo("Force sensor disconnected", "Connect the force sensor port first.")
            return

        try:
            _interval_ms, _far_step_counts, _deadband_n, targets = self._parse_force_target_settings(channels)
        except ValueError as exc:
            messagebox.showwarning("Force target settings", str(exc))
            return

        stale_channels: list[str] = []
        now_wall = time.time()
        for channel in targets:
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                stale_channels.append(f"CH{channel}: no data")
            elif now_wall - sample.timestamp > 1.5:
                stale_channels.append(f"CH{channel}: stale")
        if stale_channels:
            messagebox.showinfo("Waiting for force data", ", ".join(stale_channels))
            return

        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("Servo disconnected", "Open and connect the Encoder / Servo window first.")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("Waiting for servo data", "Wait for the first Encoder / Servo snapshot.")
            return

        offline: list[int] = []
        for channel in targets:
            motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
            if motor is None:
                continue
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]):
                offline.append(motor)
        if offline:
            messagebox.showwarning(
                "Motor offline",
                "Force target control needs these motors online: "
                + ", ".join(f"M{motor:02d}" for motor in sorted(set(offline))),
            )
            return

        self.force_target_running = True
        self.force_target_active_channels = set(targets.keys())
        self.force_target_settle_since_monotonic = 0.0
        self.force_target_move_count = 0
        self.force_target_next_channel_step_monotonic = {channel: 0.0 for channel in targets}
        self.force_target_last_targets = {
            FORCE_CHANNEL_TO_MOTOR[channel]: int(snapshot.servo_abs[FORCE_CHANNEL_TO_MOTOR[channel]])
            for channel in targets
            if FORCE_CHANNEL_TO_MOTOR.get(channel) is not None
        }
        for channel, motor in FORCE_CHANNEL_TO_MOTOR.items():
            if channel in targets and motor in self.force_target_last_targets:
                self.force_target_motor_target_vars[channel].set(str(self.force_target_last_targets[motor]))
        self._set_force_target_controls_running(True)
        window.motor_target_rejected_monotonic = 0.0
        self._queue_direct_force_mode(window)
        window.queue_text_command("status")
        self.force_target_next_step_monotonic = time.monotonic() + 0.2
        self.force_target_status_var.set(
            "Force target running: "
            + ", ".join(f"CH{channel}" for channel in sorted(self.force_target_active_channels))
            + "; switching to direct mode"
        )
        self.after(100, self._force_target_tick)

    def _stop_force_target_control(self, reason: str = "") -> None:
        if not self.force_target_running:
            self.feedback_pretension_active = False
            self._set_force_target_controls_running(False)
            return
        was_feedback_pretension = self.feedback_pretension_active
        self.force_target_running = False
        self.feedback_pretension_active = False
        self.feedback_pretension_started_monotonic = 0.0
        self.force_target_active_channels.clear()
        self.force_target_settle_since_monotonic = 0.0
        self.force_target_next_channel_step_monotonic.clear()
        self._set_force_target_controls_running(False)
        window = self._active_encoder_servo_window()
        if window is not None and window.poller is not None:
            window.queue_text_command("tension off")
        suffix = f": {reason}" if reason else ""
        if was_feedback_pretension and reason == "pretension window stable":
            self.force_target_status_var.set(
                f"反馈前预紧完成：五路已在 -15~-10 N 保持 {FEEDBACK_PRETENSION_HOLD_S:g}s；"
                f"moves={self.force_target_move_count}；电机保持当前位置"
            )
        elif was_feedback_pretension:
            self.force_target_status_var.set(
                f"反馈前预紧已停止{suffix}; moves={self.force_target_move_count}"
            )
        else:
            self.force_target_status_var.set(f"Force target stopped{suffix}; moves={self.force_target_move_count}")

    def _force_target_tick(self) -> None:
        if not self.force_target_running:
            return
        if self.poller is None:
            self._stop_force_target_control("force sensor disconnected")
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_force_target_control("Encoder / Servo disconnected")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(100, self._force_target_tick)
            return

        try:
            active_channels = sorted(self.force_target_active_channels)
            interval_ms, far_step_counts, deadband_n, targets = self._parse_force_target_settings(active_channels)
        except ValueError as exc:
            self._stop_force_target_control(str(exc))
            return

        now_wall = time.time()
        for channel in targets:
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                self._stop_force_target_control(f"CH{channel} has no force data")
                return
            if now_wall - sample.timestamp > 1.5:
                self._stop_force_target_control(f"CH{channel} force data stale")
                return

        now = time.monotonic()
        if (
            self.feedback_pretension_active
            and self.feedback_pretension_started_monotonic > 0.0
            and now - self.feedback_pretension_started_monotonic > FEEDBACK_PRETENSION_TIMEOUT_S
        ):
            window.queue_emergency_stop()
            self._stop_force_target_control(
                f"预紧超时 {FEEDBACK_PRETENSION_TIMEOUT_S:g}s，已急停"
            )
            return
        current_by_channel: dict[int, float] = {}
        error_by_channel: dict[int, float] = {}
        for channel, target_n in targets.items():
            current_n = self._force_target_current_n(channel)
            if current_n is None:
                self._stop_force_target_control(f"CH{channel} has no force data")
                return
            current_by_channel[channel] = current_n
            error_by_channel[channel] = target_n - current_n

        if self.feedback_pretension_active:
            over_tight = [
                (channel, force_n)
                for channel, force_n in current_by_channel.items()
                if force_n <= FEEDBACK_PRETENSION_ABORT_N
            ]
            if over_tight:
                details = ", ".join(
                    f"CH{channel}={force_n:.1f}N" for channel, force_n in over_tight
                )
                window.queue_emergency_stop()
                self._stop_force_target_control(f"过紧保护触发：{details}")
                return

        settle_error_n = (
            FEEDBACK_PRETENSION_DEADBAND_N
            if self.feedback_pretension_active
            else FORCE_TARGET_SETTLE_ERROR_N
        )
        settle_hold_s = (
            FEEDBACK_PRETENSION_HOLD_S
            if self.feedback_pretension_active
            else FORCE_TARGET_SETTLE_HOLD_S
        )
        all_in_settle_window = all(
            abs(error_n) <= settle_error_n for error_n in error_by_channel.values()
        )
        if all_in_settle_window:
            if self.force_target_settle_since_monotonic <= 0.0:
                self.force_target_settle_since_monotonic = now
            stable_s = now - self.force_target_settle_since_monotonic
            if stable_s >= settle_hold_s:
                reason = (
                    "pretension window stable"
                    if self.feedback_pretension_active
                    else f"within {settle_error_n:g}N for {settle_hold_s:g}s"
                )
                self._stop_force_target_control(reason)
                return
        else:
            self.force_target_settle_since_monotonic = 0.0
            stable_s = 0.0

        rejected_at = getattr(window, "motor_target_rejected_monotonic", 0.0)
        if rejected_at > 0.0 and now - rejected_at < 2.0:
            window.motor_target_rejected_monotonic = 0.0
            self._queue_direct_force_mode(window)
            window.queue_text_command("status")
            self.force_target_next_step_monotonic = now + 0.3
            self.force_target_status_var.set("Force target: resent start/direct after target rejection")
            self.after(100, self._force_target_tick)
            return

        actions: list[str] = []
        if now >= self.force_target_next_step_monotonic:
            window.queue_text_command("direct")
            for channel, target_n in targets.items():
                motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
                if motor is None:
                    continue
                online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
                if not online:
                    self._stop_force_target_control(f"M{motor:02d} offline")
                    return

                current_n = current_by_channel[channel]
                error_n = error_by_channel[channel]
                near_target = (
                    not self.feedback_pretension_active
                    and abs(error_n) <= FORCE_TARGET_NEAR_ERROR_N
                )
                channel_interval_s = (
                    FORCE_TARGET_NEAR_STEP_INTERVAL_S if near_target else interval_ms / 1000.0
                )
                channel_next_step = self.force_target_next_channel_step_monotonic.get(channel, 0.0)
                if now < channel_next_step:
                    self.force_target_motor_target_vars[channel].set(
                        str(self.force_target_last_targets.get(motor, int(snapshot.servo_abs[motor])))
                    )
                    continue

                delta, step_text = self._force_target_step_delta(
                    error_n,
                    far_step_counts=far_step_counts,
                    deadband_n=deadband_n,
                )
                self.force_target_next_channel_step_monotonic[channel] = now + channel_interval_s
                if delta == 0:
                    self.force_target_motor_target_vars[channel].set(
                        str(self.force_target_last_targets.get(motor, int(snapshot.servo_abs[motor])))
                    )
                    actions.append(f"CH{channel}/M{motor:02d} hold {current_n:.1f}/{target_n:.1f}N {step_text}")
                    continue

                current_abs = int(snapshot.servo_abs[motor])
                previous_target = int(self.force_target_last_targets.get(motor, current_abs))
                base = min(current_abs, previous_target) if delta < 0 else max(current_abs, previous_target)
                target_abs = max(FORCE_TARGET_MIN_ABS, min(FORCE_TARGET_MAX_ABS, int(base) + int(delta)))
                self.force_target_last_targets[motor] = target_abs
                self.force_target_motor_target_vars[channel].set(str(target_abs))
                window.queue_text_command(f"direct; m{motor} {target_abs}")
                self.force_target_move_count += 1
                actions.append(
                    f"CH{channel}/M{motor:02d} {current_n:.1f}->{target_n:.1f}N {step_text} target={target_abs}"
                )
            next_times = [
                self.force_target_next_channel_step_monotonic.get(channel, now + interval_ms / 1000.0)
                for channel in targets
            ]
            self.force_target_next_step_monotonic = max(now + 0.02, min(next_times, default=now + interval_ms / 1000.0))

        if actions:
            stable_text = f"; stable={stable_s:.1f}s" if all_in_settle_window else ""
            self.force_target_status_var.set(
                f"Force target running; moves={self.force_target_move_count}{stable_text}; "
                + "; ".join(actions[:4])
            )
        self.after(max(20, min(100, interval_ms // 2)), self._force_target_tick)

    def _safe_relax_force_state(self) -> tuple[Optional[list[tuple[int, int, float]]], str]:
        items: list[tuple[int, int, float]] = []
        now = time.time()
        for channel in DISPLAY_CHANNELS:
            motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
            if motor is None:
                continue
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                return None, f"缺少 CH{channel} 力传感器读数"
            if now - sample.timestamp > 1.5:
                return None, f"CH{channel} 力传感器数据超时"
            force_n = self._relative_value(sample.value, channel) / RELATIVE_UNITS_PER_NEWTON
            items.append((channel, motor, force_n))
        return items, ""

    def _toggle_safe_relax(self) -> None:
        if self.safe_relax_active:
            self._stop_safe_relax("手动停止")
            return
        self._start_safe_relax()

    def _configure_safe_relax_buttons(
        self,
        *,
        text: Optional[str] = None,
        state: Optional[str] = None,
    ) -> None:
        for attr in ("safe_relax_button", "safe_relax_top_button"):
            button = getattr(self, attr, None)
            if button is None:
                continue
            options: dict[str, str] = {}
            if text is not None:
                options["text"] = text
            if state is not None:
                options["state"] = state
            if options:
                button.configure(**options)

    def _queue_safe_relax_direct_mode(self, window: EncoderServoWindow) -> None:
        targets = window._current_host_direct_targets()
        window.queue_text_command("tension off")
        window.queue_text_command("direct")
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(targets),
            label="safe relax START snapshot",
        )
        window.queue_text_command("start")

    def _cancel_safe_relax_after(self) -> None:
        after_id = self.safe_relax_after_id
        if after_id is None:
            return
        self.safe_relax_after_id = None
        try:
            self.after_cancel(after_id)
        except tk.TclError:
            pass

    def _send_safe_relax_targets(self) -> None:
        self.safe_relax_after_id = None
        if not self.safe_relax_active:
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_safe_relax("Encoder / Servo disconnected", send_stop=False)
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self._stop_safe_relax("no Encoder / Servo data", send_stop=False)
            return

        targets = [int(snapshot.servo_abs[motor]) for motor in range(SERVO_COUNT)]
        for motor in training_target_motor_indices():
            targets[motor] = SAFE_RELAX_TARGET_ABS
            window.sent_direct_targets[motor] = SAFE_RELAX_TARGET_ABS

        window.queue_text_command("direct")
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(targets),
            label=f"abs batch22 M00-M04 {SAFE_RELAX_TARGET_ABS}",
        )
        window.queue_text_command("status")
        self.safe_relax_status_var.set(
            f"安全放松目标已发送: M00-M04 -> {SAFE_RELAX_TARGET_ABS} counts"
        )
        self.safe_relax_after_id = self.after(SAFE_RELAX_COMPLETE_DELAY_MS, self._complete_safe_relax)

    def _complete_safe_relax(self) -> None:
        self.safe_relax_after_id = None
        if not self.safe_relax_active:
            return
        self.safe_relax_active = False
        self._configure_safe_relax_buttons(text="安全放松")
        if not self.training_running:
            self.training_start_button.configure(state="normal")
        self.safe_relax_status_var.set(
            f"安全放松目标已发送完成: M00-M04 -> {SAFE_RELAX_TARGET_ABS} counts"
        )

    def _start_safe_relax(self) -> None:
        window = self._active_encoder_servo_window()
        if window is not None and window.step_test_running:
            window._stop_step_test("安全放松接管", send_stop=True)
        if window is not None and window.sine_test_running:
            window._stop_sine_test("安全放松接管", send_stop=True)
        if self.linearity_running:
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再执行安全放松。")
            return
        if self.multi_input_running:
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再执行安全放松。")
            return
        if self.training_running:
            messagebox.showinfo("采集中", "请先停止训练采集，再执行安全放松。")
            return
        if self.force_target_running:
            self._stop_force_target_control("safe relax requested")
        if self.degree_force_active:
            self._stop_degree_force_control("safe relax requested")
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待数据", "请等待 Encoder / Servo 出现第一帧数据后再执行安全放松。")
            return
        offline = [
            motor
            for motor in training_target_motor_indices()
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            messagebox.showwarning(
                "电机未在线",
                "安全放松需要 M00-M04 在线，当前未在线: "
                + ", ".join(f"M{motor:02d}" for motor in offline),
            )
            return

        self._cancel_safe_relax_after()
        self.safe_relax_active = True
        self.safe_relax_ok_since = 0.0
        self.safe_relax_next_step_monotonic = 0.0
        self.safe_relax_move_count = len(training_target_motor_indices())
        self.safe_relax_last_targets = {
            motor: SAFE_RELAX_TARGET_ABS
            for motor in training_target_motor_indices()
        }
        for motor, target in self.safe_relax_last_targets.items():
            window.sent_direct_targets[motor] = target
        self._configure_safe_relax_buttons(text="停止放松")
        window.motor_target_rejected_monotonic = 0.0
        window.clear_pending_commands()
        self._queue_safe_relax_direct_mode(window)
        self.safe_relax_status_var.set(
            f"安全放松已启动: 等待 START 后发送 M00-M04 -> {SAFE_RELAX_TARGET_ABS} counts"
        )
        self.safe_relax_after_id = self.after(SAFE_RELAX_START_DELAY_MS, self._send_safe_relax_targets)

    def _stop_safe_relax(self, reason: str = "", send_stop: bool = True) -> None:
        if not self.safe_relax_active:
            self._cancel_safe_relax_after()
            return
        self._cancel_safe_relax_after()
        self.safe_relax_active = False
        window = self._active_encoder_servo_window()
        if send_stop and window is not None and window.poller is not None:
            window.queue_text_command("stop")
            window.queue_text_command("tension off")
        self._configure_safe_relax_buttons(text="安全放松")
        if not self.training_running:
            self.training_start_button.configure(state="normal")
        suffix = f": {reason}" if reason else ""
        self.safe_relax_status_var.set(f"安全放松已停止{suffix}; step={self.safe_relax_move_count}")

    def _safe_relax_tick(self) -> None:
        self.safe_relax_active = False
        self._configure_safe_relax_buttons(text="安全放松")
        self.safe_relax_status_var.set(
            f"安全放松固定目标模式: M00-M04 -> {SAFE_RELAX_TARGET_ABS} counts"
        )

    def _parse_training_settings(
        self,
    ) -> tuple[int, float, Optional[float], list[tuple[int, float, float]], str]:
        try:
            sample_interval_ms = int(self.training_interval_var.get())
            target_period_s = float(self.training_target_period_var.get().strip())
        except (tk.TclError, ValueError) as exc:
            raise ValueError("采样间隔和目标周期需要是数字。") from exc
        if sample_interval_ms < 20:
            raise ValueError("采样间隔不能小于 20 ms。")
        if target_period_s < 0.2:
            raise ValueError("目标周期不能小于 0.2 s。")

        load_text = self.training_load_limit_var.get().strip()
        load_limit: Optional[float]
        if load_text:
            try:
                load_limit = abs(float(load_text))
            except ValueError as exc:
                raise ValueError("Load保护值需要是数字，留空表示不启用保护。") from exc
            if load_limit <= 0:
                raise ValueError("Load保护值需要大于 0，留空表示不启用保护。")
        else:
            load_limit = None

        ranges = parse_training_joint_ranges(self.training_ranges_var.get())
        target_mode = self.training_target_mode_var.get().strip().lower()
        if target_mode not in TRAINING_TARGET_MODES:
            raise ValueError(
                "采集模式需要是: " + ", ".join(TRAINING_TARGET_MODES)
            )
        return sample_interval_ms, target_period_s, load_limit, ranges, target_mode

    def _parse_pretension_settings(self) -> tuple[float, float, dict[int, int], float]:
        try:
            target_n = float(self.pretension_target_var.get().strip())
            tight_limit_n = float(self.pretension_tight_limit_var.get().strip())
            timeout_s = float(self.pretension_timeout_var.get().strip())
        except ValueError as exc:
            raise ValueError("预紧目标、过紧下限和超时需要是数字。") from exc
        if tight_limit_n >= target_n:
            raise ValueError("过紧下限需要小于预紧目标，例如 -100 < -5。")
        if timeout_s <= 0:
            raise ValueError("预紧超时需要大于 0。")
        steps = parse_pretension_steps(self.pretension_step_var.get())
        return target_n, tight_limit_n, steps, timeout_s

    def _set_training_controls_running(self, running: bool) -> None:
        editable_state = "disabled" if running else "normal"
        for widget in (
            self.training_interval_spin,
            self.training_target_period_entry,
            self.training_load_limit_entry,
            self.training_ranges_entry,
            self.training_send_targets_check,
            self.pretension_enabled_check,
            self.pretension_target_entry,
            self.pretension_tight_limit_entry,
            self.pretension_step_entry,
            self.pretension_timeout_entry,
            self.degree_force_enabled_check,
        ):
            widget.configure(state=editable_state)
        self.training_target_mode_combo.configure(state="disabled" if running else "readonly")
        self.training_start_button.configure(state="disabled" if running else "normal")
        self.training_stop_button.configure(state="normal" if running else "disabled")
        if not self.safe_relax_active:
            self._configure_safe_relax_buttons(state="disabled" if running else "normal")
        if hasattr(self, "linearity_start_button") and not self.linearity_running:
            self._set_linearity_controls_running(running)
            if running:
                self.linearity_stop_button.configure(state="disabled")
        if hasattr(self, "multi_input_start_button") and not self.multi_input_running:
            self._set_multi_input_controls_running(running)
            if running:
                self.multi_input_stop_button.configure(state="disabled")

    def _start_training_collection(self) -> None:
        if self.training_running:
            return
        if self._automatic_angle_test_is_running():
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试，再开始训练采集。")
            return
        if self.linearity_running:
            messagebox.showinfo("线性实验运行中", "请先停止线性系统判别实验，再开始训练采集。")
            return
        if self.multi_input_running:
            messagebox.showinfo("联合辨识运行中", "请先停止多输入联合辨识，再开始训练采集。")
            return
        if self.safe_relax_active:
            messagebox.showinfo("安全放松中", "请先停止安全放松，再开始训练采集。")
            return
        if self.degree_force_active:
            self._stop_degree_force_control("training collection requested")
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接力传感器 COM7。")
            return
        missing_channels = [
            channel for channel in DISPLAY_CHANNELS if channel not in self.latest_by_channel
        ]
        if missing_channels:
            messagebox.showinfo(
                "等待数据",
                "请先等待 CH1-CH5 都有读数，再开始采集。\n"
                f"当前缺少: {', '.join(f'CH{channel}' for channel in missing_channels)}",
            )
            return

        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待数据", "请等待 Encoder / Servo 出现第一帧数据后再开始采集。")
            return

        try:
            (
                sample_interval_ms,
                target_period_s,
                load_limit,
                ranges,
                target_mode,
            ) = self._parse_training_settings()
        except ValueError as exc:
            messagebox.showwarning("采集参数错误", str(exc))
            return

        if self.training_send_targets_var.get():
            valid_joints = [joint for joint, _, _ in ranges if snapshot.encoder_valid[joint]]
            if not valid_joints:
                messagebox.showinfo("等待数据", "关节范围内没有有效 Encoder 读数，暂不发送自动目标。")
                return

        pretension_enabled = bool(self.pretension_enabled_var.get())
        pretension_settings: Optional[tuple[float, float, dict[int, int], float]] = None
        if pretension_enabled:
            try:
                pretension_settings = self._parse_pretension_settings()
            except ValueError as exc:
                messagebox.showwarning("预紧参数错误", str(exc))
                return

        self.training_sample_interval_ms = sample_interval_ms
        self.training_target_period_s = target_period_s
        self.training_load_limit = load_limit
        self.training_ranges = ranges
        self.training_target_mode = target_mode
        self.training_last_target_joint = None
        self.training_last_target_relative = None
        self.training_last_target_device = None
        self.training_last_target_command = ""
        self.training_last_target_mode = target_mode
        self.training_last_target_phase = ""
        self.training_target_index = 0
        self.training_target_relative_by_joint = {}
        self.training_target_device_by_joint = {}
        self.training_next_tension_check_monotonic = 0.0
        self.training_tension_action = ""
        self.training_tension_action_count = 0
        self.training_tension_window_ok = 1
        self.training_tension_bias_by_motor = self._preserved_tension_bias_by_motor(
            window,
            self.training_tension_bias_by_motor,
        )
        self.training_target_relative_by_joint = {
            joint: 0.0 for joint, _low, _high in ranges
        }
        self.training_target_device_by_joint = {
            joint: 0.0
            for joint, _low, _high in ranges
        }
        self.training_rows = 0
        self.training_running = True
        self._set_training_controls_running(True)

        if pretension_enabled and pretension_settings is not None:
            self._start_pretension(window, snapshot, pretension_settings)
        else:
            self._begin_training_collection(window, snapshot)

    def _begin_training_collection(self, window: EncoderServoWindow, snapshot: EncoderServoSnapshot) -> None:
        default_dir = Path.cwd() / "run_data"
        try:
            default_dir.mkdir(parents=True, exist_ok=True)
            path = default_dir / f"training_collect_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            fp = open(path, "w", newline="", encoding="utf-8-sig")
            writer = csv.DictWriter(fp, fieldnames=training_csv_fieldnames(), extrasaction="ignore", restval="")
            writer.writeheader()
        except Exception as exc:
            messagebox.showerror("采集文件失败", f"无法创建训练数据 CSV: {exc}")
            self._stop_training_collection("采集文件创建失败")
            return

        self.training_file = fp
        self.training_writer = writer
        self.training_path = path
        self.training_started_monotonic = time.monotonic()
        self.training_next_sample_monotonic = self.training_started_monotonic
        self.training_next_target_monotonic = self.training_started_monotonic
        self.training_next_tension_check_monotonic = self.training_started_monotonic
        self.training_last_target_joint = None
        self.training_last_target_relative = None
        self.training_last_target_device = None
        self.training_last_target_command = ""
        self.training_last_target_mode = self.training_target_mode
        self.training_last_target_phase = "init"
        self.training_tension_action = ""
        self.training_tension_action_count = 0
        self.training_tension_window_ok = 1
        self.training_target_index = 0
        self.training_target_relative_by_joint = {
            joint: 0.0 for joint, _low, _high in self.training_ranges
        }
        self.training_target_device_by_joint = {
            joint: 0.0
            for joint, _low, _high in self.training_ranges
        }
        self.training_phase = "collecting"
        self.training_path_var.set(str(path))
        self.training_status_var.set("采集中: 0行")
        self._training_tick()

    def _start_pretension(
        self,
        window: EncoderServoWindow,
        snapshot: EncoderServoSnapshot,
        settings: tuple[float, float, dict[int, int], float],
    ) -> None:
        target_n, tight_limit_n, steps, timeout_s = settings
        self.pretension_target_n = target_n
        self.pretension_tight_limit_n = tight_limit_n
        self.pretension_steps = dict(steps)
        self.pretension_timeout_s = timeout_s
        self.pretension_started_monotonic = time.monotonic()
        self.pretension_next_step_monotonic = self.pretension_started_monotonic
        self.pretension_move_count = 0
        self.pretension_last_targets = {}
        self.training_phase = "pretension"
        self.training_path_var.set("--")
        window.queue_text_command("tension on")
        window.queue_text_command("degree")
        self._restore_current_degree_targets(window)
        self.training_status_var.set(
            f"预紧中: M00-M03 {tight_limit_n:.1f}..{target_n:.1f} N, "
            f"M04 {PRETENSION_RETURN_TIGHT_LIMIT_N:.1f}..{PRETENSION_RETURN_TARGET_N:.1f} N"
        )
        self._pretension_tick()

    def _pretension_tick(self) -> None:
        if not self.training_running or self.training_phase != "pretension":
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_training_collection("Encoder / Servo 已断开")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(100, self._pretension_tick)
            return

        violation = self._training_load_violation(snapshot)
        if violation:
            window.queue_text_command("stop")
            self._stop_training_collection(f"{violation}; 已发送 stop")
            return

        elapsed = time.monotonic() - self.pretension_started_monotonic
        if elapsed > self.pretension_timeout_s:
            window.queue_text_command("stop")
            self._stop_training_collection(f"预紧超时 {self.pretension_timeout_s:g}s; 已发送 stop")
            return

        actions = self._force_window_actions()
        if actions is None:
            self.training_status_var.set("预紧中: 等待最新力传感器数据")
            self.after(100, self._pretension_tick)
            return
        if not actions:
            self.training_status_var.set("预紧完成，开始采集")
            self.training_tension_window_ok = 1
            self._begin_training_collection(window, snapshot)
            return

        now = time.monotonic()
        if now >= self.pretension_next_step_monotonic:
            if not self._apply_force_window_actions(window, snapshot, actions, "pretension"):
                return
            self.pretension_next_step_monotonic = now + PRETENSION_STEP_INTERVAL_S

        pending_text = ", ".join(
            f"{label} CH{channel}/M{motor:02d}={force_n:.1f}N"
            for channel, motor, force_n, direction, label in actions
        )
        self.training_status_var.set(
            f"预紧中: {pending_text}; step={self.pretension_move_count}"
        )
        self.after(max(50, int(PRETENSION_STEP_INTERVAL_S * 500)), self._pretension_tick)

    def _force_window_actions_for(
        self,
        target_n: float,
        tight_limit_n: float,
        motors: Optional[set[int]] = None,
    ) -> Optional[list[tuple[int, int, float, int, str]]]:
        actions: list[tuple[int, int, float, int, str]] = []
        now = time.time()
        for channel in DISPLAY_CHANNELS:
            motor = FORCE_CHANNEL_TO_MOTOR.get(channel)
            if motor is None:
                continue
            if motors is not None and motor not in motors:
                continue
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                return None
            if now - sample.timestamp > DEGREE_FORCE_STALE_LIMIT_S:
                return None
            force_n = self._relative_value(sample.value, channel) / RELATIVE_UNITS_PER_NEWTON
            motor_target_n = self._tension_target_for_motor(motor, target_n)
            motor_tight_limit_n = self._tension_tight_limit_for_motor(motor, tight_limit_n)
            if force_n > motor_target_n:
                actions.append((channel, motor, force_n, 1, "loose"))
            elif force_n < motor_tight_limit_n:
                actions.append((channel, motor, force_n, -1, "tight"))
        return actions

    def _force_window_actions(self) -> Optional[list[tuple[int, int, float, int, str]]]:
        return self._force_window_actions_for(self.pretension_target_n, self.pretension_tight_limit_n)

    def _tension_target_for_motor(self, motor: int, target_n: float) -> float:
        if motor == PRETENSION_RETURN_MOTOR:
            return PRETENSION_RETURN_TARGET_N
        return target_n

    def _tension_tight_limit_for_motor(self, motor: int, tight_limit_n: float) -> float:
        if motor == PRETENSION_RETURN_MOTOR:
            return PRETENSION_RETURN_TIGHT_LIMIT_N
        return tight_limit_n

    def _continuous_tension_context(
        self,
    ) -> tuple[bool, str, Optional[float], Optional[float], set[int], dict[int, int]]:
        """Return the host-side force-window state used by a record row."""
        if self.degree_force_active:
            active_motors = (
                set(training_target_motor_indices())
                if self.degree_force_phase == "tension"
                else set(PRETENSION_ALWAYS_TENSION_MOTORS)
            )
            return (
                True,
                self.degree_force_phase,
                self.degree_force_target_n,
                self.degree_force_tight_limit_n,
                active_motors,
                dict(self.degree_force_bias_by_motor),
            )
        if self.training_running:
            return (
                True,
                self.training_phase,
                getattr(self, "pretension_target_n", None),
                getattr(self, "pretension_tight_limit_n", None),
                set(training_target_motor_indices()),
                dict(self.training_tension_bias_by_motor),
            )
        return False, "idle", None, None, set(), {}

    def _apply_tension_bias_actions(
        self,
        window: EncoderServoWindow,
        snapshot: EncoderServoSnapshot,
        actions: list[tuple[int, int, float, int, str]],
        steps: dict[int, int],
        bias_by_motor: dict[int, int],
        reason: str,
    ) -> tuple[bool, str, list[str]]:
        action_texts = []
        for channel, motor, force_n, direction, label in actions:
            online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
            if not online:
                return False, f"M{motor:02d} offline", action_texts
            base = bias_by_motor.get(motor, 0)
            target = int(base + steps[motor] * direction)
            limit = self._tension_bias_limit_for_motor(motor)
            target = max(-limit, min(limit, target))
            bias_by_motor[motor] = target
            self.degree_force_bias_by_motor[motor] = target
            self.training_tension_bias_by_motor[motor] = target
            window.queue_text_command(f"tensionbias m{motor} {target}")
            action_texts.append(f"{reason}:{label}:CH{channel}/M{motor:02d}={force_n:.1f}N bias={target}")
        return True, "", action_texts

    def _apply_force_window_actions(
        self,
        window: EncoderServoWindow,
        snapshot: EncoderServoSnapshot,
        actions: list[tuple[int, int, float, int, str]],
        reason: str,
    ) -> bool:
        ok, error, action_texts = self._apply_tension_bias_actions(
            window,
            snapshot,
            actions,
            self.pretension_steps,
            self.training_tension_bias_by_motor,
            reason,
        )
        if not ok:
            window.queue_text_command("stop")
            self._stop_training_collection(f"张力维护失败: {error}; 已发送 stop")
            return False
        self.pretension_move_count += len(actions)
        self.training_tension_action_count += len(actions)
        self.training_tension_window_ok = 0
        self.training_tension_action = "; ".join(action_texts)
        return True

    def _tension_bias_limit_for_motor(self, motor: int) -> int:
        if motor == PRETENSION_RETURN_MOTOR:
            return TENSION_RETURN_BIAS_ABS_LIMIT_COUNTS
        return TENSION_BIAS_ABS_LIMIT_COUNTS

    def _stop_training_collection(self, reason: str = "") -> None:
        if not self.training_running and self.training_file is None:
            return
        self.training_running = False
        self.training_phase = "idle"
        window = self._active_encoder_servo_window()
        self.pretension_last_targets.clear()
        fp = self.training_file
        self.training_file = None
        self.training_writer = None
        if fp is not None:
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass
        if hasattr(self, "training_start_button"):
            self._set_training_controls_running(False)
        suffix = f": {reason}" if reason else ""
        self.training_status_var.set(f"已停止{suffix}; {self.training_rows}行")

    def _training_tick(self) -> None:
        if not self.training_running or self.training_phase != "collecting":
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_training_collection("Encoder / Servo 已断开")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(100, self._training_tick)
            return

        violation = self._training_load_violation(snapshot)
        if violation:
            window.queue_text_command("stop")
            self._stop_training_collection(f"{violation}; 已发送 stop")
            return

        now = time.monotonic()
        if (
            self.pretension_enabled_var.get()
            and now >= self.training_next_tension_check_monotonic
        ):
            actions = self._force_window_actions()
            if actions is None:
                window.queue_text_command("stop")
                self._stop_training_collection("力传感器数据超时，无法维持张力窗口; 已发送 stop")
                return
            self.training_tension_window_ok = 1 if not actions else 0
            if actions:
                if not self._apply_force_window_actions(window, snapshot, actions, "collect"):
                    return
                self.training_next_tension_check_monotonic = now + PRETENSION_STEP_INTERVAL_S

        if (
            self.training_send_targets_var.get()
            and now >= self.training_next_target_monotonic
        ):
            self._send_training_target(window, snapshot)
            self.training_next_target_monotonic = now + self.training_target_period_s

        if now >= self.training_next_sample_monotonic:
            try:
                self._write_training_row(window, snapshot)
            except Exception as exc:
                self._stop_training_collection(f"写入失败: {exc}")
                return
            self.training_next_sample_monotonic = now + self.training_sample_interval_ms / 1000.0

        delay_ms = max(20, min(200, self.training_sample_interval_ms // 2))
        self.after(delay_ms, self._training_tick)

    def _send_training_target(self, window: EncoderServoWindow, snapshot: EncoderServoSnapshot) -> None:
        valid_ranges = [
            (joint, low, high)
            for joint, low, high in self.training_ranges
            if snapshot.encoder_valid[joint]
        ]
        if not valid_ranges:
            self.training_status_var.set(f"采集中: {self.training_rows}行, 等待有效 Encoder")
            return
        phase, active_joint, targets = build_training_target_batch(
            valid_ranges,
            self.training_target_index,
            self.training_target_mode,
        )
        commands: list[str] = []
        last_relative: Optional[float] = None
        last_device: Optional[float] = None
        for joint, relative_target in targets:
            device_target = relative_target
            command = f"degree; j{joint} {device_target:.3f}"
            if not window.queue_text_command(command):
                self._stop_training_collection("Encoder / Servo 已断开")
                return
            commands.append(command)
            self.training_target_relative_by_joint[joint] = relative_target
            self.training_target_device_by_joint[joint] = device_target
            if active_joint is not None and joint == active_joint:
                last_relative = relative_target
                last_device = device_target
        self.training_target_index += 1
        self.training_last_target_joint = active_joint
        self.training_last_target_relative = last_relative
        self.training_last_target_device = last_device
        self.training_last_target_command = " | ".join(commands)
        self.training_last_target_mode = self.training_target_mode
        self.training_last_target_phase = phase

    def _restore_current_degree_targets(self, window: EncoderServoWindow) -> bool:
        if not self.training_target_device_by_joint:
            return True
        for joint in sorted(self.training_target_device_by_joint):
            device_target = self.training_target_device_by_joint[joint]
            command = f"degree; j{joint} {device_target:.3f}"
            if not window.queue_text_command(command):
                return False
        return True

    def _training_load_violation(self, snapshot: EncoderServoSnapshot) -> str:
        if self.training_load_limit is None:
            return ""
        for motor in training_target_motor_indices():
            if not 0 <= motor < len(snapshot.servo_load):
                continue
            load = snapshot.servo_load[motor]
            if abs(load) > self.training_load_limit:
                return f"Load保护 M{motor:02d}={load} 超过 {self.training_load_limit:g}"
        return ""

    def _parse_linearity_experiment_settings(
        self,
    ) -> tuple[list[dict[int, int]], list[int], list[int], int, float, float]:
        try:
            states = parse_linearity_state_motor_positions(self.linearity_states_var.get())
            motors = parse_linearity_motors(self.linearity_motors_var.get())
            deltas = parse_linearity_deltas(self.linearity_deltas_var.get())
            repeats = int(self.linearity_repeats_var.get())
            hold_s = float(self.linearity_hold_var.get().strip())
            timeout_s = float(self.linearity_timeout_var.get().strip())
        except (tk.TclError, ValueError) as exc:
            raise ValueError(f"线性实验参数错误: {exc}") from exc
        if repeats <= 0:
            raise ValueError("重复次数需要大于 0")
        if hold_s <= 0:
            raise ValueError("到位停留时间需要大于 0")
        if timeout_s <= hold_s:
            raise ValueError("超时时间需要大于到位停留时间")
        return states, motors, deltas, repeats, hold_s, timeout_s

    def _set_linearity_controls_running(self, running: bool) -> None:
        editable_state = "disabled" if running else "normal"
        for attr in (
            "linearity_states_entry",
            "linearity_motors_entry",
            "linearity_deltas_entry",
            "linearity_repeats_spin",
            "linearity_hold_entry",
            "linearity_timeout_entry",
        ):
            widget = getattr(self, attr, None)
            if widget is not None:
                widget.configure(state=editable_state)
        if hasattr(self, "linearity_start_button"):
            self.linearity_start_button.configure(state="disabled" if running else "normal")
        if hasattr(self, "linearity_stop_button"):
            self.linearity_stop_button.configure(state="normal" if running else "disabled")
        if hasattr(self, "linearity_scan_button"):
            self.linearity_scan_button.configure(state="disabled")

    def _set_linearity_scan_ready(self, ready: bool) -> None:
        if hasattr(self, "linearity_scan_button"):
            self.linearity_scan_button.configure(
                state="normal" if ready and self.linearity_running else "disabled"
            )

    def _start_linearity_scan(self) -> None:
        if not self.linearity_running:
            messagebox.showinfo("尚未记录基准", "请先点击“步骤1 到位并记录”。")
            return
        if self.linearity_phase != "baseline_ready":
            messagebox.showinfo("基准尚未就绪", self.linearity_status_var.get())
            return
        if not self.linearity_baseline_abs_by_motor:
            messagebox.showwarning("基准数据缺失", "没有可用于扰动的 M00-M04 基准 ABS。")
            return
        self.linearity_trials = self._linearity_build_trials()
        self.linearity_trial_index = 0
        self.linearity_current_trial = None
        self._set_linearity_scan_ready(False)
        self._linearity_enter_phase("apply_perturb")
        self.linearity_status_var.set(
            f"步骤2已启动: {self.linearity_baseline_label}, "
            f"{len(self.linearity_trials)}个序列点"
        )
        self.after(0, self._linearity_experiment_tick)

    def _select_linearity_analysis_csv(self) -> None:
        initial_dir = Path.cwd() / "run_data"
        path = filedialog.askopenfilename(
            parent=self,
            title="选择局部线性实验 CSV",
            initialdir=str(initial_dir if initial_dir.exists() else Path.cwd()),
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        self.linearity_analysis_csv_path = Path(path)
        self.linearity_analysis_csv_var.set(Path(path).name)
        self.linearity_analysis_markdown = ""
        self.linearity_analysis_output_dir = None
        self.linearity_analysis_status_var.set("CSV已导入，选择步骤3、4或5开始分析。")

    def _run_linearity_csv_analysis(self, kind: str) -> None:
        if self.linearity_running or self.multi_input_running:
            messagebox.showinfo("实验运行中", "请先完成或停止当前辨识实验，再分析 CSV。")
            return
        csv_path = self.linearity_analysis_csv_path
        if csv_path is None:
            messagebox.showinfo("未导入CSV", "请先点击“导入实验CSV”。")
            return
        output_root = Path.cwd() / "run_data" / "linearity_analysis"
        labels = {
            "impact": "步骤3 影响系数",
            "hysteresis": "步骤4 回差分析",
            "linearity": "步骤5 线性判断",
        }
        try:
            result = analyze_linearity_csv(csv_path, kind, output_root)
        except (LinearityAnalysisError, OSError, csv.Error) as exc:
            self.linearity_analysis_status_var.set(f"{labels.get(kind, kind)}失败: {exc}")
            messagebox.showerror("CSV分析失败", str(exc))
            return
        self.linearity_analysis_markdown = result.markdown
        self.linearity_analysis_output_dir = result.output_dir
        self.linearity_analysis_status_var.set(
            f"{labels[kind]}完成: 有效序列{result.valid_series}, "
            f"排除{result.rejected_series}; Markdown={result.markdown_path.name}, "
            f"图={result.plot_path.name}"
        )

    def _copy_linearity_analysis_markdown(self) -> None:
        if not self.linearity_analysis_markdown:
            messagebox.showinfo("没有分析结果", "请先执行步骤3、4或5。")
            return
        self.clipboard_clear()
        self.clipboard_append(self.linearity_analysis_markdown)
        self.update_idletasks()
        self.linearity_analysis_status_var.set("最新分析 Markdown 已复制，可直接粘贴到飞书。")

    def _open_linearity_analysis_output(self) -> None:
        output_dir = self.linearity_analysis_output_dir
        if output_dir is None or not output_dir.exists():
            messagebox.showinfo("没有结果目录", "请先执行步骤3、4或5。")
            return
        try:
            os.startfile(str(output_dir))
        except (AttributeError, OSError) as exc:
            messagebox.showerror("无法打开目录", str(exc))

    def _start_linearity_experiment(self) -> None:
        if self.linearity_running:
            return
        if self._automatic_angle_test_is_running():
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试，再开始局部线性辨识。")
            return
        if self.multi_input_running:
            messagebox.showinfo("联合辨识运行中", "请先完成或停止多输入联合辨识实验。")
            return
        if self.training_running:
            messagebox.showinfo("训练采集中", "请先停止训练采集，再开始线性系统判别实验。")
            return
        if self.safe_relax_active:
            messagebox.showinfo("安全放松中", "请先停止安全放松，再开始线性系统判别实验。")
            return
        if self.poller is None:
            messagebox.showinfo("力传感器未连接", "请先连接力传感器串口。")
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("Encoder / Servo 未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待 Encoder / Servo 数据", "请等待第一帧关节角度数据。")
            return

        try:
            (
                states,
                motors,
                deltas,
                repeats,
                hold_s,
                timeout_s,
            ) = self._parse_linearity_experiment_settings()
        except ValueError as exc:
            messagebox.showwarning("线性实验参数", str(exc))
            return

        values, value_error = self._linearity_current_values(
            snapshot,
            required_joints=set(MANUAL_RECORD_JOINTS),
        )
        if values is None:
            messagebox.showinfo("等待稳定数据", value_error)
            return

        offline_motors = [
            motor
            for motor in training_target_motor_indices()
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline_motors:
            messagebox.showwarning(
                "电机未在线",
                "局部线性实验需要 M00-M04 全部在线，当前未在线: "
                + ", ".join(f"M{motor:02d}" for motor in offline_motors),
            )
            return

        safe_low = FORCE_TARGET_MIN_ABS + LINEARITY_SERVO_LIMIT_MARGIN_COUNTS
        safe_high = FORCE_TARGET_MAX_ABS - LINEARITY_SERVO_LIMIT_MARGIN_COUNTS
        for state_index, targets in enumerate(states, start=1):
            for motor in training_target_motor_indices():
                planned = [targets[motor]]
                if motor in motors:
                    planned.extend(targets[motor] + delta for delta in deltas)
                invalid = [target for target in planned if not safe_low <= target <= safe_high]
                if invalid:
                    messagebox.showwarning(
                        "初始位置或扰动超出安全范围",
                        f"S{state_index:02d} M{motor:02d} 存在目标 {invalid[0]}，"
                        f"线性实验要求所有目标位于 {safe_low}~{safe_high}。",
                    )
                    return

        transition_starts = [
            {motor: int(snapshot.servo_abs[motor]) for motor in training_target_motor_indices()},
            *states[:-1],
        ]
        transition_deltas = [
            [
                targets[motor] - starts[motor]
                for motor in training_target_motor_indices()
            ]
            for starts, targets in zip(transition_starts, states)
        ]
        largest_state_index, largest_deltas = max(
            enumerate(transition_deltas),
            key=lambda item: max(abs(delta) for delta in item[1]),
        )
        if max(abs(delta) for delta in largest_deltas) >= DIRECT_ABS_CONFIRM_DELTA_COUNTS:
            starts = transition_starts[largest_state_index]
            targets = states[largest_state_index]
            summary = "\n".join(
                f"M{motor:02d}: {starts[motor]} -> {targets[motor]} "
                f"(Δ{largest_deltas[motor]:+d})"
                for motor in training_target_motor_indices()
            )
            if not messagebox.askyesno(
                "确认初始位置大幅移动",
                f"到 S{largest_state_index + 1:02d} 的初始 ABS 跳转中，"
                "至少一路变化达到一圈（4096 counts），确认继续吗？\n\n"
                + summary,
            ):
                return

        if self.force_target_running:
            self._stop_force_target_control("linearity experiment requested")

        default_dir = Path.cwd() / "run_data"
        try:
            default_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            messagebox.showerror("文件创建失败", f"无法创建 run_data 目录: {exc}")
            return

        experiment_id = time.strftime("linearity_%Y%m%d_%H%M%S")
        self.linearity_path = default_dir / f"{experiment_id}.csv"
        self.linearity_path_var.set(str(self.linearity_path))
        self.linearity_rows = 0
        self.linearity_experiment_id = experiment_id
        self.linearity_states = states
        self.linearity_motors = motors
        self.linearity_deltas = deltas
        self.linearity_repeats = repeats
        self.linearity_hold_s = hold_s
        self.linearity_timeout_s = timeout_s
        self.linearity_state_index = 0
        self.linearity_trials = []
        self.linearity_trial_index = 0
        self.linearity_current_trial = None
        self.linearity_baseline_sample_index = 0
        self.linearity_baseline_label = ""
        self.linearity_baseline_values = {}
        self.linearity_baseline_abs_by_motor = {}
        self.linearity_expected_abs_by_motor = {}
        self.linearity_started_monotonic = time.monotonic()
        self.linearity_prev_degree_force_enabled = bool(self.degree_force_enabled_var.get())
        if self.degree_force_active:
            self._stop_degree_force_control("linearity motor-position experiment requested", send_stop=False)
        self.degree_force_enabled_var.set(False)
        self.linearity_running = True
        self._set_linearity_controls_running(True)
        self._set_linearity_scan_ready(False)
        self._linearity_enter_phase("move_state")
        self.linearity_status_var.set(
            f"线性判别启动: {len(states)}个初始状态, {len(motors)}个电机, "
            f"{len(deltas)}个扰动, repeat={repeats}"
        )
        self.after(50, self._linearity_experiment_tick)

    def _stop_linearity_experiment(self, reason: str = "", send_cleanup: bool = True) -> None:
        was_running = self.linearity_running
        self.linearity_running = False
        self.linearity_phase = "idle"
        self.linearity_expected_abs_by_motor = {}
        self._set_linearity_controls_running(False)
        self._set_linearity_scan_ready(False)
        self.degree_force_enabled_var.set(self.linearity_prev_degree_force_enabled)
        if self.degree_force_active:
            self._stop_degree_force_control("linearity experiment stopped", send_stop=False)
        if send_cleanup:
            window = self._active_encoder_servo_window()
            if window is not None and window.poller is not None:
                window.queue_emergency_stop()
        suffix = f": {reason}" if reason else ""
        if was_running:
            self.linearity_status_var.set(f"线性判别已停止{suffix}; rows={self.linearity_rows}")

    def _finish_linearity_experiment(self) -> None:
        rows = self.linearity_rows
        path = self.linearity_path
        self._stop_linearity_experiment("完成", send_cleanup=True)
        self.linearity_status_var.set(f"线性判别完成: {rows}行")
        if path is not None:
            self.status_var.set(f"线性判别数据已写入 {path}")
            self.linearity_analysis_csv_path = path
            self.linearity_analysis_csv_var.set(path.name)
            self.linearity_analysis_status_var.set("实验CSV已自动载入，可执行步骤3、4、5。")

    def _linearity_enter_phase(self, phase: str) -> None:
        self.linearity_phase = phase
        self.linearity_phase_started_monotonic = time.monotonic()
        self.linearity_next_trace_sample_monotonic = self.linearity_phase_started_monotonic
        self.linearity_phase_command_sent = False
        self._linearity_reset_stability()

    def _linearity_write_trace_if_due(
        self,
        snapshot: EncoderServoSnapshot,
        phase: str,
        trial: Optional[tuple[int, int, int]] = None,
    ) -> bool:
        now = time.monotonic()
        if now < self.linearity_next_trace_sample_monotonic:
            return True
        if not self._write_linearity_experiment_row(snapshot, phase, trial):
            return False
        self.linearity_next_trace_sample_monotonic = now + LINEARITY_TRACE_SAMPLE_INTERVAL_S
        return True

    def _linearity_reset_stability(self) -> None:
        self.linearity_stability_anchor_joints = None
        self.linearity_stability_anchor_forces = None
        self.linearity_stability_since_monotonic = 0.0
        self.linearity_last_stable_hold_s = 0.0
        self.linearity_last_max_joint_delta_deg = 0.0
        self.linearity_last_max_force_delta_n = 0.0
        self.linearity_last_max_motor_speed = 0

    def _linearity_current_values(
        self,
        snapshot: EncoderServoSnapshot,
        required_joints: Optional[set[int]] = None,
    ) -> tuple[Optional[dict[str, dict[int, float]]], str]:
        now_wall = time.time()
        if now_wall - snapshot.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
            return None, "Encoder / Servo 数据过期"

        if required_joints is None:
            required_joints = set(MANUAL_RECORD_JOINTS)

        joints: dict[int, float] = {}
        invalid_required_joints: list[int] = []
        for joint in MANUAL_RECORD_JOINTS:
            if snapshot.encoder_valid[joint]:
                joints[joint] = float(snapshot.encoder_deg[joint])
            elif joint in required_joints:
                invalid_required_joints.append(joint)
        if invalid_required_joints:
            return (
                None,
                "必需关节角度无效: "
                + ", ".join(f"J{joint:02d}" for joint in invalid_required_joints),
            )
        if not joints:
            return None, "J00-J03 都没有有效角度"

        channel_by_motor = {motor: channel for channel, motor in FORCE_CHANNEL_TO_MOTOR.items()}
        tensions: dict[int, float] = {}
        for motor in training_target_motor_indices():
            channel = channel_by_motor.get(motor)
            if channel is None:
                return None, f"M{motor:02d} 缺少力传感器映射"
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                return None, f"CH{channel}/M{motor:02d} 缺少力传感器数据"
            if now_wall - sample.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
                return None, f"CH{channel}/M{motor:02d} 力传感器数据过期"
            tensions[motor] = self._relative_value(sample.value, channel) / RELATIVE_UNITS_PER_NEWTON

        servo_abs = {
            motor: float(snapshot.servo_abs[motor])
            for motor in training_target_motor_indices()
        }
        return {"joint": joints, "tension": tensions, "servo_abs": servo_abs}, ""

    def _linearity_update_observations(self, snapshot: EncoderServoSnapshot) -> tuple[bool, str]:
        values, error = self._linearity_current_values(snapshot)
        if values is None:
            return False, error

        joints = values["joint"]
        tensions = values["tension"]
        if self.linearity_stability_anchor_joints is None or self.linearity_stability_anchor_forces is None:
            max_joint_delta = 0.0
            max_force_delta = 0.0
        else:
            joint_keys = set(joints) & set(self.linearity_stability_anchor_joints)
            max_joint_delta = max(
                (
                    abs(joints[joint] - self.linearity_stability_anchor_joints[joint])
                    for joint in joint_keys
                ),
                default=0.0,
            )
            max_force_delta = max(
                (
                    abs(tensions[motor] - self.linearity_stability_anchor_forces[motor])
                    for motor in tensions
                ),
                default=0.0,
            )
        max_motor_speed = max(
            abs(int(snapshot.servo_speed[motor]))
            for motor in training_target_motor_indices()
        )
        self.linearity_stability_anchor_joints = dict(joints)
        self.linearity_stability_anchor_forces = dict(tensions)
        self.linearity_last_max_joint_delta_deg = max_joint_delta
        self.linearity_last_max_force_delta_n = max_force_delta
        self.linearity_last_max_motor_speed = max_motor_speed
        return (
            True,
            f"record dJ/100ms={max_joint_delta:.2f}deg, "
            f"dF/100ms={max_force_delta:.2f}N, vMmax={max_motor_speed}",
        )

    def _linearity_update_position_hold(self, target_close: bool) -> tuple[bool, str]:
        now = time.monotonic()
        if target_close:
            if self.linearity_stability_since_monotonic <= 0.0:
                self.linearity_stability_since_monotonic = now
            held_s = now - self.linearity_stability_since_monotonic
        else:
            self.linearity_stability_since_monotonic = 0.0
            held_s = 0.0
        self.linearity_last_stable_hold_s = held_s
        return (
            held_s >= self.linearity_hold_s,
            f"position hold={held_s:.1f}/{self.linearity_hold_s:.1f}s",
        )

    def _linearity_state_target_closeness(self, snapshot: EncoderServoSnapshot) -> tuple[bool, str]:
        if not (0 <= self.linearity_state_index < len(self.linearity_states)):
            return False, "没有当前初始电机位置目标"
        return self._linearity_servo_target_closeness(snapshot)

    def _linearity_state_label(self) -> str:
        return f"S{self.linearity_state_index + 1:02d}"

    def _linearity_build_trials(self) -> list[tuple[int, int, int]]:
        return [
            (repeat, motor, delta)
            for repeat in range(1, self.linearity_repeats + 1)
            for motor in self.linearity_motors
            for delta in self.linearity_deltas
        ]

    def _send_linearity_state_targets(self, window: EncoderServoWindow, targets: dict[int, int]) -> None:
        self._queue_linearity_direct_targets(window, targets, 0)

    def _freeze_degree_force_updates_for_linearity(self) -> None:
        if not self.degree_force_active:
            return
        self.degree_force_active = False
        self._set_degree_force_controls_active(False)
        self.degree_force_status_var.set(
            f"Degree force frozen for linearity experiment; moves={self.degree_force_move_count}"
        )

    def _queue_linearity_direct_targets(
        self,
        window: EncoderServoWindow,
        targets_by_motor: dict[int, int],
        actual_delta_counts: int,
    ) -> None:
        commands = ["tension off", "direct"]
        window.queue_text_command("tension off")
        window.queue_text_command("direct")
        for motor in sorted(targets_by_motor):
            target = int(targets_by_motor[motor])
            window.sent_direct_targets[motor] = target
            commands.append(f"M{motor:02d}={target}")
        full_targets = window._current_host_direct_targets()
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(full_targets),
            label="linearity direct START snapshot",
        )
        window.queue_text_command("start")
        commands.append("start")
        self.linearity_expected_abs_by_motor = {
            motor: int(target)
            for motor, target in targets_by_motor.items()
        }
        self.linearity_last_command = " | ".join(commands)
        self.linearity_last_actual_delta_counts = int(actual_delta_counts)

    def _queue_linearity_direct_baseline(self, window: EncoderServoWindow) -> None:
        targets = {
            motor: int(target)
            for motor, target in self.linearity_baseline_abs_by_motor.items()
        }
        self._queue_linearity_direct_targets(window, targets, 0)

    def _queue_linearity_perturbation(
        self,
        window: EncoderServoWindow,
        motor: int,
        delta_counts: int,
    ) -> None:
        base = int(self.linearity_baseline_abs_by_motor[motor])
        target = max(FORCE_TARGET_MIN_ABS, min(FORCE_TARGET_MAX_ABS, base + int(delta_counts)))
        actual_delta = int(target - base)
        targets = {
            candidate: int(baseline)
            for candidate, baseline in self.linearity_baseline_abs_by_motor.items()
        }
        targets[motor] = int(target)
        self._queue_linearity_direct_targets(window, targets, actual_delta)

    def _linearity_check_perturbation_safety(
        self,
        snapshot: EncoderServoSnapshot,
        *,
        check_servo_deviation: bool,
    ) -> tuple[bool, str]:
        values, error = self._linearity_current_values(snapshot)
        if values is None:
            return False, error

        limit_error = self._linearity_servo_limit_error(snapshot)
        if limit_error:
            return False, limit_error

        tight = [
            (motor, tension)
            for motor, tension in sorted(values["tension"].items())
            if tension <= LINEARITY_TIGHT_ABORT_N
        ]
        if tight:
            details = ", ".join(f"M{motor:02d}={tension:.1f}N" for motor, tension in tight)
            return False, f"线性实验安全停止: 绳张力过紧 {details}"

        if check_servo_deviation and self.linearity_expected_abs_by_motor:
            deviations: list[tuple[int, int, int, int]] = []
            for motor, target in sorted(self.linearity_expected_abs_by_motor.items()):
                online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
                if not online:
                    return False, f"线性实验安全停止: M{motor:02d} offline"
                actual = int(snapshot.servo_abs[motor])
                error_counts = actual - int(target)
                if abs(error_counts) > LINEARITY_SERVO_DEVIATION_ABORT_COUNTS:
                    deviations.append((motor, actual, int(target), error_counts))
            if deviations:
                details = ", ".join(
                    f"M{motor:02d} actual={actual} target={target} err={error_counts:+d}"
                    for motor, actual, target, error_counts in deviations
                )
                return False, f"线性实验安全停止: 电机偏离目标超过 {LINEARITY_SERVO_DEVIATION_ABORT_COUNTS} counts; {details}"

        return True, ""

    def _linearity_servo_target_closeness(self, snapshot: EncoderServoSnapshot) -> tuple[bool, str]:
        if not self.linearity_expected_abs_by_motor:
            return False, "没有当前电机目标"
        errors: list[tuple[int, int]] = []
        for motor, target in sorted(self.linearity_expected_abs_by_motor.items()):
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]):
                return False, f"M{motor:02d} offline"
            errors.append((motor, int(snapshot.servo_abs[motor]) - int(target)))
        max_motor, max_error = max(errors, key=lambda item: abs(item[1]))
        close = abs(max_error) <= LINEARITY_SERVO_TARGET_TOLERANCE_COUNTS
        return (
            close,
            f"target err max=M{max_motor:02d} {max_error:+d}/"
            f"{LINEARITY_SERVO_TARGET_TOLERANCE_COUNTS} counts",
        )

    def _linearity_servo_limit_error(self, snapshot: EncoderServoSnapshot) -> str:
        near_limits: list[str] = []
        for motor in training_target_motor_indices():
            actual = int(snapshot.servo_abs[motor])
            low_margin = actual - FORCE_TARGET_MIN_ABS
            high_margin = FORCE_TARGET_MAX_ABS - actual
            if low_margin < LINEARITY_SERVO_LIMIT_MARGIN_COUNTS:
                near_limits.append(
                    f"M{motor:02d}={actual} 距下限{low_margin} counts"
                )
            elif high_margin < LINEARITY_SERVO_LIMIT_MARGIN_COUNTS:
                near_limits.append(
                    f"M{motor:02d}={actual} 距上限{high_margin} counts"
                )
        if not near_limits:
            return ""
        return "线性实验安全停止: 电机接近位置限位; " + ", ".join(near_limits)

    def _write_linearity_experiment_row(
        self,
        snapshot: EncoderServoSnapshot,
        phase: str,
        trial: Optional[tuple[int, int, int]] = None,
    ) -> bool:
        if self.linearity_path is None:
            return False
        fieldnames = linearity_experiment_csv_fieldnames()
        sample_index = self.linearity_rows + 1
        repeat_id = ""
        motor_text = ""
        delta_text = ""
        actual_delta_text = ""
        trial_index_text = ""
        sequence_step_text = ""
        if trial is not None:
            repeat_id, motor, delta_counts = trial
            motor_text = f"M{motor:02d}"
            delta_text = str(delta_counts)
            actual_delta_text = str(self.linearity_last_actual_delta_counts)
            trial_index_text = str(self.linearity_trial_index + 1)
            sequence_step_text = str((self.linearity_trial_index % max(1, len(self.linearity_deltas))) + 1)

        row = self._build_manual_record_row(snapshot)
        row["sample_index"] = sample_index
        state_targets = self.linearity_states[self.linearity_state_index]
        baseline_sample_index = sample_index if phase == "baseline" else self.linearity_baseline_sample_index
        row.update(
            {
                "experiment_id": self.linearity_experiment_id,
                "experiment_sample_index": sample_index,
                "phase": phase,
                "state_i": self.linearity_state_index + 1,
                "state_label": self._linearity_state_label(),
                "state_target": format_linearity_state_motor_positions(state_targets),
                "motor_j": motor_text,
                "delta_counts": delta_text,
                "actual_delta_counts": actual_delta_text,
                "repeat_id": repeat_id,
                "trial_index": trial_index_text,
                "sequence_step": sequence_step_text,
                "is_baseline": int(phase == "baseline"),
                "baseline_sample_index": baseline_sample_index,
                "baseline_label": self.linearity_baseline_label,
                "settle_elapsed_s": f"{time.monotonic() - self.linearity_phase_started_monotonic:.3f}",
                "stable_hold_s": f"{self.linearity_last_stable_hold_s:.3f}",
                "max_joint_delta_deg": f"{self.linearity_last_max_joint_delta_deg:.3f}",
                "max_force_delta_n": f"{self.linearity_last_max_force_delta_n:.3f}",
                "max_motor_speed": int(self.linearity_last_max_motor_speed),
                "command": self.linearity_last_command,
            }
        )

        values, error = self._linearity_current_values(snapshot)
        if values is None:
            self._stop_linearity_experiment(error)
            return False
        if self.linearity_baseline_values:
            for joint in MANUAL_RECORD_JOINTS:
                baseline = self.linearity_baseline_values.get("joint", {}).get(joint)
                if baseline is not None and joint in values["joint"]:
                    row[f"delta_joint_j{joint:02d}_deg"] = f"{values['joint'][joint] - baseline:.3f}"
            for motor in training_target_motor_indices():
                baseline = self.linearity_baseline_values.get("tension", {}).get(motor)
                if baseline is not None:
                    row[f"delta_tension_m{motor:02d}_n"] = f"{values['tension'][motor] - baseline:.3f}"
                servo_baseline = self.linearity_baseline_values.get("servo_abs", {}).get(motor)
                if servo_baseline is not None:
                    row[f"delta_servo_m{motor:02d}_abs"] = int(values["servo_abs"][motor] - servo_baseline)
        for motor in training_target_motor_indices():
            target_abs = self.linearity_expected_abs_by_motor.get(motor)
            if target_abs is not None:
                row[f"target_servo_m{motor:02d}_abs"] = int(target_abs)
                row[f"target_error_m{motor:02d}_counts"] = int(values["servo_abs"][motor] - target_abs)

        try:
            write_header = not self.linearity_path.exists() or self.linearity_path.stat().st_size == 0
            encoding = "utf-8-sig" if write_header else "utf-8"
            with open(self.linearity_path, "a", newline="", encoding=encoding) as fp:
                writer = csv.DictWriter(fp, fieldnames=fieldnames, extrasaction="ignore", restval="")
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
        except Exception as exc:
            self._stop_linearity_experiment(f"写入失败: {exc}")
            return False

        self.linearity_rows += 1
        if phase == "baseline":
            self.linearity_baseline_sample_index = sample_index
        return True

    def _linearity_experiment_tick(self) -> None:
        if not self.linearity_running:
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_linearity_experiment("Encoder / Servo 已断开", send_cleanup=False)
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(
                int(LINEARITY_TRACE_SAMPLE_INTERVAL_S * 1000),
                self._linearity_experiment_tick,
            )
            return

        phase = self.linearity_phase
        now = time.monotonic()
        if phase in ("wait_state_stable", "wait_perturb_stable"):
            if now - self.linearity_phase_started_monotonic > self.linearity_timeout_s:
                self._stop_linearity_experiment(f"{phase} 超时 {self.linearity_timeout_s:g}s")
                return

        if phase == "move_state":
            if self.linearity_state_index >= len(self.linearity_states):
                self._finish_linearity_experiment()
                return
            self.linearity_baseline_sample_index = 0
            self.linearity_baseline_label = ""
            self.linearity_baseline_values = {}
            self.linearity_baseline_abs_by_motor = {}
            targets = self.linearity_states[self.linearity_state_index]
            self._send_linearity_state_targets(window, targets)
            self._linearity_enter_phase("wait_state_stable")
            self.linearity_status_var.set(
                f"{self._linearity_state_label()} 已发送五路 ABS 初始目标，"
                "等待电机到位；角度、力和速度仅记录"
            )

        elif phase == "wait_state_stable":
            safe, safety_text = self._linearity_check_perturbation_safety(
                snapshot,
                check_servo_deviation=False,
            )
            if not safe:
                self._stop_linearity_experiment(safety_text)
                return
            observed, observation_text = self._linearity_update_observations(snapshot)
            if not observed:
                self._stop_linearity_experiment(observation_text)
                return
            target_close, target_text = self._linearity_state_target_closeness(snapshot)
            hold_complete, hold_text = self._linearity_update_position_hold(target_close)
            self.linearity_status_var.set(
                f"{self._linearity_state_label()} 初始位置检测: {target_text}; "
                f"{hold_text}; {observation_text}"
            )
            if hold_complete:
                values, error = self._linearity_current_values(snapshot)
                if values is None:
                    self._stop_linearity_experiment(error)
                    return
                self.linearity_baseline_values = {
                    key: dict(value)
                    for key, value in values.items()
                }
                self.linearity_baseline_abs_by_motor = {
                    motor: int(snapshot.servo_abs[motor])
                    for motor in training_target_motor_indices()
                }
                self.linearity_baseline_label = f"{self._linearity_state_label()}_0"
                self.linearity_last_command = "baseline"
                self.linearity_last_actual_delta_counts = 0
                limit_error = self._linearity_servo_limit_error(snapshot)
                if not self._write_linearity_experiment_row(snapshot, "baseline"):
                    return
                if limit_error:
                    self._stop_linearity_experiment(limit_error)
                    return
                self.linearity_expected_abs_by_motor = dict(self.linearity_baseline_abs_by_motor)
                self._linearity_enter_phase("baseline_ready")
                self._set_linearity_scan_ready(True)
                self.linearity_status_var.set(
                    f"步骤1完成: {self.linearity_baseline_label} 已记录，"
                    "基准为实际 ABS；核对状态后点击“步骤2 执行九点扫描”"
                )
                return
            if not self._linearity_write_trace_if_due(snapshot, "baseline_trace"):
                return

        elif phase == "apply_perturb":
            if self.linearity_trial_index >= len(self.linearity_trials):
                self.linearity_state_index += 1
                self._linearity_enter_phase("move_state")
                self.linearity_status_var.set("当前初始状态扰动完成，准备下一组初始 ABS")
            else:
                self.linearity_current_trial = self.linearity_trials[self.linearity_trial_index]
                repeat_id, motor, delta_counts = self.linearity_current_trial
                self._queue_linearity_perturbation(window, motor, delta_counts)
                self._linearity_enter_phase("wait_perturb_stable")
                self.linearity_status_var.set(
                    f"{self._linearity_state_label()} M{motor:02d} {delta_counts:+d} counts "
                    f"repeat {repeat_id}/{self.linearity_repeats}, "
                    f"step {self.linearity_trial_index + 1}/{len(self.linearity_trials)}，等待稳定"
                )

        elif phase == "wait_perturb_stable":
            check_servo_deviation = (
                now - self.linearity_phase_started_monotonic
                >= LINEARITY_SERVO_DEVIATION_GRACE_S
            )
            safe, safety_text = self._linearity_check_perturbation_safety(
                snapshot,
                check_servo_deviation=check_servo_deviation,
            )
            if not safe:
                self._stop_linearity_experiment(safety_text)
                return
            observed, observation_text = self._linearity_update_observations(snapshot)
            if not observed:
                self._stop_linearity_experiment(observation_text)
                return
            target_close, target_text = self._linearity_servo_target_closeness(snapshot)
            hold_complete, hold_text = self._linearity_update_position_hold(target_close)
            self.linearity_status_var.set(
                f"序列点到位检测: {target_text}; {hold_text}; {observation_text}"
            )
            if hold_complete:
                if not self._write_linearity_experiment_row(snapshot, "perturb", self.linearity_current_trial):
                    return
                hold_targets = dict(self.linearity_expected_abs_by_motor)
                self._queue_linearity_direct_targets(
                    window,
                    hold_targets,
                    self.linearity_last_actual_delta_counts,
                )
                self._linearity_enter_phase("post_perturb_hold")
                self.linearity_status_var.set(
                    f"当前扰动最终点已记录，已重新下发同一五路目标；"
                    f"继续保持 {LINEARITY_POST_RECORD_HOLD_S:.1f}s 后进入下一点"
                )
            elif not self._linearity_write_trace_if_due(
                snapshot,
                "perturb_trace",
                self.linearity_current_trial,
            ):
                return

        elif phase == "post_perturb_hold":
            safe, safety_text = self._linearity_check_perturbation_safety(
                snapshot,
                check_servo_deviation=(
                    now - self.linearity_phase_started_monotonic
                    >= LINEARITY_SERVO_DEVIATION_GRACE_S
                ),
            )
            if not safe:
                self._stop_linearity_experiment(safety_text)
                return
            observed, observation_text = self._linearity_update_observations(snapshot)
            if not observed:
                self._stop_linearity_experiment(observation_text)
                return
            hold_elapsed_s = now - self.linearity_phase_started_monotonic
            self.linearity_last_stable_hold_s = hold_elapsed_s
            self.linearity_status_var.set(
                f"最终点后保持原目标: {hold_elapsed_s:.1f}/"
                f"{LINEARITY_POST_RECORD_HOLD_S:.1f}s; {observation_text}"
            )
            if hold_elapsed_s >= LINEARITY_POST_RECORD_HOLD_S:
                if not self._write_linearity_experiment_row(
                    snapshot,
                    "perturb_post_hold",
                    self.linearity_current_trial,
                ):
                    return
                self.linearity_trial_index += 1
                self._linearity_enter_phase("apply_perturb")
            elif not self._linearity_write_trace_if_due(
                snapshot,
                "perturb_post_hold_trace",
                self.linearity_current_trial,
            ):
                return

        if self.linearity_running:
            self.after(
                int(LINEARITY_TRACE_SAMPLE_INTERVAL_S * 1000),
                self._linearity_experiment_tick,
            )

    def _parse_multi_input_settings(
        self,
    ) -> tuple[
        dict[int, int],
        str,
        list[float],
        int,
        int,
        float,
        float,
        float,
        float,
        tuple[float, float, float],
        float,
        list[float],
    ]:
        try:
            states = parse_linearity_state_motor_positions(self.multi_input_state_var.get())
            mode = self.multi_input_mode_var.get().strip().lower()
            amplitudes = parse_number_vector(
                self.multi_input_amplitudes_var.get(), 5, name="五路扰动幅值"
            )
            count = int(self.multi_input_count_var.get())
            seed = int(self.multi_input_seed_var.get())
            hold_s = float(self.multi_input_hold_var.get().strip())
            post_hold_s = float(self.multi_input_post_hold_var.get().strip())
            timeout_s = float(self.multi_input_timeout_var.get().strip())
            record_interval_s = int(self.multi_input_record_interval_var.get()) / 1000.0
            ratios = parse_split_ratios(self.multi_input_split_var.get())
            lambda_id = float(self.multi_input_lambda_var.get().strip())
            frequencies = parse_number_vector(
                self.multi_input_frequencies_var.get(), 5, name="多正弦五路频率"
            )
        except (tk.TclError, ValueError) as exc:
            raise ValueError(f"联合辨识参数错误: {exc}") from exc
        if len(states) != 1:
            raise ValueError("联合辨识一次只支持一个初始工作点")
        if mode not in {"random", "prbs", "multisine"}:
            raise ValueError("模式只能选择 random、prbs 或 multisine")
        if count < 30:
            raise ValueError("为保证三组数据和五路满秩，N 至少设为 30")
        if any(amplitude <= 0.0 for amplitude in amplitudes):
            raise ValueError("五路幅值都必须大于 0 counts")
        if hold_s <= 0.0 or post_hold_s <= 0.0:
            raise ValueError("到位停留和终点保持时间都必须大于 0")
        if timeout_s <= hold_s:
            raise ValueError("超时时间必须大于到位停留时间")
        if not 0.05 <= record_interval_s <= 2.0:
            raise ValueError("记录周期必须在 50~2000 ms")
        if not (0.0 <= lambda_id < float("inf")):
            raise ValueError("岭回归 λ 必须是非负有限数")
        if any(frequency <= 0.0 for frequency in frequencies):
            raise ValueError("五路多正弦频率都必须大于 0")
        labels = split_labels(count, ratios)
        if labels.count("train") < 10:
            raise ValueError("训练集至少需要 10 个样本，请增加 N 或训练比例")
        return (
            states[0],
            mode,
            amplitudes,
            count,
            seed,
            hold_s,
            post_hold_s,
            timeout_s,
            record_interval_s,
            ratios,
            lambda_id,
            frequencies,
        )

    def _set_multi_input_controls_running(self, running: bool) -> None:
        editable_state = "disabled" if running else "normal"
        for attr in (
            "multi_input_state_entry",
            "multi_input_amplitudes_entry",
            "multi_input_count_spin",
            "multi_input_seed_entry",
            "multi_input_hold_entry",
            "multi_input_post_hold_entry",
            "multi_input_timeout_entry",
            "multi_input_record_interval_spin",
            "multi_input_split_entry",
            "multi_input_lambda_entry",
            "multi_input_frequencies_entry",
        ):
            widget = getattr(self, attr, None)
            if widget is not None:
                widget.configure(state=editable_state)
        if hasattr(self, "multi_input_mode_combo"):
            self.multi_input_mode_combo.configure(state="disabled" if running else "readonly")
        if hasattr(self, "multi_input_start_button"):
            self.multi_input_start_button.configure(state="disabled" if running else "normal")
        if hasattr(self, "multi_input_stop_button"):
            self.multi_input_stop_button.configure(state="normal" if running else "disabled")
        if hasattr(self, "multi_input_scan_button"):
            self.multi_input_scan_button.configure(state="disabled")

    def _set_multi_input_scan_ready(self, ready: bool) -> None:
        if hasattr(self, "multi_input_scan_button"):
            self.multi_input_scan_button.configure(
                state="normal" if ready and self.multi_input_running else "disabled"
            )

    def _start_multi_input_experiment(self) -> None:
        if self.multi_input_running:
            return
        if self._automatic_angle_test_is_running():
            messagebox.showinfo("自动角度测试运行中", "请先停止自动阶跃/正弦测试，再开始联合辨识。")
            return
        if self.linearity_running:
            messagebox.showinfo("局部实验运行中", "请先完成或停止局部线性辨识实验。")
            return
        if self.training_running:
            messagebox.showinfo("训练采集中", "请先停止训练采集，再开始联合辨识。")
            return
        if self.safe_relax_active:
            messagebox.showinfo("安全放松中", "请先停止安全放松，再开始联合辨识。")
            return
        if self.poller is None:
            messagebox.showinfo("力传感器未连接", "请先连接力传感器串口。")
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("Encoder / Servo 未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待 Encoder / Servo 数据", "请等待第一帧关节角度和电机数据。")
            return

        try:
            (
                state,
                mode,
                amplitudes,
                count,
                seed,
                hold_s,
                post_hold_s,
                timeout_s,
                record_interval_s,
                ratios,
                lambda_id,
                frequencies,
            ) = self._parse_multi_input_settings()
        except ValueError as exc:
            messagebox.showwarning("联合辨识参数", str(exc))
            return

        values, value_error = self._linearity_current_values(snapshot)
        if values is None:
            messagebox.showinfo("等待有效数据", value_error)
            return
        offline = [
            motor
            for motor in training_target_motor_indices()
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            messagebox.showwarning(
                "电机未在线", "联合辨识要求 M00-M04 全部在线: " + ", ".join(f"M{x:02d}" for x in offline)
            )
            return

        safe_low = FORCE_TARGET_MIN_ABS + LINEARITY_SERVO_LIMIT_MARGIN_COUNTS
        safe_high = FORCE_TARGET_MAX_ABS - LINEARITY_SERVO_LIMIT_MARGIN_COUNTS
        for motor in training_target_motor_indices():
            amplitude = int(math.ceil(amplitudes[motor]))
            if state[motor] - amplitude < safe_low or state[motor] + amplitude > safe_high:
                messagebox.showwarning(
                    "初始位置或扰动超出安全范围",
                    f"M{motor:02d} 的 {state[motor]}±{amplitude} 超出 {safe_low}~{safe_high}。",
                )
                return

        transition = {
            motor: state[motor] - int(snapshot.servo_abs[motor])
            for motor in training_target_motor_indices()
        }
        if max(abs(delta) for delta in transition.values()) >= DIRECT_ABS_CONFIRM_DELTA_COUNTS:
            summary = "\n".join(
                f"M{motor:02d}: {int(snapshot.servo_abs[motor])} -> {state[motor]} (Δ{transition[motor]:+d})"
                for motor in training_target_motor_indices()
            )
            if not messagebox.askyesno(
                "确认初始位置大幅移动",
                "至少一路到初始工作点的变化达到一圈（4096 counts），确认继续吗？\n\n" + summary,
            ):
                return

        if self.force_target_running:
            self._stop_force_target_control("multi-input experiment requested")
        default_dir = Path.cwd() / "run_data"
        try:
            default_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("文件创建失败", f"无法创建 run_data 目录: {exc}")
            return

        experiment_id = time.strftime("multi_input_%Y%m%d_%H%M%S")
        self.multi_input_experiment_id = experiment_id
        self.multi_input_path = default_dir / f"{experiment_id}.csv"
        self.multi_input_path_var.set(str(self.multi_input_path))
        self.multi_input_rows = 0
        self.multi_input_state = dict(state)
        self.multi_input_mode = mode
        self.multi_input_amplitudes = list(amplitudes)
        self.multi_input_count = count
        self.multi_input_seed = seed
        self.multi_input_hold_s = hold_s
        self.multi_input_post_hold_s = post_hold_s
        self.multi_input_timeout_s = timeout_s
        self.multi_input_record_interval_s = record_interval_s
        self.multi_input_split_ratios = ratios
        self.multi_input_lambda = lambda_id
        self.multi_input_frequencies = list(frequencies)
        self.multi_input_sequence = []
        self.multi_input_split_labels = split_labels(count, ratios)
        self.multi_input_sequence_index = 0
        self.multi_input_current_delta = [0] * 5
        self.multi_input_baseline_sample_index = 0
        self.multi_input_baseline_values = {}
        self.multi_input_baseline_abs_by_motor = {}
        self.multi_input_expected_abs_by_motor = {}
        self.multi_input_analysis_markdown = ""
        self.multi_input_analysis_output_dir = None
        self.multi_input_analysis_status_var.set("结果: 等待实验完成")
        self.multi_input_prev_degree_force_enabled = bool(self.degree_force_enabled_var.get())
        if self.degree_force_active:
            self._stop_degree_force_control("multi-input motor-position experiment requested", send_stop=False)
        self.degree_force_enabled_var.set(False)
        self.multi_input_running = True
        self._set_multi_input_controls_running(True)
        self._set_multi_input_scan_ready(False)
        self._multi_input_enter_phase("move_baseline")
        self.multi_input_status_var.set(
            f"联合辨识启动: mode={mode}, N={count}；正在移动到初始 ABS"
        )
        self.after(50, self._multi_input_experiment_tick)

    def _start_multi_input_scan(self) -> None:
        if not self.multi_input_running:
            messagebox.showinfo("尚未记录基准", "请先点击“步骤1 到位并记录”。")
            return
        if self.multi_input_phase != "baseline_ready":
            messagebox.showinfo("基准尚未就绪", self.multi_input_status_var.get())
            return
        try:
            sequence = build_multi_input_sequence(
                self.multi_input_mode,
                self.multi_input_amplitudes,
                self.multi_input_count,
                self.multi_input_seed,
                step_period_s=self.multi_input_hold_s + self.multi_input_post_hold_s,
                frequencies_hz=self.multi_input_frequencies,
            )
            train_count = self.multi_input_split_labels.count("train")
            full_metrics = input_independence_metrics(sequence)
            train_metrics = input_independence_metrics(sequence[:train_count])
        except ValueError as exc:
            messagebox.showwarning("联合激励序列", str(exc))
            return
        if train_metrics.rank < 5:
            messagebox.showwarning(
                "输入不独立",
                f"训练段输入矩阵 rank={train_metrics.rank}/5；请增加 N、修改 seed 或频率。",
            )
            return
        if train_metrics.max_abs_correlation >= MULTI_INPUT_MAX_CORRELATION:
            messagebox.showwarning(
                "输入通道相关性过高",
                f"训练段最大 |r|={train_metrics.max_abs_correlation:.3f}，阈值为 "
                f"{MULTI_INPUT_MAX_CORRELATION:.2f}；请修改 seed、N 或频率。",
            )
            return
        self.multi_input_sequence = sequence.astype(int).tolist()
        self.multi_input_sequence_index = 0
        self._set_multi_input_scan_ready(False)
        self._multi_input_enter_phase("apply_excitation")
        self.multi_input_status_var.set(
            f"步骤2启动: N={len(self.multi_input_sequence)}, full rank={full_metrics.rank}/5, "
            f"train max|r|={train_metrics.max_abs_correlation:.3f}"
        )
        self.after(0, self._multi_input_experiment_tick)

    def _stop_multi_input_experiment(self, reason: str = "", send_cleanup: bool = True) -> None:
        was_running = self.multi_input_running
        self.multi_input_running = False
        self.multi_input_phase = "idle"
        self.multi_input_expected_abs_by_motor = {}
        self._set_multi_input_controls_running(False)
        self._set_multi_input_scan_ready(False)
        self.degree_force_enabled_var.set(self.multi_input_prev_degree_force_enabled)
        if self.degree_force_active:
            self._stop_degree_force_control("multi-input experiment stopped", send_stop=False)
        if send_cleanup:
            window = self._active_encoder_servo_window()
            if window is not None and window.poller is not None:
                window.queue_emergency_stop()
        if was_running:
            suffix = f": {reason}" if reason else ""
            self.multi_input_status_var.set(f"联合辨识已停止{suffix}; rows={self.multi_input_rows}")

    def _finish_multi_input_experiment(self) -> None:
        rows = self.multi_input_rows
        path = self.multi_input_path
        self._stop_multi_input_experiment("完成", send_cleanup=True)
        self.multi_input_status_var.set(f"联合辨识完成: {rows}行，已返回初始 ABS")
        if path is not None:
            self.status_var.set(f"联合辨识数据已写入 {path}")
            self._run_multi_input_analysis()

    def _multi_input_enter_phase(self, phase: str) -> None:
        self.multi_input_phase = phase
        self.multi_input_phase_started_monotonic = time.monotonic()
        self.multi_input_next_trace_sample_monotonic = self.multi_input_phase_started_monotonic
        self.multi_input_stability_anchor_joints = None
        self.multi_input_stability_anchor_forces = None
        self.multi_input_stability_since_monotonic = 0.0
        self.multi_input_last_stable_hold_s = 0.0
        self.multi_input_last_max_joint_delta_deg = 0.0
        self.multi_input_last_max_force_delta_n = 0.0
        self.multi_input_last_max_motor_speed = 0

    def _multi_input_current_values(
        self, snapshot: EncoderServoSnapshot
    ) -> tuple[Optional[dict[str, dict[int, float]]], str]:
        return self._linearity_current_values(snapshot)

    def _multi_input_update_observations(self, snapshot: EncoderServoSnapshot) -> tuple[bool, str]:
        values, error = self._multi_input_current_values(snapshot)
        if values is None:
            return False, error
        joints = values["joint"]
        tensions = values["tension"]
        if self.multi_input_stability_anchor_joints is None or self.multi_input_stability_anchor_forces is None:
            max_joint_delta = 0.0
            max_force_delta = 0.0
        else:
            max_joint_delta = max(
                abs(joints[joint] - self.multi_input_stability_anchor_joints[joint])
                for joint in joints
            )
            max_force_delta = max(
                abs(tensions[motor] - self.multi_input_stability_anchor_forces[motor])
                for motor in tensions
            )
        max_speed = max(
            abs(int(snapshot.servo_speed[motor])) for motor in training_target_motor_indices()
        )
        self.multi_input_stability_anchor_joints = dict(joints)
        self.multi_input_stability_anchor_forces = dict(tensions)
        self.multi_input_last_max_joint_delta_deg = max_joint_delta
        self.multi_input_last_max_force_delta_n = max_force_delta
        self.multi_input_last_max_motor_speed = max_speed
        return True, (
            f"record dJ={max_joint_delta:.2f}deg, dF={max_force_delta:.2f}N, vMmax={max_speed}"
        )

    def _multi_input_update_position_hold(self, target_close: bool) -> tuple[bool, str]:
        now = time.monotonic()
        if target_close:
            if self.multi_input_stability_since_monotonic <= 0.0:
                self.multi_input_stability_since_monotonic = now
            held_s = now - self.multi_input_stability_since_monotonic
        else:
            self.multi_input_stability_since_monotonic = 0.0
            held_s = 0.0
        self.multi_input_last_stable_hold_s = held_s
        return held_s >= self.multi_input_hold_s, f"position hold={held_s:.1f}/{self.multi_input_hold_s:.1f}s"

    def _queue_multi_input_targets(
        self, window: EncoderServoWindow, targets_by_motor: dict[int, int]
    ) -> None:
        commands = ["tension off", "direct"]
        window.queue_text_command("tension off")
        window.queue_text_command("direct")
        for motor in sorted(targets_by_motor):
            target = int(targets_by_motor[motor])
            window.sent_direct_targets[motor] = target
            commands.append(f"M{motor:02d}={target}")
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(window._current_host_direct_targets()),
            label="multi-input direct START snapshot",
        )
        window.queue_text_command("start")
        commands.append("start")
        self.multi_input_expected_abs_by_motor = {
            motor: int(target) for motor, target in targets_by_motor.items()
        }
        self.multi_input_last_command = " | ".join(commands)

    def _multi_input_target_closeness(self, snapshot: EncoderServoSnapshot) -> tuple[bool, str]:
        if not self.multi_input_expected_abs_by_motor:
            return False, "没有当前电机目标"
        errors: list[tuple[int, int]] = []
        for motor, target in sorted(self.multi_input_expected_abs_by_motor.items()):
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]):
                return False, f"M{motor:02d} offline"
            errors.append((motor, int(snapshot.servo_abs[motor]) - int(target)))
        max_motor, max_error = max(errors, key=lambda item: abs(item[1]))
        return (
            abs(max_error) <= LINEARITY_SERVO_TARGET_TOLERANCE_COUNTS,
            f"target err max=M{max_motor:02d} {max_error:+d}/{LINEARITY_SERVO_TARGET_TOLERANCE_COUNTS} counts",
        )

    def _multi_input_check_safety(
        self, snapshot: EncoderServoSnapshot
    ) -> tuple[bool, str]:
        values, error = self._multi_input_current_values(snapshot)
        if values is None:
            return False, error
        tight = [
            (motor, tension)
            for motor, tension in sorted(values["tension"].items())
            if tension <= LINEARITY_TIGHT_ABORT_N
        ]
        if tight:
            details = ", ".join(f"M{motor:02d}={tension:.1f}N" for motor, tension in tight)
            return False, f"联合辨识安全停止: 绳张力过紧 {details}"
        return True, ""

    def _multi_input_write_trace_if_due(self, snapshot: EncoderServoSnapshot, phase: str) -> bool:
        now = time.monotonic()
        if now < self.multi_input_next_trace_sample_monotonic:
            return True
        if not self._write_multi_input_experiment_row(snapshot, phase):
            return False
        self.multi_input_next_trace_sample_monotonic = now + self.multi_input_record_interval_s
        return True

    def _write_multi_input_experiment_row(self, snapshot: EncoderServoSnapshot, phase: str) -> bool:
        if self.multi_input_path is None:
            return False
        values, error = self._multi_input_current_values(snapshot)
        if values is None:
            self._stop_multi_input_experiment(error)
            return False
        sample_index = self.multi_input_rows + 1
        sequence_index = self.multi_input_sequence_index + 1 if self.multi_input_sequence else 0
        dataset_split = ""
        if self.multi_input_sequence and self.multi_input_sequence_index < len(self.multi_input_split_labels):
            dataset_split = self.multi_input_split_labels[self.multi_input_sequence_index]
        row = self._build_manual_record_row(snapshot)
        row["sample_index"] = sample_index
        row.update(
            {
                "experiment_id": self.multi_input_experiment_id,
                "experiment_sample_index": sample_index,
                "phase": phase,
                "dataset_split": dataset_split,
                "sequence_index": sequence_index,
                "sequence_count": self.multi_input_count,
                "excitation_mode": self.multi_input_mode,
                "random_seed": self.multi_input_seed,
                "ridge_lambda": f"{self.multi_input_lambda:g}",
                "split_train": f"{self.multi_input_split_ratios[0]:.6f}",
                "split_validation": f"{self.multi_input_split_ratios[1]:.6f}",
                "split_test": f"{self.multi_input_split_ratios[2]:.6f}",
                "state_target": format_linearity_state_motor_positions(self.multi_input_state),
                "is_baseline": int(phase == "baseline"),
                "baseline_sample_index": self.multi_input_baseline_sample_index,
                "settle_elapsed_s": f"{time.monotonic() - self.multi_input_phase_started_monotonic:.3f}",
                "stable_hold_s": f"{self.multi_input_last_stable_hold_s:.3f}",
                "max_joint_delta_deg": f"{self.multi_input_last_max_joint_delta_deg:.3f}",
                "max_force_delta_n": f"{self.multi_input_last_max_force_delta_n:.3f}",
                "max_motor_speed": self.multi_input_last_max_motor_speed,
                "command": self.multi_input_last_command,
            }
        )
        for motor in training_target_motor_indices():
            row[f"command_delta_m{motor:02d}_counts"] = self.multi_input_current_delta[motor]
        if self.multi_input_baseline_values:
            for joint in MANUAL_RECORD_JOINTS:
                baseline = self.multi_input_baseline_values["joint"].get(joint)
                if baseline is not None:
                    row[f"delta_joint_j{joint:02d}_deg"] = f"{values['joint'][joint] - baseline:.6f}"
            for motor in training_target_motor_indices():
                row[f"delta_tension_m{motor:02d}_n"] = (
                    f"{values['tension'][motor] - self.multi_input_baseline_values['tension'][motor]:.6f}"
                )
                row[f"delta_servo_m{motor:02d}_abs"] = int(
                    values["servo_abs"][motor] - self.multi_input_baseline_values["servo_abs"][motor]
                )
        for motor, target in self.multi_input_expected_abs_by_motor.items():
            row[f"target_servo_m{motor:02d}_abs"] = target
            row[f"target_error_m{motor:02d}_counts"] = int(values["servo_abs"][motor] - target)
        if phase == "baseline":
            row["baseline_sample_index"] = sample_index

        try:
            write_header = not self.multi_input_path.exists() or self.multi_input_path.stat().st_size == 0
            encoding = "utf-8-sig" if write_header else "utf-8"
            with open(self.multi_input_path, "a", newline="", encoding=encoding) as fp:
                writer = csv.DictWriter(
                    fp,
                    fieldnames=multi_input_experiment_csv_fieldnames(),
                    extrasaction="ignore",
                    restval="",
                )
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
        except (OSError, csv.Error) as exc:
            self._stop_multi_input_experiment(f"写入失败: {exc}")
            return False
        self.multi_input_rows += 1
        if phase == "baseline":
            self.multi_input_baseline_sample_index = sample_index
        return True

    def _multi_input_experiment_tick(self) -> None:
        if not self.multi_input_running:
            return
        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            self._stop_multi_input_experiment("Encoder / Servo 已断开", send_cleanup=False)
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            self.after(max(50, int(self.multi_input_record_interval_s * 1000)), self._multi_input_experiment_tick)
            return
        phase = self.multi_input_phase
        now = time.monotonic()
        if phase in {"wait_baseline", "wait_excitation", "wait_return_baseline"}:
            if now - self.multi_input_phase_started_monotonic > self.multi_input_timeout_s:
                self._stop_multi_input_experiment(f"{phase} 超时 {self.multi_input_timeout_s:g}s")
                return

        if phase == "move_baseline":
            self.multi_input_current_delta = [0] * 5
            self._queue_multi_input_targets(window, self.multi_input_state)
            self._multi_input_enter_phase("wait_baseline")
            self.multi_input_status_var.set("已发送初始五路 ABS，等待 ±10 counts 内连续保持；角度/力/速度仅记录")

        elif phase == "wait_baseline":
            safe, text = self._multi_input_check_safety(snapshot)
            if not safe:
                self._stop_multi_input_experiment(text)
                return
            observed, observation = self._multi_input_update_observations(snapshot)
            if not observed:
                self._stop_multi_input_experiment(observation)
                return
            close, target_text = self._multi_input_target_closeness(snapshot)
            complete, hold_text = self._multi_input_update_position_hold(close)
            self.multi_input_status_var.set(f"初始位置: {target_text}; {hold_text}; {observation}")
            if complete:
                values, error = self._multi_input_current_values(snapshot)
                if values is None:
                    self._stop_multi_input_experiment(error)
                    return
                self.multi_input_baseline_values = {key: dict(value) for key, value in values.items()}
                self.multi_input_baseline_abs_by_motor = {
                    motor: int(snapshot.servo_abs[motor]) for motor in training_target_motor_indices()
                }
                self.multi_input_expected_abs_by_motor = dict(self.multi_input_baseline_abs_by_motor)
                self.multi_input_last_command = "baseline"
                if not self._write_multi_input_experiment_row(snapshot, "baseline"):
                    return
                self._multi_input_enter_phase("baseline_ready")
                self._set_multi_input_scan_ready(True)
                self.multi_input_status_var.set(
                    "步骤1完成：已记录实际 ABS/四关节角/五路张力；核对后点击“步骤2 执行联合激励”"
                )
                return
            if not self._multi_input_write_trace_if_due(snapshot, "baseline_trace"):
                return

        elif phase == "apply_excitation":
            if self.multi_input_sequence_index >= len(self.multi_input_sequence):
                self.multi_input_current_delta = [0] * 5
                self._queue_multi_input_targets(window, self.multi_input_baseline_abs_by_motor)
                self._multi_input_enter_phase("wait_return_baseline")
                self.multi_input_status_var.set("全部联合激励完成，正在返回同一初始 ABS")
            else:
                self.multi_input_current_delta = list(self.multi_input_sequence[self.multi_input_sequence_index])
                targets = {
                    motor: self.multi_input_baseline_abs_by_motor[motor] + self.multi_input_current_delta[motor]
                    for motor in training_target_motor_indices()
                }
                self._queue_multi_input_targets(window, targets)
                self._multi_input_enter_phase("wait_excitation")
                split = self.multi_input_split_labels[self.multi_input_sequence_index]
                self.multi_input_status_var.set(
                    f"联合点 {self.multi_input_sequence_index + 1}/{len(self.multi_input_sequence)} "
                    f"({split})，等待五路到位"
                )

        elif phase == "wait_excitation":
            safe, text = self._multi_input_check_safety(snapshot)
            if not safe:
                self._stop_multi_input_experiment(text)
                return
            observed, observation = self._multi_input_update_observations(snapshot)
            if not observed:
                self._stop_multi_input_experiment(observation)
                return
            close, target_text = self._multi_input_target_closeness(snapshot)
            complete, hold_text = self._multi_input_update_position_hold(close)
            self.multi_input_status_var.set(
                f"联合点 {self.multi_input_sequence_index + 1}/{len(self.multi_input_sequence)}: "
                f"{target_text}; {hold_text}; {observation}"
            )
            if complete:
                if not self._write_multi_input_experiment_row(snapshot, "excitation"):
                    return
                self._queue_multi_input_targets(window, dict(self.multi_input_expected_abs_by_motor))
                self._multi_input_enter_phase("post_excitation_hold")
                self.multi_input_status_var.set(
                    f"最终点已记录并重新下发相同目标，继续保持 {self.multi_input_post_hold_s:.1f}s"
                )
            elif not self._multi_input_write_trace_if_due(snapshot, "excitation_trace"):
                return

        elif phase == "post_excitation_hold":
            safe, text = self._multi_input_check_safety(snapshot)
            if not safe:
                self._stop_multi_input_experiment(text)
                return
            observed, observation = self._multi_input_update_observations(snapshot)
            if not observed:
                self._stop_multi_input_experiment(observation)
                return
            elapsed = now - self.multi_input_phase_started_monotonic
            self.multi_input_last_stable_hold_s = elapsed
            self.multi_input_status_var.set(
                f"联合点 {self.multi_input_sequence_index + 1}/{len(self.multi_input_sequence)} "
                f"保持 {elapsed:.1f}/{self.multi_input_post_hold_s:.1f}s; {observation}"
            )
            if elapsed >= self.multi_input_post_hold_s:
                if not self._write_multi_input_experiment_row(snapshot, "excitation_post_hold"):
                    return
                self.multi_input_sequence_index += 1
                self._multi_input_enter_phase("apply_excitation")
            elif not self._multi_input_write_trace_if_due(snapshot, "excitation_post_hold_trace"):
                return

        elif phase == "wait_return_baseline":
            safe, text = self._multi_input_check_safety(snapshot)
            if not safe:
                self._stop_multi_input_experiment(text)
                return
            observed, observation = self._multi_input_update_observations(snapshot)
            if not observed:
                self._stop_multi_input_experiment(observation)
                return
            close, target_text = self._multi_input_target_closeness(snapshot)
            complete, hold_text = self._multi_input_update_position_hold(close)
            self.multi_input_status_var.set(f"返回初始点: {target_text}; {hold_text}; {observation}")
            if complete:
                if not self._write_multi_input_experiment_row(snapshot, "final_baseline"):
                    return
                self._finish_multi_input_experiment()
                return
            if not self._multi_input_write_trace_if_due(snapshot, "final_baseline_trace"):
                return

        if self.multi_input_running and self.multi_input_phase != "baseline_ready":
            self.after(max(50, int(self.multi_input_record_interval_s * 1000)), self._multi_input_experiment_tick)

    def _run_multi_input_analysis(self) -> None:
        if self.multi_input_running:
            messagebox.showinfo("实验运行中", "请先完成或停止联合辨识，再分析 CSV。")
            return
        path = self.multi_input_path
        if path is None or not path.is_file():
            messagebox.showinfo("没有联合辨识CSV", "请先完成一次多输入联合辨识实验。")
            return
        try:
            lambda_id = float(self.multi_input_lambda_var.get().strip())
            result = analyze_multi_input_csv(
                path,
                lambda_id,
                Path.cwd() / "run_data" / "multi_input_analysis",
            )
        except (ValueError, MultiInputAnalysisError, OSError, csv.Error) as exc:
            self.multi_input_analysis_status_var.set(f"分析失败: {exc}")
            messagebox.showerror("联合辨识分析失败", str(exc))
            return
        self.multi_input_analysis_markdown = result.markdown
        self.multi_input_analysis_output_dir = result.output_dir
        self.multi_input_analysis_status_var.set(
            f"4×5矩阵完成: N={result.endpoint_rows}, train/val/test="
            f"{result.train_rows}/{result.validation_rows}/{result.test_rows}, "
            f"rank={result.rank}/5, max|r|={result.max_abs_correlation:.3f}; "
            f"{result.markdown_path.name}"
        )

    def _copy_multi_input_analysis_markdown(self) -> None:
        if not self.multi_input_analysis_markdown:
            messagebox.showinfo("没有分析结果", "请先完成实验并执行步骤3。")
            return
        self.clipboard_clear()
        self.clipboard_append(self.multi_input_analysis_markdown)
        self.update_idletasks()
        self.multi_input_analysis_status_var.set("联合辨识 Markdown 已复制。")

    def _open_multi_input_analysis_output(self) -> None:
        output_dir = self.multi_input_analysis_output_dir
        if output_dir is None or not output_dir.exists():
            messagebox.showinfo("没有结果目录", "请先执行步骤3 岭回归与验证。")
            return
        try:
            os.startfile(str(output_dir))
        except (AttributeError, OSError) as exc:
            messagebox.showerror("无法打开目录", str(exc))

    def _record_snapshot_issues(self) -> tuple[Optional[EncoderServoSnapshot], list[str]]:
        issues: list[str] = []
        now = time.time()
        if self.poller is None:
            issues.append("力传感器未连接")

        for channel in sorted(FORCE_CHANNEL_TO_MOTOR):
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                issues.append(f"CH{channel}: no data")
            elif now - sample.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
                issues.append(f"CH{channel}: stale")

        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            issues.append("Encoder / Servo 未连接")
            return None, issues

        snapshot = window.latest_snapshot
        if snapshot is None:
            issues.append("Encoder / Servo: no data")
            return None, issues
        if now - snapshot.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
            issues.append("Encoder / Servo: stale")

        return (snapshot if not issues else None), issues

    def _toggle_continuous_record(self) -> None:
        if self.continuous_record_running:
            self._stop_continuous_record("手动停止")
            return
        self._start_continuous_record()

    def _start_continuous_record(self) -> None:
        if self.continuous_record_running:
            return

        try:
            interval_ms = int(self.continuous_record_interval_var.get())
        except (tk.TclError, ValueError):
            messagebox.showinfo("连续记录间隔错误", "连续记录间隔需要是整数 ms。")
            return
        if interval_ms < 20:
            messagebox.showinfo("连续记录间隔错误", "连续记录间隔不能小于 20 ms。")
            return

        snapshot, issues = self._record_snapshot_issues()
        if snapshot is None:
            messagebox.showinfo("等待记录数据", ", ".join(issues))
            return

        fp: Optional[object] = None
        event_fp: Optional[object] = None
        try:
            default_dir = Path.cwd() / "run_data" / "continuous_records"
            default_dir.mkdir(parents=True, exist_ok=True)
            path, event_path = unique_continuous_record_paths(default_dir)
            # Exclusive creation is intentional: an existing recording must
            # never be truncated, even if the clock or filename collides.
            fp = open(path, "x", newline="", encoding="utf-8-sig")
            event_fp = open(event_path, "x", newline="", encoding="utf-8-sig")
            writer = csv.DictWriter(
                fp,
                fieldnames=continuous_record_csv_fieldnames(),
                extrasaction="ignore",
                restval="",
            )
            writer.writeheader()
            fp.flush()
            event_writer = csv.DictWriter(
                event_fp,
                fieldnames=["timestamp", "local_time", "elapsed_s", "event_type", "motor", "joint", "command"],
                extrasaction="ignore",
                restval="",
            )
            event_writer.writeheader()
            event_fp.flush()
        except Exception as exc:
            if fp is not None:
                try:
                    fp.close()
                except Exception:
                    pass
            if event_fp is not None:
                try:
                    event_fp.close()
                except Exception:
                    pass
            messagebox.showerror("连续记录失败", f"无法创建连续记录 CSV: {exc}")
            return

        self.continuous_record_running = True
        self.continuous_record_file = fp
        self.continuous_record_writer = writer
        self.continuous_record_path = path
        self.continuous_event_file = event_fp
        self.continuous_event_writer = event_writer
        self.continuous_event_path = event_path
        self.continuous_record_rows = 0
        self.continuous_record_interval_ms = interval_ms
        self.continuous_record_started_monotonic = time.monotonic()
        self.continuous_record_next_sample_monotonic = self.continuous_record_started_monotonic
        self.continuous_record_path_var.set(str(path))
        self.continuous_record_status_var.set("连续记录: 0行")
        self.continuous_record_button.configure(text="停止连续记录")
        self.continuous_record_interval_spin.configure(state="disabled")
        window = self._active_encoder_servo_window()
        if window is not None:
            window._set_feedback_record_button_state(True)
        self._continuous_record_tick()

    def _stop_continuous_record(self, reason: str = "") -> None:
        if (
            not self.continuous_record_running
            and self.continuous_record_file is None
            and self.continuous_event_file is None
        ):
            return

        self.continuous_record_running = False
        fp = self.continuous_record_file
        self.continuous_record_file = None
        self.continuous_record_writer = None
        if fp is not None:
            try:
                fp.close()
            except Exception:
                pass
        event_fp = self.continuous_event_file
        self.continuous_event_file = None
        self.continuous_event_writer = None
        if event_fp is not None:
            try:
                event_fp.close()
            except Exception:
                pass

        if hasattr(self, "continuous_record_button"):
            self.continuous_record_button.configure(text="开始连续记录")
        if hasattr(self, "continuous_record_interval_spin"):
            self.continuous_record_interval_spin.configure(state="normal")
        window = self._active_encoder_servo_window()
        if window is not None:
            window._set_feedback_record_button_state(False)
        suffix = f": {reason}" if reason else ""
        self.continuous_record_status_var.set(f"连续记录已停止{suffix}; {self.continuous_record_rows}行")

    def _record_continuous_command_event(self, command: str) -> None:
        """Write control-changing commands at their serial transmission time."""
        if not self.continuous_record_running or self.continuous_event_writer is None:
            return
        text = command.strip()
        command_key = text.split(maxsplit=1)[0].rstrip(";").lower() if text else ""
        if command_key not in {
            "angle",
            "degree",
            "direct",
            "tension",
            "tensionbias",
            "kp",
            "pgain",
            "ki",
            "igain",
            "start",
            "stop",
        }:
            return
        motor_match = re.search(r"\bm(\d+)\b", text, re.IGNORECASE)
        joint_match = re.search(r"\bj(\d+)\b", text, re.IGNORECASE)
        now = time.time()
        row = {
            "timestamp": f"{now:.6f}",
            "local_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
            "elapsed_s": f"{max(0.0, time.monotonic() - self.continuous_record_started_monotonic):.3f}",
            "event_type": command_key,
            "motor": motor_match.group(1) if motor_match else "",
            "joint": joint_match.group(1) if joint_match else "",
            "command": text,
        }
        try:
            self.continuous_event_writer.writerow(row)
            if self.continuous_event_file is not None:
                self.continuous_event_file.flush()
        except Exception:
            # The main record tick owns stop/error reporting; do not let event
            # logging interrupt motion-control dispatch.
            pass

    def _continuous_record_tick(self) -> None:
        if not self.continuous_record_running:
            return

        now = time.monotonic()
        if now >= self.continuous_record_next_sample_monotonic:
            snapshot, issues = self._record_snapshot_issues()
            if snapshot is None:
                detail = ", ".join(issues[:3])
                self.continuous_record_status_var.set(
                    f"连续记录: 等待数据; {self.continuous_record_rows}行; {detail}"
                )
            else:
                try:
                    self._write_continuous_record_row(snapshot)
                except Exception as exc:
                    self._stop_continuous_record(f"写入失败: {exc}")
                    messagebox.showerror("连续记录失败", f"无法写入连续记录 CSV: {exc}")
                    return
            self.continuous_record_next_sample_monotonic = now + self.continuous_record_interval_ms / 1000.0

        delay_ms = max(20, min(200, self.continuous_record_interval_ms // 2))
        self.after(delay_ms, self._continuous_record_tick)

    def _write_continuous_record_row(self, snapshot: EncoderServoSnapshot) -> None:
        if self.continuous_record_writer is None or self.continuous_record_file is None:
            return
        sample_index = self.continuous_record_rows + 1
        row = self._build_manual_record_row(
            snapshot,
            sample_index=sample_index,
            elapsed_s=time.monotonic() - self.continuous_record_started_monotonic,
        )
        self.continuous_record_writer.writerow(row)
        self.continuous_record_file.flush()
        self.continuous_record_rows = sample_index
        self.continuous_record_status_var.set(f"连续记录: {self.continuous_record_rows}行")

    def _record_linearity_sample(self) -> None:
        if self.poller is None:
            messagebox.showinfo("力传感器未连接", "请先连接力传感器串口。")
            return

        now = time.time()
        missing_force: list[str] = []
        for channel in sorted(FORCE_CHANNEL_TO_MOTOR):
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                missing_force.append(f"CH{channel}: no data")
            elif now - sample.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
                missing_force.append(f"CH{channel}: stale")
        if missing_force:
            messagebox.showinfo("等待力传感器数据", ", ".join(missing_force))
            return

        window = self._active_encoder_servo_window()
        if window is None or window.poller is None:
            messagebox.showinfo("Encoder / Servo 未连接", "请先打开并连接 Encoder / Servo 窗口。")
            return
        snapshot = window.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待 Encoder / Servo 数据", "请等待第一帧关节角度数据。")
            return
        if now - snapshot.timestamp > MANUAL_RECORD_STALE_LIMIT_S:
            messagebox.showinfo("Encoder / Servo 数据过期", "请等待最新 Encoder / Servo 数据。")
            return

        try:
            if self.manual_record_path is None:
                default_dir = Path.cwd() / "run_data" / "linearity_records"
                default_dir.mkdir(exist_ok=True)
                self.manual_record_path = default_dir / f"linearity_record_{time.strftime('%Y%m%d_%H%M%S')}.csv"
                self.manual_record_path_var.set(str(self.manual_record_path))

            fieldnames = manual_record_csv_fieldnames()
            write_header = not self.manual_record_path.exists() or self.manual_record_path.stat().st_size == 0
            row = self._build_manual_record_row(snapshot)
            encoding = "utf-8-sig" if write_header else "utf-8"
            with open(self.manual_record_path, "a", newline="", encoding=encoding) as fp:
                writer = csv.DictWriter(fp, fieldnames=fieldnames, extrasaction="ignore", restval="")
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
            self.manual_record_rows += 1
        except Exception as exc:
            messagebox.showerror("记录失败", f"无法写入线性记录 CSV: {exc}")
            return

        self.manual_record_status_var.set(f"线性记录: {self.manual_record_rows}点")
        self.status_var.set(f"已记录线性点 {self.manual_record_rows}: {self.manual_record_path}")

    def _build_manual_record_row(
        self,
        snapshot: EncoderServoSnapshot,
        sample_index: Optional[int] = None,
        elapsed_s: Optional[float] = None,
    ) -> dict[str, object]:
        now = time.time()
        if sample_index is None:
            sample_index = self.manual_record_rows + 1
        row: dict[str, object] = {
            "timestamp": f"{now:.6f}",
            "local_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
            "sample_index": sample_index,
            "force_complete": int(all(channel in self.latest_by_channel for channel in FORCE_CHANNEL_TO_MOTOR)),
            "encoder_snapshot_ts": f"{snapshot.timestamp:.6f}",
            "encoder_age_s": f"{max(0.0, now - snapshot.timestamp):.3f}",
        }
        if elapsed_s is not None:
            row["elapsed_s"] = f"{elapsed_s:.3f}"

        window = self._active_encoder_servo_window()
        (
            tension_window_active,
            tension_window_phase,
            tension_window_target_n,
            tension_window_tight_limit_n,
            tension_window_motors,
            host_bias_by_motor,
        ) = self._continuous_tension_context()
        row["control_mode"] = window.mode_var.get() if window is not None else "--"
        row["last_host_command"] = window.sent_last_target_command if window is not None else "--"
        row["degree_force_active"] = int(tension_window_active)
        row["degree_force_phase"] = tension_window_phase
        row["degree_force_tension_window_ok"] = int(self.degree_force_tension_window_ok)
        if tension_window_target_n is not None:
            row["tension_window_target_n"] = f"{tension_window_target_n:.3f}"
        if tension_window_tight_limit_n is not None:
            row["tension_window_tight_limit_n"] = f"{tension_window_tight_limit_n:.3f}"
        sent_degree_targets = window.sent_degree_targets if window is not None else {}
        for joint in MANUAL_RECORD_JOINTS:
            if joint in sent_degree_targets:
                row[f"target_j{joint:02d}_deg"] = f"{sent_degree_targets[joint]:.3f}"

        channel_by_motor = {motor: channel for channel, motor in FORCE_CHANNEL_TO_MOTOR.items()}
        for motor in training_target_motor_indices():
            channel = channel_by_motor.get(motor)
            prefix = f"tension_m{motor:02d}"
            if channel is None:
                continue
            row[f"{prefix}_channel"] = f"CH{channel}"
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                continue
            relative = self._relative_value(sample.value, channel)
            force_n = relative / RELATIVE_UNITS_PER_NEWTON
            row[f"{prefix}_raw"] = sample.value
            row[f"{prefix}_relative"] = relative
            row[f"{prefix}_n"] = f"{force_n:.3f}"
            row[f"{prefix}_age_s"] = f"{max(0.0, now - sample.timestamp):.3f}"
            row[f"{prefix}_host_bias_counts"] = host_bias_by_motor.get(motor, 0)
            if tension_window_target_n is not None and tension_window_tight_limit_n is not None:
                motor_target_n = self._tension_target_for_motor(motor, tension_window_target_n)
                motor_tight_limit_n = self._tension_tight_limit_for_motor(motor, tension_window_tight_limit_n)
                row[f"{prefix}_target_n"] = f"{motor_target_n:.3f}"
                row[f"{prefix}_tight_limit_n"] = f"{motor_tight_limit_n:.3f}"
                row[f"{prefix}_error_to_target_n"] = f"{motor_target_n - force_n:.3f}"
                if motor not in tension_window_motors:
                    action = "inactive"
                elif force_n > motor_target_n:
                    action = "loose"
                elif force_n < motor_tight_limit_n:
                    action = "tight"
                else:
                    action = "hold"
                row[f"{prefix}_window_action"] = action

        for joint in MANUAL_RECORD_JOINTS:
            prefix = f"joint_j{joint:02d}"
            row[f"{prefix}_raw"] = snapshot.encoder_raw[joint]
            row[f"{prefix}_mapped"] = snapshot.encoder_mapped[joint]
            row[f"{prefix}_deg"] = f"{snapshot.encoder_deg[joint]:.3f}"
            row[f"{prefix}_valid"] = int(snapshot.encoder_valid[joint])

        for motor in training_target_motor_indices():
            prefix = f"servo_m{motor:02d}"
            online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
            row[f"{prefix}_abs"] = snapshot.servo_abs[motor]
            row[f"{prefix}_hardware_abs"] = snapshot.servo_hardware_abs[motor]
            row[f"{prefix}_zero_offset"] = snapshot.servo_zero_offset[motor]
            row[f"{prefix}_raw"] = snapshot.servo_raw[motor]
            row[f"{prefix}_speed"] = snapshot.servo_speed[motor]
            row[f"{prefix}_load"] = snapshot.servo_load[motor]
            row[f"{prefix}_current"] = snapshot.servo_current[motor]
            row[f"{prefix}_voltage"] = snapshot.servo_voltage[motor]
            row[f"{prefix}_temperature"] = snapshot.servo_temperature[motor]
            row[f"{prefix}_online"] = int(online)
            row[f"{prefix}_raw_online"] = int(snapshot.servo_raw_online[motor])
        for motor in training_target_motor_indices():
            debug = snapshot.joint_debug.get(motor)
            if debug is None:
                continue
            prefix = f"mcp_m{motor:02d}"
            row[f"{prefix}_target_deg"] = f"{debug.target_deg:.3f}"
            row[f"{prefix}_actual_deg"] = f"{debug.actual_deg:.3f}"
            if debug.motor_targets_valid:
                row[f"{prefix}_mapped_target"] = f"{debug.mapped_motor_target:.3f}"
                row[f"{prefix}_solver_target"] = debug.solver_output_pos
            if debug.cmd_valid:
                row[f"{prefix}_cmd_target"] = debug.cmd_target_pos
            row[f"{prefix}_cmd_valid"] = int(debug.cmd_valid)
            row[f"{prefix}_debug_age_s"] = f"{max(0.0, now - debug.timestamp):.3f}"
            row[f"{prefix}_firmware_time_ms"] = debug.firmware_time_ms
            row[f"{prefix}_q_ref_deg"] = f"{debug.q_ref_deg:.3f}"
            row[f"{prefix}_q_fb_filtered_deg"] = f"{debug.q_fb_filtered_deg:.3f}"
            row[f"{prefix}_q_fb_velocity_deg_s"] = f"{debug.q_fb_velocity_deg_s:.3f}"
            row[f"{prefix}_angle_error_deg"] = f"{debug.angle_error_deg:.3f}"
            row[f"{prefix}_angle_integral_deg_s"] = f"{debug.angle_integral_deg_s:.3f}"
            row[f"{prefix}_feedforward_counts"] = f"{debug.feedforward_counts:.3f}"
            row[f"{prefix}_angle_p_counts"] = f"{debug.angle_p_counts:.3f}"
            row[f"{prefix}_angle_i_counts"] = f"{debug.angle_i_counts:.3f}"
            row[f"{prefix}_angle_d_counts"] = f"{debug.angle_d_counts:.3f}"
            row[f"{prefix}_angle_feedback_counts"] = f"{debug.angle_feedback_counts:.3f}"
            row[f"{prefix}_tension_bias_counts"] = debug.tension_bias_counts
            row[f"{prefix}_pre_limit_target"] = debug.pre_limit_target_pos
            row[f"{prefix}_tension_bias_enabled"] = int(debug.tension_bias_enabled)
            row[f"{prefix}_command_limit_flags"] = debug.command_limit_flags
            row[f"{prefix}_command_rate_limited"] = int(bool(debug.command_limit_flags & 0x01))
            row[f"{prefix}_command_abs_limited"] = int(bool(debug.command_limit_flags & 0x02))
        return row

    def _write_training_row(self, window: EncoderServoWindow, snapshot: EncoderServoSnapshot) -> None:
        if self.training_writer is None or self.training_file is None:
            return
        row = self._build_training_row(window, snapshot)
        self.training_writer.writerow(row)
        self.training_file.flush()
        self.training_rows += 1
        target_text = self.training_last_target_phase or "--"
        if self.training_last_target_joint is not None and self.training_last_target_relative is not None:
            target_text = (
                f"{target_text} J{self.training_last_target_joint:02d}="
                f"{self.training_last_target_relative:.1f}deg"
            )
        self.training_status_var.set(
            f"采集中: {self.training_rows}行, {self.training_last_target_mode} {target_text}"
        )
        self.training_tension_action = ""

    def _build_training_row(self, window: EncoderServoWindow, snapshot: EncoderServoSnapshot) -> dict[str, object]:
        now = time.time()
        row: dict[str, object] = {
            "timestamp": f"{now:.6f}",
            "local_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
            "elapsed_s": f"{time.monotonic() - self.training_started_monotonic:.3f}",
            "target_mode": self.training_last_target_mode,
            "target_phase": self.training_last_target_phase,
            "target_joint": "" if self.training_last_target_joint is None else f"J{self.training_last_target_joint:02d}",
            "target_relative_deg": "" if self.training_last_target_relative is None else f"{self.training_last_target_relative:.3f}",
            "target_device_deg": "" if self.training_last_target_device is None else f"{self.training_last_target_device:.3f}",
            "target_command": self.training_last_target_command,
            "tension_window_ok": self.training_tension_window_ok,
            "tension_action": self.training_tension_action,
            "tension_action_count": self.training_tension_action_count,
            "tension_bias_m00_counts": self.training_tension_bias_by_motor.get(0, 0),
            "tension_bias_m01_counts": self.training_tension_bias_by_motor.get(1, 0),
            "tension_bias_m02_counts": self.training_tension_bias_by_motor.get(2, 0),
            "tension_bias_m03_counts": self.training_tension_bias_by_motor.get(3, 0),
            "tension_bias_m04_counts": self.training_tension_bias_by_motor.get(4, 0),
            "target_vector_complete": int(
                all(joint in self.training_target_relative_by_joint for joint, _low, _high in self.training_ranges)
            ),
            "force_complete": int(all(channel in self.latest_by_channel for channel in DISPLAY_CHANNELS)),
            "encoder_snapshot_ts": f"{snapshot.timestamp:.6f}",
            "encoder_age_s": f"{max(0.0, now - snapshot.timestamp):.3f}",
        }

        for joint in range(ENCODER_COUNT):
            prefix = f"target_j{joint:02d}"
            if joint in self.training_target_relative_by_joint:
                row[f"{prefix}_relative_deg"] = f"{self.training_target_relative_by_joint[joint]:.3f}"
                row[f"{prefix}_device_deg"] = f"{self.training_target_device_by_joint.get(joint, 0.0):.3f}"
                row[f"{prefix}_known"] = 1
            else:
                row[f"{prefix}_known"] = 0

        for channel in DISPLAY_CHANNELS:
            prefix = f"force_ch{channel}"
            row[f"{prefix}_motor"] = force_motor_label(channel)
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                continue
            relative = self._relative_value(sample.value, channel)
            row[f"{prefix}_raw"] = sample.value
            row[f"{prefix}_relative"] = relative
            row[f"{prefix}_n"] = f"{relative / RELATIVE_UNITS_PER_NEWTON:.3f}"
            row[f"{prefix}_age_s"] = f"{max(0.0, now - sample.timestamp):.3f}"

        channel_by_motor = {motor: channel for channel, motor in FORCE_CHANNEL_TO_MOTOR.items()}
        for motor in training_target_motor_indices():
            channel = channel_by_motor.get(motor)
            prefix = f"tension_m{motor:02d}"
            if channel is None:
                continue
            row[f"{prefix}_channel"] = f"CH{channel}"
            sample = self.latest_by_channel.get(channel)
            if sample is None:
                continue
            relative = self._relative_value(sample.value, channel)
            row[f"{prefix}_relative"] = relative
            row[f"{prefix}_n"] = f"{relative / RELATIVE_UNITS_PER_NEWTON:.3f}"

        for joint in range(ENCODER_COUNT):
            prefix = f"joint_j{joint:02d}"
            row[f"{prefix}_raw"] = snapshot.encoder_raw[joint]
            row[f"{prefix}_mapped"] = snapshot.encoder_mapped[joint]
            row[f"{prefix}_deg_abs"] = f"{snapshot.encoder_deg[joint]:.3f}"
            row[f"{prefix}_deg_rel"] = f"{snapshot.encoder_deg[joint]:.3f}"
            row[f"{prefix}_valid"] = int(snapshot.encoder_valid[joint])

        for motor in range(SERVO_COUNT):
            prefix = f"servo_m{motor:02d}"
            online = snapshot.servo_online[motor] or snapshot.servo_raw_online[motor]
            row[f"{prefix}_abs"] = snapshot.servo_abs[motor]
            row[f"{prefix}_hardware_abs"] = snapshot.servo_hardware_abs[motor]
            row[f"{prefix}_zero_offset"] = snapshot.servo_zero_offset[motor]
            row[f"{prefix}_raw"] = snapshot.servo_raw[motor]
            row[f"{prefix}_speed"] = snapshot.servo_speed[motor]
            row[f"{prefix}_load"] = snapshot.servo_load[motor]
            row[f"{prefix}_current"] = snapshot.servo_current[motor]
            row[f"{prefix}_voltage"] = snapshot.servo_voltage[motor]
            row[f"{prefix}_temperature"] = snapshot.servo_temperature[motor]
            row[f"{prefix}_online"] = int(online)
            row[f"{prefix}_raw_online"] = int(snapshot.servo_raw_online[motor])
        return row

    def _finish_force_emergency_relax(self, reason: str, *, recovered: bool) -> None:
        """Stop the all-motor release motion and hold the reached positions."""
        was_active = self.force_emergency_relax_active
        self.force_emergency_relax_active = False
        self.force_emergency_relax_targets.clear()
        window = self._active_encoder_servo_window()
        if window is not None and window.poller is not None:
            window.queue_emergency_stop()
        if recovered:
            self.force_hard_stop_latched = False
        if was_active or reason:
            self.status_var.set(
                f"全电机放松已停止：{reason}；moves={self.force_emergency_relax_move_count}"
            )

    def _start_force_emergency_relax(
        self,
        channel: int,
        force_n: float,
        *,
        trigger_limit_n: Optional[float] = None,
    ) -> None:
        """Take over motion and actively loosen all five motors."""
        if self.force_emergency_relax_active:
            return
        trigger_limit = (
            FORCE_HARD_ABORT_N if trigger_limit_n is None else float(trigger_limit_n)
        )
        self.force_hard_stop_latched = True
        self.force_emergency_relax_active = True
        self.force_emergency_relax_targets.clear()
        self.force_emergency_relax_move_count = 0
        self.force_emergency_relax_trigger_limit_n = trigger_limit
        reason = (
            f"全局拉力保护触发：CH{channel}={force_n:.1f} N "
            f"<= {trigger_limit:.1f} N"
        )

        # Do not issue STOP here: STOP freezes output and would prevent the
        # requested active release.  First silence all other command producers.
        if self.feedback_pretension_active:
            self._stop_nullspace_pretension(reason, hold_bias=False)
        if self.degree_force_active:
            self._stop_degree_force_control(reason, send_stop=False)
        self.degree_force_enabled_var.set(False)
        if self.force_target_running:
            self._stop_force_target_control(reason)
        if self.safe_relax_active:
            self._stop_safe_relax(reason, send_stop=False)
        if self.linearity_running:
            self._stop_linearity_experiment(reason, send_cleanup=False)
        if self.multi_input_running:
            self._stop_multi_input_experiment(reason, send_cleanup=False)
        if self.training_running:
            self._stop_training_collection(reason)

        window = self._active_encoder_servo_window()
        if window is not None and window.step_test_running:
            window._stop_step_test(reason, send_stop=False)
        if window is not None and window.sine_test_running:
            window._stop_sine_test(reason, send_stop=False)
        snapshot = window.latest_snapshot if window is not None else None
        if window is None or window.poller is None or snapshot is None:
            self._finish_force_emergency_relax(
                reason + "；无有效电机快照，已STOP",
                recovered=False,
            )
            return
        offline = [
            motor
            for motor in range(5)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            self._finish_force_emergency_relax(
                reason + "；电机离线 " + ", ".join(f"M{motor:02d}" for motor in offline),
                recovered=False,
            )
            return

        window.clear_pending_commands()
        for motor in range(5):
            current_abs = int(snapshot.servo_abs[motor])
            target_abs = max(FORCE_TARGET_MIN_ABS, current_abs - FORCE_HARD_RELAX_STEP_COUNTS)
            self.force_emergency_relax_targets[motor] = target_abs
            window.sent_direct_targets[motor] = target_abs
        window.queue_text_command("tension off")
        window.queue_text_command("direct")
        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(window._current_host_direct_targets()),
            label=f"{trigger_limit:.0f}N all-motor release",
        )
        window.queue_text_command("start")
        self.force_emergency_relax_move_count = 1
        self.status_var.set(
            reason + f"；正在同步放松五电机，全部 > {FORCE_HARD_RELAX_STOP_N:.1f} N 后停止"
        )
        self.after(FORCE_HARD_RELAX_INTERVAL_MS, self._force_emergency_relax_tick)

    def _force_emergency_relax_tick(self) -> None:
        if not self.force_emergency_relax_active:
            return
        window = self._active_encoder_servo_window()
        snapshot = window.latest_snapshot if window is not None else None
        if self.poller is None or window is None or window.poller is None or snapshot is None:
            self._finish_force_emergency_relax("传感器或电机连接丢失", recovered=False)
            return

        now_wall = time.time()
        forces_n: list[float] = []
        for channel in DISPLAY_CHANNELS:
            sample = self.latest_by_channel.get(channel)
            if sample is None or now_wall - sample.timestamp > 1.5:
                self._finish_force_emergency_relax(f"CH{channel}数据过期", recovered=False)
                return
            force_n = self._force_target_current_n(channel)
            if force_n is None or not math.isfinite(force_n):
                self._finish_force_emergency_relax(f"CH{channel}数据无效", recovered=False)
                return
            forces_n.append(float(force_n))

        tightest = min(forces_n)
        if all(force_n > FORCE_HARD_RELAX_STOP_N for force_n in forces_n):
            self._finish_force_emergency_relax(
                f"五路均 > {FORCE_HARD_RELAX_STOP_N:.1f} N，最紧一路={tightest:.1f} N",
                recovered=True,
            )
            return

        offline = [
            motor
            for motor in range(5)
            if not (snapshot.servo_online[motor] or snapshot.servo_raw_online[motor])
        ]
        if offline:
            self._finish_force_emergency_relax(
                "电机离线 " + ", ".join(f"M{motor:02d}" for motor in offline),
                recovered=False,
            )
            return

        at_limit = True
        for motor in range(5):
            current_abs = int(snapshot.servo_abs[motor])
            previous = int(self.force_emergency_relax_targets.get(motor, current_abs))
            base = min(current_abs, previous)
            target_abs = max(FORCE_TARGET_MIN_ABS, base - FORCE_HARD_RELAX_STEP_COUNTS)
            if target_abs > FORCE_TARGET_MIN_ABS:
                at_limit = False
            self.force_emergency_relax_targets[motor] = target_abs
            window.sent_direct_targets[motor] = target_abs
        if at_limit:
            self._finish_force_emergency_relax(
                f"已到电机放松位置下限，但最紧一路仍为 {tightest:.1f} N",
                recovered=False,
            )
            return

        window.queue_bytes_command(
            build_upper_motor_pos_abs_cmd(window._current_host_direct_targets()),
            label="all-motor release step",
        )
        self.force_emergency_relax_move_count += 1
        self.status_var.set(
            f"{self.force_emergency_relax_trigger_limit_n:.0f}N全电机放松中："
            f"最紧={tightest:.1f} N，"
            f"目标全部 > {FORCE_HARD_RELAX_STOP_N:.1f} N；"
            f"moves={self.force_emergency_relax_move_count}"
        )
        self.after(FORCE_HARD_RELAX_INTERVAL_MS, self._force_emergency_relax_tick)

    def _check_force_hard_stop(self, channel: int, relative: int) -> None:
        """At the hard limit, loosen all motors past the recovery threshold."""
        if self.force_emergency_relax_active or self.force_hard_stop_latched:
            return
        force_n = float(relative) / RELATIVE_UNITS_PER_NEWTON
        if force_n > FORCE_HARD_ABORT_N:
            return
        self._start_force_emergency_relax(channel, force_n)

    def _handle_sample(self, sample: ForceSample) -> None:
        self.samples.append(sample)
        self.latest_by_channel[sample.channel] = sample
        if (
            self.force_zero_capture_active
            and sample.channel in self.force_zero_capture_samples
        ):
            self.force_zero_capture_samples[sample.channel].append(int(sample.value))
        if len(self.samples) > MAX_SAMPLES:
            del self.samples[:1000]

        relative = self._relative_value(sample.value, sample.channel)
        self._check_force_hard_stop(sample.channel, relative)
        relative_force_n = format_force_n(relative)
        self._update_max_tension(relative, sample.channel)
        if sample.channel == int(self.channel_var.get()):
            self.value_var.set(str(sample.value))
            self.relative_var.set(relative_force_n)
            self.low_word_var.set(f"0x{sample.low_word:04X} ({sample.low_word})")
            self.high_word_var.set(f"0x{sample.high_word:04X} ({sample.high_word})")
        if sample.channel in self.channel_value_vars:
            self.channel_value_vars[sample.channel].set(str(sample.value))
            self.channel_relative_vars[sample.channel].set(relative_force_n)
        if sample.channel in self.force_target_current_vars:
            self._refresh_force_target_current_display(sample.channel)
        self.count_var.set(str(len(self.samples)))
        self.request_var.set(sample.request_hex)
        self.response_var.set(sample.response_hex)
        self._append_table_row(sample, relative)
        self._draw_plot()

    def _relative_value(self, value: int, channel: int) -> int:
        baseline = self.baselines.get(channel)
        return value if baseline is None else value - baseline

    def _update_max_tension(self, relative: int, channel: int) -> None:
        """Keep each channel's most negative relative force in this session."""
        if channel not in self.max_tension_relative_by_channel:
            return
        current = self.max_tension_relative_by_channel[channel]
        if current is not None and relative >= current:
            if channel == int(self.channel_var.get()):
                self.max_tension_var.set(format_force_n(current))
            return
        self.max_tension_relative_by_channel[channel] = int(relative)
        formatted = format_force_n(relative)
        self.channel_max_tension_vars[channel].set(formatted)
        if channel == int(self.channel_var.get()):
            self.max_tension_var.set(formatted)

    def _reset_max_tension(self) -> None:
        self.max_tension_relative_by_channel = {
            channel: None for channel in DISPLAY_CHANNELS
        }
        self.max_tension_var.set("--")
        for channel in DISPLAY_CHANNELS:
            self.channel_max_tension_vars[channel].set("--")

    def _zero_status_text(self) -> str:
        if all(channel in self.baselines for channel in DISPLAY_CHANNELS):
            values = ", ".join(
                f"CH{channel}={self.baselines[channel]}" for channel in DISPLAY_CHANNELS
            )
            return f"已保存零位: {values}"
        if self.baselines:
            values = ", ".join(
                f"CH{channel}={value}" for channel, value in sorted(self.baselines.items())
            )
            return f"部分零位: {values}"
        return "未保存零位"

    def _refresh_relative_displays(self) -> None:
        main = self.latest_by_channel.get(int(self.channel_var.get()))
        self.relative_var.set(
            format_force_n(self._relative_value(main.value, main.channel)) if main else "--"
        )
        for channel in DISPLAY_CHANNELS:
            sample = self.latest_by_channel.get(channel)
            self.channel_relative_vars[channel].set(
                format_force_n(self._relative_value(sample.value, channel)) if sample else "--"
            )
            self._refresh_force_target_current_display(channel)

    def _append_table_row(self, sample: ForceSample, relative: int) -> None:
        rows = self.table.get_children()
        if len(rows) >= MAX_TABLE_ROWS:
            self.table.delete(rows[0])
        local_time = time.strftime("%H:%M:%S", time.localtime(sample.timestamp))
        item = self.table.insert(
            "",
            "end",
            values=(
                local_time,
                f"CH{sample.channel}",
                force_motor_label(sample.channel),
                sample.value,
                format_force_n(relative),
                sample.response_hex,
            ),
        )
        self.table.see(item)

    def _draw_plot(self) -> None:
        canvas = self.plot_canvas
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 54, 16, 16, 32
        x0, y0 = pad_l, height - pad_b
        x1, y1 = width - pad_r, pad_t
        canvas.create_line(x0, y0, x1, y0, fill="#889096")
        canvas.create_line(x0, y0, x0, y1, fill="#889096")

        visible = self.samples[-800:]
        if not visible:
            canvas.create_text(width / 2, height / 2, text="等待数据...", fill="#667085")
            return
        values = [
            self._relative_value(sample.value, sample.channel) / RELATIVE_UNITS_PER_NEWTON
            for sample in visible
        ]
        v_min, v_max = min(values), max(values)
        if v_min == v_max:
            v_min -= 1
            v_max += 1
        t0 = visible[0].timestamp
        t1 = visible[-1].timestamp
        if t0 == t1:
            t1 = t0 + 1

        for i in range(5):
            y = y0 - (y0 - y1) * i / 4
            value = v_min + (v_max - v_min) * i / 4
            canvas.create_line(x0, y, x1, y, fill="#edf2f7")
            canvas.create_text(6, y, text=f"{value:.1f} N", anchor="w", fill="#667085")

        plot_channels = [
            (channel, CHANNEL_COLORS[channel], force_channel_label(channel)) for channel in DISPLAY_CHANNELS
        ]
        seen_channels = set()
        legend_x = x0 + 8
        for channel, color, label in plot_channels:
            if channel in seen_channels:
                continue
            seen_channels.add(channel)
            channel_points: list[float] = []
            for sample in visible:
                if sample.channel != channel:
                    continue
                value = self._relative_value(sample.value, sample.channel) / RELATIVE_UNITS_PER_NEWTON
                x = x0 + (x1 - x0) * (sample.timestamp - t0) / (t1 - t0)
                y = y0 - (y0 - y1) * (value - v_min) / (v_max - v_min)
                channel_points.extend((x, y))
            if len(channel_points) >= 4:
                canvas.create_line(*channel_points, fill=color, width=2)
            canvas.create_line(legend_x, y1 + 8, legend_x + 22, y1 + 8, fill=color, width=3)
            canvas.create_text(legend_x + 28, y1 + 8, text=label, anchor="w", fill=color)
            legend_x += 118
        canvas.create_text((x0 + x1) / 2, height - 10, text="时间", fill="#667085")

    def _set_baseline(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接传感器。", parent=self)
            return
        block_reason = self._force_zero_block_reason()
        if block_reason:
            messagebox.showwarning("当前不能校零", block_reason, parent=self)
            return
        missing_channels = [
            channel for channel in DISPLAY_CHANNELS if channel not in self.latest_by_channel
        ]
        if missing_channels:
            messagebox.showinfo(
                "等待数据",
                "请等待 CH1-CH5 都有读数后再校零。\n"
                f"当前缺少: {', '.join(f'CH{channel}' for channel in missing_channels)}",
                parent=self,
            )
            return

        self.force_zero_capture_active = True
        self.force_zero_capture_samples = {
            channel: [] for channel in DISPLAY_CHANNELS
        }
        self.zero_status_var.set(
            f"无载校零采集中：保持五路完全无外力 {FORCE_ZERO_CAPTURE_SECONDS:.1f}s"
        )
        self.status_var.set("正在采集无载零点，请勿触碰传感器或移动电机")
        self.force_zero_capture_after_id = self.after(
            int(round(FORCE_ZERO_CAPTURE_SECONDS * 1000.0)),
            self._finish_force_zero_capture,
        )

    def _finish_force_zero_capture(self) -> None:
        if not self.force_zero_capture_active:
            return
        self.force_zero_capture_active = False
        self.force_zero_capture_after_id = None

        try:
            results = {
                channel: force_zero_baseline_from_samples(
                    self.force_zero_capture_samples[channel]
                )
                for channel in DISPLAY_CHANNELS
            }
        except ValueError as exc:
            self.zero_status_var.set(self._zero_status_text())
            messagebox.showwarning(
                "校零样本不足",
                f"{exc}。请保持连接稳定后重试。",
                parent=self,
            )
            return

        unstable = {
            channel: span
            for channel, (_baseline, span) in results.items()
            if span > FORCE_ZERO_MAX_SPAN_UNITS
        }
        if unstable:
            detail = ", ".join(
                f"CH{channel}波动{span / RELATIVE_UNITS_PER_NEWTON:.1f}N"
                for channel, span in unstable.items()
            )
            self.zero_status_var.set(self._zero_status_text())
            messagebox.showwarning(
                "无载读数不稳定",
                f"{detail}；本次零点未保存。请确认无外力、平台静止后重试。",
                parent=self,
            )
            return

        self.baselines = {
            channel: baseline
            for channel, (baseline, _span) in results.items()
        }
        try:
            save_force_zero_baselines(self.baselines)
        except Exception as exc:
            messagebox.showerror("保存失败", f"零位保存失败: {exc}")
            return
        self.zero_status_var.set(self._zero_status_text())
        sample_counts = ", ".join(
            f"CH{channel}:{len(self.force_zero_capture_samples[channel])}"
            for channel in DISPLAY_CHANNELS
        )
        self.status_var.set(
            f"无载校零完成（{sample_counts}），零位已保存: {FORCE_ZERO_CONFIG_PATH}"
        )
        self._reset_max_tension()
        self._refresh_relative_displays()
        self._draw_plot()

    def _cancel_force_zero_capture(self) -> None:
        self.force_zero_capture_active = False
        after_id = self.force_zero_capture_after_id
        self.force_zero_capture_after_id = None
        if after_id is not None:
            try:
                self.after_cancel(after_id)
            except tk.TclError:
                pass
        self.force_zero_capture_samples = {
            channel: [] for channel in DISPLAY_CHANNELS
        }

    def _clear_baseline(self) -> None:
        self._cancel_force_zero_capture()
        self.baselines.clear()
        try:
            delete_force_zero_baselines()
        except Exception as exc:
            messagebox.showerror("清除失败", f"零位配置删除失败: {exc}")
            return
        self.zero_status_var.set(self._zero_status_text())
        self.status_var.set("零位已清除")
        self._reset_max_tension()
        self._refresh_relative_displays()
        self._draw_plot()

    def _clear_samples(self) -> None:
        self.samples.clear()
        self.latest_by_channel.clear()
        self._reset_max_tension()
        self.table.delete(*self.table.get_children())
        self.count_var.set("0")
        self.value_var.set("--")
        self.relative_var.set("--")
        for channel in DISPLAY_CHANNELS:
            self.channel_value_vars[channel].set("--")
            self.channel_relative_vars[channel].set("--")
            self.channel_max_tension_vars[channel].set("--")
            self.force_target_current_vars[channel].set("--")
            self.force_target_motor_target_vars[channel].set("--")
        self.low_word_var.set("--")
        self.high_word_var.set("--")
        self.request_var.set("--")
        self.response_var.set("--")
        self._draw_plot()

    def _export_csv(self) -> None:
        if not self.samples:
            messagebox.showinfo("没有数据", "当前没有可导出的样本。")
            return
        default_dir = Path.cwd() / "run_data"
        default_dir.mkdir(exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="导出 CSV",
            initialdir=str(default_dir),
            initialfile=f"force_sensor_{time.strftime('%Y%m%d_%H%M%S')}.csv",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(
                [
                    "timestamp",
                    "local_time",
                    "channel",
                    "motor",
                    "value",
                    "relative_value",
                    "relative_force_n",
                    "low_word",
                    "high_word",
                    "request_hex",
                    "response_hex",
                ]
            )
            for sample in self.samples:
                writer.writerow(
                    [
                        f"{sample.timestamp:.6f}",
                        time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(sample.timestamp)),
                        sample.channel,
                        force_motor_label(sample.channel),
                        sample.value,
                        relative_value := self._relative_value(sample.value, sample.channel),
                        f"{relative_value / RELATIVE_UNITS_PER_NEWTON:.1f}",
                        sample.low_word,
                        sample.high_word,
                        sample.request_hex,
                        sample.response_hex,
                    ]
                )
        self.status_var.set(f"已导出 {path}")

    def _on_close(self) -> None:
        self._stop_safe_relax("窗口关闭")
        self._stop_linearity_experiment("窗口关闭")
        self._stop_multi_input_experiment("窗口关闭")
        self._stop_degree_force_control("窗口关闭")
        self._stop_training_collection("窗口关闭")
        self._stop_continuous_record("窗口关闭")
        if self.encoder_servo_window is not None and self.encoder_servo_window.winfo_exists():
            self.encoder_servo_window._disconnect()
            self.encoder_servo_window.destroy()
        self._stop_force_target_control("window closed")
        self._disconnect()
        self.destroy()


def run_protocol_selftest() -> None:
    if FORCE_HARD_ABORT_N != -90.0:
        raise AssertionError(f"force hard-stop threshold mismatch: {FORCE_HARD_ABORT_N}")
    if FORCE_HARD_RELAX_STOP_N != -70.0:
        raise AssertionError(f"force release stop threshold mismatch: {FORCE_HARD_RELAX_STOP_N}")
    if not NULLSPACE_PRETENSION_AUTOSTART_ENABLED:
        raise AssertionError("staged tension control must start with degree feedback")
    if NULLSPACE_PRETENSION_RELEASE_N != -90.0:
        raise AssertionError(
            f"null-space release threshold mismatch: {NULLSPACE_PRETENSION_RELEASE_N}"
        )
    if LINEARITY_TIGHT_ABORT_N != FORCE_HARD_ABORT_N:
        raise AssertionError("linearity hard-stop threshold is not global")
    if FEEDBACK_PRETENSION_ABORT_N != FORCE_HARD_ABORT_N:
        raise AssertionError("pretension hard-stop threshold is not global")

    record_path, event_path = unique_continuous_record_paths(Path("records"))
    if re.fullmatch(r"continuous_record_\d{8}_\d{6}_\d{6}\.csv", record_path.name) is None:
        raise AssertionError(f"continuous record filename mismatch: {record_path.name}")
    if event_path.name != f"{record_path.stem}_events.csv":
        raise AssertionError((record_path, event_path))

    multi_sequence = build_multi_input_sequence(
        "prbs",
        [200] * 5,
        120,
        20260716,
        frequencies_hz=[0.037, 0.053, 0.071, 0.089, 0.113],
    )
    multi_metrics = input_independence_metrics(multi_sequence[:72])
    if multi_sequence.shape != (120, 5) or multi_metrics.rank != 5:
        raise AssertionError((multi_sequence.shape, multi_metrics))
    multi_splits = split_labels(120, parse_split_ratios("0.6;0.2;0.2"))
    if (multi_splits.count("train"), multi_splits.count("validation"), multi_splits.count("test")) != (72, 24, 24):
        raise AssertionError("multi-input split mismatch")
    request = build_read_holding_registers(1, 0, 2)
    expected_request = bytes.fromhex("01 03 00 00 00 02 C4 0B")
    if request != expected_request:
        raise AssertionError(f"request mismatch: {format_hex(request)}")
    diagnostic_request = build_read_holding_registers(1, 0, 6)
    expected_diagnostic_request = bytes.fromhex("01 03 00 00 00 06 C5 C8")
    if diagnostic_request != expected_diagnostic_request:
        raise AssertionError(f"diagnostic request mismatch: {format_hex(diagnostic_request)}")
    channel5_request = build_read_holding_registers(1, weight_register_for_channel(5), 2)
    expected_channel5_request = bytes.fromhex("01 03 00 08 00 02 45 C9")
    if channel5_request != expected_channel5_request:
        raise AssertionError(f"CH5 request mismatch: {format_hex(channel5_request)}")
    tare_request = build_write_single_register(1, 0x0015, 1)
    expected_tare_request = bytes.fromhex("01 06 00 15 00 01 59 CE")
    if tare_request != expected_tare_request:
        raise AssertionError(f"tare request mismatch: {format_hex(tare_request)}")
    decode_write_single_register_response(tare_request, tare_request, 1)
    zero_baseline, zero_span = force_zero_baseline_from_samples(
        [99, 100, 100, 100, 101, 100, 99, 100, 100, 500]
    )
    if zero_baseline != 100 or zero_span != 401:
        raise AssertionError((zero_baseline, zero_span))
    response = bytes.fromhex("01 03 04 04 D2 00 00 5B 3A")
    value, low_word, high_word = decode_live_weight_response(response, 1)
    if (value, low_word, high_word) != (1234, 0x04D2, 0x0000):
        raise AssertionError((value, low_word, high_word))
    stream_mode = build_upper_stream_mode_cmd()
    if stream_mode != bytes.fromhex("FE 03 D1 01 FF"):
        raise AssertionError(f"stream mode mismatch: {format_hex(stream_mode)}")
    angle_frame = build_upper_angle_cmd([0.0] * ENCODER_COUNT)
    if (
        angle_frame[0] != UPPER_PROTOCOL_HEADER
        or angle_frame[1] != (1 + ENCODER_COUNT * 4 + 1)
        or angle_frame[2] != CMD_ANGLE_CTRL
        or angle_frame[3:7] != struct.pack("<f", 0.0)
        or angle_frame[-1] != UPPER_PROTOCOL_TAIL
    ):
        raise AssertionError(f"angle frame mismatch: {format_hex(angle_frame)}")
    if parse_j00_j03_target_values("-12;20;5;-8") != [-12.0, 20.0, 5.0, -8.0]:
        raise AssertionError("J00-J03 batch target parsing mismatch")
    if parse_j00_j03_target_values("3.5") != [3.5, 3.5, 3.5, 3.5]:
        raise AssertionError("J00-J03 broadcast target parsing mismatch")
    try:
        parse_j00_j03_target_values("1;2;3")
    except ValueError:
        pass
    else:
        raise AssertionError("short J00-J03 batch target should be rejected")
    step_poses = parse_step_test_poses(STEP_TEST_DEFAULT_POSES)
    step_amplitudes = parse_step_test_amplitudes(STEP_TEST_DEFAULT_AMPLITUDES)
    validate_step_test_targets(step_poses, step_amplitudes)
    step_trials = build_step_test_trials(step_poses, step_amplitudes, STEP_TEST_DEFAULT_REPEATS)
    if len(step_poses) != 27 or len(step_trials) != 648:
        raise AssertionError((len(step_poses), len(step_trials)))
    if any(pose[1] <= 0.0 for pose in step_poses):
        raise AssertionError("default step-test poses must keep J01 positive")
    if any(
        step_poses[pose_index][1] + (signed_step if joint == 1 else 0.0) <= 0.0
        for pose_index, joint, signed_step, _repeat in step_trials
    ):
        raise AssertionError("default step-test targets must keep J01 positive")
    step_target_ranges = []
    for joint in MANUAL_RECORD_JOINTS:
        targets = [
            step_poses[pose_index][joint] + signed_amplitude
            for pose_index in range(len(step_poses))
            for signed_amplitude in (-step_amplitudes[0], step_amplitudes[0])
        ]
        step_target_ranges.append((min(targets), max(targets)))
    if step_target_ranges != [(-15.0, 15.0), (1.0, 30.0), (0.0, 60.0), (-45.0, 45.0)]:
        raise AssertionError(f"unexpected step-test coverage: {step_target_ranges}")
    try:
        invalid_step_poses = list(step_poses)
        invalid_step_poses[0] = (
            invalid_step_poses[0][0],
            0.0,
            invalid_step_poses[0][2],
            invalid_step_poses[0][3],
        )
        parse_step_test_poses(
            "|".join(";".join(f"{value:g}" for value in pose) for pose in invalid_step_poses)
        )
    except ValueError:
        pass
    else:
        raise AssertionError("J01=0 pose must be rejected")
    if "settling_time_all_joints_s" not in step_test_summary_fieldnames():
        raise AssertionError("step-test summary fields incomplete")
    sine_poses = parse_sine_test_poses(SINE_TEST_DEFAULT_POSES)
    sine_frequencies = parse_sine_test_frequencies(SINE_TEST_DEFAULT_FREQUENCIES_HZ)
    sine_amplitudes = parse_step_test_amplitudes(SINE_TEST_DEFAULT_AMPLITUDES)
    validate_sine_test_targets(sine_poses, sine_amplitudes)
    sine_trials = build_sine_test_trials(
        sine_poses, sine_amplitudes, sine_frequencies, SINE_TEST_DEFAULT_REPEATS
    )
    if len(sine_poses) != 27 or len(sine_trials) != 648:
        raise AssertionError((len(sine_poses), len(sine_trials)))
    if any(
        sine_poses[pose_index][1] - (amplitude if joint == 1 else 0.0) <= 0.0
        for pose_index, joint, amplitude, _frequency, _repeat in sine_trials
    ):
        raise AssertionError("default sine-test targets must keep J01 positive")
    sine_target_ranges = []
    for joint in MANUAL_RECORD_JOINTS:
        targets = [
            sine_poses[pose_index][joint] + signed_amplitude
            for pose_index in range(len(sine_poses))
            for signed_amplitude in (-sine_amplitudes[0], sine_amplitudes[0])
        ]
        sine_target_ranges.append((min(targets), max(targets)))
    if sine_target_ranges != [(-15.0, 15.0), (2.0, 28.0), (0.0, 60.0), (-45.0, 45.0)]:
        raise AssertionError(f"unexpected sine-test coverage: {sine_target_ranges}")
    synthetic_sine = [
        (
            index * 0.05,
            3.0 + 2.0 * math.sin(2.0 * math.pi * 0.2 * index * 0.05 - math.radians(30.0)),
        )
        for index in range(200)
    ]
    sine_fit = fit_sine_signal(synthetic_sine, 0.2)
    if sine_fit is None:
        raise AssertionError("sine fit unexpectedly failed")
    fit_amplitude, fit_phase, fit_bias, fit_rmse = sine_fit
    if (
        abs(fit_amplitude - 2.0) > 1e-6
        or abs(fit_phase + 30.0) > 1e-6
        or abs(fit_bias - 3.0) > 1e-6
        or fit_rmse > 1e-6
    ):
        raise AssertionError((fit_amplitude, fit_phase, fit_bias, fit_rmse))
    if "phase_lag_deg" not in sine_test_summary_fieldnames():
        raise AssertionError("sine-test summary fields incomplete")
    class _SelftestStringVar:
        def set(self, _value: str) -> None:
            pass

    target_record_window = object.__new__(EncoderServoWindow)
    target_record_window.sent_degree_targets = {}
    target_record_window.sent_direct_targets = {}
    target_record_window.sent_last_target_command = "--"
    target_record_window.sent_target_var = _SelftestStringVar()
    target_record_window._record_sent_target_command("tensionbias m4 7200")
    if target_record_window.sent_direct_targets:
        raise AssertionError("tension bias must not be recorded as a direct motor target")
    target_record_window._record_sent_target_command("direct; m4 20263")
    if target_record_window.sent_direct_targets != {4: 20263}:
        raise AssertionError("direct motor target recording mismatch")
    degree_feedback = [float(index) for index in range(ENCODER_COUNT)]
    degree_valid = [True] * ENCODER_COUNT
    degree_valid[2] = False
    resumed_degree = build_host_degree_target_vector(
        {0: 10.0, 3: -5.0},
        degree_feedback,
        degree_valid,
    )
    if resumed_degree[0] != 10.0 or resumed_degree[1] != 1.0 or resumed_degree[2] != 0.0 or resumed_degree[3] != -5.0:
        raise AssertionError(f"host degree target merge mismatch: {resumed_degree[:4]}")
    resumed_direct = build_host_direct_target_vector(
        {0: 1200, 4: -600},
        [100 + index for index in range(SERVO_COUNT)],
    )
    if resumed_direct[0] != 1200 or resumed_direct[1] != 101 or resumed_direct[4] != -600:
        raise AssertionError(f"host direct target merge mismatch: {resumed_direct[:5]}")
    safe_relax_frame = build_upper_motor_pos_abs_cmd([SAFE_RELAX_TARGET_ABS] * SERVO_COUNT)
    if (
        safe_relax_frame[0] != UPPER_PROTOCOL_HEADER
        or safe_relax_frame[1] != (1 + SERVO_COUNT * 2 + 1)
        or safe_relax_frame[2] != CMD_MOTOR_POS_ABS
        or safe_relax_frame[3:5] != struct.pack(">h", SAFE_RELAX_TARGET_ABS)
        or safe_relax_frame[-1] != UPPER_PROTOCOL_TAIL
    ):
        raise AssertionError(f"safe relax frame mismatch: {format_hex(safe_relax_frame)}")
    d3_ack = parse_proto_ack_payload(bytes((CMD_MOTOR_POS_ABS, 1, 0)))
    if d3_ack != (CMD_MOTOR_POS_ABS, 1, 0):
        raise AssertionError(f"D3 ACK parse mismatch: {d3_ack}")
    if parse_proto_ack_payload(bytes((CMD_MOTOR_POS_ABS, 1))) is not None:
        raise AssertionError("short protocol ACK should be rejected")
    upper_buffer = bytearray(bytes.fromhex("FE 04 01 00 01 FF") + b"<<<SYS_READY>>>\r\n")
    frames, lines = parse_upper_frame_bytes(upper_buffer)
    if frames != [(PACKET_TYPE_SENSOR, bytes.fromhex("00 01"))]:
        raise AssertionError(frames)
    if lines != ["<<<SYS_READY>>>"]:
        raise AssertionError(lines)
    parsed_telem = parse_servo_telem_payload(struct.pack(">hhhBBB", 12, -34, 456, 74, 31, 1))
    if parsed_telem is None:
        raise AssertionError("new servo telemetry parse failed")
    speed, load, current, voltage, temperature, online = parsed_telem
    if (speed[0], load[0], current[0], voltage[0], temperature[0], online[0]) != (12, -34, 456, 74, 31, True):
        raise AssertionError((speed[0], load[0], current[0], voltage[0], temperature[0], online[0]))
    legacy_telem = parse_servo_telem_payload(struct.pack(">hhBBB", 12, -34, 74, 31, 1))
    if legacy_telem is None or legacy_telem[2][0] != 0:
        raise AssertionError("legacy servo telemetry parse failed")
    ranges = parse_training_joint_ranges("J0:-10:10,J1=0:30")
    if ranges != [(0, -10.0, 10.0), (1, 0.0, 30.0)]:
        raise AssertionError(ranges)
    coverage_targets = [
        (joint, round(target, 1))
        for joint, target in (
            build_training_coverage_target(
                [(2, 0.0, 70.0), (3, -45.0, 45.0)],
                index,
                jitter_fraction=0.0,
            )
            for index in range(4)
        )
    ]
    if coverage_targets != [(2, 5.6), (3, -37.8), (2, 64.4), (3, 37.8)]:
        raise AssertionError(coverage_targets)
    batch_ranges = [(0, -10.0, 10.0), (1, 0.0, 30.0), (2, 0.0, 70.0), (3, -45.0, 45.0)]
    phase, active_joint, single_targets = build_training_target_batch(batch_ranges, 0, "single")
    if phase != "single_j00" or active_joint != 0 or len(single_targets) != len(batch_ranges):
        raise AssertionError((phase, active_joint, single_targets))
    if [target for joint, target in single_targets if joint != active_joint] != [0.0, 0.0, 0.0]:
        raise AssertionError(single_targets)
    phase, active_joint, combo_targets = build_training_target_batch(batch_ranges, 0, "combo")
    if phase != "combo_01" or active_joint is not None:
        raise AssertionError((phase, active_joint, combo_targets))
    if dict(combo_targets)[2] <= 60.0 or dict(combo_targets)[3] <= 30.0:
        raise AssertionError(combo_targets)
    phase, active_joint, roundtrip_targets = build_training_target_batch(batch_ranges, 1, "roundtrip")
    if phase != "roundtrip_j00_2" or active_joint != 0 or dict(roundtrip_targets)[0] <= 8.0:
        raise AssertionError((phase, active_joint, roundtrip_targets))
    phase, active_joint, step_targets = build_training_target_batch(batch_ranges, 0, "step")
    if phase != "step_10_20_20_35" or active_joint is not None:
        raise AssertionError((phase, active_joint, step_targets))
    if dict(step_targets) != {0: 10.0, 1: 20.0, 2: 20.0, 3: 35.0}:
        raise AssertionError(step_targets)
    if force_target_step_delta_from_error(4.0, 20) != -1:
        raise AssertionError("small positive force error should loosen by 1 count")
    if force_target_step_delta_from_error(5.0, 20) != -1:
        raise AssertionError("5N positive force error should loosen by 1 count")
    if force_target_step_delta_from_error(5.1, 20) != -20:
        raise AssertionError("positive force error above 5N should loosen by far step")
    if force_target_step_delta_from_error(10.0, 20) != -20:
        raise AssertionError("positive force error should loosen with negative motor delta")
    if force_target_step_delta_from_error(-4.0, 20) != 1:
        raise AssertionError("small negative force error should tighten by 1 count")
    if force_target_step_delta_from_error(-5.1, 20) != 20:
        raise AssertionError("negative force error should tighten with positive motor delta")
    if force_target_step_delta_from_error(0.0, 20) != 0:
        raise AssertionError("zero force error should hold")
    if FEEDBACK_PRETENSION_TARGET_N != -12.5 or FEEDBACK_PRETENSION_DEADBAND_N != 2.5:
        raise AssertionError("feedback pretension window constants mismatch")
    for force_n in (FEEDBACK_PRETENSION_TIGHT_LIMIT_N, FEEDBACK_PRETENSION_LOOSE_LIMIT_N):
        if force_target_step_delta_from_error(
            FEEDBACK_PRETENSION_TARGET_N - force_n,
            FEEDBACK_PRETENSION_STEP_COUNTS,
            FEEDBACK_PRETENSION_DEADBAND_N,
        ) != 0:
            raise AssertionError(f"pretension boundary should hold: {force_n}")
    if force_target_step_delta_from_error(
        FEEDBACK_PRETENSION_TARGET_N,
        FEEDBACK_PRETENSION_STEP_COUNTS,
        FEEDBACK_PRETENSION_DEADBAND_N,
    ) != FEEDBACK_PRETENSION_STEP_COUNTS:
        raise AssertionError("zero-tension pretension step should tighten")
    if force_target_step_delta_from_error(
        FEEDBACK_PRETENSION_TARGET_N - (-20.0),
        FEEDBACK_PRETENSION_STEP_COUNTS,
        FEEDBACK_PRETENSION_DEADBAND_N,
    ) != -FEEDBACK_PRETENSION_STEP_COUNTS:
        raise AssertionError("over-tight pretension step should loosen")
    pretension_test_app = object.__new__(ForceSensorApp)
    pretension_test_app.feedback_pretension_active = True
    pretension_near_delta, _ = ForceSensorApp._force_target_step_delta(
        pretension_test_app,
        FEEDBACK_PRETENSION_TARGET_N - (-9.0),
        far_step_counts=FEEDBACK_PRETENSION_STEP_COUNTS,
        deadband_n=FEEDBACK_PRETENSION_DEADBAND_N,
    )
    if pretension_near_delta != FEEDBACK_PRETENSION_STEP_COUNTS:
        raise AssertionError("feedback pretension must not use the 1-count near-target step")
    null_bias = nullspace_pretension_bias_counts(50.0)
    if null_bias != {0: 20, 1: 20, 2: 31, 3: 19, 4: 20}:
        raise AssertionError(f"null-space bias mapping mismatch: {null_bias}")
    allocation = allocate_nullspace_internal_tension(
        {motor: -NULLSPACE_TENSION_MIN_N for motor in range(5)}
    )
    reconstructed = [
        allocation.task_tension_by_motor[motor]
        + allocation.alpha_desired_n * NULLSPACE_PRETENSION_VECTOR[motor]
        for motor in range(5)
    ]
    if not allocation.feasible or min(reconstructed) < NULLSPACE_TENSION_MIN_N - 1.0e-8:
        raise AssertionError((allocation, reconstructed))
    if max(reconstructed) > NULLSPACE_TENSION_MAX_N + 1.0e-8:
        raise AssertionError((allocation, reconstructed))
    if not math.isclose(
        allocation.alpha_desired_n,
        allocation.alpha_measured_n,
        rel_tol=0.0,
        abs_tol=1.0e-8,
    ):
        raise AssertionError("all-f_min state should preserve measured internal tension")
    infeasible_allocation = allocate_nullspace_internal_tension(
        {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: -200.0}
    )
    if infeasible_allocation.feasible:
        raise AssertionError("allocator accepted incompatible f_min/f_max bounds")
    if nullspace_alpha_count_step(10.0) != 50:
        raise AssertionError("positive alpha step must respect the count limit")
    if nullspace_alpha_count_step(-10.0) != -50:
        raise AssertionError("negative alpha step must respect the count limit")
    if nullspace_alpha_count_step(0.1) != 0:
        raise AssertionError("alpha force deadband must hold")
    action, loosest, tightest = nullspace_pretension_action(
        {motor: 0.0 for motor in range(5)}
    )
    if (action, loosest, tightest) != ("tighten", 0.0, 0.0):
        raise AssertionError((action, loosest, tightest))
    action, _, _ = nullspace_pretension_action({motor: -20.0 for motor in range(5)})
    if action != "hold":
        raise AssertionError("forces inside the null-space window should hold")
    release_forces = {motor: -20.0 for motor in range(5)}
    release_forces[3] = -91.0
    release_forces[1] = -2.0
    action, _, _ = nullspace_pretension_action(release_forces)
    if action != "release":
        raise AssertionError("tight-side release must have priority over a loose tendon")
    emergency_forces = {motor: -20.0 for motor in range(5)}
    emergency_forces[4] = -100.0
    action, _, _ = nullspace_pretension_action(emergency_forces)
    if action != "release":
        raise AssertionError("null-space layer must release while global all-motor protection takes over")
    fitted_r = (
        (3.6109, 3.4232, 0.0, 0.0),
        (-2.9767, 4.0643, 0.0, 0.0),
        (-1.9980, -1.3476, 2.9110, 0.0),
        (2.5066, -1.5569, 0.0, 3.6078),
        (0.0, -3.8985, -4.3955, -3.3443),
    )
    null_residual = [
        sum(fitted_r[motor][joint] * NULLSPACE_PRETENSION_VECTOR[motor] for motor in range(5))
        for joint in range(4)
    ]
    if max(abs(value) for value in null_residual) > 5e-4:
        raise AssertionError(f"R^T n is not zero enough: {null_residual}")
    try:
        parse_training_joint_ranges("J99:0:1")
    except ValueError:
        pass
    else:
        raise AssertionError("invalid joint range accepted")
    pretension_steps = parse_pretension_steps("M00:20,M01:-20,M02:15,M03:10,M04:5")
    if pretension_steps != {0: 20, 1: -20, 2: 15, 3: 10, 4: 5}:
        raise AssertionError(pretension_steps)
    try:
        parse_pretension_steps("M00:20")
    except ValueError:
        pass
    else:
        raise AssertionError("incomplete pretension steps accepted")
    training_fields = training_csv_fieldnames()
    for field in (
        "target_mode",
        "target_phase",
        "tension_window_ok",
        "tension_action",
        "tension_bias_m00_counts",
        "tension_m00_n",
        "tension_m01_n",
        "target_j00_relative_deg",
        "target_j03_known",
        "joint_j00_deg_rel",
        "servo_m00_abs",
        "servo_m00_speed",
        "servo_m00_load",
        "servo_m00_current",
    ):
        if field not in training_fields:
            raise AssertionError(f"missing training CSV field: {field}")
    manual_fields = manual_record_csv_fieldnames()
    for field in (
        "sample_index",
        "target_j00_deg",
        "target_j03_deg",
        "tension_m00_n",
        "tension_m04_n",
        "joint_j00_deg",
        "joint_j03_deg",
        "servo_m00_abs",
        "servo_m00_hardware_abs",
        "servo_m00_zero_offset",
        "servo_m00_raw",
        "servo_m04_voltage",
        "servo_m04_raw_online",
        "mcp_m03_mapped_target",
        "mcp_m03_solver_target",
        "mcp_m03_cmd_target",
        "mcp_m04_tension_bias_counts",
        "mcp_m04_pre_limit_target",
    ):
        if field not in manual_fields:
            raise AssertionError(f"missing manual record CSV field: {field}")
    if "joint_j04_deg" in manual_fields:
        raise AssertionError("manual record CSV should only include J00-J03")
    continuous_fields = continuous_record_csv_fieldnames()
    for field in (
        "elapsed_s",
        "sample_index",
        "target_j00_deg",
        "target_j03_deg",
        "tension_m04_n",
        "joint_j03_deg",
        "servo_m04_abs",
        "servo_m04_hardware_abs",
        "servo_m04_zero_offset",
        "servo_m04_raw",
        "servo_m04_raw_online",
        "mcp_m03_mapped_target",
        "mcp_m03_solver_target",
        "mcp_m03_cmd_target",
        "mcp_m04_tension_bias_counts",
        "mcp_m04_command_rate_limited",
    ):
        if field not in continuous_fields:
            raise AssertionError(f"missing continuous record CSV field: {field}")
    if "joint_j04_deg" in continuous_fields:
        raise AssertionError("continuous record CSV should only include J00-J03")
    debug_payload = struct.pack(
        ">BB8fihBBh",
        3,
        1,
        0.0,
        16.5,
        1.25,
        84.0,
        2.5,
        3.0,
        4.0,
        11600.0,
        0,
        120,
        0,
        1,
        84,
    )
    debug = parse_joint_debug_payload(debug_payload)
    if debug is None:
        raise AssertionError("extended joint debug payload did not parse")
    if (
        debug.joint_index != 3
        or not debug.valid
        or round(debug.actual_deg, 3) != 16.5
        or round(debug.mapped_motor_target, 3) != 11600.0
        or debug.solver_output_pos != 120
        or not debug.motor_targets_valid
        or debug.cmd_target_pos != 84
        or not debug.cmd_valid
    ):
        raise AssertionError(f"joint debug parse mismatch: {debug}")
    debug_v2_payload = debug_payload + struct.pack(
        ">I10fhhBB",
        123456,
        2.0,
        1.5,
        -4.0,
        0.5,
        12.0,
        100.0,
        10.0,
        20.0,
        -2.0,
        28.0,
        3600,
        3720,
        1,
        3,
    )
    debug_v2 = parse_joint_debug_payload(debug_v2_payload)
    if (
        debug_v2 is None
        or debug_v2.firmware_time_ms != 123456
        or round(debug_v2.q_fb_velocity_deg_s, 3) != -4.0
        or round(debug_v2.angle_i_counts, 3) != 20.0
        or debug_v2.tension_bias_counts != 3600
        or debug_v2.pre_limit_target_pos != 3720
        or not debug_v2.tension_bias_enabled
        or debug_v2.command_limit_flags != 3
    ):
        raise AssertionError(f"v2 joint debug parse mismatch: {debug_v2}")
    linearity_states = parse_linearity_state_motor_positions(
        "3500;4500;4000;3000;2000 | M00=3600;M01=4600;M02=4100;M03=3100;M04=2100"
    )
    if linearity_states != [
        {0: 3500, 1: 4500, 2: 4000, 3: 3000, 4: 2000},
        {0: 3600, 1: 4600, 2: 4100, 3: 3100, 4: 2100},
    ]:
        raise AssertionError(linearity_states)
    if parse_linearity_motors("M00,M04,2") != [0, 4, 2]:
        raise AssertionError("linearity motor parse failed")
    if parse_linearity_deltas("0,50,100,50,0,-50,-100,-50,0") != [0, 50, 100, 50, 0, -50, -100, -50, 0]:
        raise AssertionError("linearity delta parse failed")
    linearity_fields = linearity_experiment_csv_fieldnames()
    for field in (
        "experiment_id",
        "phase",
        "state_i",
        "motor_j",
        "delta_counts",
        "repeat_id",
        "baseline_sample_index",
        "sequence_step",
        "max_joint_delta_deg",
        "max_force_delta_n",
        "max_motor_speed",
        "delta_joint_j00_deg",
        "delta_tension_m00_n",
        "delta_servo_m00_abs",
        "target_servo_m00_abs",
        "target_error_m00_counts",
    ):
        if field not in linearity_fields:
            raise AssertionError(f"missing linearity CSV field: {field}")
    if "delta_joint_j04_deg" in linearity_fields:
        raise AssertionError("linearity CSV should only include J00-J03 delta joints")
    print("protocol selftest ok")


def run_cli_diagnostic(port_name: str, baudrate: int, slave_id: int, channel: int) -> None:
    start_register = weight_register_for_channel(channel)
    request = build_read_holding_registers(slave_id, start_register, 2)
    print(f"Opening {port_name} @ {baudrate} 8N1, slave {slave_id}, CH{channel}")
    print(f"Register: 0x{start_register:04X}")
    print(f"Request:  {format_hex(request)}")
    port = open_serial_port(port_name, baudrate, timeout_s=0.5)
    try:
        if hasattr(port, "reset_input_buffer"):
            port.reset_input_buffer()
        port.write(request)
        if hasattr(port, "flush"):
            port.flush()
        response = read_exact(port, 9, timeout_s=0.8)
        print(f"Response: {format_hex(response)}")
        registers = decode_holding_registers_response(response, slave_id)
        if len(registers) < 2:
            raise ValueError(f"expected 2 registers, got {len(registers)}")
        weight = words_to_i32_low_first(registers[0], registers[1])
        print(f"CH{channel}重量: {weight}")
        print(f"低字:       0x{registers[0]:04X} ({registers[0]})")
        print(f"高字:       0x{registers[1]:04X} ({registers[1]})")
    finally:
        port.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Force sensor Modbus-RTU monitor")
    parser.add_argument("--port", default="COM7", help="serial port, default: COM7")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"serial baudrate, default: {DEFAULT_BAUDRATE}",
    )
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus slave id, default: 1")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL, help=f"weight channel, default: {DEFAULT_CHANNEL}")
    parser.add_argument(
        "--servo-port",
        default=ENCODER_SERVO_DEFAULT_PORT,
        help=f"Encoder / Servo serial port, default: {ENCODER_SERVO_DEFAULT_PORT}",
    )
    parser.add_argument(
        "--servo-baudrate",
        type=int,
        default=ENCODER_SERVO_DEFAULT_BAUDRATE,
        help=f"Encoder / Servo baudrate, default: {ENCODER_SERVO_DEFAULT_BAUDRATE}",
    )
    parser.add_argument("--diagnose", action="store_true", help="read the selected channel once")
    parser.add_argument("--selftest", action="store_true", help="check CRC and frame decoding")
    args = parser.parse_args()

    if args.selftest:
        run_protocol_selftest()
        return 0
    if args.diagnose:
        run_cli_diagnostic(args.port, args.baudrate, args.slave_id, args.channel)
        return 0

    app = ForceSensorApp(
        default_port=args.port,
        default_servo_port=args.servo_port,
        default_servo_baudrate=args.servo_baudrate,
    )
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
