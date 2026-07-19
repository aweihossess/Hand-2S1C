#!/usr/bin/env python3
"""Guarded multi-pose step/sine validation over 27 broad joint poses.

The script deliberately calls ``run_direct_j03_sine.py`` for every trial so
each CSV remains self-contained.  It checkpoints after every physical trial
and can resume after an interruption without overwriting completed data.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
import subprocess
import sys

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_direct_j03_sine.py")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.control_validation.run_direct_pi_search import fit_sine, settling_time, stop_controller, wrap_pi


# These leave a one-degree margin to the runner's target limits while keeping
# J01 >= 2 deg even under the -5 deg excitation.
POSES = tuple(
    (j0, j1, j2, j3)
    for j0 in (-9.0, 0.0, 9.0)
    for j2 in (6.0, 30.0, 54.0)
    for j3 in (-39.0, 0.0, 39.0)
    for j1 in ((7.0, 15.0, 23.0)[
        (int((j0 + 9.0) / 9.0) + int(j2 / 24.0) + int((j3 + 39.0) / 39.0)) % 3
    ],)
)


def parse_poses(text: str) -> tuple[tuple[float, float, float, float], ...]:
    poses: list[tuple[float, float, float, float]] = []
    for chunk in text.split("|"):
        values = [float(value.strip()) for value in chunk.split(";") if value.strip()]
        if len(values) != 4:
            raise argparse.ArgumentTypeError("each pose must contain J00;J01;J02;J03")
        pose = tuple(values)
        if pose[1] <= 5.0:
            raise argparse.ArgumentTypeError("J01 must exceed 5 deg for the -5 deg test")
        poses.append(pose)  # target extrema are checked again by the child runner
    if not poses:
        raise argparse.ArgumentTypeError("at least one pose is required")
    return tuple(poses)


@dataclass
class TrialMetric:
    pose_index: int
    pose: str
    mode: str
    joint: int
    frequency_hz: float
    rmse_deg: float
    steady_error_deg: float
    effective_overshoot_deg: float
    settling_s: float
    amplitude_ratio: float
    phase_lag_deg: float
    phase_lag_s: float
    cross_coupling_deg: float
    peak_tension_n: float
    csv_path: str


def pose_text(pose: tuple[float, float, float, float]) -> str:
    return ";".join(f"{value:g}" for value in pose)


def nearest_neighbor_order(
    poses: tuple[tuple[float, float, float, float], ...],
    start: tuple[float, float, float, float] = (10.0, 10.0, 20.0, 10.0),
) -> list[tuple[float, float, float, float]]:
    remaining = list(poses)
    ordered: list[tuple[float, float, float, float]] = []
    current = np.asarray(start, dtype=float)
    scale = np.asarray((18.0, 21.0, 48.0, 78.0), dtype=float)
    while remaining:
        best = min(
            remaining,
            key=lambda item: float(np.linalg.norm((np.asarray(item) - current) / scale)),
        )
        ordered.append(best)
        remaining.remove(best)
        current = np.asarray(best, dtype=float)
    return ordered


def csv_is_complete(path: Path, mode: str) -> bool:
    if not path.exists() or path.stat().st_size < 2048:
        return False
    try:
        frame = pd.read_csv(path, usecols=["phase", "t_rel"])
    except Exception:
        return False
    required = {"sine"} if mode == "sine" else {"step_plus", "step_minus"}
    return required.issubset(set(frame.phase.astype(str)))


def run_trial(
    *,
    mode: str,
    pose: tuple[float, float, float, float],
    joint: int,
    frequency_hz: float,
    kp: float,
    ki: float,
    r_blend: float,
    p_blend: float,
    abort_tension_n: float,
    output: Path,
    transition: bool = False,
) -> None:
    command = [
        sys.executable,
        str(RUNNER),
        "--mode", mode,
        # Use the --name=value form because a pose beginning with a negative
        # J00 value (for example -9;7;30;39) is otherwise parsed as an option.
        f"--base-targets={pose_text(pose)}",
        "--kp", str(kp),
        "--ki", str(ki),
        "--r-blend", str(r_blend),
        "--p-blend", str(p_blend),
        "--abort-tension-n", str(abort_tension_n),
        "--startup-grace-s", "5.0" if transition else "2.0",
        "--output", str(output),
    ]
    if transition:
        command += [
            "--baseline-s", "12",
            "--step-amplitude-deg", "0",
            "--step-hold-s", "0",
            "--center-hold-s", "0",
            "--return-s", "0",
            "--step-joint", str(joint),
        ]
    elif mode == "step":
        command += [
            "--baseline-s", "1.5",
            "--step-amplitude-deg", "5",
            "--step-hold-s", "4",
            "--center-hold-s", "2",
            "--return-s", "3",
            "--step-joint", str(joint),
        ]
    else:
        command += [
            "--baseline-s", "1.5",
            "--duration-s", str(1.0 / frequency_hz),
            "--return-s", "3",
            "--frequency-hz", str(frequency_hz),
            "--amplitude-deg", "5",
            "--mean-deg", str(pose[joint]),
            "--sine-joint", str(joint),
        ]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    stop_controller()
    if completed.returncode != 0:
        raise RuntimeError(
            f"physical trial aborted: mode={mode}, pose={pose}, joint={joint}, "
            f"code={completed.returncode}"
        )


def verify_transition(path: Path, pose: tuple[float, float, float, float]) -> list[float]:
    frame = pd.read_csv(path)
    if len(frame) < 10:
        raise RuntimeError(f"transition CSV has too few rows: {path}")
    tail = frame.tail(10)
    errors = [
        float(abs(tail[f"j{joint}_actual"].mean() - pose[joint]))
        for joint in range(4)
    ]
    # Broad-space validation is meant to measure pose-dependent steady error.
    # A moderate transition error is therefore a result, not a safety fault.
    # Keep a 5-degree sanity bound so a clearly failed/wrong-direction move is
    # still rejected before excitation starts.
    if max(errors) > 5.0:
        raise RuntimeError(f"pose transition did not converge within 5 deg: {errors}")
    if float(tail["j1_actual"].min()) <= 0.0:
        raise RuntimeError("J01 actual was not strictly positive after transition")
    return errors


def analyze_step(pose_index: int, pose, joint: int, path: Path) -> TrialMetric:
    frame = pd.read_csv(path)
    rmses: list[float] = []
    steadies: list[float] = []
    overshoots: list[float] = []
    settlements: list[float] = []
    cross_values: list[float] = []
    for phase, direction in (("step_plus", 1.0), ("step_minus", -1.0)):
        part = frame[frame.phase == phase]
        t = part.t_rel.to_numpy(float)
        target = part[f"j{joint}_target"].to_numpy(float)
        actual = part[f"j{joint}_actual"].to_numpy(float)
        error = target - actual
        rmses.append(float(np.sqrt(np.mean(error * error))))
        tail = max(1, min(len(part), int(round(1.0 / max(0.05, np.median(np.diff(t)))))))
        steadies.append(float(abs(np.mean(error[-tail:]))))
        overshoots.append(float(max(0.0, np.max(direction * (actual - target)) - 1.0)))
        settlements.append(settling_time(t, error, band=1.0, hold_s=1.0))
        for other in range(4):
            if other != joint:
                cross_values.append(float(np.ptp(part[f"j{other}_actual"].to_numpy(float))))
    return TrialMetric(
        pose_index=pose_index,
        pose=pose_text(pose),
        mode="step",
        joint=joint,
        frequency_hz=0.0,
        rmse_deg=float(np.mean(rmses)),
        steady_error_deg=float(np.mean(steadies)),
        effective_overshoot_deg=float(np.mean(overshoots)),
        settling_s=float(np.mean(settlements)),
        amplitude_ratio=math.nan,
        phase_lag_deg=math.nan,
        phase_lag_s=math.nan,
        cross_coupling_deg=max(cross_values, default=0.0),
        peak_tension_n=float(frame.peak_tension_n.max()),
        csv_path=str(path),
    )


def analyze_sine(pose_index: int, pose, joint: int, frequency_hz: float, path: Path) -> TrialMetric:
    all_frame = pd.read_csv(path)
    frame = all_frame[all_frame.phase == "sine"].copy()
    t = frame.t_rel.to_numpy(float).copy()
    t -= t[0]
    target = frame[f"command_j{joint:02d}_target"].to_numpy(float)
    actual = frame[f"j{joint}_actual"].to_numpy(float)
    target_amp, target_phase, _ = fit_sine(t, target, frequency_hz)
    actual_amp, actual_phase, _ = fit_sine(t, actual, frequency_hz)
    lag = wrap_pi(target_phase - actual_phase)
    cross = [
        fit_sine(t, frame[f"j{other}_actual"].to_numpy(float), frequency_hz)[0]
        for other in range(4)
        if other != joint
    ]
    return TrialMetric(
        pose_index=pose_index,
        pose=pose_text(pose),
        mode="sine",
        joint=joint,
        frequency_hz=frequency_hz,
        rmse_deg=float(np.sqrt(np.mean((target - actual) ** 2))),
        steady_error_deg=math.nan,
        effective_overshoot_deg=math.nan,
        settling_s=math.nan,
        amplitude_ratio=float(actual_amp / max(target_amp, 1.0e-9)),
        phase_lag_deg=float(math.degrees(lag)),
        phase_lag_s=float(lag / (2.0 * math.pi * frequency_hz)),
        cross_coupling_deg=float(max(cross)),
        peak_tension_n=float(all_frame.peak_tension_n.max()),
        csv_path=str(path),
    )


def write_metrics(path: Path, rows: list[TrialMetric]) -> None:
    payload = [asdict(row) for row in rows]
    path.with_suffix(".json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True), encoding="utf-8"
    )
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(payload[0]))
        writer.writeheader()
        writer.writerows(payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kp", type=float, default=0.70)
    parser.add_argument("--ki", type=float, default=0.02)
    parser.add_argument("--r-blend", type=float, default=0.0)
    parser.add_argument("--p-blend", type=float, default=0.25)
    parser.add_argument("--abort-tension-n", type=float, default=70.0)
    parser.add_argument(
        "--poses",
        type=parse_poses,
        help="Optional J00;J01;J02;J03 poses separated by |; defaults to the broad 27-pose set.",
    )
    parser.add_argument("--max-poses", type=int, default=27)
    parser.add_argument("--output-dir")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("physical motion is disabled unless --execute is supplied")
    selected_poses = args.poses or POSES
    if not 1 <= args.max_poses <= len(selected_poses):
        parser.error(f"--max-poses must be within 1..{len(selected_poses)}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(
        args.output_dir
        or ROOT / "run_data" / "direct_autotune" / f"multipose_{stamp}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    config = {
        "kp": args.kp,
        "ki": args.ki,
        "r_blend": args.r_blend,
        "p_blend": args.p_blend,
        "abort_tension_n": args.abort_tension_n,
        "step_amplitude_deg": 5.0,
        "sine_amplitude_deg": 5.0,
        "poses": [list(pose) for pose in nearest_neighbor_order(selected_poses)[: args.max_poses]],
    }
    (output_dir / "config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    rows: list[TrialMetric] = []
    metrics_path = output_dir / "metrics.csv"
    if args.resume and metrics_path.exists():
        for item in pd.read_csv(metrics_path).to_dict("records"):
            rows.append(TrialMetric(**item))

    poses = nearest_neighbor_order(selected_poses)[: args.max_poses]
    total = len(poses) * 8
    completed_count = 0
    for pose_index, pose in enumerate(poses, start=1):
        pose_dir = output_dir / f"pose_{pose_index:02d}_{pose_text(pose).replace(';', '_')}"
        pose_dir.mkdir(parents=True, exist_ok=True)
        transition_path = pose_dir / "transition.csv"
        try:
            if not args.resume or not transition_path.exists():
                print(f"POSE {pose_index}/{len(poses)} transition -> {pose}", flush=True)
                run_trial(
                    mode="step", pose=pose, joint=0, frequency_hz=0.1,
                    kp=args.kp, ki=args.ki, r_blend=args.r_blend, p_blend=args.p_blend,
                    abort_tension_n=args.abort_tension_n, output=transition_path, transition=True,
                )
            transition_errors = verify_transition(transition_path, pose)
            (pose_dir / "transition_errors.json").write_text(
                json.dumps(transition_errors, indent=2), encoding="utf-8"
            )
        except Exception as exc:
            # A broad-space pose can be unreachable even though the previous
            # pose was valid.  Preserve the reason and continue with the next
            # nearest-neighbour target; the child runner has already issued
            # STOP and cleared null-space bias on any physical abort.
            skipped = {
                "pose_index": pose_index,
                "pose": list(pose),
                "reason": str(exc),
                "transition_csv": str(transition_path),
            }
            (pose_dir / "skipped.json").write_text(
                json.dumps(skipped, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            stop_controller()
            print(f"POSE {pose_index}/{len(poses)} SKIPPED: {exc}", flush=True)
            continue

        for joint in range(4):
            step_path = pose_dir / f"step_j{joint}.csv"
            if not (args.resume and csv_is_complete(step_path, "step")):
                print(f"  [{completed_count + 1}/{total}] step J{joint}", flush=True)
                run_trial(
                    mode="step", pose=pose, joint=joint, frequency_hz=0.0,
                    kp=args.kp, ki=args.ki, r_blend=args.r_blend, p_blend=args.p_blend,
                    abort_tension_n=args.abort_tension_n, output=step_path,
                )
            rows = [row for row in rows if not (
                row.pose_index == pose_index and row.mode == "step" and row.joint == joint
            )]
            rows.append(analyze_step(pose_index, pose, joint, step_path))
            completed_count += 1
            write_metrics(metrics_path, rows)

            frequency_hz = (0.05, 0.10, 0.20)[(pose_index + joint - 1) % 3]
            sine_path = pose_dir / f"sine_j{joint}_{frequency_hz:.2f}hz.csv"
            if not (args.resume and csv_is_complete(sine_path, "sine")):
                print(
                    f"  [{completed_count + 1}/{total}] sine J{joint} {frequency_hz:.2f}Hz",
                    flush=True,
                )
                run_trial(
                    mode="sine", pose=pose, joint=joint, frequency_hz=frequency_hz,
                    kp=args.kp, ki=args.ki, r_blend=args.r_blend, p_blend=args.p_blend,
                    abort_tension_n=args.abort_tension_n, output=sine_path,
                )
            rows = [row for row in rows if not (
                row.pose_index == pose_index and row.mode == "sine" and row.joint == joint
            )]
            rows.append(analyze_sine(pose_index, pose, joint, frequency_hz, sine_path))
            completed_count += 1
            write_metrics(metrics_path, rows)

    stop_controller()
    print(f"COMPLETED={len(rows)} trials")
    print(f"OUTPUT={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
