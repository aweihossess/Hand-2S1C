#!/usr/bin/env python3
"""Run guarded direct-serial PI candidates with both step and sine excitation."""

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
import time

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_direct_j03_sine.py")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from desktop.force_sensor_monitor import open_serial_port


@dataclass
class Result:
    kp: float
    ki: float
    score: float
    step_rmse_deg: float
    step_steady_error_deg: float
    step_overshoot_deg: float
    step_settling_s: float
    sine_rmse_deg: float
    sine_amplitude_ratio: float
    sine_phase_lag_deg: float
    sine_phase_lag_s: float
    cross_coupling_amp_deg: float
    peak_tension_n: float
    step_csv: str
    sine_csv: str


def parse_candidates(text: str) -> list[tuple[float, float]]:
    values: list[tuple[float, float]] = []
    for chunk in text.split(";"):
        if not chunk.strip():
            continue
        parts = [part.strip() for part in chunk.split(",")]
        if len(parts) != 2:
            raise argparse.ArgumentTypeError("candidate format is Kp,Ki;Kp,Ki")
        kp, ki = (float(parts[0]), float(parts[1]))
        if not (0.0 <= kp <= 2.0 and 0.0 <= ki <= 0.5):
            raise argparse.ArgumentTypeError("safe search range is 0<=Kp<=2, 0<=Ki<=0.5")
        values.append((kp, ki))
    if not values:
        raise argparse.ArgumentTypeError("at least one candidate is required")
    return values


def fit_sine(t: np.ndarray, y: np.ndarray, frequency_hz: float = 0.1) -> tuple[float, float, float]:
    omega = 2.0 * np.pi * frequency_hz
    design = np.c_[np.sin(omega * t), np.cos(omega * t), np.ones(len(t))]
    coefficient = np.linalg.lstsq(design, y, rcond=None)[0]
    return (
        float(np.hypot(coefficient[0], coefficient[1])),
        float(math.atan2(coefficient[1], coefficient[0])),
        float(coefficient[2]),
    )


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def settling_time(t: np.ndarray, error: np.ndarray, band: float = 1.0, hold_s: float = 1.0) -> float:
    for index, start in enumerate(t):
        end = int(np.searchsorted(t, start + hold_s, side="left"))
        if end < len(t) and np.all(np.abs(error[index : end + 1]) <= band):
            return float(start - t[0])
    return float(t[-1] - t[0])


def analyze_pair(kp: float, ki: float, step_path: Path, sine_path: Path) -> Result:
    step = pd.read_csv(step_path)
    sine_all = pd.read_csv(sine_path)
    step_rmse: list[float] = []
    step_steady: list[float] = []
    step_over: list[float] = []
    step_settle: list[float] = []
    for phase in ("step_plus", "step_minus"):
        frame = step[step.phase == phase]
        if frame.empty:
            raise RuntimeError(f"missing {phase} rows in {step_path}")
        t = frame.t_rel.to_numpy(float)
        for joint in range(4):
            target = frame[f"j{joint}_target"].to_numpy(float)
            actual = frame[f"j{joint}_actual"].to_numpy(float)
            error = target - actual
            step_rmse.append(float(np.sqrt(np.mean(error * error))))
            tail = max(1, int(round(2.0 / max(0.05, float(np.median(np.diff(t)))))))
            step_steady.append(float(abs(np.mean(error[-tail:]))))
            direction = 1.0 if phase == "step_plus" else -1.0
            signed_beyond = direction * (actual - target)
            step_over.append(float(max(0.0, np.max(signed_beyond) - 1.0)))
            step_settle.append(settling_time(t, error))

    sine = sine_all[sine_all.phase == "sine"].copy()
    # Pandas can expose a read-only view here (notably with the bundled
    # runtime's copy-on-write mode).  The phase fit shifts time to zero, so
    # explicitly own the array instead of mutating a read-only view.
    t = sine.t_rel.to_numpy(float).copy()
    t -= t[0]
    target = sine.command_j03_target.to_numpy(float)
    actual = sine.j3_actual.to_numpy(float)
    target_amp, target_phase, _ = fit_sine(t, target)
    actual_amp, actual_phase, _ = fit_sine(t, actual)
    lag_rad = wrap_pi(target_phase - actual_phase)
    cross = [fit_sine(t, sine[f"j{joint}_actual"].to_numpy(float))[0] for joint in range(3)]
    step_peak = float(step.peak_tension_n.max())
    sine_peak = float(sine_all.peak_tension_n.max())

    mean_step_rmse = float(np.mean(step_rmse))
    mean_step_steady = float(np.mean(step_steady))
    mean_step_over = float(np.mean(step_over))
    mean_step_settle = float(np.mean(step_settle))
    sine_rmse = float(np.sqrt(np.mean((target - actual) ** 2)))
    amp_ratio = actual_amp / max(target_amp, 1.0e-9)
    lag_s = lag_rad / (2.0 * math.pi * 0.1)
    cross_amp = float(max(cross))
    score = (
        mean_step_rmse
        + 1.8 * mean_step_steady
        + 0.20 * mean_step_over
        + 0.08 * mean_step_settle
        + 0.55 * sine_rmse
        + 1.2 * abs(lag_s)
        + 3.0 * abs(1.0 - amp_ratio)
        + 0.5 * cross_amp
    )
    return Result(
        kp=kp,
        ki=ki,
        score=float(score),
        step_rmse_deg=mean_step_rmse,
        step_steady_error_deg=mean_step_steady,
        step_overshoot_deg=mean_step_over,
        step_settling_s=mean_step_settle,
        sine_rmse_deg=sine_rmse,
        sine_amplitude_ratio=float(amp_ratio),
        sine_phase_lag_deg=float(math.degrees(lag_rad)),
        sine_phase_lag_s=float(lag_s),
        cross_coupling_amp_deg=cross_amp,
        peak_tension_n=max(step_peak, sine_peak),
        step_csv=str(step_path),
        sine_csv=str(sine_path),
    )


def stop_controller() -> None:
    port = open_serial_port("COM6", 921600, timeout_s=0.05)
    try:
        for command in ("stop", "tension off"):
            port.write((command + "\n").encode("ascii"))
            port.flush()
            time.sleep(0.03)
        for motor in range(5):
            port.write((f"tensionbias m{motor} 0\n").encode("ascii"))
            port.flush()
    finally:
        port.close()


def run_one(
    mode: str,
    kp: float,
    ki: float,
    output: Path,
    abort_tension_n: float,
    r_blend: float = 0.0,
    p_blend: float = 0.0,
) -> None:
    command = [
        sys.executable,
        str(RUNNER),
        "--mode", mode,
        "--kp", str(kp),
        "--ki", str(ki),
        "--r-blend", str(r_blend),
        "--p-blend", str(p_blend),
        "--baseline-s", "3",
        "--return-s", "5",
        "--abort-tension-n", str(abort_tension_n),
        "--output", str(output),
    ]
    if mode == "sine":
        command += ["--duration-s", "20", "--frequency-hz", "0.1", "--amplitude-deg", "10", "--mean-deg", "20"]
    else:
        command += ["--step-amplitude-deg", "5", "--step-hold-s", "6", "--center-hold-s", "4"]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    try:
        stop_controller()
    except Exception as exc:
        raise RuntimeError(f"failed to stop controller after {mode}: {exc}") from exc
    if completed.returncode != 0:
        raise RuntimeError(f"{mode} candidate Kp={kp}, Ki={ki} aborted with code {completed.returncode}")


def preposition(output_dir: Path, abort_tension_n: float) -> None:
    output = output_dir / "preposition_to_10_10_20_10.csv"
    command = [
        sys.executable,
        str(RUNNER),
        "--mode", "step",
        "--kp", "1.0",
        "--ki", "0.3",
        "--baseline-s", "30",
        "--step-amplitude-deg", "0",
        "--step-hold-s", "0",
        "--center-hold-s", "0",
        "--return-s", "0",
        "--startup-grace-s", "5",
        "--abort-tension-n", str(abort_tension_n),
        "--output", str(output),
    ]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    stop_controller()
    if completed.returncode != 0:
        raise RuntimeError(f"preposition aborted with code {completed.returncode}")
    frame = pd.read_csv(output)
    tail = frame.tail(10)
    errors = [abs(float(tail[f"j{joint}_actual"].mean()) - value) for joint, value in enumerate((10, 10, 20, 10))]
    if max(errors) > 2.0:
        raise RuntimeError(f"preposition did not reach base pose; mean errors={errors}")


def write_report(path: Path, results: list[Result]) -> None:
    ordered = sorted(results, key=lambda item: item.score)
    lines = [
        "# 直连 PI 自动调参结果",
        "",
        "每个候选均执行联合 ±5° 阶跃与 J03 20°±10°、0.1 Hz、2周期正弦。超调采用目标外 ±1° 死区。",
        "",
        "|排名|Kp|Ki (1/s)|得分|阶跃RMSE°|稳态误差°|有效超调°|稳定时间s|正弦RMSE°|幅值比|相位滞后° / s|耦合幅值°|峰值张力N|",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for rank, item in enumerate(ordered, start=1):
        lines.append(
            f"|{rank}|{item.kp:.3f}|{item.ki:.3f}|{item.score:.3f}|{item.step_rmse_deg:.3f}|"
            f"{item.step_steady_error_deg:.3f}|{item.step_overshoot_deg:.3f}|{item.step_settling_s:.3f}|"
            f"{item.sine_rmse_deg:.3f}|{item.sine_amplitude_ratio:.3f}|"
            f"{item.sine_phase_lag_deg:.1f} / {item.sine_phase_lag_s:.3f}|"
            f"{item.cross_coupling_amp_deg:.3f}|{item.peak_tension_n:.1f}|"
        )
    best = ordered[0]
    lines += ["", f"推荐：`Kp={best.kp:.3f}`，`Ki={best.ki:.3f} 1/s`。", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--candidates",
        type=parse_candidates,
        default=parse_candidates("0.45,0.03;0.65,0.05;0.80,0.10;1.00,0.10;1.00,0.20;1.00,0.30;1.20,0.15"),
    )
    parser.add_argument("--abort-tension-n", type=float, default=40.0)
    parser.add_argument("--r-blend", type=float, default=0.0)
    parser.add_argument("--p-blend", type=float, default=0.0)
    parser.add_argument("--output-dir")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed candidate CSVs in --output-dir and continue the remaining search.",
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("physical motion is disabled unless --execute is supplied")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir or ROOT / "run_data" / "direct_autotune" / f"pi_search_{stamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    if not args.resume:
        print("Prepositioning to [10,10,20,10] deg...", flush=True)
        preposition(output_dir, args.abort_tension_n)
    else:
        print(f"Resuming search in {output_dir}", flush=True)
    results: list[Result] = []
    for index, (kp, ki) in enumerate(args.candidates, start=1):
        print(f"[{index}/{len(args.candidates)}] Kp={kp:.3f}, Ki={ki:.3f}", flush=True)
        stem = f"c{index:02d}_kp{kp:.3f}_ki{ki:.3f}"
        step_path = output_dir / f"{stem}_step.csv"
        sine_path = output_dir / f"{stem}_sine.csv"
        if args.resume and step_path.exists() and sine_path.exists():
            print("  reusing completed step and sine CSVs", flush=True)
        else:
            run_one(
                "step", kp, ki, step_path, args.abort_tension_n,
                args.r_blend, args.p_blend,
            )
            run_one(
                "sine", kp, ki, sine_path, args.abort_tension_n,
                args.r_blend, args.p_blend,
            )
        result = analyze_pair(kp, ki, step_path, sine_path)
        results.append(result)
        (output_dir / "results.json").write_text(
            json.dumps([asdict(item) for item in results], indent=2, ensure_ascii=False), encoding="utf-8"
        )
        with (output_dir / "results.csv").open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=list(asdict(results[0]).keys()))
            writer.writeheader()
            writer.writerows(asdict(item) for item in results)
        write_report(output_dir / "report.md", results)
        print(f"  score={result.score:.3f}, peak={result.peak_tension_n:.1f}N", flush=True)

    best = min(results, key=lambda item: item.score)
    print(f"BEST Kp={best.kp:.3f} Ki={best.ki:.3f} score={best.score:.3f}")
    print(f"OUTPUT={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
