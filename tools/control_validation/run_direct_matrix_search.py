#!/usr/bin/env python3
"""Guarded online search for feedforward-R and feedback-P blend factors."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_direct_j03_sine.py")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.control_validation.run_direct_pi_search import analyze_pair, stop_controller


def parse_pairs(text: str) -> list[tuple[float, float]]:
    pairs: list[tuple[float, float]] = []
    for chunk in text.split(";"):
        if not chunk.strip():
            continue
        values = [float(value.strip()) for value in chunk.split(",")]
        if len(values) != 2 or not all(0.0 <= value <= 1.0 for value in values):
            raise argparse.ArgumentTypeError("pairs must be rblend,pblend in [0,1]")
        pairs.append((values[0], values[1]))
    if not pairs:
        raise argparse.ArgumentTypeError("at least one matrix pair is required")
    return pairs


def run_mode(
    mode: str,
    kp: float,
    ki: float,
    r_blend: float,
    p_blend: float,
    output: Path,
    abort_tension_n: float,
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
        command += [
            "--duration-s", "20",
            "--frequency-hz", "0.1",
            "--amplitude-deg", "10",
            "--mean-deg", "20",
            "--sine-joint", "3",
        ]
    else:
        command += [
            "--step-amplitude-deg", "5",
            "--step-hold-s", "6",
            "--center-hold-s", "4",
            "--step-joint", "-1",
        ]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    stop_controller()
    if completed.returncode != 0:
        raise RuntimeError(
            f"{mode} matrix candidate r={r_blend}, p={p_blend} aborted "
            f"with code {completed.returncode}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", type=parse_pairs, required=True)
    parser.add_argument("--kp", type=float, default=0.70)
    parser.add_argument("--ki", type=float, default=0.02)
    parser.add_argument("--abort-tension-n", type=float, default=40.0)
    parser.add_argument("--output-dir")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("physical motion is disabled unless --execute is supplied")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(
        args.output_dir
        or ROOT / "run_data" / "direct_autotune" / f"matrix_search_{stamp}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for index, (r_blend, p_blend) in enumerate(args.pairs, start=1):
        print(
            f"[{index}/{len(args.pairs)}] rblend={r_blend:.2f}, "
            f"pblend={p_blend:.2f}",
            flush=True,
        )
        stem = f"m{index:02d}_r{r_blend:.2f}_p{p_blend:.2f}"
        step_path = output_dir / f"{stem}_step.csv"
        sine_path = output_dir / f"{stem}_sine.csv"
        run_mode("step", args.kp, args.ki, r_blend, p_blend, step_path, args.abort_tension_n)
        run_mode("sine", args.kp, args.ki, r_blend, p_blend, sine_path, args.abort_tension_n)
        metrics = asdict(analyze_pair(args.kp, args.ki, step_path, sine_path))
        row = {"r_blend": r_blend, "p_blend": p_blend, **metrics}
        rows.append(row)
        (output_dir / "results.json").write_text(
            json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        with (output_dir / "results.csv").open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(
            f"  score={row['score']:.3f}, lag={row['sine_phase_lag_deg']:.1f}deg, "
            f"amp={row['sine_amplitude_ratio']:.3f}, peak={row['peak_tension_n']:.1f}N",
            flush=True,
        )
    best = min(rows, key=lambda item: float(item["score"]))
    print(
        f"BEST rblend={best['r_blend']:.2f} pblend={best['p_blend']:.2f} "
        f"score={best['score']:.3f}"
    )
    print(f"OUTPUT={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
