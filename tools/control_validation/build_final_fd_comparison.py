#!/usr/bin/env python3
"""Build the final evidence plot for selecting the motor finite-difference limit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from tools.control_validation.run_direct_multipose_validation import analyze_sine


POSE = (0.0, 32.0, 35.0, 52.0)


def setup_plot() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "figure.dpi": 120,
            "savefig.dpi": 190,
        }
    )


def first_cycle(path: Path, joint: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    frame = pd.read_csv(path)
    frame = frame[frame.phase == "sine"].copy()
    t = frame.t_rel.to_numpy(float).copy()
    t -= t[0]
    keep = t <= 10.0
    target = frame[f"command_j{joint:02d}_target"].to_numpy(float)[keep]
    actual = frame[f"j{joint}_actual"].to_numpy(float)[keep]
    return t[keep], target, actual


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    setup_plot()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    transition_summary = json.loads(
        (args.workspace / "output/control_autotune_20260719/31_fd60_transition_summary.json").read_text(
            encoding="utf-8"
        )
    )
    fd_values = (20, 40, 60)
    transition = [transition_summary[f"fd{fd}"] for fd in fd_values]
    errors = np.array(
        [
            [item["joint"][f"J0{joint}"]["final_error_deg"] for joint in range(4)]
            for item in transition
        ],
        dtype=float,
    )
    mean_errors = np.mean(np.abs(errors), axis=1)
    peak_tensions = np.array([item["peak_tension_n"] for item in transition], dtype=float)

    fd60_sine_path = (
        args.workspace
        / "run_data/direct_autotune/fd60_safe3_repair/pose_03_0_32_35_52/sine_j1_0.10hz.csv"
    )
    fd80_sine_path = (
        args.workspace
        / "run_data/direct_autotune/fd80_single_pose_20260719/sine_j1_0.10hz_10cycles.csv"
    )
    fd60_metric = analyze_sine(1, POSE, 1, 0.10, fd60_sine_path)
    fd80_metric = analyze_sine(1, POSE, 1, 0.10, fd80_sine_path)
    t60, target60, actual60 = first_cycle(fd60_sine_path, 1)
    t80, target80, actual80 = first_cycle(fd80_sine_path, 1)

    fig = plt.figure(figsize=(14, 9.2))
    grid = fig.add_gridspec(2, 2, width_ratios=(1.08, 0.92), hspace=0.34, wspace=0.28)
    ax0 = fig.add_subplot(grid[0, 0])
    x = np.arange(4)
    width = 0.24
    for index, fd in enumerate(fd_values):
        ax0.bar(x + (index - 1) * width, np.abs(errors[index]), width, label=f"FD={fd}")
    ax0.set_xticks(x, [f"J0{joint}" for joint in range(4)])
    ax0.set_ylabel("末端绝对误差 (°)")
    ax0.set_title("同一大步姿态转换：20/40/60 counts")
    ax0.legend(ncol=3, fontsize=8)

    ax1 = fig.add_subplot(grid[0, 1])
    ax1.plot(fd_values, mean_errors, "o-", color="#3b73b9", label="四关节平均末端误差")
    ax1.set_xlabel("有限差分 (counts/10 ms)")
    ax1.set_ylabel("平均末端误差 (°)", color="#3b73b9")
    ax1.tick_params(axis="y", labelcolor="#3b73b9")
    ax1.set_xticks(fd_values)
    ax1b = ax1.twinx()
    ax1b.plot(fd_values, peak_tensions, "s--", color="#d95f5f", label="峰值拉力")
    ax1b.set_ylabel("峰值拉力幅值 (N)", color="#d95f5f")
    ax1b.tick_params(axis="y", labelcolor="#d95f5f")
    ax1.set_title("速度收益与张力代价")
    lines = ax1.get_lines() + ax1b.get_lines()
    ax1.legend(lines, [line.get_label() for line in lines], loc="best", fontsize=8)

    ax2 = fig.add_subplot(grid[1, 0])
    ax2.plot(t60, target60, color="#222222", lw=1.2, label="目标")
    ax2.plot(t60, actual60, color="#7b6fd0", lw=1.7, label="FD60 实测")
    ax2.plot(t80, actual80, color="#56a64b", lw=1.7, label="FD80 实测")
    ax2.set_xlabel("J01 0.10 Hz 首周期时间 (s)")
    ax2.set_ylabel("J01 (°)")
    ax2.set_title("同姿态、同频率 J01 正弦响应")
    ax2.legend(ncol=3, fontsize=8)

    ax3 = fig.add_subplot(grid[1, 1])
    labels = ("RMSE (°)", "滞后 (s)", "幅值误差 |1-r|")
    fd60_values = (
        fd60_metric.rmse_deg,
        fd60_metric.phase_lag_s,
        abs(1.0 - fd60_metric.amplitude_ratio),
    )
    fd80_values = (
        fd80_metric.rmse_deg,
        fd80_metric.phase_lag_s,
        abs(1.0 - fd80_metric.amplitude_ratio),
    )
    y = np.arange(3)
    ax3.barh(y + 0.18, fd60_values, 0.34, label="FD60")
    ax3.barh(y - 0.18, fd80_values, 0.34, label="FD80")
    ax3.set_yticks(y, labels)
    ax3.invert_yaxis()
    ax3.set_title("J01 正弦指标（越小越好）")
    ax3.legend(fontsize=8)

    fig.suptitle("有限差分阈值证据审计：80 counts 为当前最佳默认候选", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    figure_path = args.output_dir / "fd_limit_comparison.png"
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    result = {
        "transition_comparison": {
            str(fd): {
                "mean_abs_final_error_deg": float(mean_errors[index]),
                "peak_tension_n": float(peak_tensions[index]),
            }
            for index, fd in enumerate(fd_values)
        },
        "same_pose_same_frequency_j01": {
            "fd60": {
                "rmse_deg": fd60_metric.rmse_deg,
                "amplitude_ratio": fd60_metric.amplitude_ratio,
                "phase_lag_s": fd60_metric.phase_lag_s,
                "peak_tension_n": fd60_metric.peak_tension_n,
            },
            "fd80": {
                "rmse_deg": fd80_metric.rmse_deg,
                "amplitude_ratio": fd80_metric.amplitude_ratio,
                "phase_lag_s": fd80_metric.phase_lag_s,
                "peak_tension_n": fd80_metric.peak_tension_n,
            },
        },
        "recommendation": {
            "default_counts": 80,
            "reason": "best observed J01 tracking with safe measured tension; broad-pose FD80 validation remains incomplete",
            "experiment_guard_tension_n": [5, 80],
            "host_hard_release_trigger_n": 90,
        },
        "figure": str(figure_path),
    }
    (args.output_dir / "fd_limit_comparison.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
