#!/usr/bin/env python3
"""Plot the FD=80 single-pose long step and ten-cycle sine validation."""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from tools.control_validation.run_direct_multipose_validation import (
    analyze_sine,
    analyze_step,
)


JOINT_COLORS = ("#d95f5f", "#e69f00", "#56a64b", "#7b6fd0")
POSE = (0.0, 32.0, 35.0, 52.0)


def setup_plot() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.dpi": 120,
            "savefig.dpi": 190,
            "axes.grid": True,
            "grid.alpha": 0.22,
        }
    )


def phase_edges(frame: pd.DataFrame) -> list[tuple[str, float, float]]:
    order = list(dict.fromkeys(frame["phase"].astype(str)))
    return [
        (
            phase,
            float(frame.loc[frame.phase == phase, "t_rel"].min()),
            float(frame.loc[frame.phase == phase, "t_rel"].max()),
        )
        for phase in order
    ]


def plot_step(frame: pd.DataFrame, metrics: list, output: Path) -> None:
    labels = {
        "baseline": "基准稳定",
        "step_plus": "+5° 平台（30 s）",
        "center": "回中",
        "step_minus": "-5° 平台（30 s）",
        "return": "最终回中",
    }
    phases = phase_edges(frame)
    fig, axes = plt.subplots(4, 1, figsize=(14, 11.2), sharex=True)
    for joint, axis in enumerate(axes):
        t = frame.t_rel.to_numpy(float)
        target = frame[f"j{joint}_target"].to_numpy(float)
        actual = frame[f"j{joint}_actual"].to_numpy(float)
        axis.plot(t, target, color="#222222", lw=1.45, label="目标")
        axis.plot(t, actual, color=JOINT_COLORS[joint], lw=1.75, label="实测")
        for index, (phase, start, end) in enumerate(phases):
            if index % 2:
                axis.axvspan(start, end, color="#999999", alpha=0.055, lw=0)
            if joint == 0:
                axis.text(
                    (start + end) / 2,
                    0.955,
                    labels.get(phase, phase),
                    transform=axis.get_xaxis_transform(),
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )
        metric = metrics[joint]
        axis.set_title(
            f"J0{joint}: RMSE={metric.rmse_deg:.2f}°，稳态误差={metric.steady_error_deg:.2f}°，"
            f"±1°稳定时间={metric.settling_s:.2f}s，有效超调={metric.effective_overshoot_deg:.2f}°",
            fontsize=10,
        )
        axis.set_ylabel(f"J0{joint} (°)")
        axis.legend(loc="best", ncol=2, fontsize=8)
    axes[-1].set_xlabel("实验时间 (s)")
    fig.suptitle(
        "FD=80 counts：单姿态四关节联合长阶跃（姿态 0°, 32°, 35°, 52°）",
        fontsize=15,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def plot_valid_sines(
    sine_frames: dict[int, pd.DataFrame], metrics: dict[int, object], output: Path
) -> None:
    fig, axes = plt.subplots(len(sine_frames), 1, figsize=(14, 7.2), sharex=True)
    if not isinstance(axes, np.ndarray):
        axes = np.array([axes])
    for axis, joint in zip(axes, sorted(sine_frames)):
        part = sine_frames[joint]
        t = part.t_rel.to_numpy(float).copy()
        t -= t[0]
        target = part[f"command_j{joint:02d}_target"].to_numpy(float)
        actual = part[f"j{joint}_actual"].to_numpy(float)
        metric = metrics[joint]
        axis.plot(t, target, color="#222222", lw=1.35, label="目标")
        axis.plot(t, actual, color=JOINT_COLORS[joint], lw=1.65, label="实测")
        for cycle in range(11):
            axis.axvline(cycle * 10.0, color="#777777", lw=0.65, alpha=0.22)
        axis.set_xlim(0, 100)
        axis.set_ylabel(f"J0{joint} (°)")
        axis.set_title(
            f"J0{joint}: 0.10 Hz，10周期，RMSE={metric.rmse_deg:.2f}°，"
            f"幅值比={metric.amplitude_ratio:.3f}，滞后={metric.phase_lag_deg:.1f}°/{metric.phase_lag_s:.3f}s",
            fontsize=10,
        )
        axis.legend(loc="upper right", ncol=2, fontsize=8)
    axes[-1].set_xlabel("正弦阶段时间 (s)；竖线为周期边界")
    fig.suptitle(
        "FD=80 counts：单姿态十周期正弦响应（力传感器有效部分）",
        fontsize=15,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def plot_j2_invalid(frame: pd.DataFrame, metric, output: Path) -> None:
    sine = frame[frame.phase == "sine"].copy()
    t = sine.t_rel.to_numpy(float).copy()
    t -= t[0]
    fig, axes = plt.subplots(2, 1, figsize=(14, 7.8), sharex=True)
    axes[0].plot(t, sine.command_j02_target, color="#222222", lw=1.35, label="J02目标")
    axes[0].plot(t, sine.j2_actual, color=JOINT_COLORS[2], lw=1.65, label="J02实测")
    axes[0].set_ylabel("J02 (°)")
    axes[0].set_title(
        f"仅供角度诊断：RMSE={metric.rmse_deg:.2f}°，幅值比={metric.amplitude_ratio:.3f}，"
        f"滞后={metric.phase_lag_deg:.1f}°/{metric.phase_lag_s:.3f}s"
    )
    axes[0].legend(loc="upper right", ncol=2, fontsize=8)
    for motor in range(5):
        axes[1].plot(
            t,
            sine[f"force_m0{motor}_n"],
            lw=1.2,
            label=f"M0{motor}传感器",
        )
    axes[1].axhspan(-80, -5, color="#56a64b", alpha=0.08, label="有效负拉力范围")
    axes[1].axhline(0, color="#222222", lw=1.0)
    axes[1].set_ylabel("传感器读数 (N)")
    axes[1].set_xlabel("正弦阶段时间 (s)")
    axes[1].legend(loc="upper left", ncol=3, fontsize=8)
    fig.suptitle(
        "J02 十周期重试：CH1/M01 跳为 +130～+275 N，力安全验证无效",
        color="#a52020",
        fontsize=15,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def write_inline_visual(output_dir: Path, destination: Path) -> None:
    figures = (
        ("fd80_long_step_response.png", "80 counts 单姿态四关节联合长阶跃响应"),
        ("fd80_ten_cycle_sine_valid_j0_j1.png", "J00、J01 有效十周期正弦响应"),
        ("fd80_j2_force_sensor_fault.png", "J02 角度诊断与 CH1/M01 传感器故障证据"),
    )
    blocks: list[str] = []
    for name, alt in figures:
        payload = base64.b64encode((output_dir / name).read_bytes()).decode("ascii")
        blocks.append(
            f'<figure><img src="data:image/png;base64,{payload}" alt="{alt}" '
            f'style="display:block;width:100%;height:auto"><figcaption class="sr-only">{alt}</figcaption></figure>'
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        '<div id="fd80-single-pose-validation">\n'
        + "\n".join(blocks)
        + "\n</div>\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--vis-html", type=Path)
    args = parser.parse_args()
    setup_plot()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    step_path = args.data_dir / "combined_step_long_fd80.csv"
    step = pd.read_csv(step_path)
    step_metrics = [analyze_step(1, POSE, joint, step_path) for joint in range(4)]

    valid_sine_frames: dict[int, pd.DataFrame] = {}
    valid_sine_metrics: dict[int, object] = {}
    rows: list[dict] = []
    for joint, name in (
        (0, "sine_j0_0.10hz_10cycles.csv"),
        (1, "sine_j1_0.10hz_10cycles.csv"),
    ):
        path = args.data_dir / name
        frame = pd.read_csv(path)
        valid_sine_frames[joint] = frame[frame.phase == "sine"].copy()
        valid_sine_metrics[joint] = analyze_sine(1, POSE, joint, 0.10, path)

    invalid_path = args.data_dir / "sine_j2_0.10hz_10cycles_retry.csv"
    invalid_frame = pd.read_csv(invalid_path)
    invalid_metric = analyze_sine(1, POSE, 2, 0.10, invalid_path)

    plot_step(step, step_metrics, args.output_dir / "fd80_long_step_response.png")
    plot_valid_sines(
        valid_sine_frames,
        valid_sine_metrics,
        args.output_dir / "fd80_ten_cycle_sine_valid_j0_j1.png",
    )
    plot_j2_invalid(
        invalid_frame,
        invalid_metric,
        args.output_dir / "fd80_j2_force_sensor_fault.png",
    )

    for metric in step_metrics:
        rows.append(
            {
                "test": "long_joint_step",
                "joint": metric.joint,
                "valid": True,
                "rmse_deg": metric.rmse_deg,
                "steady_error_deg": metric.steady_error_deg,
                "settling_1deg_s": metric.settling_s,
                "effective_overshoot_deg": metric.effective_overshoot_deg,
                "amplitude_ratio": np.nan,
                "phase_lag_deg": np.nan,
                "phase_lag_s": np.nan,
                "peak_tension_n": metric.peak_tension_n,
                "note": "30 s positive and negative platforms",
            }
        )
    for joint, metric in valid_sine_metrics.items():
        rows.append(
            {
                "test": "sine_0.10hz_10cycles",
                "joint": joint,
                "valid": True,
                "rmse_deg": metric.rmse_deg,
                "steady_error_deg": np.nan,
                "settling_1deg_s": np.nan,
                "effective_overshoot_deg": np.nan,
                "amplitude_ratio": metric.amplitude_ratio,
                "phase_lag_deg": metric.phase_lag_deg,
                "phase_lag_s": metric.phase_lag_s,
                "peak_tension_n": metric.peak_tension_n,
                "note": "force sensors valid",
            }
        )
    rows.append(
        {
            "test": "sine_0.10hz_10cycles",
            "joint": 2,
            "valid": False,
            "rmse_deg": invalid_metric.rmse_deg,
            "steady_error_deg": np.nan,
            "settling_1deg_s": np.nan,
            "effective_overshoot_deg": np.nan,
            "amplitude_ratio": invalid_metric.amplitude_ratio,
            "phase_lag_deg": invalid_metric.phase_lag_deg,
            "phase_lag_s": invalid_metric.phase_lag_s,
            "peak_tension_n": invalid_metric.peak_tension_n,
            "note": "angle-only; CH1/M01 force sensor invalid (+130 to +275 N)",
        }
    )
    metrics = pd.DataFrame(rows)
    metrics.to_csv(args.output_dir / "metrics.csv", index=False, encoding="utf-8-sig")
    summary = {
        "finite_difference_counts": 80,
        "pose_deg": list(POSE),
        "step_csv": str(step_path),
        "valid_sine_joints": [0, 1],
        "invalid_sine_joints": [2],
        "not_run_sine_joints": [3],
        "blocking_fault": "CH1/M01 force sensor became nonphysical positive (+130 to +275 N)",
        "max_abs_cmd_minus_now_counts": int(
            max((step[f"m{i}_cmd"] - step[f"m{i}_now"]).abs().max() for i in range(5))
        ),
        "step_peak_tension_n": float(step.peak_tension_n.max()),
        "artifacts": [
            "fd80_long_step_response.png",
            "fd80_ten_cycle_sine_valid_j0_j1.png",
            "fd80_j2_force_sensor_fault.png",
            "metrics.csv",
        ],
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if args.vis_html:
        write_inline_visual(args.output_dir, args.vis_html)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
