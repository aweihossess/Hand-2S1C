#!/usr/bin/env python3
"""Summarize the 27-pose broad-space validation and generate report figures."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "run_data" / "direct_autotune" / "multipose_20260719_broad70"
OUT = ROOT / "output" / "control_autotune_20260719"
DOC = ROOT / "docs" / "broad_validation_report_20260719.md"


def setup_plot() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.dpi": 120,
            "savefig.dpi": 180,
            "axes.grid": True,
            "grid.alpha": 0.22,
        }
    )


def load_skips() -> list[dict]:
    return [
        json.loads(path.read_text(encoding="utf-8"))
        for path in sorted(DATA.glob("pose_*/skipped.json"))
    ]


def plot_feasibility(frame: pd.DataFrame, skips: list[dict]) -> Path:
    feasible = {
        tuple(float(value) for value in pose.split(";"))
        for pose in frame.pose.unique()
    }
    skipped = {tuple(float(value) for value in item["pose"]) for item in skips}
    path = OUT / "20_broad_feasibility.png"
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8), sharex=True, sharey=True)
    for axis, j0 in zip(axes, (-9.0, 0.0, 9.0)):
        for pose in sorted(feasible | skipped):
            if pose[0] != j0:
                continue
            color = "#1f9d73" if pose in feasible else "#d9534f"
            marker = "o" if pose in feasible else "X"
            axis.scatter(pose[3], pose[2], s=110, marker=marker, c=color, edgecolors="white", linewidths=0.8)
            axis.annotate(f"J1={pose[1]:g}", (pose[3], pose[2]), xytext=(4, 5), textcoords="offset points", fontsize=7)
        axis.set_title(f"J00 = {j0:g}°")
        axis.set_xlabel("J03 target (deg)")
        axis.set_xticks((-39, 0, 39))
        axis.set_yticks((6, 30, 54))
    axes[0].set_ylabel("J02 target (deg)")
    fig.suptitle("27姿态广域验证可行性（绿圆：完成8项；红叉：过渡不可行）", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_metrics(frame: pd.DataFrame) -> Path:
    path = OUT / "21_broad_metrics.png"
    step = frame[frame.mode == "step"]
    sine = frame[frame.mode == "sine"]
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.7))

    joints = np.arange(4)
    steady = step.groupby("joint").steady_error_deg.mean().reindex(joints)
    settle = step.groupby("joint").settling_s.mean().reindex(joints)
    axes[0].bar(joints - 0.18, steady, 0.36, label="稳态误差 (deg)", color="#e76f51")
    axes[0].bar(joints + 0.18, settle, 0.36, label="±1°稳定时间 (s)", color="#2a9d8f")
    axes[0].set_xticks(joints, [f"J0{i}" for i in joints])
    axes[0].set_title("40个阶跃：按关节汇总")
    axes[0].legend(fontsize=8)

    freq = sorted(sine.frequency_hz.unique())
    amp = sine.groupby("frequency_hz").amplitude_ratio.mean().reindex(freq)
    lag = sine.groupby("frequency_hz").phase_lag_deg.mean().reindex(freq)
    ax2 = axes[1]
    ax2.plot(freq, amp, "o-", color="#2a9d8f", label="幅值比")
    ax2.set_ylim(0, 1.05)
    ax2.set_xlabel("频率 (Hz)")
    ax2.set_ylabel("幅值比")
    ax2b = ax2.twinx()
    ax2b.plot(freq, lag, "s--", color="#e76f51", label="相位滞后")
    ax2b.set_ylabel("相位滞后 (deg)")
    ax2.set_title("40个正弦：频率响应")
    lines = ax2.get_lines() + ax2b.get_lines()
    ax2.legend(lines, [line.get_label() for line in lines], fontsize=8, loc="upper left")

    peak = frame.groupby(["pose_index", "pose"]).peak_tension_n.max().reset_index().sort_values("pose_index")
    axes[2].bar(np.arange(len(peak)), peak.peak_tension_n, color="#457b9d")
    axes[2].axhline(70, color="#d62728", linestyle="--", label="70 N stop")
    axes[2].set_xticks(np.arange(len(peak)), [str(int(value)) for value in peak.pose_index], rotation=0)
    axes[2].set_xlabel("可行姿态原序号")
    axes[2].set_ylabel("峰值张力 (N)")
    axes[2].set_title("各可行姿态峰值张力")
    axes[2].legend(fontsize=8)

    fig.suptitle("广域验证性能汇总：10个可行姿态 / 80项", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def main() -> None:
    setup_plot()
    OUT.mkdir(parents=True, exist_ok=True)
    frame = pd.read_csv(DATA / "metrics.csv")
    skips = load_skips()
    feasibility = plot_feasibility(frame, skips)
    metrics = plot_metrics(frame)

    step = frame[frame.mode == "step"]
    sine = frame[frame.mode == "sine"]
    lines = [
        "# 27姿态广域验证报告（70 N绝对保护）",
        "",
        "## 结论",
        "",
        f"- 27个预设姿态全部尝试；{frame.pose_index.nunique()}个姿态通过过渡并完成全部测试，{len(skips)}个姿态因角度不可达/越界被安全跳过。",
        f"- 完成{len(frame)}个有效项目：{len(step)}个双向阶跃、{len(sine)}个单周期正弦。",
        f"- 全程最大张力为 **{frame.peak_tension_n.max():.1f} N**，没有触发70 N绝对停止线。",
        "- 负J03=-39°组合大多无法跟随；正J03与J02较高的部分组合仍可完成，呈明显方向非对称。",
        "",
        "![广域可行性](../output/control_autotune_20260719/20_broad_feasibility.png)",
        "",
        "![广域指标](../output/control_autotune_20260719/21_broad_metrics.png)",
        "",
        "## 阶跃结果",
        "",
        "| 关节 | RMSE | 稳态误差 | ±1°稳定时间 | 有效超调 | 峰值张力 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for joint, group in step.groupby("joint"):
        lines.append(
            f"| J0{joint} | {group.rmse_deg.mean():.3f}° | {group.steady_error_deg.mean():.3f}° | "
            f"{group.settling_s.mean():.3f} s | {group.effective_overshoot_deg.mean():.3f}° | {group.peak_tension_n.max():.1f} N |"
        )
    lines += [
        "",
        "## 正弦结果",
        "",
        "| 频率 | 数量 | RMSE | 幅值比 | 相位滞后 | 等效时间滞后 | 峰值张力 |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for frequency, group in sine.groupby("frequency_hz"):
        lines.append(
            f"| {frequency:.2f} Hz | {len(group)} | {group.rmse_deg.mean():.3f}° | "
            f"{group.amplitude_ratio.mean():.3f} | {group.phase_lag_deg.mean():.1f}° | "
            f"{group.phase_lag_s.mean():.3f} s | {group.peak_tension_n.max():.1f} N |"
        )
    lines += [
        "",
        "## 数据位置",
        "",
        "- 汇总指标：`run_data/direct_autotune/multipose_20260719_broad70/metrics.csv`",
        "- 每个可行姿态：对应 `pose_XX_*/step_*.csv` 与 `sine_*.csv`。",
        "- 每个不可行姿态：对应 `pose_XX_*/skipped.json` 与 `transition.csv`。",
        "",
        "## 工程判断",
        "",
        "把主机停止线从40 N改为70 N是合理的：此前第一个姿态的峰值约39 N，本轮可正常完成；但角度边界和过渡误差检查仍必须保留。当前主要限制不是张力增长速度，而是R/P局部模型、回差和机构走线造成的姿态方向相关可达性。",
    ]
    DOC.write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary = {
        "attempted_poses": 27,
        "feasible_poses": int(frame.pose_index.nunique()),
        "skipped_poses": len(skips),
        "completed_trials": len(frame),
        "step_trials": len(step),
        "sine_trials": len(sine),
        "peak_tension_n": float(frame.peak_tension_n.max()),
        "figures": [str(feasibility), str(metrics)],
    }
    (DATA / "broad_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
