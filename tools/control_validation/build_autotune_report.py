#!/usr/bin/env python3
"""Build the 2026-07-19 controller tuning and multi-pose reports."""

from __future__ import annotations

import json
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output" / "control_autotune_20260719"
DOCS = ROOT / "docs"
MULTI = ROOT / "run_data" / "direct_autotune" / "multipose_20260719_local"
FAIL = ROOT / "run_data" / "direct_autotune" / "multipose_20260719_screen" / "pose_01_9_15_30_0" / "transition.csv"

P_BASE = np.array(
    [
        [4.3, 5.9, 0.0, 0.0],
        [-4.3, 5.9, 0.0, 0.0],
        [-1.2, -3.0, 6.0, 0.0],
        [1.2, -3.0, 0.0, 4.8],
        [0.0, -6.2, -6.0, -6.2],
    ],
    dtype=float,
)
P_IDENTIFIED = np.array(
    [
        [3.7902, 4.2846, 0.0, 0.0],
        [-2.7764, 4.3217, 0.0, 0.0],
        [-1.8163, -3.3031, 5.9482, 0.0],
        [2.3439, -1.9732, 0.0, 4.7259],
        [0.0, -3.4184, -6.1187, -4.0970],
    ],
    dtype=float,
)
P_FINAL = 0.75 * P_BASE + 0.25 * P_IDENTIFIED


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 140,
            "savefig.dpi": 180,
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def save(fig: plt.Figure, name: str) -> Path:
    path = OUT / name
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def rel(path: Path, base: Path) -> str:
    return Path(os.path.relpath(path, base)).as_posix()


def load_pi_results() -> pd.DataFrame:
    files = [
        ROOT / "run_data/direct_autotune/pi_search_20260719_070557/results.csv",
        ROOT / "run_data/direct_autotune/pi_refine_20260719_071810/results.csv",
        ROOT / "run_data/direct_autotune/joint_refine_20260719_075737/results.csv",
    ]
    frames = []
    for path in files:
        frame = pd.read_csv(path)
        frame["batch"] = path.parent.name
        frames.append(frame)
    return pd.concat(frames, ignore_index=True)


def plot_pi_search(pi: pd.DataFrame) -> Path:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    points = axes[0].scatter(
        pi.kp, pi.ki, c=pi.score, s=85, cmap="viridis_r", edgecolor="white", linewidth=0.8
    )
    axes[0].scatter([0.70], [0.02], s=180, marker="*", c="#e63946", label="推荐 0.70/0.02")
    axes[0].set(xlabel="Kp", ylabel="Ki (1/s)", title="PI 候选综合评分（越低越好）")
    axes[0].legend(frameon=False)
    fig.colorbar(points, ax=axes[0], label="综合评分")
    ordered = pi.sort_values("score").head(10).copy()
    labels = [f"{kp:.2f}/{ki:.3f}" for kp, ki in zip(ordered.kp, ordered.ki)]
    axes[1].barh(labels[::-1], ordered.score.to_numpy()[::-1], color="#2a9d8f")
    axes[1].axvline(ordered.score.min(), color="#e63946", ls="--", lw=1)
    axes[1].set(xlabel="综合评分", ylabel="Kp / Ki", title="评分最优的 10 组参数")
    return save(fig, "01_pi_search.png")


def load_matrix_results() -> pd.DataFrame:
    files = [
        ROOT / "run_data/direct_autotune/matrix_search_20260719_074321/results.csv",
        ROOT / "run_data/direct_autotune/matrix_search_20260719_074850/results.csv",
        ROOT / "run_data/direct_autotune/matrix_search_20260719_075311/results.csv",
    ]
    return pd.concat([pd.read_csv(path) for path in files], ignore_index=True)


def plot_matrix_search(matrix: pd.DataFrame) -> Path:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.3))
    r = matrix[matrix.p_blend == 0].sort_values("r_blend")
    p = matrix[matrix.r_blend == 0].sort_values("p_blend")
    axes[0].plot(r.r_blend, r.score, "o-", label="综合评分")
    axes[0].plot(r.r_blend, r.sine_phase_lag_deg / 6.0, "s--", label="相位滞后/6")
    axes[0].set(xlabel="前馈 R 混合比例", title="前馈 R：阶跃略有改善、相位变差")
    axes[0].legend(frameon=False)
    axes[1].plot(p.p_blend, p.score, "o-", color="#2a9d8f", label="综合评分")
    axes[1].axvline(0.25, color="#e63946", ls="--", label="推荐 25%")
    axes[1].set(xlabel="反馈 P 混合比例", title="反馈 P 的在线搜索")
    axes[1].legend(frameon=False)
    axes[2].plot(p.p_blend, p.sine_phase_lag_deg, "o-", label="相位滞后 (deg)")
    axes[2].plot(p.p_blend, p.peak_tension_n, "s--", label="峰值张力 (N)")
    axes[2].set(xlabel="反馈 P 混合比例", title="相位与张力的折衷")
    axes[2].legend(frameon=False)
    return save(fig, "02_matrix_search.png")


def plot_before_after() -> Path:
    before = pd.read_csv(ROOT / "run_data/direct_autotune/matrix_search_20260719_074321/m01_r0.00_p0.00_sine.csv")
    after = pd.read_csv(ROOT / "run_data/direct_autotune/matrix_search_20260719_074850/m01_r0.00_p0.25_sine.csv")
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=False)
    for ax, frame, title in (
        (axes[0], before, "修改前：P 混合 0%"),
        (axes[1], after, "修改后：P 混合 25%"),
    ):
        sine = frame[frame.phase == "sine"].copy()
        t = sine.t_rel - sine.t_rel.iloc[0]
        ax.plot(t, sine.command_j03_target, lw=2.0, color="#333333", label="J03 目标")
        ax.plot(t, sine.j3_actual, lw=1.8, color="#e76f51", label="J03 实测")
        ax.set(ylabel="角度 (deg)", title=title)
        ax.legend(ncol=2, frameon=False)
    axes[-1].set_xlabel("正弦段时间 (s)")
    return save(fig, "03_before_after_j03_sine.png")


def plot_failure() -> Path:
    frame = pd.read_csv(FAIL)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for motor in range(5):
        axes[0].plot(frame.t_rel, -frame[f"force_m{motor:02d}_n"], label=f"M{motor:02d}")
    axes[0].axhline(40, color="#d00000", ls="--", lw=1.5, label="主机停止线 40 N")
    axes[0].set(ylabel="拉力幅值 (N)", title="广域姿态扩展失败：M02 张力快速增长")
    axes[0].legend(ncol=6, frameon=False)
    for joint in range(4):
        axes[1].plot(frame.t_rel, frame[f"j{joint}_actual"], label=f"J{joint:02d} 实测")
        axes[1].plot(frame.t_rel, frame[f"j{joint}_target"], ls="--", alpha=0.55)
    axes[1].set(xlabel="时间 (s)", ylabel="角度 (deg)")
    axes[1].legend(ncol=4, frameon=False)
    return save(fig, "04_broad_pose_abort.png")


def plot_pose_trials(metrics: pd.DataFrame) -> list[Path]:
    paths: list[Path] = []
    for pose_index in sorted(metrics.pose_index.unique()):
        pose_metrics = metrics[metrics.pose_index == pose_index]
        pose = str(pose_metrics.pose.iloc[0])
        fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=False)
        for joint, ax in enumerate(axes):
            item = pose_metrics[(pose_metrics["mode"] == "step") & (pose_metrics.joint == joint)].iloc[0]
            frame = pd.read_csv(item.csv_path)
            t = frame.t_rel - frame.t_rel.iloc[0]
            ax.plot(t, frame[f"command_j{joint:02d}_target"], color="#333333", lw=1.8, label="目标")
            ax.plot(t, frame[f"j{joint}_actual"], color="#e76f51", lw=1.6, label="实测")
            ax.set_ylabel(f"J{joint:02d} (deg)")
            ax.set_title(
                f"RMSE={item.rmse_deg:.2f}°, 稳态误差={item.steady_error_deg:.2f}°, "
                f"1°稳定时间={item.settling_s:.2f}s"
            )
        axes[0].legend(ncol=2, frameon=False)
        axes[-1].set_xlabel("时间 (s)")
        fig.suptitle(f"姿态 {pose}：四关节 ±5° 阶跃", y=1.005, fontsize=15)
        paths.append(save(fig, f"pose_{pose_index:02d}_steps.png"))

        fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=False)
        for joint, ax in enumerate(axes):
            item = pose_metrics[(pose_metrics["mode"] == "sine") & (pose_metrics.joint == joint)].iloc[0]
            frame = pd.read_csv(item.csv_path)
            frame = frame[frame.phase == "sine"]
            t = frame.t_rel - frame.t_rel.iloc[0]
            ax.plot(t, frame[f"command_j{joint:02d}_target"], color="#333333", lw=1.8, label="目标")
            ax.plot(t, frame[f"j{joint}_actual"], color="#457b9d", lw=1.6, label="实测")
            ax.set_ylabel(f"J{joint:02d} (deg)")
            ax.set_title(
                f"{item.frequency_hz:.2f}Hz, 幅值比={item.amplitude_ratio:.2f}, "
                f"相位滞后={item.phase_lag_deg:.1f}° ({item.phase_lag_s:.2f}s)"
            )
        axes[0].legend(ncol=2, frameon=False)
        axes[-1].set_xlabel("正弦段时间 (s)")
        fig.suptitle(f"姿态 {pose}：四关节正弦", y=1.005, fontsize=15)
        paths.append(save(fig, f"pose_{pose_index:02d}_sines.png"))
    return paths


def plot_frequency(metrics: pd.DataFrame) -> Path:
    sine = metrics[metrics["mode"] == "sine"]
    grouped = sine.groupby("frequency_hz").agg(
        rmse=("rmse_deg", "mean"),
        ratio=("amplitude_ratio", "mean"),
        lag=("phase_lag_deg", "mean"),
        lag_s=("phase_lag_s", "mean"),
    ).reset_index()
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    axes[0].plot(grouped.frequency_hz, grouped.rmse, "o-", color="#e76f51")
    axes[0].set(xlabel="频率 (Hz)", ylabel="平均 RMSE (deg)", title="误差随频率上升")
    axes[1].plot(grouped.frequency_hz, grouped.ratio, "o-", color="#2a9d8f")
    axes[1].axhline(1.0, color="#555555", ls="--")
    axes[1].set(xlabel="频率 (Hz)", ylabel="平均幅值比", title="高频跟踪掉幅")
    axes[2].plot(grouped.frequency_hz, grouped.lag, "o-", label="相位滞后 (deg)")
    axes[2].plot(grouped.frequency_hz, grouped.lag_s, "s--", label="等效时间滞后 (s)")
    axes[2].set(xlabel="频率 (Hz)", title="相位滞后随频率显著增加")
    axes[2].legend(frameon=False)
    return save(fig, "11_frequency_summary.png")


def matrix_markdown(matrix: np.ndarray) -> str:
    return "\n".join(
        "[" + ", ".join(f"{value: .4f}" for value in row) + "]" for row in matrix
    )


def build_reports(figures: dict[str, Path], metrics: pd.DataFrame, pi: pd.DataFrame, matrix: pd.DataFrame) -> None:
    best_pi = pi.loc[pi.score.idxmin()]
    p_search = matrix[matrix.r_blend == 0].copy()
    best_p = p_search.loc[p_search.score.idxmin()]
    base_p = matrix[(matrix.r_blend == 0) & (matrix.p_blend == 0)].iloc[0]
    step = metrics[metrics["mode"] == "step"]
    sine = metrics[metrics["mode"] == "sine"]
    by_freq = sine.groupby("frequency_hz").agg(
        rmse=("rmse_deg", "mean"),
        amp=("amplitude_ratio", "mean"),
        lag=("phase_lag_deg", "mean"),
        lag_s=("phase_lag_s", "mean"),
    )

    control_lines = [
        "# 五腱四关节控制自动调参与矩阵优化报告（2026-07-19）",
        "",
        "## 结论先行",
        "",
        "- 局部综合推荐：`Kp=0.70`、`Ki=0.02 1/s`、前馈 `R blend=0`、反馈 `P blend=0.25`。",
        "- `P blend=0.25` 是局部推荐，不应在尚未完成广域安全验证前设为全局上电默认。",
        "- 单纯增大 Ki 不能解决正弦相位滞后；剩余误差主要是频率相关带宽/延迟，而不是静态矩阵的单一比例错误。",
        "- 1° 内不计超调时，推荐参数在三姿态阶跃中平均有效超调接近 0°，但 J02 的稳态误差仍偏大。",
        "",
        "## 1. 控制结构与被优化参数",
        "",
        "当前控制分为前馈、PI 任务反馈、任务空间投影和独立零空间张力分配。前馈使用 `R`，反馈使用 `P`，二者必须分开：",
        "",
        "```text",
        "ΔL_ff = 200 R (q_ref - q_entry)",
        "ΔL_PI = 200 P [Kp e + Ki ∫e dt]",
        "ΔL_task = (R R⁺)(ΔL_ff + ΔL_PI)",
        "x_requested = x_entry + ΔL_task + α n",
        "x_cmd = x_now + clip(x_requested - x_now, -20, 20)",
        "```",
        "",
        "本轮没有用张力 ΔF 做前馈；张力仅用于零空间分配和安全保护。",
        "",
        "## 2. Kp/Ki 搜索",
        "",
        f"共汇总 {len(pi)} 组 PI 候选。评分最小点为 `Kp={best_pi.kp:.2f}`、`Ki={best_pi.ki:.3f}`，综合评分 {best_pi.score:.3f}。",
        "高 Ki 候选并没有稳定降低相位滞后，部分候选反而增加张力峰值和阶跃误差。",
        "",
        f"![PI 搜索]({rel(figures['pi'], DOCS)})",
        "",
        "## 3. 为什么以及怎样优化矩阵",
        "",
        "1. 先固定 PI，分别做四个关节的 ±5° 独立阶跃；用稳定段的实际电机位移和实际角度增量拟合局部 5×4 映射。",
        "2. 保留腱绳物理稀疏结构，并用小岭回归向原矩阵约束，避免把噪声拟合成不存在的耦合。",
        "3. 不直接整矩阵替换，而用 0%～100% 混合比例逐点实测。",
        "4. 分开扫描前馈 R 和反馈 P。R 改动使相位变差，因此最终 R 保持原值；P 的 25% 混合综合评分最低。",
        "",
        f"原 P 综合评分 {base_p.score:.3f}，P 混合 25% 后 {best_p.score:.3f}（下降 {(base_p.score-best_p.score)/base_p.score*100:.1f}%）；"
        f"阶跃稳态误差由 {base_p.step_steady_error_deg:.3f}° 降至 {best_p.step_steady_error_deg:.3f}°。",
        "",
        f"![矩阵搜索]({rel(figures['matrix'], DOCS)})",
        "",
        f"![J03 修改前后]({rel(figures['before_after'], DOCS)})",
        "",
        "### 局部推荐反馈矩阵 P（mm/rad）",
        "",
        "`P_final = 0.75 P_base + 0.25 P_identified`：",
        "",
        "```text",
        matrix_markdown(P_FINAL),
        "```",
        "",
        "## 4. 最终参数与边界",
        "",
        "| 参数 | 推荐值 | 说明 |",
        "|---|---:|---|",
        "| Kp | 0.70 | 三姿态局部综合值 |",
        "| Ki | 0.02 1/s | 再增大不能消除相位滞后 |",
        "| 前馈 R blend | 0 | 拟合 R 的混合使正弦相位更差 |",
        "| 反馈 P blend | 0.25 | 局部评分最佳；暂不设全局上电默认 |",
        "| 主机试验保护 | 40 N | 本轮保守验证线 |",
        "| 单周期电机有限差分 | 20 counts/10 ms | 高频相位滞后的重要来源之一 |",
        "",
        "## 5. 仍需改进的地方",
        "",
        "- 静态 P 矩阵不能抵消频率相关的 0.20 Hz 相位滞后。下一阶段应考虑目标速度前馈/相位超前，而不是继续提高 Ki。",
        "- J02 在三姿态中的阶跃稳态误差约 1.7°～2.0°，低频正弦幅值比也最低，需要单独辨识 J02 通道或增加列增益，而不能把全局 Kp 一起放大。",
        "- 若加入 D 项，必须先对角速度低通并做很小的增益扫描；当前证据更支持速度前馈而非直接高增益 D。",
        "- 广域姿态必须采用可行域搜索和分段路径，不能直接跳到 27 个预设点。",
        "",
    ]
    (DOCS / "control_autotune_report_20260719.md").write_text("\n".join(control_lines), encoding="utf-8")

    test_lines = [
        "# 三姿态阶跃与正弦验证报告（2026-07-19）",
        "",
        "## 测试范围",
        "",
        "安全完成三个姿态：`[10,10,20,10]°`、`[10,15,20,10]°`、`[5,10,20,10]°`。每个姿态包含四个关节的 ±5° 双向阶跃和四个关节的一周期正弦，共 24 项。J01 目标和实测始终大于 0°。",
        "",
        f"阶跃平均 RMSE 为 {step.rmse_deg.mean():.3f}°，平均稳态误差 {step.steady_error_deg.mean():.3f}°，"
        f"按 1° 带宽的平均稳定时间 {step.settling_s.mean():.3f}s，有效超调 {step.effective_overshoot_deg.mean():.3f}°。",
        f"正弦平均 RMSE 为 {sine.rmse_deg.mean():.3f}°，全程峰值张力 {metrics.peak_tension_n.max():.1f}N。",
        "",
        "## 频率特性",
        "",
        "| 频率 | 平均 RMSE | 平均幅值比 | 平均相位滞后 | 等效时间滞后 |",
        "|---:|---:|---:|---:|---:|",
    ]
    for frequency, row in by_freq.iterrows():
        test_lines.append(
            f"| {frequency:.2f} Hz | {row.rmse:.3f}° | {row.amp:.3f} | {row.lag:.1f}° | {row.lag_s:.3f}s |"
        )
    test_lines += [
        "",
        f"![频率汇总]({rel(figures['frequency'], DOCS)})",
        "",
        "## 广域扩展的安全失败",
        "",
        "尝试从基准附近移动到 `[9,15,30,0]°` 时，M02 张力在约 3.2s 内由 24.6N 上升到 39.5N，并在下一采样达到 40N 主机停止线。所有 host tensionbias 均为 0，因此该问题来自任务姿态/耦合，而不是额外零空间偏置。该姿态及后续广域点未继续执行。",
        "",
        f"![广域失败]({rel(figures['failure'], DOCS)})",
        "",
        "## 每个姿态的完整曲线",
        "",
    ]
    for pose_index in sorted(metrics.pose_index.unique()):
        pose = metrics[metrics.pose_index == pose_index].pose.iloc[0]
        test_lines += [
            f"### 姿态 {pose_index}: `{pose}°`",
            "",
            f"![阶跃]({rel(OUT / f'pose_{pose_index:02d}_steps.png', DOCS)})",
            "",
            f"![正弦]({rel(OUT / f'pose_{pose_index:02d}_sines.png', DOCS)})",
            "",
        ]
    test_lines += [
        "## 建议的下一轮实验",
        "",
        "1. 先在这三个安全姿态增加 0.02/0.05/0.10/0.15/0.20Hz 扫频，辨识每个关节的带宽与纯延迟。",
        "2. 对 J02 单独重辨识反馈矩阵第二列（程序索引 J02 为第 3 列），限制其余列不变。",
        "3. 加入速度前馈后，先只在 0.10Hz 和 0.20Hz 比较相位，不改变零空间张力逻辑。",
        "4. 用张力可行域引导姿态扩展：每次目标增量不超过 2°，预测/实测任一路超过 35N 就回退，禁止直接跨到广域目标。",
        "",
    ]
    (DOCS / "multipose_validation_report_20260719.md").write_text("\n".join(test_lines), encoding="utf-8")


def build_inline_html(metrics: pd.DataFrame, matrix: pd.DataFrame) -> Path:
    vis_root = Path(r"C:\Users\A\.codex\visualizations\2026\07\15\019f64e7-4f3c-7eb2-8039-9f460689dd18")
    vis_root.mkdir(parents=True, exist_ok=True)
    path = vis_root / "control-autotune-20260719.html"
    sine = metrics[metrics["mode"] == "sine"].sort_values(["frequency_hz", "joint"])
    step = metrics[metrics["mode"] == "step"]
    freq = sine.groupby("frequency_hz").agg(
        rmse=("rmse_deg", "mean"), amp=("amplitude_ratio", "mean"), lag=("phase_lag_deg", "mean")
    ).reset_index()
    payload = {
        "frequency": freq.round(4).to_dict("records"),
        "steps": step[["pose", "joint", "steady_error_deg", "settling_s", "peak_tension_n"]].round(4).to_dict("records"),
        "matrix": P_FINAL.round(4).tolist(),
    }
    html = f"""<!doctype html>
<html lang=\"zh-CN\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
<title>五腱控制自动调参</title><style>
:root{{--bg:#f5f7fb;--card:#fff;--ink:#172033;--muted:#667085;--accent:#176b87;--good:#087f5b;--warn:#c2410c}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font:14px/1.55 system-ui,'Microsoft YaHei',sans-serif}}
main{{max-width:1180px;margin:auto;padding:28px}}h1{{font-size:28px;margin:0 0 8px}}.sub{{color:var(--muted);margin-bottom:22px}}
.grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}}.card{{background:var(--card);border:1px solid #e3e7ef;border-radius:14px;padding:18px;box-shadow:0 3px 12px #1720330c}}
.value{{font-size:28px;font-weight:750;color:var(--accent)}}.label{{color:var(--muted)}}.wide{{grid-column:span 2}}table{{width:100%;border-collapse:collapse}}
th,td{{padding:9px 10px;border-bottom:1px solid #edf0f5;text-align:right}}th:first-child,td:first-child{{text-align:left}}.bar{{height:9px;background:#e8edf3;border-radius:8px;overflow:hidden}}.bar i{{display:block;height:100%;background:var(--accent)}}
.ok{{color:var(--good)}}.warn{{color:var(--warn)}}@media(max-width:800px){{.grid{{grid-template-columns:1fr 1fr}}.wide{{grid-column:span 2}}}}
</style></head><body><main><h1>五腱控制自动调参结果</h1><div class=\"sub\">2026-07-19 · 24 项三姿态实测 · 1° 内不计超调</div>
<section class=\"grid\"><div class=\"card\"><div class=\"label\">推荐 Kp</div><div class=\"value\">0.70</div></div>
<div class=\"card\"><div class=\"label\">推荐 Ki</div><div class=\"value\">0.02/s</div></div>
<div class=\"card\"><div class=\"label\">前馈 R blend</div><div class=\"value\">0</div></div>
<div class=\"card\"><div class=\"label\">反馈 P blend</div><div class=\"value\">25%</div></div>
<div class=\"card wide\"><h2>正弦频率特性</h2><table><thead><tr><th>频率</th><th>RMSE</th><th>幅值比</th><th>相位滞后</th></tr></thead><tbody id=\"freq\"></tbody></table></div>
<div class=\"card wide\"><h2>结论</h2><p class=\"ok\">低频 0.05 Hz 已可较好跟踪，矩阵 P 的 25% 混合改善阶跃综合指标。</p><p class=\"warn\">0.20 Hz 相位滞后约 107°，静态矩阵无法消除频率相关延迟；下一步应采用速度前馈/相位超前。</p><p>广域目标 [9,15,30,0]° 使 M02 张力快速接近 40 N，已安全停止，未继续扩大姿态。</p></div>
<div class=\"card wide\"><h2>局部推荐 P 矩阵 (mm/rad)</h2><table id=\"matrix\"></table></div>
<div class=\"card wide\"><h2>阶跃摘要</h2><table><thead><tr><th>姿态 / 关节</th><th>稳态误差</th><th>1°稳定时间</th><th>峰值张力</th></tr></thead><tbody id=\"steps\"></tbody></table></div>
</section></main><script>const d={json.dumps(payload, ensure_ascii=False)};
document.querySelector('#freq').innerHTML=d.frequency.map(x=>`<tr><td>${{x.frequency_hz.toFixed(2)}} Hz</td><td>${{x.rmse.toFixed(2)}}°</td><td>${{x.amp.toFixed(2)}}</td><td>${{x.lag.toFixed(1)}}°</td></tr>`).join('');
document.querySelector('#matrix').innerHTML=d.matrix.map((r,i)=>`<tr><th>M0${{i}}</th>${{r.map(v=>`<td>${{v.toFixed(4)}}</td>`).join('')}}</tr>`).join('');
document.querySelector('#steps').innerHTML=d.steps.map(x=>`<tr><td>${{x.pose}} / J0${{x.joint}}</td><td>${{x.steady_error_deg.toFixed(2)}}°</td><td>${{x.settling_s.toFixed(2)}}s</td><td>${{x.peak_tension_n.toFixed(1)}}N</td></tr>`).join('');
</script></body></html>"""
    path.write_text(html, encoding="utf-8")
    return path


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    DOCS.mkdir(parents=True, exist_ok=True)
    configure_style()
    pi = load_pi_results()
    matrix = load_matrix_results()
    metrics = pd.read_csv(MULTI / "metrics.csv")
    figures = {
        "pi": plot_pi_search(pi),
        "matrix": plot_matrix_search(matrix),
        "before_after": plot_before_after(),
        "failure": plot_failure(),
        "frequency": plot_frequency(metrics),
    }
    plot_pose_trials(metrics)
    build_reports(figures, metrics, pi, matrix)
    inline = build_inline_html(metrics, matrix)
    summary = {
        "recommended": {"kp": 0.70, "ki": 0.02, "r_blend": 0.0, "p_blend": 0.25},
        "p_final": P_FINAL.round(6).tolist(),
        "metrics_csv": str(MULTI / "metrics.csv"),
        "control_report": str(DOCS / "control_autotune_report_20260719.md"),
        "test_report": str(DOCS / "multipose_validation_report_20260719.md"),
        "inline_html": str(inline),
    }
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
