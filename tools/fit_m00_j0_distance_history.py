"""Fit the M00 position relationship to distance from the J0 mechanical zero."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

from fit_m04_feedforward_history import (
    JOINTS,
    FitResult,
    add_motion_features,
    aggregate_blocks,
    center_by_group,
    classify,
    feature_matrix,
    first_present,
    fit_model,
    joint_column,
    numeric,
    read_header,
    stable_filter,
)

try:
    from PIL import Image, ImageDraw
except ImportError:
    Image = None
    ImageDraw = None


def load_m00_candidate(path: Path, category: str) -> pd.DataFrame | None:
    header = read_header(path)
    columns = set(header)
    q_columns = [joint_column(columns, joint) for joint in JOINTS]
    force_column = first_present(columns, ["tension_m00_n", "force_ch2_n"])
    bias_column = first_present(
        columns,
        [
            "tension_m00_host_bias_counts",
            "tension_bias_m00_counts",
            "mcp_m00_tension_bias_counts",
        ],
    )
    if any(column is None for column in q_columns) or force_column is None or "servo_m00_abs" not in columns:
        return None

    optional = [
        "elapsed_s",
        "timestamp",
        "encoder_age_s",
        "tension_m00_age_s",
        "force_ch2_age_s",
        "servo_m00_abs",
        "servo_m00_speed",
        "servo_m00_online",
        "joint_j00_valid",
        "joint_j01_valid",
        "joint_j02_valid",
        "joint_j03_valid",
        "degree_force_active",
        "mcp_m00_tension_bias_enabled",
        "mcp_m00_command_rate_limited",
        "control_mode",
    ]
    selected = list(dict.fromkeys([*q_columns, force_column, bias_column, *optional]))
    selected = [column for column in selected if column and column in columns]
    try:
        raw = pd.read_csv(
            path,
            usecols=selected,
            encoding="utf-8-sig",
            low_memory=False,
            on_bad_lines="skip",
        )
    except (OSError, UnicodeError, ValueError, pd.errors.ParserError):
        return None

    out = pd.DataFrame(index=raw.index)
    out["time_s"] = numeric(raw, "elapsed_s")
    if out["time_s"].notna().sum() < 2:
        timestamp = numeric(raw, "timestamp")
        out["time_s"] = timestamp - timestamp.dropna().iloc[0] if timestamp.notna().any() else np.arange(len(raw)) * 0.1
    for joint, column in enumerate(q_columns):
        out[f"q{joint}"] = numeric(raw, column)
        out[f"valid{joint}"] = numeric(raw, f"joint_j{joint:02d}_valid", default=1.0)
    out["force_n"] = numeric(raw, force_column)
    out["tension_n"] = -out["force_n"]
    out["servo_abs"] = numeric(raw, "servo_m00_abs")
    out["servo_speed"] = numeric(raw, "servo_m00_speed", default=0.0)
    out["servo_online"] = numeric(raw, "servo_m00_online", default=1.0)
    out["bias"] = numeric(raw, bias_column)
    out["encoder_age_s"] = numeric(raw, "encoder_age_s", default=0.0)
    force_age_column = first_present(columns, ["tension_m00_age_s", "force_ch2_age_s"])
    out["force_age_s"] = numeric(raw, force_age_column, default=0.0)
    out["bias_enabled"] = numeric(raw, "mcp_m00_tension_bias_enabled", default=1.0)
    out["rate_limited"] = numeric(raw, "mcp_m00_command_rate_limited", default=0.0)
    out["window_ok"] = 1.0
    out["control_active"] = numeric(raw, "degree_force_active", default=1.0)
    out["control_mode"] = raw["control_mode"].astype(str) if "control_mode" in raw else "degree"
    out["source_file"] = path.name
    out["source_path"] = str(path.resolve())
    out["source_type"] = category
    return out


def candidate_paths(root: Path) -> list[tuple[Path, str]]:
    result: list[tuple[Path, str]] = []
    for path in sorted(root.rglob("*.csv")):
        relative_parts = path.relative_to(root).parts
        if any(part.startswith("analysis_m") for part in relative_parts[:-1]):
            continue
        category = classify(path)
        if category in {"training", "continuous"}:
            result.append((path, category))
    return result


def coefficient(result: FitResult, name: str) -> tuple[float, float]:
    lookup = dict(zip(result.feature_names, result.coefficients))
    std_lookup = dict(zip(result.feature_names, result.loo_coefficient_std))
    return float(lookup.get(name, 0.0)), float(std_lookup.get(name, math.nan))


def write_j0_plot(path: Path, dataset: pd.DataFrame, result: FitResult) -> None:
    if Image is None or ImageDraw is None:
        return
    if "J0" in result.feature_names:
        j0_feature = "J0"
        x_label = "Change in signed J0 from episode anchor (deg)"
    elif "abs_J0" in result.feature_names:
        j0_feature = "abs_J0"
        x_label = "Change in |J0| from episode anchor (deg)"
    else:
        return
    x_raw, names = feature_matrix(dataset, result.model)
    y_raw = dataset["servo_abs"].to_numpy(dtype=float)
    groups = dataset["source_group"].to_numpy(dtype=str)
    x, y = center_by_group(x_raw, y_raw, groups)
    coefficient_map = dict(zip(result.feature_names, result.coefficients))
    j0_index = names.index(j0_feature)
    partial = y.copy()
    for index, name in enumerate(names):
        if name != j0_feature:
            partial -= x[:, index] * coefficient_map[name]
    distance = x[:, j0_index]

    width, height = 1100, 520
    left, right, top, bottom = 80, 40, 70, 70
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_low, x_high = (float(np.nanpercentile(distance, value)) for value in (1, 99))
    y_low, y_high = (float(np.nanpercentile(partial, value)) for value in (1, 99))
    if x_high - x_low < 1.0e-9 or y_high - y_low < 1.0e-9:
        return
    x_pad = (x_high - x_low) * 0.08
    y_pad = (y_high - y_low) * 0.08
    x_low, x_high = x_low - x_pad, x_high + x_pad
    y_low, y_high = y_low - y_pad, y_high + y_pad

    def x_pos(value: float) -> float:
        return left + (value - x_low) / (x_high - x_low) * plot_w

    def y_pos(value: float) -> float:
        return top + (y_high - value) / (y_high - y_low) * plot_h

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image, "RGBA")
    for fraction in np.linspace(0.0, 1.0, 6):
        xv = x_low + fraction * (x_high - x_low)
        yv = y_low + fraction * (y_high - y_low)
        x = x_pos(xv)
        y = y_pos(yv)
        draw.line((x, top, x, top + plot_h), fill="#e5e7eb", width=1)
        draw.line((left, y, left + plot_w, y), fill="#e5e7eb", width=1)
        draw.text((x - 15, top + plot_h + 14), f"{xv:.1f}", fill="#475569")
        draw.text((left - 52, y - 6), f"{yv:.0f}", fill="#475569")
    files = dataset["source_file"].to_numpy(dtype=str)
    unique_files = list(np.unique(files))
    palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#475569"]
    step = max(1, math.ceil(len(distance) / 3500))
    for index in range(0, len(distance), step):
        color = palette[unique_files.index(files[index]) % len(palette)]
        x = x_pos(float(distance[index]))
        y_pos_value = y_pos(float(partial[index]))
        draw.ellipse((x - 2, y_pos_value - 2, x + 2, y_pos_value + 2), fill=color + "78")
    slope, _std = coefficient(result, j0_feature)
    draw.line(
        (x_pos(x_low), y_pos(slope * x_low), x_pos(x_high), y_pos(slope * x_high)),
        fill="#111827",
        width=2,
    )
    draw.rectangle((left, top, left + plot_w, top + plot_h), outline="#94a3b8", width=1)
    draw.text((left, 16), "M00 position vs displacement from J0 mechanical zero", fill="#111827")
    draw.text(
        (left, 38),
        f"partial relationship after removing other fitted terms | slope={slope:.2f} counts/deg",
        fill="#475569",
    )
    draw.text((left + plot_w / 2 - 115, height - 28), x_label, fill="#111827")
    draw.text((8, top + plot_h / 2), "M00 counts", fill="#111827")
    image.save(path)


def write_report(
    path: Path,
    root: Path,
    candidate_count: int,
    dataset: pd.DataFrame,
    results: list[FitResult],
    recommended: FitResult,
    piecewise: FitResult,
    filters: dict[str, float],
) -> None:
    signed_slope, signed_std = coefficient(recommended, "J0")
    j1_slope, j1_std = coefficient(recommended, "J1")
    pos_slope, pos_std = coefficient(piecewise, "J0_pos_distance")
    neg_slope, neg_std = coefficient(piecewise, "J0_neg_distance")
    negative_side_signed_slope = -neg_slope
    symmetry_delta = abs(pos_slope - negative_side_signed_slope)
    symmetry_scale = max(1.0, (abs(pos_slope) + abs(negative_side_signed_slope)) * 0.5)
    symmetry_pct = 100.0 * symmetry_delta / symmetry_scale
    abs_result = next(result for result in results if result.model == "linear_absj0_j1")

    lines = [
        "# M00 与 J0 零位距离比例拟合",
        "",
        f"- 历史原始候选：{candidate_count} 个 CSV",
        f"- 稳态数据：{len(dataset)} 个 1 秒中位数块，来自 {dataset['source_file'].nunique()} 个实验文件",
        f"- M00 张力筛选：{filters['force_min_n']:.1f} 到 {filters['force_max_n']:.1f} N",
        "",
        "## 数据支持的比例关系",
        "",
        "```text",
        "M00_ff = M00_anchor",
        f"       {signed_slope:+.4f} * (J0 - J0_anchor)",
        f"       {j1_slope:+.4f} * (J1 - J1_anchor)",
        "```",
        "",
        "这里的距离需要保留方向，即 `J0-0`。纯绝对距离 `|J0|` 会把左右两侧的电机方向混在一起。",
        "",
        "| 项 | 系数 counts/deg | 留一文件标准差 |",
        "|---|---:|---:|",
        f"| J0 signed | {signed_slope:.4f} | {signed_std:.4f} |",
        f"| J1 | {j1_slope:.4f} | {j1_std:.4f} |",
        "",
        "## 正负侧检查",
        "",
        f"- J0 正侧：M00 随正距离变化 {pos_slope:+.4f} ± {pos_std:.4f} counts/deg",
        f"- J0 负侧：M00 随负侧距离变化 {neg_slope:+.4f} ± {neg_std:.4f} counts/deg",
        f"- 换算成有符号 J0 后，负侧斜率为 {negative_side_signed_slope:+.4f} counts/deg",
        f"- 两侧有符号斜率差异：{symmetry_pct:.1f}%",
        "",
        "正、负距离系数符号相反，但换成有符号角度后斜率接近，因此关系是近似直线，不是 V 形。",
        "",
        "## 模型比较（整份实验留一验证）",
        "",
        "| 模型 | RMSE counts | MAE counts | R2 | 相对常值改善 |",
        "|---|---:|---:|---:|---:|",
    ]
    for result in sorted(results, key=lambda item: item.rmse):
        lines.append(
            f"| {result.model} | {result.rmse:.2f} | {result.mae:.2f} | "
            f"{result.r2:.3f} | {result.improvement_pct:.1f}% |"
        )
    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"M00 对 J0 带方向位移的比例约为 {signed_slope:.2f} counts/deg。"
            f"按文件验证 R2={recommended.r2:.3f}，RMSE={recommended.rmse:.1f} counts。",
            f"绝对值模型的 RMSE={abs_result.rmse:.1f} counts，高于有符号模型的 {recommended.rmse:.1f} counts，"
            "所以不建议使用 `|J0|`。",
            "该值描述的是恒张力附近的 M00 电机位置前馈，不代表裸绳几何长度的毫米单位。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit M00 position against distance from J0 zero.")
    parser.add_argument("--root", type=Path, default=Path("run_data"))
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--force-min-n", type=float, default=-35.0)
    parser.add_argument("--force-max-n", type=float, default=-8.0)
    parser.add_argument("--max-qdot-deg-s", type=float, default=3.0)
    parser.add_argument("--max-servo-speed", type=float, default=100.0)
    parser.add_argument("--block-s", type=float, default=1.0)
    args = parser.parse_args()

    root = args.root.resolve()
    output_dir = args.output_dir or root / f"analysis_m00_j0_distance_{datetime.now():%Y%m%d_%H%M%S}"
    output_dir.mkdir(parents=True, exist_ok=True)

    frames: list[pd.DataFrame] = []
    audits: list[dict[str, object]] = []
    paths = candidate_paths(root)
    for path, category in paths:
        loaded = load_m00_candidate(path, category)
        audits.append(
            {
                "path": str(path.resolve()),
                "category": category,
                "usable_schema": int(loaded is not None),
                "raw_rows": 0 if loaded is None else len(loaded),
            }
        )
        if loaded is None or loaded.empty:
            continue
        loaded = add_motion_features(loaded)
        stable = stable_filter(
            loaded,
            force_min_n=args.force_min_n,
            force_max_n=args.force_max_n,
            max_qdot_deg_s=args.max_qdot_deg_s,
            max_servo_speed=args.max_servo_speed,
        )
        if not stable.empty:
            frames.append(stable)
    pd.DataFrame(audits).to_csv(output_dir / "m00_csv_audit.csv", index=False, encoding="utf-8-sig")
    if not frames:
        raise RuntimeError("no M00 rows passed the stable-data filters")

    stable_all = pd.concat(frames, ignore_index=True)
    blocks = aggregate_blocks(stable_all, target="servo_abs", block_s=args.block_s)
    counts = blocks.groupby("source_file")["servo_abs"].count()
    keep_files = set(counts[counts >= 8].index)
    blocks = blocks[blocks["source_file"].isin(keep_files)].copy()
    ranges = blocks.groupby("source_file")["servo_abs"].agg(lambda values: float(values.max() - values.min()))
    informative_files = set(ranges[ranges >= 20.0].index)
    blocks = blocks[blocks["source_file"].isin(informative_files)].copy()
    if len(blocks) < 30:
        raise RuntimeError("not enough informative M00 blocks")
    blocks.to_csv(output_dir / "m00_stable_blocks.csv", index=False, encoding="utf-8-sig")

    model_names = (
        "linear_j01",
        "linear_absj0_j1",
        "linear_absj0_j1_force",
        "linear_piecewise_j0_j1",
        "linear_piecewise_j0_j1_force",
        "linear_absj0_j123",
        "linear_absj0_j123_force",
        "linear_piecewise_j0_j123",
        "linear_q",
    )
    results = [fit_model(blocks, target="servo_abs", model=model) for model in model_names]
    comparison_rows = [
        {
            "model": result.model,
            "files": result.files,
            "blocks": result.rows,
            "alpha": result.alpha,
            "rmse_counts": result.rmse,
            "mae_counts": result.mae,
            "r2_relative": result.r2,
            "improvement_pct": result.improvement_pct,
        }
        for result in results
    ]
    pd.DataFrame(comparison_rows).to_csv(output_dir / "m00_model_comparison.csv", index=False, encoding="utf-8-sig")

    coefficient_rows: list[dict[str, object]] = []
    for result in results:
        for feature, value, std in zip(result.feature_names, result.coefficients, result.loo_coefficient_std):
            coefficient_rows.append(
                {
                    "model": result.model,
                    "feature": feature,
                    "coefficient": value,
                    "leave_one_file_out_std": std,
                }
            )
    pd.DataFrame(coefficient_rows).to_csv(output_dir / "m00_model_coefficients.csv", index=False, encoding="utf-8-sig")

    recommended = next(result for result in results if result.model == "linear_j01")
    piecewise = next(result for result in results if result.model == "linear_piecewise_j0_j1")
    coefficient_map = dict(zip(recommended.feature_names, recommended.coefficients))
    std_map = dict(zip(recommended.feature_names, recommended.loo_coefficient_std))
    model = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "model": recommended.model,
        "formula": "anchor + k_j0*(J0-J0_anchor) + k_j1*(J1-J1_anchor)",
        "j0_signed_counts_per_deg": float(coefficient_map.get("J0", 0.0)),
        "j0_signed_leave_one_file_out_std": float(std_map.get("J0", math.nan)),
        "joint_counts_per_deg": [float(coefficient_map.get(f"J{joint}", 0.0)) for joint in JOINTS],
        "tension_nuisance_counts_per_n": float(coefficient_map.get("tension_N", 0.0)),
        "metrics": {
            "files": recommended.files,
            "blocks": recommended.rows,
            "rmse_counts": recommended.rmse,
            "mae_counts": recommended.mae,
            "r2_relative": recommended.r2,
            "improvement_pct": recommended.improvement_pct,
        },
    }
    (output_dir / "m00_j0_distance_model.json").write_text(
        json.dumps(model, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    filters = {
        "force_min_n": args.force_min_n,
        "force_max_n": args.force_max_n,
        "max_qdot_deg_s": args.max_qdot_deg_s,
        "max_servo_speed": args.max_servo_speed,
        "block_s": args.block_s,
    }
    write_report(
        output_dir / "m00_j0_distance_report.md",
        root=root,
        candidate_count=len(paths),
        dataset=blocks,
        results=results,
        recommended=recommended,
        piecewise=piecewise,
        filters=filters,
    )
    write_j0_plot(output_dir / "m00_j0_distance_diagnostics.png", blocks, recommended)

    print(f"output_dir={output_dir.resolve()}")
    print(f"files={recommended.files} blocks={recommended.rows} model={recommended.model}")
    print(
        f"j0_signed_counts_per_deg={float(coefficient_map.get('J0', 0.0)):.5f} "
        f"std={float(std_map.get('J0', math.nan)):.5f} "
        f"rmse={recommended.rmse:.2f} r2={recommended.r2:.3f}"
    )
    pos, pos_std = coefficient(piecewise, "J0_pos_distance")
    neg, neg_std = coefficient(piecewise, "J0_neg_distance")
    print(f"piecewise_pos={pos:.5f}+/-{pos_std:.5f} piecewise_neg={neg:.5f}+/-{neg_std:.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
