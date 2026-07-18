"""Fit an M04 return-tendon feedforward model from historical CSV logs.

The primary model is relative to a runtime anchor:

    m04_ff = anchor_counts + sum(a_j * (q_j - anchor_q_j))

Centering every experiment independently removes servo-zero and preload-offset
changes between runs. The force term is fitted as a nuisance variable so the
joint coefficients describe geometry at a constant tension.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

try:
    from PIL import Image, ImageDraw
except ImportError:  # SVG output remains available without Pillow.
    Image = None
    ImageDraw = None


RAW_PATTERNS = {
    "training": re.compile(r"^training_collect_\d{8}_\d{6}(?:_KEEP_dl_feedforward_dataset)?\.csv$"),
    "continuous": re.compile(r"^continuous_record_\d{8}_\d{6}\.csv$"),
    "linearity": re.compile(r"^linearity_\d{8}_\d{6}\.csv$"),
    "legacy_mcp": re.compile(r"^mcp_control_\d{8}_\d{6}\.csv$"),
}
JOINTS = tuple(range(4))
EPS = 1.0e-9
RIDGE_ALPHAS = (0.0, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0)


@dataclass
class FileAudit:
    path: Path
    category: str
    rows: int
    columns: int
    has_q: bool
    has_force: bool
    has_servo: bool
    has_bias: bool
    status: str


@dataclass
class FitResult:
    target: str
    model: str
    alpha: float
    files: int
    rows: int
    rmse: float
    mae: float
    r2: float
    baseline_rmse: float
    improvement_pct: float
    feature_names: list[str]
    coefficients: np.ndarray
    loo_coefficient_std: np.ndarray
    predictions: np.ndarray
    actual: np.ndarray
    source_files: np.ndarray


def classify(path: Path) -> str:
    for category, pattern in RAW_PATTERNS.items():
        if pattern.match(path.name):
            return category
    return "derived_or_other"


def read_header(path: Path) -> list[str]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            return next(csv.reader(stream), [])
    except (OSError, UnicodeError):
        return []


def count_rows(path: Path) -> int:
    try:
        with path.open("r", encoding="utf-8-sig", errors="replace") as stream:
            return max(0, sum(1 for _ in stream) - 1)
    except OSError:
        return 0


def first_present(columns: set[str], candidates: list[str]) -> str | None:
    return next((name for name in candidates if name in columns), None)


def joint_column(columns: set[str], joint: int) -> str | None:
    prefix = f"joint_j{joint:02d}"
    return first_present(
        columns,
        [f"{prefix}_deg_rel", f"{prefix}_deg", f"{prefix}_deg_abs"],
    )


def audit_files(root: Path) -> list[FileAudit]:
    audits: list[FileAudit] = []
    for path in sorted(root.rglob("*.csv")):
        relative_parts = path.relative_to(root).parts
        if any(part.startswith("analysis_m04_feedforward_") for part in relative_parts[:-1]):
            continue
        header = read_header(path)
        columns = set(header)
        category = classify(path)
        has_q = all(joint_column(columns, joint) is not None for joint in JOINTS)
        has_force = first_present(columns, ["tension_m04_n", "force_ch5_n"]) is not None
        has_servo = "servo_m04_abs" in columns
        has_bias = first_present(
            columns,
            [
                "tension_m04_host_bias_counts",
                "tension_bias_m04_counts",
                "mcp_m04_tension_bias_counts",
            ],
        ) is not None
        if category not in {"training", "continuous"}:
            status = "audited_not_primary_raw"
        elif not has_q:
            status = "missing_joint_angles"
        elif not has_force:
            status = "missing_m04_force"
        elif not (has_servo or has_bias):
            status = "missing_m04_target"
        else:
            status = "candidate"
        audits.append(
            FileAudit(
                path=path,
                category=category,
                rows=count_rows(path),
                columns=len(header),
                has_q=has_q,
                has_force=has_force,
                has_servo=has_servo,
                has_bias=has_bias,
                status=status,
            )
        )
    return audits


def numeric(df: pd.DataFrame, column: str | None, default: float = math.nan) -> pd.Series:
    if column is None or column not in df.columns:
        return pd.Series(np.full(len(df), default), index=df.index, dtype=float)
    return pd.to_numeric(df[column], errors="coerce")


def load_candidate(path: Path, category: str) -> pd.DataFrame | None:
    header = read_header(path)
    columns = set(header)
    q_columns = [joint_column(columns, joint) for joint in JOINTS]
    force_column = first_present(columns, ["tension_m04_n", "force_ch5_n"])
    bias_column = first_present(
        columns,
        [
            "tension_m04_host_bias_counts",
            "tension_bias_m04_counts",
            "mcp_m04_tension_bias_counts",
        ],
    )
    if any(column is None for column in q_columns) or force_column is None:
        return None

    optional = [
        "elapsed_s",
        "timestamp",
        "encoder_age_s",
        "tension_m04_age_s",
        "force_ch5_age_s",
        "servo_m04_abs",
        "servo_m04_speed",
        "servo_m04_online",
        "joint_j00_valid",
        "joint_j01_valid",
        "joint_j02_valid",
        "joint_j03_valid",
        "degree_force_active",
        "degree_force_phase",
        "degree_force_tension_window_ok",
        "tension_window_ok",
        "mcp_m04_tension_bias_enabled",
        "mcp_m04_command_rate_limited",
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
        valid_column = f"joint_j{joint:02d}_valid"
        out[f"valid{joint}"] = numeric(raw, valid_column, default=1.0)
    out["force_n"] = numeric(raw, force_column)
    out["tension_n"] = -out["force_n"]
    out["servo_abs"] = numeric(raw, "servo_m04_abs")
    out["servo_speed"] = numeric(raw, "servo_m04_speed", default=0.0)
    out["servo_online"] = numeric(raw, "servo_m04_online", default=1.0)
    out["bias"] = numeric(raw, bias_column)
    out["encoder_age_s"] = numeric(raw, "encoder_age_s", default=0.0)
    force_age_column = first_present(columns, ["tension_m04_age_s", "force_ch5_age_s"])
    out["force_age_s"] = numeric(raw, force_age_column, default=0.0)
    out["bias_enabled"] = numeric(raw, "mcp_m04_tension_bias_enabled", default=1.0)
    out["rate_limited"] = numeric(raw, "mcp_m04_command_rate_limited", default=0.0)
    out["window_ok"] = numeric(
        raw,
        first_present(columns, ["degree_force_tension_window_ok", "tension_window_ok"]),
        default=1.0,
    )
    out["control_active"] = numeric(raw, "degree_force_active", default=1.0)
    out["control_mode"] = raw["control_mode"].astype(str) if "control_mode" in raw else "degree"
    out["source_file"] = path.name
    out["source_path"] = str(path.resolve())
    out["source_type"] = category
    return out


def add_motion_features(df: pd.DataFrame) -> pd.DataFrame:
    df = df.sort_values("time_s").copy()
    time_values = df["time_s"].to_numpy(dtype=float)
    max_qdot = np.zeros(len(df), dtype=float)
    if len(df) > 1:
        dt = np.diff(time_values)
        finite_positive = dt[np.isfinite(dt) & (dt > 1.0e-4)]
        fallback_dt = float(np.median(finite_positive)) if finite_positive.size else 0.1
        dt[~np.isfinite(dt) | (dt <= 1.0e-4)] = fallback_dt
        for joint in JOINTS:
            values = df[f"q{joint}"].to_numpy(dtype=float)
            velocity = np.zeros(len(df), dtype=float)
            velocity[1:] = np.abs(np.diff(values) / dt)
            velocity[:-1] = np.maximum(velocity[:-1], velocity[1:])
            max_qdot = np.maximum(max_qdot, velocity)
    df["max_qdot_deg_s"] = max_qdot
    gap = np.zeros(len(df), dtype=bool)
    active_rising = np.zeros(len(df), dtype=bool)
    bias_reset = np.zeros(len(df), dtype=bool)
    if len(df) > 1:
        dt = np.diff(time_values)
        finite_positive = dt[np.isfinite(dt) & (dt > 1.0e-4)]
        median_dt = float(np.median(finite_positive)) if finite_positive.size else 0.1
        gap[1:] = dt > max(5.0, median_dt * 8.0)
        active = df["control_active"].fillna(0.0).to_numpy(dtype=float) > 0.5
        active_rising[1:] = active[1:] & ~active[:-1]
        bias = df["bias"].to_numpy(dtype=float)
        finite_pair = np.isfinite(bias[1:]) & np.isfinite(bias[:-1])
        bias_reset[1:] = finite_pair & (np.abs(np.diff(bias)) > 500.0)
    episode = np.cumsum(gap | active_rising | bias_reset)
    df["source_group"] = df["source_file"].astype(str) + "#" + pd.Series(episode, index=df.index).astype(str)
    return df


def stable_filter(
    df: pd.DataFrame,
    force_min_n: float,
    force_max_n: float,
    max_qdot_deg_s: float,
    max_servo_speed: float,
) -> pd.DataFrame:
    mask = np.ones(len(df), dtype=bool)
    required = [*[f"q{joint}" for joint in JOINTS], "force_n", "time_s"]
    for column in required:
        mask &= np.isfinite(df[column].to_numpy(dtype=float))
    for joint in JOINTS:
        mask &= df[f"valid{joint}"].to_numpy(dtype=float) > 0.5
    mask &= df["servo_online"].to_numpy(dtype=float) > 0.5
    mask &= df["encoder_age_s"].to_numpy(dtype=float) <= 0.5
    mask &= df["force_age_s"].to_numpy(dtype=float) <= 0.5
    mask &= df["force_n"].to_numpy(dtype=float) >= force_min_n
    mask &= df["force_n"].to_numpy(dtype=float) <= force_max_n
    mask &= df["max_qdot_deg_s"].to_numpy(dtype=float) <= max_qdot_deg_s
    mask &= np.abs(df["servo_speed"].to_numpy(dtype=float)) <= max_servo_speed
    mask &= df["rate_limited"].to_numpy(dtype=float) < 0.5
    mode = df["control_mode"].astype(str).str.lower()
    mask &= mode.isin(["degree", "joint", "nan", ""]).to_numpy()
    return df.loc[mask].copy()


def aggregate_blocks(df: pd.DataFrame, target: str, block_s: float) -> pd.DataFrame:
    target_column = "bias" if target == "bias" else "servo_abs"
    usable = df[np.isfinite(df[target_column].to_numpy(dtype=float))].copy()
    if target == "bias":
        usable = usable[usable["bias_enabled"] > 0.5]
    if usable.empty:
        return usable
    usable["block"] = np.floor(usable["time_s"] / block_s).astype(int)
    value_columns = [
        "time_s",
        *[f"q{joint}" for joint in JOINTS],
        "force_n",
        "tension_n",
        "servo_abs",
        "servo_speed",
        "bias",
        "max_qdot_deg_s",
    ]
    grouped = usable.groupby(
        ["source_file", "source_path", "source_type", "source_group", "block"],
        as_index=False,
    )
    result = grouped[value_columns].median(numeric_only=True)
    counts = grouped.size().rename(columns={"size": "raw_rows"})
    return result.merge(
        counts,
        on=["source_file", "source_path", "source_type", "source_group", "block"],
        how="left",
    )


def feature_matrix(df: pd.DataFrame, model: str) -> tuple[np.ndarray, list[str]]:
    q = [df[f"q{joint}"].to_numpy(dtype=float) for joint in JOINTS]
    values: list[np.ndarray] | None = None
    names: list[str] | None = None
    if model.startswith("linear_absj0_j123"):
        values = [np.abs(q[0]), q[1], q[2], q[3]]
        names = ["abs_J0", "J1", "J2", "J3"]
    elif model.startswith("linear_absj0_j1"):
        values = [np.abs(q[0]), q[1]]
        names = ["abs_J0", "J1"]
    elif model.startswith("linear_piecewise_j0_j123"):
        values = [np.maximum(q[0], 0.0), np.maximum(-q[0], 0.0), q[1], q[2], q[3]]
        names = ["J0_pos_distance", "J0_neg_distance", "J1", "J2", "J3"]
    elif model.startswith("linear_piecewise_j0_j1"):
        values = [np.maximum(q[0], 0.0), np.maximum(-q[0], 0.0), q[1]]
        names = ["J0_pos_distance", "J0_neg_distance", "J1"]
    elif model == "linear_j01":
        feature_joints = (0, 1)
    elif model == "linear_j23":
        feature_joints = (2, 3)
    elif model == "linear_j123":
        feature_joints = (1, 2, 3)
    else:
        feature_joints = JOINTS
    if values is None or names is None:
        values = [q[joint] for joint in feature_joints]
        names = [f"J{joint}" for joint in feature_joints]
    if model.startswith("quadratic"):
        for joint in JOINTS:
            values.append(q[joint] * q[joint])
            names.append(f"J{joint}^2")
        for left in JOINTS:
            for right in range(left + 1, len(JOINTS)):
                values.append(q[left] * q[right])
                names.append(f"J{left}*J{right}")
    if model.endswith("force"):
        values.append(df["tension_n"].to_numpy(dtype=float))
        names.append("tension_N")
    return np.column_stack(values), names


def center_by_group(x: np.ndarray, y: np.ndarray, groups: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x_centered = np.empty_like(x, dtype=float)
    y_centered = np.empty_like(y, dtype=float)
    for source in np.unique(groups):
        mask = groups == source
        x_centered[mask] = x[mask] - np.median(x[mask], axis=0)
        y_centered[mask] = y[mask] - np.median(y[mask])
    return x_centered, y_centered


def file_equal_weights(files: np.ndarray) -> np.ndarray:
    weights = np.zeros(len(files), dtype=float)
    unique = np.unique(files)
    for source in unique:
        mask = files == source
        weights[mask] = 1.0 / max(1, int(np.sum(mask)))
    weights *= len(files) / max(EPS, float(np.sum(weights)))
    return weights


def ridge_fit_raw(
    x: np.ndarray,
    y: np.ndarray,
    files: np.ndarray,
    alpha: float,
) -> np.ndarray:
    weights = file_equal_weights(files)
    mean = np.average(x, axis=0, weights=weights)
    variance = np.average((x - mean) ** 2, axis=0, weights=weights)
    std = np.sqrt(variance)
    std[std < EPS] = 1.0
    scaled = (x - mean) / std
    sqrt_w = np.sqrt(weights)
    xw = scaled * sqrt_w[:, None]
    yw = y * sqrt_w
    regularizer = np.eye(xw.shape[1]) * alpha
    beta_scaled = np.linalg.solve(xw.T @ xw + regularizer, xw.T @ yw)
    return beta_scaled / std


def metrics(y: np.ndarray, prediction: np.ndarray) -> tuple[float, float, float]:
    residual = prediction - y
    rmse = float(np.sqrt(np.mean(residual * residual)))
    mae = float(np.mean(np.abs(residual)))
    total = float(np.sum((y - np.mean(y)) ** 2))
    r2 = math.nan if total <= EPS else 1.0 - float(np.sum(residual * residual)) / total
    return rmse, mae, r2


def choose_alpha_lofo(x: np.ndarray, y: np.ndarray, files: np.ndarray) -> float:
    unique = np.unique(files)
    if len(unique) < 2:
        return 1.0
    best_alpha = RIDGE_ALPHAS[0]
    best_score = math.inf
    for alpha in RIDGE_ALPHAS:
        scores: list[float] = []
        for held_out in unique:
            train = files != held_out
            test = ~train
            if np.sum(train) <= x.shape[1] or np.sum(test) < 2:
                continue
            beta = ridge_fit_raw(x[train], y[train], files[train], alpha)
            scores.append(metrics(y[test], x[test] @ beta)[0])
        score = float(np.mean(scores)) if scores else math.inf
        if score < best_score:
            best_score = score
            best_alpha = alpha
    return best_alpha


def fit_model(df: pd.DataFrame, target: str, model: str) -> FitResult:
    target_column = "bias" if target == "bias" else "servo_abs"
    x_raw, feature_names = feature_matrix(df, model)
    y_raw = df[target_column].to_numpy(dtype=float)
    files = df["source_file"].to_numpy(dtype=str)
    groups = df["source_group"].to_numpy(dtype=str)
    x, y = center_by_group(x_raw, y_raw, groups)
    alpha = choose_alpha_lofo(x, y, files)
    coefficients = ridge_fit_raw(x, y, files, alpha)
    unique_files = np.unique(files)
    prediction = np.full(len(y), math.nan, dtype=float)
    if len(unique_files) >= 2:
        for held_out in unique_files:
            train = files != held_out
            test = ~train
            if np.sum(train) <= x.shape[1]:
                continue
            fold_coefficients = ridge_fit_raw(x[train], y[train], files[train], alpha)
            prediction[test] = x[test] @ fold_coefficients
    if not np.all(np.isfinite(prediction)):
        prediction = x @ coefficients
    rmse, mae, r2 = metrics(y, prediction)
    baseline_rmse = float(np.sqrt(np.mean(y * y)))
    improvement = 100.0 * (baseline_rmse - rmse) / baseline_rmse if baseline_rmse > EPS else math.nan

    leave_one_out_coefficients: list[np.ndarray] = []
    for held_out in unique_files:
        train = files != held_out
        if np.sum(train) > x.shape[1]:
            leave_one_out_coefficients.append(ridge_fit_raw(x[train], y[train], files[train], alpha))
    coefficient_std = (
        np.std(np.vstack(leave_one_out_coefficients), axis=0)
        if leave_one_out_coefficients
        else np.full(len(feature_names), math.nan)
    )
    return FitResult(
        target=target,
        model=model,
        alpha=alpha,
        files=len(np.unique(files)),
        rows=len(df),
        rmse=rmse,
        mae=mae,
        r2=r2,
        baseline_rmse=baseline_rmse,
        improvement_pct=improvement,
        feature_names=feature_names,
        coefficients=coefficients,
        loo_coefficient_std=coefficient_std,
        predictions=prediction,
        actual=y,
        source_files=files,
    )


def validation_by_file(result: FitResult) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for source in np.unique(result.source_files):
        mask = result.source_files == source
        rmse, mae, r2 = metrics(result.actual[mask], result.predictions[mask])
        rows.append(
            {
                "target": result.target,
                "model": result.model,
                "source_file": source,
                "rows": int(np.sum(mask)),
                "rmse_counts": rmse,
                "mae_counts": mae,
                "r2_relative": r2,
            }
        )
    return rows


def svg_scatter(path: Path, result: FitResult) -> None:
    width, height = 1100, 520
    left, right, top, bottom = 80, 40, 55, 70
    plot_w = width - left - right
    plot_h = height - top - bottom
    values = np.concatenate([result.actual, result.predictions])
    low = float(np.nanpercentile(values, 1))
    high = float(np.nanpercentile(values, 99))
    if not math.isfinite(low) or not math.isfinite(high) or high - low < EPS:
        low, high = -1.0, 1.0
    padding = (high - low) * 0.08
    low -= padding
    high += padding

    def x_pos(value: float) -> float:
        return left + (value - low) / (high - low) * plot_w

    def y_pos(value: float) -> float:
        return top + (high - value) / (high - low) * plot_h

    unique_files = list(np.unique(result.source_files))
    palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#475569"]
    points: list[str] = []
    max_points = 3500
    step = max(1, math.ceil(len(result.actual) / max_points))
    for index in range(0, len(result.actual), step):
        source_index = unique_files.index(result.source_files[index])
        color = palette[source_index % len(palette)]
        points.append(
            f'<circle cx="{x_pos(float(result.actual[index])):.2f}" '
            f'cy="{y_pos(float(result.predictions[index])):.2f}" r="2.2" '
            f'fill="{color}" fill-opacity="0.48" />'
        )
    ticks: list[str] = []
    for fraction in np.linspace(0.0, 1.0, 6):
        value = low + fraction * (high - low)
        x = x_pos(value)
        y = y_pos(value)
        ticks.extend(
            [
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#e5e7eb" />',
                f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#e5e7eb" />',
                f'<text x="{x:.2f}" y="{top + plot_h + 28}" text-anchor="middle" font-size="13" fill="#475569">{value:.0f}</text>',
                f'<text x="{left - 12}" y="{y + 4:.2f}" text-anchor="end" font-size="13" fill="#475569">{value:.0f}</text>',
            ]
        )
    title = f"M04 {result.target} relative feedforward: measured vs predicted"
    subtitle = f"{result.model}, files={result.files}, blocks={result.rows}, RMSE={result.rmse:.1f} counts, R2={result.r2:.3f}"
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#ffffff" />
<text x="{left}" y="28" font-family="Arial, sans-serif" font-size="21" font-weight="700" fill="#111827">{title}</text>
<text x="{left}" y="48" font-family="Arial, sans-serif" font-size="13" fill="#475569">{subtitle}</text>
{''.join(ticks)}
<line x1="{x_pos(low):.2f}" y1="{y_pos(low):.2f}" x2="{x_pos(high):.2f}" y2="{y_pos(high):.2f}" stroke="#111827" stroke-width="1.5" />
{''.join(points)}
<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#94a3b8" />
<text x="{left + plot_w / 2:.2f}" y="{height - 18}" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#111827">Measured relative command (counts)</text>
<text x="20" y="{top + plot_h / 2:.2f}" text-anchor="middle" transform="rotate(-90 20 {top + plot_h / 2:.2f})" font-family="Arial, sans-serif" font-size="15" fill="#111827">Predicted relative command (counts)</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def png_scatter(path: Path, result: FitResult) -> None:
    if Image is None or ImageDraw is None:
        return
    width, height = 1100, 520
    left, right, top, bottom = 80, 40, 70, 70
    plot_w = width - left - right
    plot_h = height - top - bottom
    values = np.concatenate([result.actual, result.predictions])
    low = float(np.nanpercentile(values, 1))
    high = float(np.nanpercentile(values, 99))
    if not math.isfinite(low) or not math.isfinite(high) or high - low < EPS:
        low, high = -1.0, 1.0
    padding = (high - low) * 0.08
    low -= padding
    high += padding

    def x_pos(value: float) -> float:
        return left + (value - low) / (high - low) * plot_w

    def y_pos(value: float) -> float:
        return top + (high - value) / (high - low) * plot_h

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image, "RGBA")
    for fraction in np.linspace(0.0, 1.0, 6):
        value = low + fraction * (high - low)
        x = x_pos(value)
        y = y_pos(value)
        draw.line((x, top, x, top + plot_h), fill="#e5e7eb", width=1)
        draw.line((left, y, left + plot_w, y), fill="#e5e7eb", width=1)
        draw.text((x - 16, top + plot_h + 14), f"{value:.0f}", fill="#475569")
        draw.text((left - 54, y - 6), f"{value:.0f}", fill="#475569")
    draw.line((x_pos(low), y_pos(low), x_pos(high), y_pos(high)), fill="#111827", width=2)
    palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#475569"]
    unique_files = list(np.unique(result.source_files))
    step = max(1, math.ceil(len(result.actual) / 3500))
    for index in range(0, len(result.actual), step):
        source_index = unique_files.index(result.source_files[index])
        color = palette[source_index % len(palette)]
        x = x_pos(float(result.actual[index]))
        y = y_pos(float(result.predictions[index]))
        draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=color + "80")
    draw.rectangle((left, top, left + plot_w, top + plot_h), outline="#94a3b8", width=1)
    draw.text((left, 16), f"M04 {result.target} relative feedforward: measured vs predicted", fill="#111827")
    draw.text(
        (left, 38),
        f"file-held-out validation | {result.model} | files={result.files} | blocks={result.rows} | RMSE={result.rmse:.1f} | R2={result.r2:.3f}",
        fill="#475569",
    )
    draw.text((left + plot_w / 2 - 110, height - 28), "Measured relative command (counts)", fill="#111827")
    draw.text((8, top + plot_h / 2), "Predicted", fill="#111827")
    image.save(path)


def write_report(
    path: Path,
    root: Path,
    audits: list[FileAudit],
    datasets: dict[str, pd.DataFrame],
    results: list[FitResult],
    recommended: FitResult,
    anchor: dict[str, float | str],
    filters: dict[str, float],
) -> None:
    coefficient_map = dict(zip(recommended.feature_names, recommended.coefficients))
    coefficient_std = dict(zip(recommended.feature_names, recommended.loo_coefficient_std))
    q_coefficients = [float(coefficient_map.get(f"J{joint}", 0.0)) for joint in JOINTS]
    abs_j0_coefficient = float(coefficient_map.get("abs_J0", 0.0))
    j0_pos_coefficient = float(coefficient_map.get("J0_pos_distance", 0.0))
    j0_neg_coefficient = float(coefficient_map.get("J0_neg_distance", 0.0))
    anchor_q = [float(anchor[f"q{joint}"]) for joint in JOINTS]
    anchor_y = float(anchor[recommended.target])
    anchor_contribution = sum(coef * q for coef, q in zip(q_coefficients, anchor_q))
    anchor_contribution += abs_j0_coefficient * abs(anchor_q[0])
    anchor_contribution += j0_pos_coefficient * max(anchor_q[0], 0.0)
    anchor_contribution += j0_neg_coefficient * max(-anchor_q[0], 0.0)
    absolute_intercept = anchor_y - anchor_contribution
    formula_terms: list[str] = []
    if "abs_J0" in coefficient_map:
        formula_terms.append(
            f"       {abs_j0_coefficient:+.4f} * (|J0| - |J0_anchor|)"
        )
    elif "J0_pos_distance" in coefficient_map:
        formula_terms.extend(
            [
                f"       {j0_pos_coefficient:+.4f} * (max(J0,0) - max(J0_anchor,0))",
                f"       {j0_neg_coefficient:+.4f} * (max(-J0,0) - max(-J0_anchor,0))",
            ]
        )
    elif "J0" in coefficient_map:
        formula_terms.append(f"       {q_coefficients[0]:+.4f} * (J0 - J0_anchor)")
    for joint in (1, 2, 3):
        if f"J{joint}" in coefficient_map:
            formula_terms.append(
                f"       {q_coefficients[joint]:+.4f} * (J{joint} - J{joint}_anchor)"
            )

    candidate_count = sum(audit.status == "candidate" for audit in audits)
    lines = [
        "# M04 历史数据前馈拟合",
        "",
        f"- 数据根目录：`{root.resolve()}`",
        f"- 审计 CSV：{len(audits)} 个；原始候选：{candidate_count} 个",
        f"- 推荐目标：`{recommended.target}`；模型：`{recommended.model}`",
        f"- 稳态聚合样本：{recommended.rows} 个，来自 {recommended.files} 个实验文件",
        "",
        "## 推荐模型",
        "",
        "不同实验执行过多次 ZERO，因此不建议把所有文件拟合成一个固定全局截距。使用运行时锚点：",
        "",
        "```text",
        "M04_ff = M04_anchor",
        *formula_terms,
        "```",
        "",
        "系数单位为 `motor counts / deg`。M04_anchor 是进入角度控制时已经建立好的张力 bias。",
        "J0 使用到机械零位的距离 `|J0|`，不是有符号角度；J1、J2、J3 保留有符号线性项。",
        "",
        "当前最新可用实验锚点仅用于回放检查：",
        "",
        f"```text\nsource = {anchor['source_file']}\nM04_anchor = {anchor_y:.1f} counts\n"
        + "\n".join(f"J{joint}_anchor = {anchor_q[joint]:.3f} deg" for joint in JOINTS)
        + f"\n等价固定截距 b = {absolute_intercept:.1f} counts（只对该次零位有效）\n```",
        "",
        "## 系数稳定性",
        "",
        "| 项 | 系数 | 留一文件标准差 |",
        "|---|---:|---:|",
    ]
    for name in recommended.feature_names:
        lines.append(f"| {name} | {coefficient_map[name]:.4f} | {coefficient_std[name]:.4f} |")
    lines.extend(
        [
            "",
            "## 模型比较",
            "",
            "| 目标 | 模型 | 文件 | 样本块 | RMSE counts | MAE counts | R2 | 相对常值改善 | alpha |",
            "|---|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for result in sorted(results, key=lambda item: (item.target, item.rmse)):
        lines.append(
            f"| {result.target} | {result.model} | {result.files} | {result.rows} | "
            f"{result.rmse:.2f} | {result.mae:.2f} | {result.r2:.3f} | "
            f"{result.improvement_pct:.1f}% | {result.alpha:g} |"
        )
    lines.extend(
        [
            "",
            "## 数据筛选",
            "",
            f"- M04 力窗口：{filters['force_min_n']:.1f} 到 {filters['force_max_n']:.1f} N",
            f"- 最大关节速度：{filters['max_qdot_deg_s']:.1f} deg/s",
            f"- 最大 M04 servo speed：{filters['max_servo_speed']:.1f}",
            f"- 聚合窗口：{filters['block_s']:.1f} s，中位数聚合",
            "- 仅使用 J00-J03 有效、力/编码器数据新鲜、M04 在线且未触发命令限速的 degree 样本。",
            "- 线性扰动 CSV 和旧 MCP 控制 CSV 只做审计，不用于恒张力主模型，避免把人为扰动和旧控制结构混入前馈。",
            "",
            "## 解释",
            "",
            f"推荐模型相对常值 bias 的 RMSE 改善为 {recommended.improvement_pct:.1f}%，"
            f"相对变化 R2={recommended.r2:.3f}。"
            + (
                "`tension_N` 仅作为恒张力拟合中的干扰修正，固件前馈只使用 J00-J03 系数；"
                if "tension_N" in recommended.feature_names
                else "加入 `tension_N` 后没有改善按文件验证误差，因此采用更简单的纯角度线性模型；"
            )
            + "剩余张力误差仍交给低带宽 PI 修正。",
            "",
            "## 固件形式",
            "",
            "```cpp",
            f"static const float kM04J0AbsCountsPerDeg = {abs_j0_coefficient:.4f}f;",
            "static const float kM04FeedforwardCountsPerDeg[4] = {",
            "    " + ", ".join(f"{value:.4f}f" for value in q_coefficients),
            "};",
            "```",
            "",
            "在正式写入固件前，建议用一次独立慢速姿态扫描验证方向和最大误差。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit M04 feedforward from all historical CSV logs.")
    parser.add_argument("--root", type=Path, default=Path("run_data"))
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--force-min-n", type=float, default=-21.0)
    parser.add_argument("--force-max-n", type=float, default=-14.0)
    parser.add_argument("--max-qdot-deg-s", type=float, default=3.0)
    parser.add_argument("--max-servo-speed", type=float, default=100.0)
    parser.add_argument("--block-s", type=float, default=1.0)
    args = parser.parse_args()

    root = args.root.resolve()
    if not root.exists():
        raise FileNotFoundError(root)
    output_dir = args.output_dir or root / f"analysis_m04_feedforward_{datetime.now():%Y%m%d_%H%M%S}"
    output_dir.mkdir(parents=True, exist_ok=True)

    audits = audit_files(root)
    audit_rows = [
        {
            "path": str(audit.path.resolve()),
            "category": audit.category,
            "rows": audit.rows,
            "columns": audit.columns,
            "has_q": int(audit.has_q),
            "has_force": int(audit.has_force),
            "has_servo": int(audit.has_servo),
            "has_bias": int(audit.has_bias),
            "status": audit.status,
        }
        for audit in audits
    ]
    pd.DataFrame(audit_rows).to_csv(output_dir / "m04_csv_audit.csv", index=False, encoding="utf-8-sig")

    stable_frames: list[pd.DataFrame] = []
    for audit in audits:
        if audit.status != "candidate":
            continue
        loaded = load_candidate(audit.path, audit.category)
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
            stable_frames.append(stable)
    if not stable_frames:
        raise RuntimeError("no stable M04 rows passed the filters")
    stable_all = pd.concat(stable_frames, ignore_index=True)

    datasets: dict[str, pd.DataFrame] = {}
    for target in ("bias", "servo_abs"):
        blocks = aggregate_blocks(stable_all, target=target, block_s=args.block_s)
        if blocks.empty:
            continue
        target_column = "bias" if target == "bias" else "servo_abs"
        if target == "bias":
            blocks = blocks[np.abs(blocks["bias"].to_numpy(dtype=float)) < 7000.0].copy()
        per_file_count = blocks.groupby("source_file")[target_column].count()
        keep_files = set(per_file_count[per_file_count >= 8].index)
        blocks = blocks[blocks["source_file"].isin(keep_files)].copy()
        per_file_range = blocks.groupby("source_file")[target_column].agg(lambda values: float(values.max() - values.min()))
        informative_files = set(per_file_range[per_file_range >= 20.0].index)
        blocks = blocks[blocks["source_file"].isin(informative_files)].copy()
        if len(blocks) >= 20 and blocks["source_file"].nunique() >= 1:
            datasets[target] = blocks
            blocks.to_csv(output_dir / f"m04_{target}_stable_blocks.csv", index=False, encoding="utf-8-sig")
    if not datasets:
        raise RuntimeError("stable rows exist, but no target has enough informative blocks")

    results: list[FitResult] = []
    validation_rows: list[dict[str, object]] = []
    for target, dataset in datasets.items():
        for model in (
            "linear_j23",
            "linear_j123",
            "linear_absj0_j123",
            "linear_absj0_j123_force",
            "linear_piecewise_j0_j123",
            "linear_q",
            "linear_q_force",
            "quadratic_q_force",
        ):
            result = fit_model(dataset, target=target, model=model)
            results.append(result)
            validation_rows.extend(validation_by_file(result))
    pd.DataFrame(validation_rows).to_csv(
        output_dir / "m04_validation_by_file.csv",
        index=False,
        encoding="utf-8-sig",
    )

    summary_rows: list[dict[str, object]] = []
    coefficient_rows: list[dict[str, object]] = []
    for result in results:
        summary_rows.append(
            {
                "target": result.target,
                "model": result.model,
                "alpha": result.alpha,
                "files": result.files,
                "rows": result.rows,
                "rmse_counts": result.rmse,
                "mae_counts": result.mae,
                "r2_relative": result.r2,
                "baseline_rmse_counts": result.baseline_rmse,
                "improvement_pct": result.improvement_pct,
            }
        )
        for name, coefficient, std in zip(
            result.feature_names,
            result.coefficients,
            result.loo_coefficient_std,
        ):
            coefficient_rows.append(
                {
                    "target": result.target,
                    "model": result.model,
                    "feature": name,
                    "coefficient": coefficient,
                    "leave_one_file_out_std": std,
                }
            )
    pd.DataFrame(summary_rows).to_csv(output_dir / "m04_model_comparison.csv", index=False, encoding="utf-8-sig")
    pd.DataFrame(coefficient_rows).to_csv(output_dir / "m04_model_coefficients.csv", index=False, encoding="utf-8-sig")

    preferred_target = "bias" if "bias" in datasets and datasets["bias"]["source_file"].nunique() >= 2 else "servo_abs"
    preferred_results = [result for result in results if result.target == preferred_target]
    linear_force = next(
        (result for result in preferred_results if result.model == "linear_absj0_j123_force"),
        None,
    )
    linear = next(result for result in preferred_results if result.model == "linear_absj0_j123")
    recommended = linear_force if linear_force is not None and linear_force.rmse < linear.rmse * 0.95 else linear

    recommended_dataset = datasets[recommended.target]
    latest_file = max(
        recommended_dataset["source_path"].unique(),
        key=lambda value: Path(value).stat().st_mtime,
    )
    latest_rows = recommended_dataset[recommended_dataset["source_path"] == latest_file]
    latest_group = latest_rows.sort_values("time_s")["source_group"].iloc[-1]
    latest_rows = latest_rows[latest_rows["source_group"] == latest_group]
    target_column = "bias" if recommended.target == "bias" else "servo_abs"
    anchor = {
        "source_file": Path(latest_file).name,
        "bias": float(np.median(latest_rows["bias"])),
        "servo_abs": float(np.median(latest_rows["servo_abs"])),
        "force_n": float(np.median(latest_rows["force_n"])),
        **{f"q{joint}": float(np.median(latest_rows[f"q{joint}"])) for joint in JOINTS},
    }
    coefficient_map = dict(zip(recommended.feature_names, recommended.coefficients))
    model_json = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "target": recommended.target,
        "model": recommended.model,
        "relative_anchor_required": True,
        "formula": (
            "anchor_counts + j0_abs_counts_per_deg * (abs(q0) - abs(anchor_q0)) "
            "+ sum(joint_counts_per_deg[j] * (q[j] - anchor_q[j]))"
        ),
        "j0_abs_counts_per_deg": float(coefficient_map.get("abs_J0", 0.0)),
        "j0_positive_distance_counts_per_deg": float(coefficient_map.get("J0_pos_distance", 0.0)),
        "j0_negative_distance_counts_per_deg": float(coefficient_map.get("J0_neg_distance", 0.0)),
        "joint_counts_per_deg": [float(coefficient_map.get(f"J{joint}", 0.0)) for joint in JOINTS],
        "tension_nuisance_counts_per_n": float(coefficient_map.get("tension_N", 0.0)),
        "anchor": anchor,
        "metrics": {
            "files": recommended.files,
            "blocks": recommended.rows,
            "rmse_counts": recommended.rmse,
            "mae_counts": recommended.mae,
            "r2_relative": recommended.r2,
            "baseline_rmse_counts": recommended.baseline_rmse,
            "improvement_pct": recommended.improvement_pct,
        },
        "filters": {
            "force_min_n": args.force_min_n,
            "force_max_n": args.force_max_n,
            "max_qdot_deg_s": args.max_qdot_deg_s,
            "max_servo_speed": args.max_servo_speed,
            "block_s": args.block_s,
        },
    }
    (output_dir / "m04_feedforward_model.json").write_text(
        json.dumps(model_json, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    svg_scatter(output_dir / "m04_feedforward_diagnostics.svg", recommended)
    png_scatter(output_dir / "m04_feedforward_diagnostics.png", recommended)
    write_report(
        output_dir / "m04_feedforward_report.md",
        root=root,
        audits=audits,
        datasets=datasets,
        results=results,
        recommended=recommended,
        anchor=anchor,
        filters=model_json["filters"],
    )

    print(f"output_dir={output_dir.resolve()}")
    print(f"audited_files={len(audits)} candidates={sum(audit.status == 'candidate' for audit in audits)}")
    print(
        f"recommended={recommended.target}/{recommended.model} files={recommended.files} "
        f"blocks={recommended.rows} rmse={recommended.rmse:.2f} r2={recommended.r2:.3f}"
    )
    print(
        "joint_counts_per_deg="
        + ",".join(f"{float(coefficient_map.get(f'J{joint}', 0.0)):.5f}" for joint in JOINTS)
    )
    print(f"j0_abs_counts_per_deg={float(coefficient_map.get('abs_J0', 0.0)):.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
