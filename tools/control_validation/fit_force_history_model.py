"""Fit force/tension models using motor load, position, direction and history.

The target is positive rope tension in N:
    tension_N = max(0, -force_N)

Inputs intentionally exclude force sensor history so the result can guide a
future controller that does not carry force sensors.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


CHANNEL_MOTOR = {
    2: 0,
    1: 1,
    3: 2,
    4: 3,
    5: 4,
}
MOTORS = sorted(CHANNEL_MOTOR.values())
WINDOWS_S = (0.6, 0.8, 1.0)
ALPHAS = (0.0, 0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0)
EPS = 1.0e-9


@dataclass
class ModelResult:
    motor: int
    channel: int
    model: str
    rows: int
    train_rows: int
    val_rows: int
    test_rows: int
    alpha: float
    rmse: float
    mae: float
    r2: float
    baseline_rmse: float
    improvement_vs_baseline_pct: float
    top_features: str


def find_latest_nonpositive_csv(run_data: Path) -> Path:
    pattern = re.compile(r"^training_collect_\d{8}_\d{6}_nonpositive_force\.csv$")
    files = sorted(
        (path for path in run_data.glob("training_collect_*_nonpositive_force.csv") if pattern.match(path.name)),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not files:
        raise FileNotFoundError(f"no training_collect_*_nonpositive_force.csv found in {run_data}")
    return files[0]


def to_float_series(df: pd.DataFrame, column: str) -> np.ndarray:
    if column not in df.columns:
        raise KeyError(column)
    return pd.to_numeric(df[column], errors="coerce").to_numpy(dtype=float)


def sign(value: float) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 0.0


def metric_rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def metric_mae(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.mean(np.abs(y_true - y_pred)))


def metric_r2(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y_true - y_pred) ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if ss_tot <= EPS:
        return math.nan
    return 1.0 - ss_res / ss_tot


def ridge_fit_predict(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_eval: np.ndarray,
    alpha: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = np.mean(x_train, axis=0)
    std = np.std(x_train, axis=0)
    std[std < EPS] = 1.0
    train_scaled = (x_train - mean) / std
    eval_scaled = (x_eval - mean) / std
    train_design = np.column_stack([np.ones(len(train_scaled)), train_scaled])
    eval_design = np.column_stack([np.ones(len(eval_scaled)), eval_scaled])
    reg = np.eye(train_design.shape[1]) * alpha
    reg[0, 0] = 0.0
    if alpha == 0:
        beta = np.linalg.lstsq(train_design, y_train, rcond=None)[0]
    else:
        beta = np.linalg.solve(train_design.T @ train_design + reg, train_design.T @ y_train)
    return eval_design @ beta, beta, mean, std


def split_indices(n: int) -> tuple[slice, slice, slice, slice]:
    train_end = max(1, int(n * 0.60))
    val_end = max(train_end + 1, int(n * 0.80))
    if val_end >= n:
        val_end = n - 1
    return slice(0, train_end), slice(train_end, val_end), slice(0, val_end), slice(val_end, n)


def choose_alpha(x: np.ndarray, y: np.ndarray) -> float:
    train_slice, val_slice, _train_val_slice, _test_slice = split_indices(len(x))
    best_alpha = ALPHAS[0]
    best_rmse = math.inf
    for alpha in ALPHAS:
        pred, _beta, _mean, _std = ridge_fit_predict(x[train_slice], y[train_slice], x[val_slice], alpha)
        rmse = metric_rmse(y[val_slice], pred)
        if rmse < best_rmse:
            best_rmse = rmse
            best_alpha = alpha
    return best_alpha


def fit_and_score(x: np.ndarray, y: np.ndarray, feature_names: list[str]) -> tuple[dict[str, float], str, np.ndarray]:
    train_slice, _val_slice, train_val_slice, test_slice = split_indices(len(x))
    alpha = choose_alpha(x, y)
    y_test = y[test_slice]
    pred, beta, _mean, _std = ridge_fit_predict(x[train_val_slice], y[train_val_slice], x[test_slice], alpha)
    baseline_pred = np.full_like(y_test, fill_value=float(np.mean(y[train_slice])), dtype=float)
    rmse = metric_rmse(y_test, pred)
    baseline_rmse = metric_rmse(y_test, baseline_pred)
    top_features = top_feature_text(beta[1:], feature_names)
    metrics = {
        "alpha": alpha,
        "rmse": rmse,
        "mae": metric_mae(y_test, pred),
        "r2": metric_r2(y_test, pred),
        "baseline_rmse": baseline_rmse,
        "improvement_vs_baseline_pct": 100.0 * (baseline_rmse - rmse) / baseline_rmse if baseline_rmse > EPS else math.nan,
    }
    return metrics, top_features, pred


def top_feature_text(coefficients: np.ndarray, feature_names: list[str], limit: int = 8) -> str:
    pairs = sorted(
        ((abs(float(value)), float(value), name) for value, name in zip(coefficients, feature_names)),
        reverse=True,
    )
    return "; ".join(f"{name}={coef:.3g}" for _abs_coef, coef, name in pairs[:limit])


def add_current_motor_features(
    features: list[float],
    names: list[str],
    data: dict[str, np.ndarray],
    i: int,
    motor: int,
    prefix: str,
) -> None:
    elapsed = data["elapsed_s"]
    abs_pos = data[f"m{motor:02d}_abs"]
    speed = data[f"m{motor:02d}_speed"]
    load = data[f"m{motor:02d}_load"]
    previous = max(0, i - 1)
    dt = max(EPS, elapsed[i] - elapsed[previous])
    velocity = 0.0 if i == 0 else (abs_pos[i] - abs_pos[previous]) / dt
    values = {
        f"{prefix}_abs": abs_pos[i],
        f"{prefix}_speed": speed[i],
        f"{prefix}_load": load[i],
        f"{prefix}_abs_load": abs(load[i]),
        f"{prefix}_velocity": velocity,
        f"{prefix}_dir_speed": sign(speed[i]),
        f"{prefix}_dir_velocity": sign(velocity),
    }
    for name, value in values.items():
        names.append(name)
        features.append(float(value))


def add_motor_history_features(
    features: list[float],
    names: list[str],
    data: dict[str, np.ndarray],
    i: int,
    motor: int,
    window_s: float,
    prefix: str,
) -> None:
    elapsed = data["elapsed_s"]
    start_t = elapsed[i] - window_s
    start = int(np.searchsorted(elapsed, start_t, side="left"))
    if start >= i:
        start = max(0, i - 1)
    span = slice(start, i + 1)
    dt = max(EPS, elapsed[i] - elapsed[start])
    abs_pos = data[f"m{motor:02d}_abs"][span]
    speed = data[f"m{motor:02d}_speed"][span]
    load = data[f"m{motor:02d}_load"][span]
    abs_load = np.abs(load)

    stats = {
        f"{prefix}_abs_delta_w": abs_pos[-1] - abs_pos[0],
        f"{prefix}_abs_slope_w": (abs_pos[-1] - abs_pos[0]) / dt,
        f"{prefix}_abs_mean_w": float(np.mean(abs_pos)),
        f"{prefix}_speed_mean_w": float(np.mean(speed)),
        f"{prefix}_speed_max_w": float(np.max(speed)),
        f"{prefix}_speed_min_w": float(np.min(speed)),
        f"{prefix}_load_mean_w": float(np.mean(load)),
        f"{prefix}_load_delta_w": load[-1] - load[0],
        f"{prefix}_load_max_w": float(np.max(load)),
        f"{prefix}_load_min_w": float(np.min(load)),
        f"{prefix}_abs_load_mean_w": float(np.mean(abs_load)),
        f"{prefix}_abs_load_max_w": float(np.max(abs_load)),
        f"{prefix}_abs_load_delta_w": abs_load[-1] - abs_load[0],
        f"{prefix}_dir_abs_delta_w": sign(abs_pos[-1] - abs_pos[0]),
        f"{prefix}_dir_load_delta_w": sign(load[-1] - load[0]),
    }
    for name, value in stats.items():
        names.append(name)
        features.append(float(value))


def build_dataset(
    data: dict[str, np.ndarray],
    target_motor: int,
    target_channel: int,
    model: str,
    window_s: float | None,
) -> tuple[np.ndarray, np.ndarray, list[str], np.ndarray]:
    elapsed = data["elapsed_s"]
    force = data[f"tension_m{target_motor:02d}_n"]
    y_all = np.maximum(0.0, -force)
    x_rows: list[list[float]] = []
    y_rows: list[float] = []
    t_rows: list[float] = []
    feature_names: list[str] | None = None

    min_i = 1
    if window_s is not None:
        min_i = int(np.searchsorted(elapsed, elapsed[0] + window_s, side="left"))
    for i in range(min_i, len(elapsed)):
        if not math.isfinite(y_all[i]):
            continue
        features: list[float] = []
        names: list[str] = []

        if model == "load_only":
            load = data[f"m{target_motor:02d}_load"][i]
            names.extend(["load", "abs_load"])
            features.extend([float(load), float(abs(load))])
        elif model == "current_motor":
            add_current_motor_features(features, names, data, i, target_motor, f"m{target_motor:02d}")
        elif model == "history_motor":
            assert window_s is not None
            add_current_motor_features(features, names, data, i, target_motor, f"m{target_motor:02d}")
            add_motor_history_features(features, names, data, i, target_motor, window_s, f"m{target_motor:02d}_{window_s:.1f}s")
        elif model == "history_all_motors":
            assert window_s is not None
            for motor in MOTORS:
                add_current_motor_features(features, names, data, i, motor, f"m{motor:02d}")
                add_motor_history_features(features, names, data, i, motor, window_s, f"m{motor:02d}_{window_s:.1f}s")
        else:
            raise ValueError(model)

        if feature_names is None:
            feature_names = names
        x_rows.append(features)
        y_rows.append(float(y_all[i]))
        t_rows.append(float(elapsed[i]))

    if feature_names is None:
        raise ValueError("no feature rows")
    x = np.asarray(x_rows, dtype=float)
    y = np.asarray(y_rows, dtype=float)
    t = np.asarray(t_rows, dtype=float)
    mask = np.isfinite(x).all(axis=1) & np.isfinite(y)
    return x[mask], y[mask], feature_names, t[mask]


def load_data(path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(path, encoding="utf-8-sig")
    data: dict[str, np.ndarray] = {
        "elapsed_s": to_float_series(df, "elapsed_s"),
    }
    for motor in MOTORS:
        data[f"tension_m{motor:02d}_n"] = to_float_series(df, f"tension_m{motor:02d}_n")
        for field in ("abs", "speed", "load"):
            data[f"m{motor:02d}_{field}"] = to_float_series(df, f"servo_m{motor:02d}_{field}")
    return data


def model_specs() -> list[tuple[str, float | None, str]]:
    specs = [
        ("load_only", None, "load_only"),
        ("current_motor", None, "current_motor"),
    ]
    for window_s in WINDOWS_S:
        specs.append(("history_motor", window_s, f"history_motor_{window_s:.1f}s"))
        specs.append(("history_all_motors", window_s, f"history_all_motors_{window_s:.1f}s"))
    return specs


def write_report(path: Path, source: Path, results: list[ModelResult]) -> None:
    lines = [
        "# Force history model report",
        "",
        f"- source: `{source.name}`",
        "- target: `tension_N = max(0, -force_N)`",
        "- split: chronological 60% train, 20% validation, 20% test",
        "- model: standardized Ridge regression, alpha selected on validation split",
        "",
        "## Best Model Per Motor",
        "",
        "| Motor | Channel | Best model | Test RMSE N | Test MAE N | Test R2 | Baseline RMSE N | Improvement | Top features |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for motor in MOTORS:
        motor_results = [item for item in results if item.motor == motor]
        best = min(motor_results, key=lambda item: item.rmse)
        lines.append(
            f"| M{best.motor:02d} | CH{best.channel} | {best.model} | {best.rmse:.3f} | {best.mae:.3f} | {best.r2:.3f} | {best.baseline_rmse:.3f} | {best.improvement_vs_baseline_pct:.1f}% | {best.top_features} |"
        )

    lines.extend(
        [
            "",
            "## All Results",
            "",
            "| Motor | Model | Rows | Alpha | RMSE N | MAE N | R2 | Improvement |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for item in sorted(results, key=lambda r: (r.motor, r.rmse)):
        lines.append(
            f"| M{item.motor:02d} | {item.model} | {item.rows} | {item.alpha:g} | {item.rmse:.3f} | {item.mae:.3f} | {item.r2:.3f} | {item.improvement_vs_baseline_pct:.1f}% |"
        )

    lines.extend(
        [
            "",
            "Interpretation:",
            "",
            "- If a history model beats `load_only`, the tension state is not captured by instantaneous load alone.",
            "- If `history_all_motors` wins, cross-coupling from other tendons/motors matters.",
            "- Negative test R2 means the model generalizes worse than predicting the training mean on the last 20% of the run.",
            "- With only one run, these results are diagnostic rather than final calibration.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_predictions(path: Path, source: Path, data: dict[str, np.ndarray], best_results: list[ModelResult]) -> None:
    rows: list[dict[str, object]] = []
    for result in best_results:
        model_name, window_s = parse_model_name(result.model)
        x, y, names, t = build_dataset(data, result.motor, result.channel, model_name, window_s)
        _train_slice, _val_slice, train_val_slice, test_slice = split_indices(len(x))
        pred, _beta, _mean, _std = ridge_fit_predict(x[train_val_slice], y[train_val_slice], x[test_slice], result.alpha)
        for elapsed, actual, predicted in zip(t[test_slice], y[test_slice], pred):
            rows.append(
                {
                    "source": source.name,
                    "motor": f"M{result.motor:02d}",
                    "channel": f"CH{result.channel}",
                    "model": result.model,
                    "elapsed_s": f"{elapsed:.3f}",
                    "actual_tension_n": f"{actual:.3f}",
                    "predicted_tension_n": f"{predicted:.3f}",
                    "error_n": f"{predicted - actual:.3f}",
                }
            )
    rows.sort(key=lambda row: (str(row["motor"]), float(row["elapsed_s"])))
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        fieldnames = [
            "source",
            "motor",
            "channel",
            "model",
            "elapsed_s",
            "actual_tension_n",
            "predicted_tension_n",
            "error_n",
        ]
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def svg_escape(text: object) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def nice_range(values: list[float], pad_ratio: float = 0.08) -> tuple[float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return 0.0, 1.0
    lo, hi = min(finite), max(finite)
    if lo == hi:
        lo -= 1.0
        hi += 1.0
    pad = (hi - lo) * pad_ratio
    return lo - pad, hi + pad


def write_prediction_svg(path: Path, predictions_csv: Path) -> None:
    rows = list(csv.DictReader(open(predictions_csv, "r", encoding="utf-8-sig")))
    width, height = 1200, 900
    margin_l, margin_r = 72, 22
    panel_h = 138
    gap = 30
    top = 58
    plot_w = width - margin_l - margin_r
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Segoe UI,Arial,sans-serif;font-size:13px;fill:#334155}.title{font-size:18px;font-weight:600}.axis{stroke:#64748b;stroke-width:1}.grid{stroke:#e2e8f0;stroke-width:1}.actual{stroke:#2563eb;stroke-width:1.6;fill:none}.pred{stroke:#dc2626;stroke-width:1.4;fill:none;stroke-dasharray:5 4}</style>',
        '<text class="title" x="72" y="30">Best history model: test-segment actual vs predicted tension</text>',
        '<text x="72" y="50">blue = actual tension, red dashed = predicted tension</text>',
    ]

    def polyline(points: list[tuple[float, float]], css_class: str) -> None:
        if len(points) < 2:
            return
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
        elements.append(f'<polyline class="{css_class}" points="{d}"/>')

    for idx, motor in enumerate(f"M{m:02d}" for m in MOTORS):
        part = [row for row in rows if row["motor"] == motor]
        if not part:
            continue
        y_top = top + idx * (panel_h + gap)
        times = [float(row["elapsed_s"]) for row in part]
        actual = [float(row["actual_tension_n"]) for row in part]
        pred = [float(row["predicted_tension_n"]) for row in part]
        x0, x1 = min(times), max(times)
        y0, y1 = nice_range(actual + pred)
        if x0 == x1:
            x1 = x0 + 1.0

        elements.append(f'<text x="{margin_l}" y="{y_top - 12}" class="title">{svg_escape(motor)} {svg_escape(part[0]["channel"])} {svg_escape(part[0]["model"])}</text>')
        for tick in range(5):
            gx = margin_l + plot_w * tick / 4
            gy = y_top + panel_h * tick / 4
            xv = x0 + (x1 - x0) * tick / 4
            yv = y1 - (y1 - y0) * tick / 4
            elements.append(f'<line class="grid" x1="{gx:.1f}" y1="{y_top}" x2="{gx:.1f}" y2="{y_top + panel_h}"/>')
            elements.append(f'<line class="grid" x1="{margin_l}" y1="{gy:.1f}" x2="{margin_l + plot_w}" y2="{gy:.1f}"/>')
            elements.append(f'<text x="{gx:.1f}" y="{y_top + panel_h + 17}" text-anchor="middle">{xv:.0f}</text>')
            elements.append(f'<text x="{margin_l - 10}" y="{gy + 4:.1f}" text-anchor="end">{yv:.1f}</text>')
        elements.append(f'<line class="axis" x1="{margin_l}" y1="{y_top + panel_h}" x2="{margin_l + plot_w}" y2="{y_top + panel_h}"/>')
        elements.append(f'<line class="axis" x1="{margin_l}" y1="{y_top}" x2="{margin_l}" y2="{y_top + panel_h}"/>')

        def scale(t: float, value: float) -> tuple[float, float]:
            px = margin_l + (t - x0) * plot_w / (x1 - x0)
            py = y_top + panel_h - (value - y0) * panel_h / (y1 - y0)
            return px, py

        polyline([scale(t, v) for t, v in zip(times, actual)], "actual")
        polyline([scale(t, v) for t, v in zip(times, pred)], "pred")

    elements.append("</svg>")
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def parse_model_name(name: str) -> tuple[str, float | None]:
    if name == "load_only" or name == "current_motor":
        return name, None
    match = re.match(r"^(history_motor|history_all_motors)_(\d+\.\d+)s$", name)
    if not match:
        raise ValueError(name)
    return match.group(1), float(match.group(2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--run-data", type=Path, default=Path("run_data"))
    args = parser.parse_args()

    source = args.csv or find_latest_nonpositive_csv(args.run_data)
    data = load_data(source)
    results: list[ModelResult] = []
    for channel, motor in sorted(CHANNEL_MOTOR.items(), key=lambda item: item[1]):
        for model_name, window_s, label in model_specs():
            x, y, feature_names, _elapsed = build_dataset(data, motor, channel, model_name, window_s)
            if len(x) < 30:
                continue
            metrics, top_features, _pred = fit_and_score(x, y, feature_names)
            train_slice, val_slice, _train_val_slice, test_slice = split_indices(len(x))
            results.append(
                ModelResult(
                    motor=motor,
                    channel=channel,
                    model=label,
                    rows=len(x),
                    train_rows=train_slice.stop - train_slice.start,
                    val_rows=val_slice.stop - val_slice.start,
                    test_rows=test_slice.stop - test_slice.start,
                    alpha=metrics["alpha"],
                    rmse=metrics["rmse"],
                    mae=metrics["mae"],
                    r2=metrics["r2"],
                    baseline_rmse=metrics["baseline_rmse"],
                    improvement_vs_baseline_pct=metrics["improvement_vs_baseline_pct"],
                    top_features=top_features,
                )
            )

    stem = source.with_suffix("")
    summary_path = stem.with_name(stem.name + "_history_model_summary.csv")
    with open(summary_path, "w", newline="", encoding="utf-8-sig") as fp:
        fieldnames = list(ModelResult.__dataclass_fields__.keys())
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for item in results:
            writer.writerow(item.__dict__)

    report_path = stem.with_name(stem.name + "_history_model_report.md")
    write_report(report_path, source, results)

    best_results = [min([item for item in results if item.motor == motor], key=lambda item: item.rmse) for motor in MOTORS]
    predictions_path = stem.with_name(stem.name + "_history_model_predictions.csv")
    write_predictions(predictions_path, source, data, best_results)
    plot_path = stem.with_name(stem.name + "_history_model_predictions.svg")
    write_prediction_svg(plot_path, predictions_path)

    print(f"source={source}")
    print(f"summary={summary_path}")
    print(f"report={report_path}")
    print(f"predictions={predictions_path}")
    print(f"prediction_plot={plot_path}")
    for item in best_results:
        print(
            f"M{item.motor:02d}/CH{item.channel}: best={item.model}, "
            f"rmse={item.rmse:.3f}N, mae={item.mae:.3f}N, r2={item.r2:.3f}, "
            f"baseline={item.baseline_rmse:.3f}N, improvement={item.improvement_vs_baseline_pct:.1f}%"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
