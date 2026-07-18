"""Filter latest force/load training CSV and summarize force-load relations."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


CHANNEL_MOTOR = {
    2: 0,
    1: 1,
    3: 2,
    4: 3,
    5: 4,
}
LAG_SAMPLE_SECONDS = 0.2
MAX_LAG_SAMPLES = 10


def find_latest_training_csv(run_data: Path) -> Path:
    pattern = re.compile(r"^training_collect_\d{8}_\d{6}\.csv$")
    files = sorted(
        (path for path in run_data.glob("training_collect_*.csv") if pattern.match(path.name)),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not files:
        raise FileNotFoundError(f"no training_collect_*.csv found in {run_data}")
    return files[0]


def to_float(value: object) -> float:
    try:
        if value is None:
            return math.nan
        text = str(value).strip()
        if not text:
            return math.nan
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def pearson(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 2:
        return math.nan
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    sxx = sum((x - x_mean) ** 2 for x in xs)
    syy = sum((y - y_mean) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return math.nan
    sxy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    return sxy / math.sqrt(sxx * syy)


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    n = len(xs)
    if n < 2:
        return math.nan, math.nan, math.nan
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    sxx = sum((x - x_mean) ** 2 for x in xs)
    if sxx <= 0:
        return math.nan, math.nan, math.nan
    sxy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = y_mean - slope * x_mean
    ss_tot = sum((y - y_mean) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = math.nan if ss_tot <= 0 else 1.0 - ss_res / ss_tot
    return slope, intercept, r2


def best_lag_corr(xs: list[float], ys: list[float], max_lag: int = MAX_LAG_SAMPLES) -> tuple[int, float]:
    best_lag = 0
    best_corr = math.nan
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            x_aligned = xs[-lag:]
            y_aligned = ys[: len(x_aligned)]
        elif lag > 0:
            x_aligned = xs[:-lag]
            y_aligned = ys[lag:]
        else:
            x_aligned = xs
            y_aligned = ys
        corr = pearson(x_aligned, y_aligned)
        if math.isfinite(corr) and (
            not math.isfinite(best_corr) or abs(corr) > abs(best_corr)
        ):
            best_lag = lag
            best_corr = corr
    return best_lag, best_corr


def summarize_pair(rows: list[dict[str, str]], motor: int, channel: int) -> dict[str, object]:
    force_field = f"tension_m{motor:02d}_n"
    if force_field not in rows[0]:
        force_field = f"force_ch{channel}_n"
    load_field = f"servo_m{motor:02d}_load"
    abs_field = f"servo_m{motor:02d}_abs"
    speed_field = f"servo_m{motor:02d}_speed"

    force_n: list[float] = []
    tension_mag: list[float] = []
    load: list[float] = []
    abs_load: list[float] = []
    motor_abs: list[float] = []
    speed: list[float] = []

    for row in rows:
        f = to_float(row.get(force_field))
        l = to_float(row.get(load_field))
        if math.isfinite(f) and math.isfinite(l):
            force_n.append(f)
            tension_mag.append(max(0.0, -f))
            load.append(l)
            abs_load.append(abs(l))
            a = to_float(row.get(abs_field))
            s = to_float(row.get(speed_field))
            if math.isfinite(a):
                motor_abs.append(a)
            if math.isfinite(s):
                speed.append(s)

    slope_load_force, intercept_load_force, r2_load_force = linear_fit(load, force_n)
    slope_absload_tension, intercept_absload_tension, r2_absload_tension = linear_fit(abs_load, tension_mag)
    lag_force_load, corr_force_load_lag = best_lag_corr(force_n, load)
    lag_tension_absload, corr_tension_absload_lag = best_lag_corr(tension_mag, abs_load)
    return {
        "motor": f"M{motor:02d}",
        "channel": f"CH{channel}",
        "n": len(force_n),
        "force_min_n": min(force_n) if force_n else math.nan,
        "force_max_n": max(force_n) if force_n else math.nan,
        "force_mean_n": sum(force_n) / len(force_n) if force_n else math.nan,
        "tension_mag_mean_n": sum(tension_mag) / len(tension_mag) if tension_mag else math.nan,
        "load_min": min(load) if load else math.nan,
        "load_max": max(load) if load else math.nan,
        "load_mean": sum(load) / len(load) if load else math.nan,
        "abs_load_mean": sum(abs_load) / len(abs_load) if abs_load else math.nan,
        "corr_force_load": pearson(force_n, load),
        "corr_tension_absload": pearson(tension_mag, abs_load),
        "fit_force_vs_load_slope": slope_load_force,
        "fit_force_vs_load_intercept": intercept_load_force,
        "fit_force_vs_load_r2": r2_load_force,
        "fit_tension_vs_absload_slope": slope_absload_tension,
        "fit_tension_vs_absload_intercept": intercept_absload_tension,
        "fit_tension_vs_absload_r2": r2_absload_tension,
        "best_lag_force_load_samples": lag_force_load,
        "best_lag_force_load_seconds": lag_force_load * LAG_SAMPLE_SECONDS,
        "best_lag_force_load_corr": corr_force_load_lag,
        "best_lag_tension_absload_samples": lag_tension_absload,
        "best_lag_tension_absload_seconds": lag_tension_absload * LAG_SAMPLE_SECONDS,
        "best_lag_tension_absload_corr": corr_tension_absload_lag,
        "motor_abs_min": min(motor_abs) if motor_abs else math.nan,
        "motor_abs_max": max(motor_abs) if motor_abs else math.nan,
        "speed_min": min(speed) if speed else math.nan,
        "speed_max": max(speed) if speed else math.nan,
    }


def fmt(value: object, digits: int = 3) -> str:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}"
    return str(value)


def write_summary_markdown(path: Path, source: Path, filtered: Path, counts: dict[str, int], summaries: list[dict[str, object]]) -> None:
    lines = [
        "# Force sensor vs motor load analysis",
        "",
        f"- source: `{source.name}`",
        f"- filtered_csv: `{filtered.name}`",
        f"- raw_rows: {counts['raw_rows']}",
        f"- kept_rows: {counts['kept_rows']}",
        f"- removed_positive_force_rows: {counts['removed_rows']}",
        f"- removed_ratio: {counts['removed_rows'] / counts['raw_rows']:.1%}" if counts["raw_rows"] else "- removed_ratio: ",
        "",
        "Filtering rule: remove rows where any of `force_ch1_n`..`force_ch5_n` is greater than 0.",
        "",
        "## Per-motor relation",
        "",
        "| Motor | Force CH | N | force range N | load range | corr(force, load) | best lag force/load | corr(-force, abs(load)) | best lag tension/absload | R2 force~load |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for item in summaries:
        force_range = f"{fmt(item['force_min_n'], 2)}..{fmt(item['force_max_n'], 2)}"
        load_range = f"{fmt(item['load_min'], 0)}..{fmt(item['load_max'], 0)}"
        lines.append(
            "| {motor} | {channel} | {n} | {force_range} | {load_range} | {corr_fl} | {lag_fl} | {corr_tl} | {lag_tl} | {r2_fl} |".format(
                motor=item["motor"],
                channel=item["channel"],
                n=item["n"],
                force_range=force_range,
                load_range=load_range,
                corr_fl=fmt(item["corr_force_load"], 3),
                lag_fl=f"{fmt(item['best_lag_force_load_corr'], 3)} @ {fmt(item['best_lag_force_load_seconds'], 1)}s",
                corr_tl=fmt(item["corr_tension_absload"], 3),
                lag_tl=f"{fmt(item['best_lag_tension_absload_corr'], 3)} @ {fmt(item['best_lag_tension_absload_seconds'], 1)}s",
                r2_fl=fmt(item["fit_force_vs_load_r2"], 3),
            )
        )
    lines.extend(
        [
            "",
            "Interpretation notes:",
            "",
            "- `force` is signed sensor force in N. In this setup, rope tension is usually negative, so `-force` is the positive tension magnitude.",
            "- `corr(force, load)` shows signed relation. A strong negative or positive value means the servo load sign carries direction information.",
            "- `corr(-force, abs(load))` tests whether load magnitude tracks tension magnitude independent of sign.",
            "- Low R2 means a single-frame linear model using load alone is weak; motor position, velocity, direction and recent history may be needed.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def collect_points(rows: list[dict[str, str]], motor: int, channel: int) -> list[tuple[float, float, float, float]]:
    force_field = f"tension_m{motor:02d}_n"
    if force_field not in rows[0]:
        force_field = f"force_ch{channel}_n"
    load_field = f"servo_m{motor:02d}_load"
    points: list[tuple[float, float, float, float]] = []
    for row in rows:
        force_n = to_float(row.get(force_field))
        load = to_float(row.get(load_field))
        if math.isfinite(force_n) and math.isfinite(load):
            points.append((load, force_n, abs(load), max(0.0, -force_n)))
    return points


def nice_range(values: list[float], pad_ratio: float = 0.08) -> tuple[float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return 0.0, 1.0
    lo = min(finite)
    hi = max(finite)
    if lo == hi:
        lo -= 1.0
        hi += 1.0
    pad = (hi - lo) * pad_ratio
    return lo - pad, hi + pad


def svg_escape(text: object) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def write_scatter_svg(path: Path, rows: list[dict[str, str]]) -> None:
    colors = {
        0: "#2563eb",
        1: "#dc2626",
        2: "#16a34a",
        3: "#9333ea",
        4: "#ea580c",
    }
    series = [
        (motor, channel, collect_points(rows, motor, channel))
        for channel, motor in sorted(CHANNEL_MOTOR.items(), key=lambda item: item[1])
    ]

    left_values_x = [point[0] for _, _, points in series for point in points]
    left_values_y = [point[1] for _, _, points in series for point in points]
    right_values_x = [point[2] for _, _, points in series for point in points]
    right_values_y = [point[3] for _, _, points in series for point in points]
    lx0, lx1 = nice_range(left_values_x)
    ly0, ly1 = nice_range(left_values_y)
    rx0, rx1 = nice_range(right_values_x)
    ry0, ry1 = nice_range(right_values_y)

    width, height = 1200, 560
    left = (72, 54, 520, 390)
    right = (670, 54, 520, 390)
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Segoe UI,Arial,sans-serif;font-size:13px;fill:#334155}.title{font-size:18px;font-weight:600}.axis{stroke:#64748b;stroke-width:1}.grid{stroke:#e2e8f0;stroke-width:1}.legend{font-size:12px}</style>',
        '<text class="title" x="72" y="28">Force sensor vs motor load after removing positive-force rows</text>',
    ]

    def draw_axes(rect: tuple[int, int, int, int], title: str, x_label: str, y_label: str, xr: tuple[float, float], yr: tuple[float, float]) -> None:
        x, y, w, h = rect
        elements.append(f'<text x="{x}" y="{y - 18}" class="title">{svg_escape(title)}</text>')
        for i in range(5):
            gx = x + w * i / 4
            gy = y + h * i / 4
            xv = xr[0] + (xr[1] - xr[0]) * i / 4
            yv = yr[1] - (yr[1] - yr[0]) * i / 4
            elements.append(f'<line class="grid" x1="{gx:.1f}" y1="{y}" x2="{gx:.1f}" y2="{y + h}"/>')
            elements.append(f'<line class="grid" x1="{x}" y1="{gy:.1f}" x2="{x + w}" y2="{gy:.1f}"/>')
            elements.append(f'<text x="{gx:.1f}" y="{y + h + 20}" text-anchor="middle">{xv:.0f}</text>')
            elements.append(f'<text x="{x - 10}" y="{gy + 4:.1f}" text-anchor="end">{yv:.1f}</text>')
        elements.append(f'<line class="axis" x1="{x}" y1="{y + h}" x2="{x + w}" y2="{y + h}"/>')
        elements.append(f'<line class="axis" x1="{x}" y1="{y}" x2="{x}" y2="{y + h}"/>')
        elements.append(f'<text x="{x + w / 2:.1f}" y="{y + h + 44}" text-anchor="middle">{svg_escape(x_label)}</text>')
        elements.append(f'<text x="{x - 54}" y="{y + h / 2:.1f}" text-anchor="middle" transform="rotate(-90 {x - 54} {y + h / 2:.1f})">{svg_escape(y_label)}</text>')

    def scale(rect: tuple[int, int, int, int], xr: tuple[float, float], yr: tuple[float, float], xv: float, yv: float) -> tuple[float, float]:
        x, y, w, h = rect
        px = x + (xv - xr[0]) * w / (xr[1] - xr[0])
        py = y + h - (yv - yr[0]) * h / (yr[1] - yr[0])
        return px, py

    draw_axes(left, "Signed relation", "servo load", "force N", (lx0, lx1), (ly0, ly1))
    draw_axes(right, "Magnitude relation", "abs(servo load)", "tension magnitude N = -force", (rx0, rx1), (ry0, ry1))

    for motor, channel, points in series:
        color = colors.get(motor, "#475569")
        for load, force_n, abs_load, tension_mag in points[:: max(1, len(points) // 1600)]:
            px, py = scale(left, (lx0, lx1), (ly0, ly1), load, force_n)
            elements.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="1.35" fill="{color}" opacity="0.35"/>')
            px, py = scale(right, (rx0, rx1), (ry0, ry1), abs_load, tension_mag)
            elements.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="1.35" fill="{color}" opacity="0.35"/>')

    legend_x = 72
    legend_y = 520
    for motor, channel, _points in series:
        color = colors.get(motor, "#475569")
        elements.append(f'<rect x="{legend_x}" y="{legend_y - 10}" width="16" height="4" fill="{color}"/>')
        elements.append(f'<text class="legend" x="{legend_x + 22}" y="{legend_y - 4}">M{motor:02d} / CH{channel}</text>')
        legend_x += 126
    elements.append("</svg>")
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--run-data", type=Path, default=Path("run_data"))
    args = parser.parse_args()

    source = args.csv or find_latest_training_csv(args.run_data)
    with open(source, "r", newline="", encoding="utf-8-sig") as fp:
        reader = csv.DictReader(fp)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    if not rows:
        raise ValueError(f"{source} has no rows")

    force_cols = [f"force_ch{channel}_n" for channel in sorted(CHANNEL_MOTOR)]
    missing = [col for col in force_cols if col not in fieldnames]
    if missing:
        raise ValueError(f"missing force columns: {missing}")

    kept: list[dict[str, str]] = []
    removed = 0
    positive_counts = {col: 0 for col in force_cols}
    for row in rows:
        positives = []
        for col in force_cols:
            value = to_float(row.get(col))
            is_positive = math.isfinite(value) and value > 0
            positives.append(is_positive)
            if is_positive:
                positive_counts[col] += 1
        if any(positives):
            removed += 1
        else:
            kept.append(row)

    stem = source.with_suffix("")
    filtered_path = stem.with_name(stem.name + "_nonpositive_force.csv")
    with open(filtered_path, "w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(kept)

    summaries = [
        summarize_pair(kept, motor=motor, channel=channel)
        for channel, motor in sorted(CHANNEL_MOTOR.items(), key=lambda item: item[1])
    ]
    summary_path = stem.with_name(stem.name + "_force_load_summary.csv")
    summary_fields = list(summaries[0].keys())
    with open(summary_path, "w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summaries)

    report_path = stem.with_name(stem.name + "_force_load_report.md")
    counts = {
        "raw_rows": len(rows),
        "kept_rows": len(kept),
        "removed_rows": removed,
    }
    write_summary_markdown(report_path, source, filtered_path, counts, summaries)
    svg_path = stem.with_name(stem.name + "_force_load_scatter.svg")
    write_scatter_svg(svg_path, kept)

    print(f"source={source}")
    print(f"filtered={filtered_path}")
    print(f"summary={summary_path}")
    print(f"report={report_path}")
    print(f"scatter_svg={svg_path}")
    print(f"raw_rows={len(rows)} kept_rows={len(kept)} removed_rows={removed}")
    print("positive_counts=" + ", ".join(f"{key}:{value}" for key, value in positive_counts.items()))
    for item in summaries:
        print(
            "{motor} {channel}: n={n}, force={fmin:.2f}..{fmax:.2f}N, load={lmin:.0f}..{lmax:.0f}, "
            "corr(force,load)={corr:.3f}, corr(-force,absload)={corr_abs:.3f}, r2={r2:.3f}".format(
                motor=item["motor"],
                channel=item["channel"],
                n=item["n"],
                fmin=item["force_min_n"],
                fmax=item["force_max_n"],
                lmin=item["load_min"],
                lmax=item["load_max"],
                corr=item["corr_force_load"],
                corr_abs=item["corr_tension_absload"],
                r2=item["fit_force_vs_load_r2"],
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
