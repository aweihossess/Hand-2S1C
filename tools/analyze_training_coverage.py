from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont


@dataclass(frozen=True)
class AxisSpec:
    name: str
    label: str
    unit: str


JOINT_AXES = [
    AxisSpec("joint_j00_deg_rel", "J00 rel", "deg"),
    AxisSpec("joint_j01_deg_rel", "J01 rel", "deg"),
    AxisSpec("joint_j02_deg_rel", "J02 rel", "deg"),
    AxisSpec("joint_j03_deg_rel", "J03 rel", "deg"),
]

FORCE_AXES = [
    AxisSpec("tension_m00_n", "M00/CH2", "N"),
    AxisSpec("tension_m01_n", "M01/CH1", "N"),
    AxisSpec("tension_m02_n", "M02/CH3", "N"),
    AxisSpec("tension_m03_n", "M03/CH4", "N"),
    AxisSpec("tension_m04_n", "M04/CH5", "N"),
]

SERVO_FIELDS = ["abs", "speed", "load", "zero_offset", "raw"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze training dataset coverage.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument(
        "--plot-j2-j3-j4",
        action="store_true",
        help="Also write an alias plot using 1-based labels J2/J3/J4 for data columns J01/J02/J03.",
    )
    return parser.parse_args()


def numeric_frame(df: pd.DataFrame, columns: Iterable[str]) -> pd.DataFrame:
    present = [col for col in columns if col in df.columns]
    return df[present].apply(pd.to_numeric, errors="coerce")


def stats_table(df: pd.DataFrame, columns: list[AxisSpec]) -> pd.DataFrame:
    data = numeric_frame(df, [axis.name for axis in columns])
    rows = []
    for axis in columns:
        if axis.name not in data:
            continue
        s = data[axis.name].dropna()
        if s.empty:
            continue
        rows.append(
            {
                "field": axis.name,
                "label": axis.label,
                "unit": axis.unit,
                "count": int(s.count()),
                "min": float(s.min()),
                "p01": float(s.quantile(0.01)),
                "p05": float(s.quantile(0.05)),
                "median": float(s.quantile(0.50)),
                "p95": float(s.quantile(0.95)),
                "p99": float(s.quantile(0.99)),
                "max": float(s.max()),
                "span": float(s.max() - s.min()),
            }
        )
    return pd.DataFrame(rows)


def servo_stats(df: pd.DataFrame) -> pd.DataFrame:
    axes = []
    for motor in range(5):
        for field in SERVO_FIELDS:
            name = f"servo_m{motor:02d}_{field}"
            if name in df.columns:
                unit = "counts" if field in {"abs", "zero_offset", "raw"} else ""
                axes.append(AxisSpec(name, f"M{motor:02d} {field}", unit))
    return stats_table(df, axes)


def occupancy_summary(df: pd.DataFrame) -> pd.DataFrame:
    columns = ["joint_j01_deg_rel", "joint_j02_deg_rel", "joint_j03_deg_rel"]
    data = numeric_frame(df, columns).dropna()
    rows = []
    if data.empty:
        return pd.DataFrame(rows)

    for step in (5.0, 10.0, 15.0):
        mins = data.min()
        maxs = data.max()
        bins = [
            np.arange(math.floor(mins[col] / step) * step, math.ceil(maxs[col] / step) * step + step, step)
            for col in columns
        ]
        ok = np.ones(len(data), dtype=bool)
        digitized = []
        for col, edges in zip(columns, bins):
            idx = np.digitize(data[col].to_numpy(dtype=float), edges) - 1
            ok &= (idx >= 0) & (idx < len(edges) - 1)
            digitized.append(idx)
        occupied = len(set(zip(digitized[0][ok], digitized[1][ok], digitized[2][ok])))
        total = int(np.prod([len(edges) - 1 for edges in bins]))
        rows.append(
            {
                "axes": "J01/J02/J03",
                "bin_deg": step,
                "occupied_bins": occupied,
                "total_bins_in_range_box": total,
                "occupancy_ratio": occupied / total if total else 0.0,
            }
        )
    return pd.DataFrame(rows)


def target_actual_summary(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for joint in range(4):
        target_col = f"target_j{joint:02d}_relative_deg"
        actual_col = f"joint_j{joint:02d}_deg_rel"
        if target_col not in df.columns or actual_col not in df.columns:
            continue
        target = pd.to_numeric(df[target_col], errors="coerce")
        actual = pd.to_numeric(df[actual_col], errors="coerce")
        mask = target.notna() & actual.notna()
        if not mask.any():
            continue
        rows.append(
            {
                "joint": f"J{joint:02d}",
                "target_min": float(target[mask].min()),
                "target_max": float(target[mask].max()),
                "actual_min": float(actual[mask].min()),
                "actual_max": float(actual[mask].max()),
                "corr_target_actual": float(actual[mask].corr(target[mask])),
            }
        )
    return pd.DataFrame(rows)


def force_quality_summary(df: pd.DataFrame) -> pd.DataFrame:
    data = numeric_frame(df, [axis.name for axis in FORCE_AXES])
    rows = []
    for axis in FORCE_AXES:
        if axis.name not in data:
            continue
        s = data[axis.name].dropna()
        if s.empty:
            continue
        rows.append(
            {
                "label": axis.label,
                "rows": int(s.count()),
                "positive_rows": int((s > 0).sum()),
                "above_minus_10n_rows": int((s > -10.0).sum()),
                "below_minus_150n_rows": int((s < -150.0).sum()),
                "within_minus_150_to_minus_10_ratio": float(((s >= -150.0) & (s <= -10.0)).mean()),
            }
        )
    return pd.DataFrame(rows)


def monotonic_hull(points: np.ndarray) -> np.ndarray:
    unique = sorted({(float(x), float(y)) for x, y in points})
    if len(unique) <= 1:
        return np.asarray(unique, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return np.asarray(lower[:-1] + upper[:-1], dtype=float)


def load_font(size: int) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeui.ttf"),
        Path("C:/Windows/Fonts/calibri.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def project_points(points: np.ndarray, mins: np.ndarray, maxs: np.ndarray, width: int, height: int) -> np.ndarray:
    span = np.maximum(maxs - mins, 1e-9)
    normalized = (points - mins) / span - 0.5
    x = normalized[:, 0]
    y = normalized[:, 1]
    z = normalized[:, 2]
    angle = math.radians(38)
    u = (x - y) * math.cos(angle)
    v = z + (x + y) * math.sin(angle) * 0.58
    scale = min(width, height) * 0.72
    px = width * 0.52 + u * scale
    py = height * 0.55 - v * scale
    return np.column_stack([px, py])


def color_from_value(value: float, lo: float, hi: float) -> tuple[int, int, int, int]:
    if hi <= lo:
        t = 0.5
    else:
        t = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    r = int(39 + 184 * t)
    g = int(118 - 70 * t)
    b = int(220 - 120 * t)
    return r, g, b, 160


def draw_poly(draw: ImageDraw.ImageDraw, pts: np.ndarray, fill: tuple[int, int, int, int], outline: tuple[int, int, int, int]) -> None:
    if len(pts) < 3:
        return
    xy = [(float(x), float(y)) for x, y in pts]
    draw.polygon(xy, fill=fill)
    draw.line(xy + [xy[0]], fill=outline, width=2)


def draw_3d_envelope(
    df: pd.DataFrame,
    out_png: Path,
    axis_cols: list[str] | None = None,
    axis_labels: list[str] | None = None,
    color_col: str = "joint_j00_deg_rel",
    color_label: str = "J00 rel deg",
    title: str = "Measured Joint Coverage Envelope",
    subtitle: str = "3D axes: J01/J02/J03 relative angle; point color: J00 relative angle",
) -> dict[str, float | int | str]:
    if axis_cols is None:
        axis_cols = ["joint_j01_deg_rel", "joint_j02_deg_rel", "joint_j03_deg_rel"]
    if axis_labels is None:
        axis_labels = ["J01 rel deg", "J02 rel deg", "J03 rel deg"]
    valid_cols = ["joint_j00_valid", "joint_j01_valid", "joint_j02_valid", "joint_j03_valid"]

    work = df.copy()
    for col in axis_cols + [color_col] + valid_cols:
        if col in work.columns:
            work[col] = pd.to_numeric(work[col], errors="coerce")
    mask = np.ones(len(work), dtype=bool)
    for col in valid_cols:
        if col in work.columns:
            mask &= work[col].fillna(0).astype(float).to_numpy() > 0
    for col in axis_cols + [color_col]:
        mask &= work[col].notna().to_numpy()
    pts4 = work.loc[mask, axis_cols + [color_col]].to_numpy(dtype=float)
    if len(pts4) == 0:
        raise RuntimeError("No valid joint rows found for 3D envelope.")

    # Downsample deterministically for scatter clarity.
    if len(pts4) > 2500:
        idx = np.linspace(0, len(pts4) - 1, 2500).astype(int)
        scatter = pts4[idx]
    else:
        scatter = pts4

    xyz = pts4[:, :3]
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    pad = (maxs - mins) * 0.08
    mins_p = mins - pad
    maxs_p = maxs + pad

    width, height = 1800, 1280
    scale = 2
    img = Image.new("RGBA", (width * scale, height * scale), (248, 250, 252, 255))
    draw = ImageDraw.Draw(img, "RGBA")
    font_title = load_font(42 * scale)
    font = load_font(24 * scale)
    font_small = load_font(18 * scale)

    def proj(p: np.ndarray) -> np.ndarray:
        return project_points(p.reshape(-1, 3), mins_p, maxs_p, width * scale, height * scale)

    # Bounding box.
    corners = np.array(
        [
            [mins[0], mins[1], mins[2]],
            [maxs[0], mins[1], mins[2]],
            [maxs[0], maxs[1], mins[2]],
            [mins[0], maxs[1], mins[2]],
            [mins[0], mins[1], maxs[2]],
            [maxs[0], mins[1], maxs[2]],
            [maxs[0], maxs[1], maxs[2]],
            [mins[0], maxs[1], maxs[2]],
        ]
    )
    pc = proj(corners)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    for a, b in edges:
        draw.line([tuple(pc[a]), tuple(pc[b])], fill=(148, 163, 184, 115), width=2 * scale)

    # Layered x/y hulls across z bins. This is a practical envelope surface for sparse measured data.
    z = xyz[:, 2]
    bins = np.linspace(z.min(), z.max(), 14)
    for i in range(len(bins) - 1):
        low, high = bins[i], bins[i + 1]
        if i == len(bins) - 2:
            layer_mask = (z >= low) & (z <= high)
        else:
            layer_mask = (z >= low) & (z < high)
        layer = xyz[layer_mask]
        if len(layer) < 8:
            continue
        hull2 = monotonic_hull(layer[:, :2])
        if len(hull2) < 3:
            continue
        z_mid = float(np.median(layer[:, 2]))
        hull3 = np.column_stack([hull2[:, 0], hull2[:, 1], np.full(len(hull2), z_mid)])
        ph = proj(hull3)
        fill = (59, 130, 246, 32 + int(75 * i / max(1, len(bins) - 2)))
        outline = (37, 99, 235, 65)
        draw_poly(draw, ph, fill, outline)

    # Scatter points, colored by J00.
    scatter_xyz = scatter[:, :3]
    scatter_j00 = scatter[:, 3]
    pp = proj(scatter_xyz)
    cmin = float(np.nanmin(pts4[:, 3]))
    cmax = float(np.nanmax(pts4[:, 3]))
    order = np.argsort(scatter_xyz[:, 2])
    radius = 3 * scale
    for idx in order:
        x2, y2 = pp[idx]
        col = color_from_value(float(scatter_j00[idx]), cmin, cmax)
        draw.ellipse((x2 - radius, y2 - radius, x2 + radius, y2 + radius), fill=col)

    draw.text((60 * scale, 42 * scale), title, font=font_title, fill=(15, 23, 42, 255))
    draw.text((64 * scale, 96 * scale), subtitle, font=font, fill=(71, 85, 105, 255))

    labels = [
        (axis_labels[0], np.array([maxs[0], mins[1], mins[2]])),
        (axis_labels[1], np.array([mins[0], maxs[1], mins[2]])),
        (axis_labels[2], np.array([mins[0], mins[1], maxs[2]])),
    ]
    for text, point in labels:
        x2, y2 = proj(point)[0]
        draw.text((x2 + 12 * scale, y2 - 8 * scale), text, font=font, fill=(15, 23, 42, 255))

    stats_lines = [
        f"Rows plotted: {len(pts4):,}",
        f"{color_label} range: {cmin:.2f} to {cmax:.2f}",
        f"{axis_labels[0]} range: {mins[0]:.2f} to {maxs[0]:.2f}",
        f"{axis_labels[1]} range: {mins[1]:.2f} to {maxs[1]:.2f}",
        f"{axis_labels[2]} range: {mins[2]:.2f} to {maxs[2]:.2f}",
    ]
    y = 1040 * scale
    for line in stats_lines:
        draw.text((62 * scale, y), line, font=font_small, fill=(51, 65, 85, 255))
        y += 30 * scale

    # Simple color legend.
    lx, ly, lw, lh = 1370 * scale, 1040 * scale, 300 * scale, 22 * scale
    for i in range(lw):
        value = cmin + (cmax - cmin) * i / max(1, lw - 1)
        draw.line([(lx + i, ly), (lx + i, ly + lh)], fill=color_from_value(value, cmin, cmax))
    draw.rectangle((lx, ly, lx + lw, ly + lh), outline=(100, 116, 139, 255), width=2)
    draw.text((lx, ly - 30 * scale), color_label, font=font_small, fill=(51, 65, 85, 255))
    draw.text((lx, ly + 30 * scale), f"{cmin:.1f}", font=font_small, fill=(51, 65, 85, 255))
    draw.text((lx + lw - 60 * scale, ly + 30 * scale), f"{cmax:.1f}", font=font_small, fill=(51, 65, 85, 255))

    img = img.convert("RGB").resize((width, height), Image.Resampling.LANCZOS)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png, quality=95)

    return {
        "valid_joint_rows": int(len(pts4)),
        "j00_min": cmin,
        "j00_max": cmax,
        "axis0_min": float(mins[0]),
        "axis0_max": float(maxs[0]),
        "axis1_min": float(mins[1]),
        "axis1_max": float(maxs[1]),
        "axis2_min": float(mins[2]),
        "axis2_max": float(maxs[2]),
    }


def write_markdown_report(
    out_md: Path,
    csv_path: Path,
    df: pd.DataFrame,
    joint_stats: pd.DataFrame,
    force_stats: pd.DataFrame,
    servo_summary: pd.DataFrame,
    occupancy: pd.DataFrame,
    target_actual: pd.DataFrame,
    force_quality: pd.DataFrame,
    envelope_meta: dict[str, float | int | str],
) -> None:
    duration = float(pd.to_numeric(df.get("elapsed_s", pd.Series(dtype=float)), errors="coerce").max())
    complete_force_ratio = None
    if "force_complete" in df.columns:
        complete_force_ratio = float(pd.to_numeric(df["force_complete"], errors="coerce").fillna(0).mean())
    tension_ok_ratio = None
    if "tension_window_ok" in df.columns:
        tension_ok_ratio = float(pd.to_numeric(df["tension_window_ok"], errors="coerce").fillna(0).mean())

    lines = [
        "# Dataset Coverage Report",
        "",
        f"- Dataset: `{csv_path.name}`",
        f"- Rows: {len(df):,}",
        f"- Duration: {duration:.2f} s",
    ]
    if complete_force_ratio is not None:
        lines.append(f"- Force-complete rows: {complete_force_ratio * 100:.1f}%")
    if tension_ok_ratio is not None:
        lines.append(f"- Tension-window-ok rows: {tension_ok_ratio * 100:.1f}%")
    lines.extend(
        [
            f"- 3D envelope rows: {envelope_meta['valid_joint_rows']:,}",
            "",
            "## Joint Actual Relative Angle Coverage",
            "",
            "| joint | min | p05 | median | p95 | max | span |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in joint_stats.iterrows():
        lines.append(
            f"| {row['label']} | {row['min']:.2f} | {row['p05']:.2f} | {row['median']:.2f} | "
            f"{row['p95']:.2f} | {row['max']:.2f} | {row['span']:.2f} |"
        )

    lines.extend(
        [
            "",
            "## Force Coverage",
            "",
            "| channel | min | p05 | median | p95 | max | span |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in force_stats.iterrows():
        lines.append(
            f"| {row['label']} | {row['min']:.2f} | {row['p05']:.2f} | {row['median']:.2f} | "
            f"{row['p95']:.2f} | {row['max']:.2f} | {row['span']:.2f} |"
        )

    lines.extend(
        [
            "",
            "## Force Window Quality",
            "",
            "| channel | positive rows | rows > -10 N | rows < -150 N | in [-150,-10] ratio |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for _, row in force_quality.iterrows():
        lines.append(
            f"| {row['label']} | {int(row['positive_rows'])} | {int(row['above_minus_10n_rows'])} | "
            f"{int(row['below_minus_150n_rows'])} | {row['within_minus_150_to_minus_10_ratio'] * 100:.1f}% |"
        )

    lines.extend(
        [
            "",
            "## Servo Coverage, M00-M04",
            "",
            "| field | min | p05 | median | p95 | max | span |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in servo_summary.iterrows():
        lines.append(
            f"| {row['label']} | {row['min']:.2f} | {row['p05']:.2f} | {row['median']:.2f} | "
            f"{row['p95']:.2f} | {row['max']:.2f} | {row['span']:.2f} |"
        )

    lines.extend(
        [
            "",
            "## 3D Occupancy",
            "",
            "Occupancy is computed inside the measured J01/J02/J03 range box.",
            "",
            "| axes | bin deg | occupied bins | total bins | ratio |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for _, row in occupancy.iterrows():
        lines.append(
            f"| {row['axes']} | {row['bin_deg']:.0f} | {int(row['occupied_bins'])} | "
            f"{int(row['total_bins_in_range_box'])} | {row['occupancy_ratio'] * 100:.1f}% |"
        )

    lines.extend(
        [
            "",
            "## Target vs Actual Joint Range",
            "",
            "| joint | target min | target max | actual min | actual max | corr |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in target_actual.iterrows():
        lines.append(
            f"| {row['joint']} | {row['target_min']:.2f} | {row['target_max']:.2f} | "
            f"{row['actual_min']:.2f} | {row['actual_max']:.2f} | {row['corr_target_actual']:.3f} |"
        )

    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    csv_path = args.csv_path.resolve()
    if args.out_dir is None:
        stem = csv_path.stem.replace("training_collect_", "analysis_")
        out_dir = csv_path.parent / stem
    else:
        out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(csv_path, low_memory=False)

    joint_stats = stats_table(df, JOINT_AXES)
    force_stats = stats_table(df, FORCE_AXES)
    servo_summary = servo_stats(df)
    occupancy = occupancy_summary(df)
    target_actual = target_actual_summary(df)
    force_quality = force_quality_summary(df)

    joint_stats.to_csv(out_dir / "joint_coverage_stats.csv", index=False, quoting=csv.QUOTE_MINIMAL)
    force_stats.to_csv(out_dir / "force_coverage_stats.csv", index=False, quoting=csv.QUOTE_MINIMAL)
    servo_summary.to_csv(out_dir / "servo_m00_m04_coverage_stats.csv", index=False, quoting=csv.QUOTE_MINIMAL)
    occupancy.to_csv(out_dir / "joint_3d_occupancy.csv", index=False, quoting=csv.QUOTE_MINIMAL)
    target_actual.to_csv(out_dir / "target_actual_range.csv", index=False, quoting=csv.QUOTE_MINIMAL)
    force_quality.to_csv(out_dir / "force_window_quality.csv", index=False, quoting=csv.QUOTE_MINIMAL)

    envelope_png = out_dir / "joint_j01_j02_j03_3d_envelope.png"
    envelope_meta = draw_3d_envelope(df, envelope_png)
    if args.plot_j2_j3_j4:
        draw_3d_envelope(
            df,
            out_dir / "joint_J2_J3_J4_3d_envelope.png",
            axis_cols=["joint_j01_deg_rel", "joint_j02_deg_rel", "joint_j03_deg_rel"],
            axis_labels=["J2 rel deg", "J3 rel deg", "J4 rel deg"],
            color_col="joint_j00_deg_rel",
            color_label="J1 rel deg",
            title="Measured J2/J3/J4 Coverage Envelope",
            subtitle="3D axes: J2/J3/J4 relative angle; point color: J1 relative angle",
        )

    write_markdown_report(
        out_dir / "coverage_report.md",
        csv_path,
        df,
        joint_stats,
        force_stats,
        servo_summary,
        occupancy,
        target_actual,
        force_quality,
        envelope_meta,
    )

    print(f"rows={len(df)}")
    print(f"out_dir={out_dir}")
    print(f"envelope_png={envelope_png}")
    print(joint_stats.to_string(index=False))
    print(force_stats.to_string(index=False))
    print(occupancy.to_string(index=False))
    print(target_actual.to_string(index=False))
    print(force_quality.to_string(index=False))


if __name__ == "__main__":
    main()
