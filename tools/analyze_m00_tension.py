from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
M00_TENSION_BIAS_LIMIT_COUNTS = 7200


def latest_training_csv() -> Path:
    files = sorted(
        ROOT.joinpath("run_data").glob("training_collect*.csv"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    files = [p for p in files if "summary" not in p.name and "prediction" not in p.name]
    if not files:
        raise FileNotFoundError("No training_collect*.csv file found in run_data")
    return files[0]


def load_font(size: int) -> ImageFont.ImageFont:
    for name in ("arial.ttf", "segoeui.ttf", "calibri.ttf"):
        path = Path("C:/Windows/Fonts") / name
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def longest_true_run(mask: np.ndarray, t: np.ndarray) -> tuple[int, float, float, float]:
    best_len = 0
    best_start = 0
    best_end = -1
    cur_start = None
    for i, value in enumerate(mask):
        if value and cur_start is None:
            cur_start = i
        if (not value or i == len(mask) - 1) and cur_start is not None:
            cur_end = i if value and i == len(mask) - 1 else i - 1
            cur_len = cur_end - cur_start + 1
            if cur_len > best_len:
                best_len = cur_len
                best_start = cur_start
                best_end = cur_end
            cur_start = None
    if best_len == 0:
        return 0, 0.0, 0.0, 0.0
    return best_len, float(t[best_start]), float(t[best_end]), float(t[best_end] - t[best_start])


def plot_m00(df: pd.DataFrame, out_png: Path) -> None:
    t = pd.to_numeric(df["elapsed_s"], errors="coerce").to_numpy(dtype=float)
    tension = pd.to_numeric(df["tension_m00_n"], errors="coerce").to_numpy(dtype=float)
    bias = pd.to_numeric(df["tension_bias_m00_counts"], errors="coerce").to_numpy(dtype=float)
    pos = pd.to_numeric(df["servo_m00_abs"], errors="coerce").to_numpy(dtype=float)

    width, height = 1800, 980
    img = Image.new("RGB", (width, height), "#f8fafc")
    draw = ImageDraw.Draw(img)
    title_font = load_font(42)
    font = load_font(24)
    small = load_font(19)

    left, right = 105, 80
    top, bottom = 125, 120
    plot_w = width - left - right
    plot_h = height - top - bottom

    t_min, t_max = np.nanmin(t), np.nanmax(t)
    y_min = min(-60.0, np.nanmin(tension) - 5.0)
    y_max = max(5.0, np.nanmax(tension) + 5.0)

    def x_of(v: float) -> float:
        return left + (v - t_min) / max(1e-9, t_max - t_min) * plot_w

    def y_of(v: float) -> float:
        return top + (y_max - v) / max(1e-9, y_max - y_min) * plot_h

    draw.text((55, 38), "M00 Tension and Host Tension Bias", fill="#0f172a", font=title_font)
    draw.text(
        (58, 90),
        f"M00 tension should stay <= -10 N. Current bias limit for M00 is +/-{M00_TENSION_BIAS_LIMIT_COUNTS} counts.",
        fill="#475569",
        font=font,
    )

    for i in range(8):
        y = top + plot_h * i / 7
        value = y_max - (y_max - y_min) * i / 7
        draw.line((left, y, width - right, y), fill="#dbe3ee", width=1)
        draw.text((24, y - 12), f"{value:5.1f}", fill="#64748b", font=small)
    for i in range(7):
        x = left + plot_w * i / 6
        value = t_min + (t_max - t_min) * i / 6
        draw.line((x, top, x, height - bottom), fill="#e2e8f0", width=1)
        draw.text((x - 32, height - bottom + 20), f"{value:.0f}", fill="#64748b", font=small)
    draw.rectangle((left, top, width - right, height - bottom), outline="#94a3b8", width=2)
    draw.text((width / 2 - 70, height - 56), "elapsed time (s)", fill="#334155", font=font)
    draw.text((24, top - 35), "N", fill="#334155", font=font)

    # Loose regions.
    loose = tension > -10.0
    if loose.any():
        start = None
        for i, value in enumerate(loose):
            if value and start is None:
                start = i
            if (not value or i == len(loose) - 1) and start is not None:
                end = i if value and i == len(loose) - 1 else i - 1
                draw.rectangle((x_of(t[start]), top, x_of(t[end]), height - bottom), fill="#fee2e2")
                start = None

    # Threshold.
    y_thr = y_of(-10.0)
    draw.line((left, y_thr, width - right, y_thr), fill="#ef4444", width=3)
    draw.text((width - right - 190, y_thr - 30), "-10 N loose threshold", fill="#b91c1c", font=small)

    # Tension line.
    tension_points = [(x_of(float(tt)), y_of(float(v))) for tt, v in zip(t, tension) if np.isfinite(tt) and np.isfinite(v)]
    if len(tension_points) >= 2:
        draw.line(tension_points, fill="#2563eb", width=4)

    # Bias line scaled to right-side overlay.
    b_min, b_max = 0.0, float(M00_TENSION_BIAS_LIMIT_COUNTS)
    def yb_of(v: float) -> float:
        return height - bottom - (v - b_min) / max(1e-9, b_max - b_min) * plot_h

    bias_points = [(x_of(float(tt)), yb_of(float(v))) for tt, v in zip(t, bias) if np.isfinite(tt) and np.isfinite(v)]
    if len(bias_points) >= 2:
        draw.line(bias_points, fill="#f97316", width=3)
    draw.text((width - right - 220, top + 25), "orange: M00 bias counts", fill="#c2410c", font=small)
    draw.text((width - right - 220, top + 55), "blue: M00 tension N", fill="#1d4ed8", font=small)

    # Position hint at bottom as gray min/max text.
    draw.text(
        (left, height - 88),
        f"M00 abs pos: {np.nanmin(pos):.0f}..{np.nanmax(pos):.0f} counts; bias: {np.nanmin(bias):.0f}..{np.nanmax(bias):.0f} counts",
        fill="#475569",
        font=small,
    )

    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png, quality=95)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze M00 tension behavior in a training CSV.")
    parser.add_argument("csv_path", nargs="?", type=Path, default=None)
    args = parser.parse_args()
    csv_path = args.csv_path.resolve() if args.csv_path else latest_training_csv()
    df = pd.read_csv(csv_path, low_memory=False)

    out_dir = csv_path.parent / f"analysis_{csv_path.stem}_m00_tension"
    out_dir.mkdir(parents=True, exist_ok=True)

    t = pd.to_numeric(df["elapsed_s"], errors="coerce")
    m00 = pd.to_numeric(df["tension_m00_n"], errors="coerce")
    bias = pd.to_numeric(df["tension_bias_m00_counts"], errors="coerce")
    action_count = pd.to_numeric(df.get("tension_action_count", pd.Series([0] * len(df))), errors="coerce").fillna(0)
    window_ok = pd.to_numeric(df.get("tension_window_ok", pd.Series([1] * len(df))), errors="coerce").fillna(1)
    pos = pd.to_numeric(df["servo_m00_abs"], errors="coerce")
    load = pd.to_numeric(df["servo_m00_load"], errors="coerce")

    loose = (m00 > -10.0).to_numpy()
    very_loose = (m00 > -5.0).to_numpy()
    maxed = (bias >= M00_TENSION_BIAS_LIMIT_COUNTS).to_numpy()
    longest = longest_true_run(loose, t.to_numpy(dtype=float))

    action_text = df.get("tension_action", pd.Series([""] * len(df))).fillna("").astype(str)
    m00_actions = action_text.str.contains("M00|m00", regex=True, na=False)

    summary = {
        "csv": str(csv_path),
        "rows": int(len(df)),
        "duration_s": float(t.max()),
        "m00_min_n": float(m00.min()),
        "m00_median_n": float(m00.median()),
        "m00_p95_n": float(m00.quantile(0.95)),
        "m00_max_n": float(m00.max()),
        "m00_loose_rows_gt_minus_10": int(loose.sum()),
        "m00_loose_ratio_gt_minus_10": float(loose.mean()),
        "m00_very_loose_rows_gt_minus_5": int(very_loose.sum()),
        "m00_bias_min": float(bias.min()),
        "m00_bias_max": float(bias.max()),
        "m00_bias_at_limit_rows": int(maxed.sum()),
        "m00_bias_at_limit_ratio": float(maxed.mean()),
        "m00_bias_unique_count": int(bias.nunique(dropna=True)),
        "rows_with_any_tension_action": int((action_count > 0).sum()),
        "rows_with_m00_action_text": int(m00_actions.sum()),
        "tension_window_not_ok_rows": int((window_ok == 0).sum()),
        "m00_abs_min": float(pos.min()),
        "m00_abs_max": float(pos.max()),
        "m00_load_min": float(load.min()),
        "m00_load_max": float(load.max()),
        "longest_loose_run_rows": int(longest[0]),
        "longest_loose_run_start_s": float(longest[1]),
        "longest_loose_run_end_s": float(longest[2]),
        "longest_loose_run_duration_s": float(longest[3]),
        "first_row_m00_n": float(m00.iloc[0]),
        "first_row_m00_bias": float(bias.iloc[0]),
        "last_row_m00_n": float(m00.iloc[-1]),
        "last_row_m00_bias": float(bias.iloc[-1]),
    }

    pd.Series(summary).to_csv(out_dir / "m00_tension_summary.csv", header=False)
    plot_m00(df, out_dir / "m00_tension_bias_plot.png")

    # Save the loosest rows for audit.
    columns = [
        "local_time",
        "elapsed_s",
        "target_phase",
        "target_joint",
        "target_command",
        "tension_m00_n",
        "tension_bias_m00_counts",
        "tension_action",
        "tension_action_count",
        "servo_m00_abs",
        "servo_m00_speed",
        "servo_m00_load",
        "joint_j00_deg_rel",
        "joint_j01_deg_rel",
        "joint_j02_deg_rel",
        "joint_j03_deg_rel",
    ]
    present = [col for col in columns if col in df.columns]
    df.loc[m00.sort_values(ascending=False).head(120).index, present].to_csv(
        out_dir / "m00_loosest_rows.csv",
        index=False,
    )

    print(f"csv={csv_path}")
    print(f"out_dir={out_dir}")
    for key, value in summary.items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()
