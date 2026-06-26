#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics
import sys


def to_float(value, default=math.nan):
    try:
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def load_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def series(rows, name):
    return [to_float(row.get(name, "")) for row in rows]


def finite_pairs(x_vals, y_vals):
    out = []
    for x, y in zip(x_vals, y_vals):
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


def crop_to_step_start(rows, step_joint=3, step_target=10.0, pre_step_sec=5.0):
    prev_target = math.nan
    step_idx = None
    target_field = f"j{step_joint}_target"
    for idx, row in enumerate(rows):
        target = to_float(row.get(target_field, ""))
        if math.isfinite(target) and math.isclose(target, step_target, abs_tol=1e-6):
            if not math.isfinite(prev_target) or not math.isclose(prev_target, step_target, abs_tol=1e-6):
                step_idx = idx
                break
        if math.isfinite(target):
            prev_target = target
    if step_idx is None:
        return rows, None

    step_t = to_float(rows[step_idx].get("t_rel", ""))
    crop_idx = step_idx
    if math.isfinite(step_t) and pre_step_sec > 0.0:
        crop_start_t = step_t - pre_step_sec
        for idx, row in enumerate(rows):
            t = to_float(row.get("t_rel", ""))
            if math.isfinite(t) and t >= crop_start_t:
                crop_idx = idx
                break

    cropped = [dict(row) for row in rows[crop_idx:]]
    t0 = step_t
    if math.isfinite(t0):
        for row in cropped:
            t = to_float(row.get("t_rel", ""))
            if math.isfinite(t):
                row["t_rel"] = f"{t - t0:.6f}"
    return cropped, t0


def nice_bounds(values, pad=0.08):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return -1.0, 1.0
    lo, hi = min(vals), max(vals)
    if lo == hi:
        span = max(1.0, abs(lo) * 0.2)
        return lo - span, hi + span
    span = hi - lo
    return lo - span * pad, hi + span * pad


def panel_values(rows, panels):
    values = []
    for panel in panels:
        for item in panel["items"]:
            values.extend(series(rows, item["field"]))
        for hline in panel.get("hlines", []):
            values.append(float(hline["y"]))
    return values


def apply_shared_y_bounds(rows, panels):
    y_min, y_max = nice_bounds(panel_values(rows, panels))
    for panel in panels:
        panel["y_min"] = y_min
        panel["y_max"] = y_max


def polyline(points, x_min, x_max, y_min, y_max, left, top, width, height):
    if not points:
        return ""
    dx = x_max - x_min if x_max != x_min else 1.0
    dy = y_max - y_min if y_max != y_min else 1.0
    coords = []
    for x, y in points:
        px = left + (x - x_min) / dx * width
        py = top + height - (y - y_min) / dy * height
        coords.append(f"{px:.1f},{py:.1f}")
    return " ".join(coords)


def write_svg(path, title, rows, panels):
    x_vals = series(rows, "t_rel")
    x_finite = [x for x in x_vals if math.isfinite(x)]
    if not x_finite:
        x_finite = [0.0, 1.0]
    x_min, x_max = min(x_finite), max(x_finite)
    if x_min == x_max:
        x_max = x_min + 1.0

    panel_h = 190
    left = 72
    right = 24
    top_margin = 48
    bottom = 42
    width = 1180
    height = top_margin + panel_h * len(panels) + bottom
    plot_w = width - left - right

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{left}" y="26" font-family="Arial" font-size="18" font-weight="700">{title}</text>',
    ]

    for panel_idx, panel in enumerate(panels):
        panel_top = top_margin + panel_idx * panel_h
        plot_h = panel_h - 46
        y_values = []
        for item in panel["items"]:
            y_values.extend(series(rows, item["field"]))
        for hline in panel.get("hlines", []):
            y_values.append(float(hline["y"]))
        y_min = panel.get("y_min")
        y_max = panel.get("y_max")
        if y_min is None or y_max is None:
            y_min, y_max = nice_bounds(y_values)

        parts.append(f'<rect x="{left}" y="{panel_top}" width="{plot_w}" height="{plot_h}" fill="#fafafa" stroke="#ddd"/>')
        parts.append(f'<text x="10" y="{panel_top + 18}" font-family="Arial" font-size="13">{panel["label"]}</text>')

        ticks = panel.get("y_ticks")
        if ticks is None:
            ticks = [y_max - (y_max - y_min) * tick_idx / 4.0 for tick_idx in range(5)]
        for y_val in ticks:
            if y_max == y_min:
                ratio = 0.0
            else:
                ratio = (y_max - y_val) / (y_max - y_min)
            y_px = panel_top + plot_h * ratio
            parts.append(f'<line x1="{left}" y1="{y_px:.1f}" x2="{left + plot_w}" y2="{y_px:.1f}" stroke="#e7e7e7" stroke-width="0.7"/>')
            parts.append(f'<text x="{left - 6}" y="{y_px + 3:.1f}" text-anchor="end" font-family="Arial" font-size="10">{y_val:.2f}</text>')

        zero_points = finite_pairs(x_vals, [0.0] * len(rows))
        if y_min <= 0.0 <= y_max:
            pts = polyline(zero_points, x_min, x_max, y_min, y_max, left, panel_top, plot_w, plot_h)
            parts.append(f'<polyline points="{pts}" fill="none" stroke="#888" stroke-width="0.7" opacity="0.55"/>')

        for hline in panel.get("hlines", []):
            pts = polyline(finite_pairs(x_vals, [float(hline["y"])] * len(rows)), x_min, x_max, y_min, y_max, left, panel_top, plot_w, plot_h)
            parts.append(f'<polyline points="{pts}" fill="none" stroke="{hline.get("color", "#d62728")}" stroke-width="0.8" stroke-dasharray="5 4"/>')

        legend_x = left
        for item_idx, item in enumerate(panel["items"]):
            pts = polyline(finite_pairs(x_vals, series(rows, item["field"])), x_min, x_max, y_min, y_max, left, panel_top, plot_w, plot_h)
            color = item.get("color", colors[item_idx % len(colors)])
            parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.4"/>')
            parts.append(f'<text x="{legend_x}" y="{panel_top + plot_h + 22}" font-family="Arial" font-size="11" fill="{color}">{item["name"]}</text>')
            legend_x += 92

    parts.append(f'<text x="{left}" y="{height - 12}" font-family="Arial" font-size="12">time (s), {x_min:.2f} to {x_max:.2f}</text>')
    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def write_summary(rows, out_path):
    lines = [f"rows: {len(rows)}"]
    t = series(rows, "t_rel")
    t = [v for v in t if math.isfinite(v)]
    if t:
        lines.append(f"duration_s: {max(t) - min(t):.3f}")
    for j in range(4):
        err = [v for v in series(rows, f"j{j}_error") if math.isfinite(v)]
        if err:
            lines.append(f"J{j} error mean={statistics.fmean(err):.3f} max_abs={max(abs(v) for v in err):.3f} final={err[-1]:.3f}")
    for m in range(5):
        load = [v for v in series(rows, f"m{m}_load") if math.isfinite(v)]
        if load:
            lines.append(f"M{m} load max_abs={max(abs(v) for v in load):.0f} over160={sum(abs(v) > 160 for v in load)} over180={sum(abs(v) > 180 for v in load)}")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Plot MCP5 control CSV to SVG files without third-party packages.")
    parser.add_argument("csv_path")
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--no-step-crop", action="store_true", help="Plot the full record instead of cropping to the first J3 0->10 step.")
    parser.add_argument("--step-joint", type=int, default=3)
    parser.add_argument("--step-target", type=float, default=10.0)
    parser.add_argument("--pre-step-sec", type=float, default=5.0, help="Seconds to keep before the detected step while using step-cropped plots.")
    parser.add_argument("--joint-y-10", action="store_true", help="Use fixed -10..10 y-axis ticks for joint plots.")
    args = parser.parse_args()

    rows = load_csv(args.csv_path)
    if not rows:
        print("CSV is empty.")
        return 1
    step_t0 = None
    if not args.no_step_crop:
        rows, step_t0 = crop_to_step_start(rows, args.step_joint, args.step_target, args.pre_step_sec)
        if step_t0 is None:
            print("No step start found; plotting full record.")
    out_dir = args.out_dir or os.path.splitext(args.csv_path)[0] + "_plots"
    os.makedirs(out_dir, exist_ok=True)

    joint_panels = []
    for j in range(4):
        joint_panels.append({
            "label": f"J{j}",
            "items": [
                {"field": f"j{j}_target", "name": "target", "color": "#1f77b4"},
                {"field": f"j{j}_actual", "name": "actual", "color": "#d62728"},
                {"field": f"j{j}_error", "name": "error", "color": "#2ca02c"},
            ],
        })
    if args.joint_y_10:
        for panel in joint_panels:
            panel["y_min"] = -10.0
            panel["y_max"] = 10.0
            panel["y_ticks"] = [-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
    else:
        apply_shared_y_bounds(rows, joint_panels)
    title_suffix = "" if step_t0 is None else f" (t=0 at J{args.step_joint}->{args.step_target:g} deg step)"
    write_svg(os.path.join(out_dir, "joints.svg"), "MCP Joint Targets / Actual / Error" + title_suffix, rows, joint_panels)

    motor_panels = []
    for m in range(5):
        motor_panels.append({
            "label": f"M{m}",
            "items": [
                {"field": f"m{m}_solver", "name": "solver", "color": "#1f77b4"},
                {"field": f"m{m}_cmd", "name": "cmd", "color": "#d62728"},
                {"field": f"m{m}_now", "name": "now", "color": "#2ca02c"},
                {"field": f"m{m}_bias", "name": "bias", "color": "#9467bd"},
            ],
            "hlines": [{"y": 6400, "color": "#cc0000"}, {"y": -6400, "color": "#cc0000"}],
        })
    apply_shared_y_bounds(rows, motor_panels)
    write_svg(os.path.join(out_dir, "motors.svg"), "MCP Motor Solver / Cmd / Now / Bias" + title_suffix, rows, motor_panels)

    load_panel = [{
        "label": "load",
        "items": [{"field": f"m{m}_load", "name": f"M{m}", "color": c} for m, c in enumerate(["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"])],
        "hlines": [{"y": 160, "color": "#ff9900"}, {"y": -160, "color": "#ff9900"}, {"y": 180, "color": "#cc0000"}, {"y": -180, "color": "#cc0000"}],
    }]
    write_svg(os.path.join(out_dir, "loads.svg"), "MCP Loads" + title_suffix, rows, load_panel)
    write_summary(rows, os.path.join(out_dir, "summary.txt"))
    print(f"Saved SVG plots to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
