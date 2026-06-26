#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw

from plot_joint_step_response import (
    COLORS,
    JOINTS,
    draw_dashed_line,
    draw_panel,
    draw_polyline,
    font,
    nice_ticks,
    project,
)


def fnum(value, default=math.nan):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except ValueError:
        return default


def load_sine_rows(path, joint):
    rows = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            target = fnum(row.get(f"{joint}_target"))
            if not math.isfinite(target) or target <= 1.0:
                continue
            item = {"t_rel": fnum(row.get("t_rel"))}
            for name in JOINTS:
                item[f"{name}_actual"] = fnum(row.get(f"{name}_actual"))
                item[f"{name}_target"] = fnum(row.get(f"{name}_target"))
                item[f"{name}_error"] = fnum(row.get(f"{name}_error"))
            if math.isfinite(item["t_rel"]) and math.isfinite(item[f"{joint}_actual"]):
                rows.append(item)
    if not rows:
        raise ValueError(f"No sine rows found for {joint} in {path}")
    first_t = rows[0]["t_rel"]
    for row in rows:
        row["t_plot"] = row["t_rel"] - first_t
    return rows


def metrics(rows, joint):
    errors = [r[f"{joint}_target"] - r[f"{joint}_actual"] for r in rows]
    actual = [r[f"{joint}_actual"] for r in rows]
    target = [r[f"{joint}_target"] for r in rows]
    return {
        "rows": len(rows),
        "duration": rows[-1]["t_plot"] - rows[0]["t_plot"],
        "target_min": min(target),
        "target_max": max(target),
        "actual_min": min(actual),
        "actual_max": max(actual),
        "rmse": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "mae": sum(abs(e) for e in errors) / len(errors),
        "mean_error": sum(errors) / len(errors),
        "max_abs_error": max(abs(e) for e in errors),
    }


def draw_metric_panel(draw, rect, lines):
    left, top, right, bottom = rect
    draw.rounded_rectangle(rect, radius=8, outline=(203, 213, 225), fill=(248, 250, 252), width=2)
    text_font = font(15)
    y = top + 12
    for line in lines:
        draw.text((left + 14, y), line, fill=(15, 23, 42), font=text_font)
        y += 22


def render(rows, joint, output, title):
    width, height = 1500, 1050
    margin_left, margin_right = 115, 80
    img = Image.new("RGB", (width, height), (248, 250, 252))
    draw = ImageDraw.Draw(img)

    title_font = font(28, bold=True)
    subtitle_font = font(16)
    draw.text((55, 30), title, fill=(15, 23, 42), font=title_font)
    draw.text(
        (55, 68),
        "solid = actual, dashed = target; final reset-to-zero rows are excluded",
        fill=(71, 85, 105),
        font=subtitle_font,
    )

    xvals = [r["t_plot"] for r in rows]
    draw_panel(
        draw,
        (margin_left, 135, width - margin_right, 445),
        f"{joint.upper()} target vs actual",
        xvals,
        [(f"{joint} actual", [r[f"{joint}_actual"] for r in rows], COLORS[joint])],
        [(f"{joint} target", [r[f"{joint}_target"] for r in rows], (15, 23, 42))],
        "deg",
        "seconds",
        None,
    )

    errors = [r[f"{joint}_target"] - r[f"{joint}_actual"] for r in rows]
    draw_panel(
        draw,
        (margin_left, 535, width - margin_right, 735),
        f"{joint.upper()} error (target - actual)",
        xvals,
        [("error", errors, (220, 38, 38))],
        [],
        "deg",
        "seconds",
        None,
    )

    passive = [name for name in JOINTS if name != joint]
    draw_panel(
        draw,
        (margin_left, 895, width - margin_right, 1000),
        "Passive joint actual angles",
        xvals,
        [(name, [r[f"{name}_actual"] for r in rows], COLORS[name]) for name in passive],
        [],
        "deg",
        "seconds",
        None,
    )

    xmin, xmax = min(xvals), max(xvals)
    for rect in [(margin_left, 135, width - margin_right, 445), (margin_left, 535, width - margin_right, 735)]:
        left, top, right, bottom = rect
        yvals = errors if top == 535 else [r[f"{joint}_target"] for r in rows] + [r[f"{joint}_actual"] for r in rows]
        ymin, ymax = min(yvals), max(yvals)
        pad = max(1.0, (ymax - ymin) * 0.12)
        ymin -= pad
        ymax += pad
        if ymin <= 0 <= ymax:
            pts = project([(xmin, 0), (xmax, 0)], rect, xmin, xmax, ymin, ymax)
            draw_dashed_line(draw, pts, (100, 116, 139), width=1, dash=9, gap=8)

    m = metrics(rows, joint)
    lines = [
        f"rows={m['rows']} | duration={m['duration']:.2f}s",
        f"target {m['target_min']:.2f}..{m['target_max']:.2f} deg | actual {m['actual_min']:.2f}..{m['actual_max']:.2f} deg",
        f"RMSE={m['rmse']:.2f} deg | MAE={m['mae']:.2f} deg | mean error={m['mean_error']:.2f} deg | max abs={m['max_abs_error']:.2f} deg",
    ]
    draw_metric_panel(draw, (margin_left, 755, width - margin_right, 835), lines)
    img.save(output)


def main():
    parser = argparse.ArgumentParser(description="Plot a sine tracking run from MCP control CSV.")
    parser.add_argument("csv", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--joint", default="j2", choices=JOINTS)
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    rows = load_sine_rows(args.csv, args.joint)
    output = args.output or args.csv.with_name(args.csv.stem + f"_{args.joint}_sine_tracking.png")
    title = args.title or f"{args.joint.upper()} sine tracking"
    render(rows, args.joint, output, title)

    m = metrics(rows, args.joint)
    print(output)
    print(
        f"rows={m['rows']} duration={m['duration']:.3f}s "
        f"target={m['target_min']:.2f}..{m['target_max']:.2f} "
        f"actual={m['actual_min']:.2f}..{m['actual_max']:.2f} "
        f"rmse={m['rmse']:.3f} mae={m['mae']:.3f} "
        f"mean_error={m['mean_error']:.3f} max_abs={m['max_abs_error']:.3f}"
    )


if __name__ == "__main__":
    main()
