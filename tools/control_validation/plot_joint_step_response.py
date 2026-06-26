#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


JOINTS = ["j0", "j1", "j2", "j3"]
COLORS = {
    "j0": (31, 119, 180),
    "j1": (255, 127, 14),
    "j2": (44, 160, 44),
    "j3": (214, 39, 40),
}


def fnum(value, default=math.nan):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except ValueError:
        return default


def load_rows(path):
    rows = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            item = {"t_rel": fnum(row.get("t_rel"))}
            for joint in JOINTS:
                item[f"{joint}_actual"] = fnum(row.get(f"{joint}_actual"))
                item[f"{joint}_target"] = fnum(row.get(f"{joint}_target"))
                item[f"{joint}_error"] = fnum(row.get(f"{joint}_error"))
            rows.append(item)
    rows = [r for r in rows if math.isfinite(r["t_rel"])]
    if not rows:
        raise ValueError(f"No usable rows in {path}")
    first_t = rows[0]["t_rel"]
    for row in rows:
        row["t_plot"] = row["t_rel"] - first_t
    return rows


def detect_step_time(rows):
    best_i = None
    best_delta = 0.0
    for i in range(1, len(rows)):
        delta = sum(
            abs(rows[i][f"{j}_target"] - rows[i - 1][f"{j}_target"])
            for j in JOINTS
            if math.isfinite(rows[i][f"{j}_target"])
            and math.isfinite(rows[i - 1][f"{j}_target"])
        )
        if delta > best_delta:
            best_delta = delta
            best_i = i
    if best_i is None or best_delta < 0.1:
        return rows[0]["t_plot"], None
    return rows[best_i]["t_plot"], best_i


def font(size, bold=False):
    candidates = [
        r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\segoeuib.ttf" if bold else r"C:\Windows\Fonts\segoeui.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            pass
    return ImageFont.load_default()


def nice_ticks(vmin, vmax, count=6):
    if not math.isfinite(vmin) or not math.isfinite(vmax) or vmin == vmax:
        return [0]
    span = vmax - vmin
    raw = span / max(1, count - 1)
    power = 10 ** math.floor(math.log10(abs(raw)))
    step = min([1, 2, 2.5, 5, 10], key=lambda m: abs(m * power - raw)) * power
    start = math.floor(vmin / step) * step
    end = math.ceil(vmax / step) * step
    ticks = []
    val = start
    while val <= end + step * 0.5:
        ticks.append(round(val, 6))
        val += step
    return ticks


def project(points, rect, xmin, xmax, ymin, ymax):
    left, top, right, bottom = rect
    width = right - left
    height = bottom - top
    out = []
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        px = left + (x - xmin) / (xmax - xmin) * width
        py = bottom - (y - ymin) / (ymax - ymin) * height
        out.append((px, py))
    return out


def draw_polyline(draw, pts, color, width=3):
    if len(pts) < 2:
        return
    draw.line(pts, fill=color, width=width, joint="curve")


def draw_dashed_line(draw, pts, color, width=2, dash=12, gap=8):
    if len(pts) < 2:
        return
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        if length <= 0:
            continue
        ux = (x2 - x1) / length
        uy = (y2 - y1) / length
        pos = 0.0
        while pos < length:
            end = min(pos + dash, length)
            draw.line(
                [(x1 + ux * pos, y1 + uy * pos), (x1 + ux * end, y1 + uy * end)],
                fill=color,
                width=width,
            )
            pos += dash + gap


def draw_panel(draw, rect, title, xvals, series, targets, y_label, x_label, step_x):
    left, top, right, bottom = rect
    all_y = []
    for _, values, _ in series:
        all_y.extend([v for v in values if math.isfinite(v)])
    for _, values, _ in targets:
        all_y.extend([v for v in values if math.isfinite(v)])
    ymin = min(all_y)
    ymax = max(all_y)
    if ymin == ymax:
        ymin -= 1
        ymax += 1
    margin = max(1.0, (ymax - ymin) * 0.12)
    ymin -= margin
    ymax += margin
    xmin = min(xvals)
    xmax = max(xvals)
    if xmin == xmax:
        xmax = xmin + 1.0

    draw.rectangle(rect, outline=(205, 213, 223), width=2)
    title_font = font(24, bold=True)
    label_font = font(18)
    small_font = font(16)
    draw.text((left, top - 34), title, fill=(33, 37, 41), font=title_font)

    grid_color = (226, 232, 240)
    tick_color = (71, 85, 105)
    for tick in nice_ticks(ymin, ymax):
        if tick < ymin or tick > ymax:
            continue
        y = project([(xmin, tick)], rect, xmin, xmax, ymin, ymax)[0][1]
        draw.line([(left, y), (right, y)], fill=grid_color, width=1)
        draw.text((left - 74, y - 10), f"{tick:g}", fill=tick_color, font=small_font)
    for tick in nice_ticks(xmin, xmax):
        if tick < xmin or tick > xmax:
            continue
        x = project([(tick, ymin)], rect, xmin, xmax, ymin, ymax)[0][0]
        draw.line([(x, top), (x, bottom)], fill=grid_color, width=1)
        draw.text((x - 16, bottom + 8), f"{tick:g}", fill=tick_color, font=small_font)

    if step_x is not None and xmin <= step_x <= xmax:
        step_pts = project([(step_x, ymin), (step_x, ymax)], rect, xmin, xmax, ymin, ymax)
        draw_dashed_line(draw, step_pts, (120, 53, 15), width=2, dash=10, gap=8)

    zero_pts = project([(xmin, 0), (xmax, 0)], rect, xmin, xmax, ymin, ymax)
    if ymin <= 0 <= ymax:
        draw_dashed_line(draw, zero_pts, (100, 116, 139), width=1, dash=9, gap=8)

    for name, values, color in targets:
        pts = project(list(zip(xvals, values)), rect, xmin, xmax, ymin, ymax)
        draw_dashed_line(draw, pts, color, width=2, dash=16, gap=10)
    for name, values, color in series:
        pts = project(list(zip(xvals, values)), rect, xmin, xmax, ymin, ymax)
        draw_polyline(draw, pts, color, width=4)

    draw.text((left + (right - left) // 2 - 55, bottom + 38), x_label, fill=tick_color, font=label_font)
    draw.text((left - 96, top - 6), y_label, fill=tick_color, font=label_font)


def summarize(rows, step_idx):
    tail_rows = [r for r in rows if r["t_plot"] >= rows[-1]["t_plot"] - 10.0]
    if not tail_rows:
        tail_rows = rows[-max(1, min(len(rows), 80)) :]
    last = rows[-1]
    tail_mean = {}
    tail_err = {}
    for joint in JOINTS:
        tail_mean[joint] = sum(r[f"{joint}_actual"] for r in tail_rows) / len(tail_rows)
        tail_err[joint] = sum(r[f"{joint}_error"] for r in tail_rows) / len(tail_rows)
    return [
        "Targets: " + ", ".join(f"{j.upper()} {last[f'{j}_target']:.2f}" for j in JOINTS),
        "Final: " + ", ".join(f"{j.upper()} {last[f'{j}_actual']:.2f}" for j in JOINTS),
        "Tail mean: " + ", ".join(f"{j.upper()} {tail_mean[j]:.2f}" for j in JOINTS),
        "Tail err: " + ", ".join(f"{j.upper()} {tail_err[j]:.2f}" for j in JOINTS),
        "",
    ]


def render(rows, output, title):
    step_x, step_idx = detect_step_time(rows)
    for row in rows:
        row["x"] = row["t_plot"] - step_x
    xvals = [r["x"] for r in rows]

    scale = 1
    width, height = 1500, 1000
    im = Image.new("RGB", (width * scale, height * scale), (248, 250, 252))
    draw = ImageDraw.Draw(im)

    def srect(rect):
        return tuple(int(v * scale) for v in rect)

    def scaled_font(size, bold=False):
        return font(size * scale, bold)

    title_font = scaled_font(28, bold=True)
    sub_font = scaled_font(17)
    draw.text((60 * scale, 28 * scale), title, fill=(15, 23, 42), font=title_font)
    draw.text(
        (60 * scale, 68 * scale),
        "time zero = detected target step. solid = actual angle, dashed = target, gray dashed = zero line",
        fill=(71, 85, 105),
        font=sub_font,
    )

    top_rect = srect((130, 135, 1425, 475))
    bottom_rect = srect((130, 570, 1425, 820))

    series_actual = [
        (j.upper(), [r[f"{j}_actual"] for r in rows], COLORS[j]) for j in JOINTS
    ]
    targets = [
        (j.upper() + " target", [r[f"{j}_target"] for r in rows], COLORS[j])
        for j in JOINTS
    ]
    draw_panel(
        draw,
        top_rect,
        "Joint actual angles",
        xvals,
        series_actual,
        targets,
        "deg",
        "seconds from step",
        0.0,
    )

    series_error = [
        (j.upper(), [r[f"{j}_error"] for r in rows], COLORS[j]) for j in JOINTS
    ]
    draw_panel(
        draw,
        bottom_rect,
        "Joint error (target - actual)",
        xvals,
        series_error,
        [],
        "deg",
        "seconds from step",
        0.0,
    )

    legend_x = 1040 * scale
    legend_y = 32 * scale
    small_font = scaled_font(17)
    for idx, j in enumerate(JOINTS):
        y = legend_y + idx * 26 * scale
        draw.line([(legend_x, y + 9 * scale), (legend_x + 40 * scale, y + 9 * scale)], fill=COLORS[j], width=5 * scale)
        draw.text((legend_x + 52 * scale, y), j.upper(), fill=(33, 37, 41), font=small_font)
    draw_dashed_line(
        draw,
        [(legend_x + 160 * scale, legend_y + 9 * scale), (legend_x + 210 * scale, legend_y + 9 * scale)],
        (51, 65, 85),
        width=3 * scale,
        dash=15 * scale,
        gap=8 * scale,
    )
    draw.text((legend_x + 222 * scale, legend_y), "targets", fill=(33, 37, 41), font=small_font)

    summary_font = scaled_font(15)
    summary_lines = summarize(rows, step_idx)
    box_x, box_y = 130 * scale, 905 * scale
    box_w, box_h = 1295 * scale, 72 * scale
    draw.rounded_rectangle(
        (box_x, box_y, box_x + box_w, box_y + box_h),
        radius=8 * scale,
        fill=(255, 255, 255),
        outline=(203, 213, 225),
        width=1 * scale,
    )
    draw.text((box_x + 16 * scale, box_y + 11 * scale), summary_lines[0], fill=(15, 23, 42), font=summary_font)
    draw.text((box_x + 16 * scale, box_y + 33 * scale), summary_lines[1], fill=(15, 23, 42), font=summary_font)
    draw.text((box_x + 16 * scale, box_y + 55 * scale), summary_lines[2] + " | " + summary_lines[3], fill=(15, 23, 42), font=summary_font)

    im = im.resize((width, height), Image.Resampling.LANCZOS)
    output.parent.mkdir(parents=True, exist_ok=True)
    im.save(output)
    return summary_lines


def main():
    parser = argparse.ArgumentParser(description="Plot MCP joint step response CSV.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    csv_path = args.csv_path
    output = args.output or csv_path.with_name(csv_path.stem + "_joint_step_response.png")
    title = args.title or f"Joint step response: {csv_path.name}"
    rows = load_rows(csv_path)
    summary = render(rows, output, title)
    print(output)
    for line in summary:
        print(line)


if __name__ == "__main__":
    main()
