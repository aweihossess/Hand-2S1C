from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ZERO_CONFIG = ROOT / "desktop" / "config" / "encoder_zero.json"


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def load_zero_deg(path: Path) -> dict[int, float]:
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    zero = data.get("zero_deg", {})
    return {int(key): float(value) for key, value in zero.items()}


def load_font(size: int) -> ImageFont.ImageFont:
    for name in ("arial.ttf", "segoeui.ttf", "calibri.ttf"):
        path = Path("C:/Windows/Fonts") / name
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def build_trajectory(
    duration_s: float,
    dt_s: float,
    mcp_max_deg: float,
    pip_max_deg: float,
    dip_ratio: float,
    dip_sign: float,
    zero_deg: dict[int, float],
) -> list[dict[str, object]]:
    steps = max(2, int(round(duration_s / dt_s)) + 1)
    rows: list[dict[str, object]] = []
    for index in range(steps):
        t_s = min(duration_s, index * dt_s)
        s = 0.0 if duration_s <= 0 else t_s / duration_s

        mcp_phase = smoothstep(s)
        pip_phase = smoothstep((s - 0.04) / 0.92)
        dip_phase = smoothstep((s - 0.08) / 0.84)

        mcp = mcp_max_deg * mcp_phase
        pip = pip_max_deg * pip_phase
        dip_abs = dip_ratio * pip * (0.45 + 0.55 * dip_phase)
        dip = dip_sign * dip_abs

        j01_device = mcp + zero_deg.get(1, 0.0)
        j02_device = pip + zero_deg.get(2, 0.0)
        j03_device = dip + zero_deg.get(3, 0.0)

        row = {
            "index": index,
            "t_s": round(t_s, 3),
            "s": round(s, 5),
            "mcp_fe_j01_rel_deg": round(mcp, 3),
            "pip_fe_j02_rel_deg": round(pip, 3),
            "dip_fe_j03_rel_deg": round(dip, 3),
            "dip_abs_over_pip": round(abs(dip) / pip, 3) if abs(pip) > 1e-6 else 0.0,
            "j01_device_deg": round(j01_device, 3),
            "j02_device_deg": round(j02_device, 3),
            "j03_device_deg": round(j03_device, 3),
            "command_j01": f"degree; j1 {j01_device:.3f}",
            "command_j02": f"degree; j2 {j02_device:.3f}",
            "command_j03": f"degree; j3 {j03_device:.3f}",
        }
        row["command_sequence"] = f"{row['command_j01']} | {row['command_j02']} | {row['command_j03']}"
        rows.append(row)
    return rows


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def draw_chart(rows: list[dict[str, object]], path: Path) -> None:
    width, height = 1600, 950
    img = Image.new("RGB", (width, height), "#f8fafc")
    draw = ImageDraw.Draw(img)
    title_font = load_font(44)
    font = load_font(24)
    small = load_font(20)

    margin_left, margin_right = 110, 60
    margin_top, margin_bottom = 130, 120
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    t_values = [float(row["t_s"]) for row in rows]
    series = [
        ("MCP-FE / J01", "mcp_fe_j01_rel_deg", "#2563eb"),
        ("PIP-FE / J02", "pip_fe_j02_rel_deg", "#16a34a"),
        ("DIP-FE / J03", "dip_fe_j03_rel_deg", "#dc2626"),
    ]
    y_values = []
    for _, key, _ in series:
        y_values.extend(float(row[key]) for row in rows)
    y_min = min(0.0, min(y_values))
    y_max = max(5.0, max(y_values))
    y_pad = max(5.0, (y_max - y_min) * 0.08)
    y_min -= y_pad
    y_max += y_pad

    def x_of(t: float) -> float:
        return margin_left + (t - min(t_values)) / max(1e-9, max(t_values) - min(t_values)) * plot_w

    def y_of(v: float) -> float:
        return margin_top + (y_max - v) / max(1e-9, y_max - y_min) * plot_h

    draw.text((60, 42), "Human-like Finger Curl Trajectory", fill="#0f172a", font=title_font)
    draw.text(
        (64, 95),
        "J01=MCP-FE, J02=PIP-FE, J03=DIP-FE; DIP is coupled to PIP with a slight lag",
        fill="#475569",
        font=font,
    )

    # Grid and axes.
    for i in range(8):
        y = margin_top + plot_h * i / 7
        value = y_max - (y_max - y_min) * i / 7
        draw.line((margin_left, y, width - margin_right, y), fill="#dbe3ee", width=1)
        draw.text((20, y - 12), f"{value:5.1f}", fill="#64748b", font=small)
    for i in range(7):
        x = margin_left + plot_w * i / 6
        t = min(t_values) + (max(t_values) - min(t_values)) * i / 6
        draw.line((x, margin_top, x, height - margin_bottom), fill="#e2e8f0", width=1)
        draw.text((x - 26, height - margin_bottom + 18), f"{t:.1f}", fill="#64748b", font=small)
    draw.rectangle((margin_left, margin_top, width - margin_right, height - margin_bottom), outline="#94a3b8", width=2)
    draw.text((width / 2 - 46, height - 55), "time (s)", fill="#334155", font=font)
    draw.text((18, margin_top - 38), "deg", fill="#334155", font=font)

    for label, key, color in series:
        points = [(x_of(float(row["t_s"])), y_of(float(row[key]))) for row in rows]
        draw.line(points, fill=color, width=5, joint="curve")
        x_end, y_end = points[-1]
        draw.ellipse((x_end - 7, y_end - 7, x_end + 7, y_end + 7), fill=color)
        draw.text((x_end - 220, y_end - 36), f"{label}: {float(rows[-1][key]):.1f} deg", fill=color, font=small)

    legend_x, legend_y = 1160, 150
    for i, (label, _, color) in enumerate(series):
        y = legend_y + i * 38
        draw.line((legend_x, y + 12, legend_x + 55, y + 12), fill=color, width=6)
        draw.text((legend_x + 68, y), label, fill="#0f172a", font=small)

    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, quality=95)


def write_note(
    path: Path,
    csv_path: Path,
    png_path: Path,
    duration_s: float,
    dt_s: float,
    mcp_max_deg: float,
    pip_max_deg: float,
    dip_ratio: float,
    dip_sign: float,
) -> None:
    direction = "positive" if dip_sign >= 0 else "negative"
    text = f"""# Finger Curl Trajectory

This trajectory is intended as a conservative human-like closing motion for the current MCP/PIP/DIP setup.

- J01 = MCP-FE
- J02 = PIP-FE
- J03 = DIP-FE
- Duration: {duration_s:.2f} s
- Interval: {dt_s:.3f} s
- MCP max: {mcp_max_deg:.2f} deg
- PIP max: {pip_max_deg:.2f} deg
- DIP/PIP final coupling ratio: {dip_ratio:.2f}
- DIP sign: {direction}

Formula:

```text
s = t / duration
smoothstep(x) = clamp(x, 0, 1)^2 * (3 - 2 * clamp(x, 0, 1))
MCP = {mcp_max_deg:.2f} * smoothstep(s)
PIP = {pip_max_deg:.2f} * smoothstep((s - 0.04) / 0.92)
DIP = sign * {dip_ratio:.2f} * PIP * (0.45 + 0.55 * smoothstep((s - 0.08) / 0.84))
```

Use the relative columns for conceptual control. Use the `command_j01`, `command_j02`, and `command_j03` columns if sending directly to firmware, because those include the saved encoder display zero offset.

- CSV: `{csv_path.name}`
- PNG: `{png_path.name}`
"""
    path.write_text(text, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a human-like MCP/PIP/DIP finger curl trajectory.")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--dt", type=float, default=0.2)
    parser.add_argument("--mcp-max", type=float, default=42.0)
    parser.add_argument("--pip-max", type=float, default=68.0)
    parser.add_argument("--dip-ratio", type=float, default=0.66)
    parser.add_argument("--dip-sign", type=float, default=1.0, choices=(-1.0, 1.0))
    parser.add_argument("--zero-config", type=Path, default=DEFAULT_ZERO_CONFIG)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "run_data" / "finger_curl_trajectory_j01_j02_j03")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    zero_deg = load_zero_deg(args.zero_config)
    rows = build_trajectory(
        duration_s=args.duration,
        dt_s=args.dt,
        mcp_max_deg=args.mcp_max,
        pip_max_deg=args.pip_max,
        dip_ratio=args.dip_ratio,
        dip_sign=args.dip_sign,
        zero_deg=zero_deg,
    )
    out_dir = args.out_dir.resolve()
    csv_path = out_dir / "finger_curl_trajectory_j01_j02_j03.csv"
    png_path = out_dir / "finger_curl_trajectory_j01_j02_j03.png"
    note_path = out_dir / "finger_curl_trajectory_j01_j02_j03.md"

    write_csv(rows, csv_path)
    draw_chart(rows, png_path)
    write_note(
        note_path,
        csv_path,
        png_path,
        args.duration,
        args.dt,
        args.mcp_max,
        args.pip_max,
        args.dip_ratio,
        args.dip_sign,
    )
    print(f"csv={csv_path}")
    print(f"png={png_path}")
    print(f"note={note_path}")
    print("final relative targets:")
    print(
        f"J01={rows[-1]['mcp_fe_j01_rel_deg']} deg, "
        f"J02={rows[-1]['pip_fe_j02_rel_deg']} deg, "
        f"J03={rows[-1]['dip_fe_j03_rel_deg']} deg"
    )


if __name__ == "__main__":
    main()
