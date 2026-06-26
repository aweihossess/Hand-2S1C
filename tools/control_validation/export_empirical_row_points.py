#!/usr/bin/env python3
import argparse
import csv
import math
import os


def to_float(value, default=math.nan):
    try:
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def infer_active_effective_joint(row, threshold_deg):
    targets = [to_float(row.get(f"j{j}_target", "")) for j in range(4)]
    finite = [(j, abs(v), v) for j, v in enumerate(targets) if math.isfinite(v)]
    if not finite:
        return "", ""
    finite.sort(key=lambda item: item[1], reverse=True)
    if finite[0][1] <= threshold_deg:
        return "", ""
    if len(finite) > 1 and finite[1][1] > threshold_deg:
        return "multi", finite[0][2]
    return finite[0][0], finite[0][2]


def write_points(path, rows, active_threshold_deg):
    fields = [
        "t_rel",
        "active_effective_joint",
        "active_effective_target_deg",
    ]
    fields += [f"j{j}_{name}" for j in range(4) for name in ("target", "actual", "error")]
    fields += [f"m{m}_{name}" for m in range(5) for name in ("now", "map", "solver", "cmd", "load", "cur", "bias")]

    out_rows = []
    for row in rows:
        if not row.get("line", "").startswith("[MCP5 CTRL]"):
            continue
        active_joint, active_target = infer_active_effective_joint(row, active_threshold_deg)
        rec = {
            "t_rel": row.get("t_rel", ""),
            "active_effective_joint": active_joint,
            "active_effective_target_deg": active_target,
        }
        for j in range(4):
            for name in ("target", "actual", "error"):
                rec[f"j{j}_{name}"] = row.get(f"j{j}_{name}", "")
        for m in range(5):
            for name in ("now", "map", "solver", "cmd", "load", "cur", "bias"):
                rec[f"m{m}_{name}"] = row.get(f"m{m}_{name}", "")
        out_rows.append(rec)

    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(out_rows)
    return out_rows


def write_report(path, source_path, points):
    lines = [
        f"source: {source_path}",
        f"rows: {len(points)}",
        "",
        "joint_actual_ranges:",
    ]
    for j in range(4):
        vals = [to_float(row.get(f"j{j}_actual", "")) for row in points]
        vals = [v for v in vals if math.isfinite(v)]
        if vals:
            lines.append(f"  J{j}: {min(vals):.3f}..{max(vals):.3f}")
    lines.append("")
    lines.append("motor_now_ranges:")
    for m in range(5):
        vals = [to_float(row.get(f"m{m}_now", "")) for row in points]
        vals = [v for v in vals if math.isfinite(v)]
        if vals:
            lines.append(f"  M{m}: {min(vals):.0f}..{max(vals):.0f}")
    lines.append("")
    lines.append("note:")
    lines.append("  This export preserves every MCP control row, including transient overshoot and coupling.")
    lines.append("  active_effective_joint is inferred from the firmware-effective target columns.")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Export every MCP control row as empirical measured angle/motor-position points.")
    parser.add_argument("csv_path")
    parser.add_argument("--active-threshold-deg", type=float, default=0.25)
    parser.add_argument("--out-prefix", default="")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    prefix = args.out_prefix or os.path.splitext(args.csv_path)[0] + "_empirical_rows"
    os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
    points = write_points(prefix + "_points.csv", rows, args.active_threshold_deg)
    write_report(prefix + "_report.txt", args.csv_path, points)
    print(f"Saved row empirical points to {prefix}_points.csv")
    print(f"Saved report to {prefix}_report.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
