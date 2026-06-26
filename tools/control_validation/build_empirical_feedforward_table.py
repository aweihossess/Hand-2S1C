#!/usr/bin/env python3
import argparse
import csv
import math
import os
from collections import defaultdict


def to_float(value, default=math.nan):
    try:
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def mean(values):
    vals = [v for v in values if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else math.nan


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def row_motor(row, motor_index, suffix):
    return to_float(row.get(f"m{motor_index}_{suffix}_mean", ""))


def sample_ok(row, max_bias_abs, max_load_abs):
    if max_bias_abs is not None:
        for m in range(5):
            bias = row_motor(row, m, "bias")
            if math.isfinite(bias) and abs(bias) > max_bias_abs:
                return False
    if max_load_abs is not None:
        for m in range(5):
            load = row_motor(row, m, "load")
            if math.isfinite(load) and abs(load) > max_load_abs:
                return False
    return True


def find_base_now(rows, zero_tol):
    zero_rows = []
    for row in rows:
        targets = [to_float(row.get(f"j{j}_target_mean", "")) for j in range(4)]
        if all(math.isfinite(v) and abs(v) <= zero_tol for v in targets):
            zero_rows.append(row)
    if not zero_rows:
        for row in rows:
            target = to_float(row.get("target_deg", ""))
            if math.isfinite(target) and abs(target) <= zero_tol:
                zero_rows.append(row)

    base = []
    base_now_minus_bias = []
    for m in range(5):
        base.append(mean(row_motor(row, m, "now") for row in zero_rows))
        base_now_minus_bias.append(
            mean(
                row_motor(row, m, "now") - row_motor(row, m, "bias")
                for row in zero_rows
                if math.isfinite(row_motor(row, m, "now")) and math.isfinite(row_motor(row, m, "bias"))
            )
        )
    return base, base_now_minus_bias, zero_rows


def build_points(rows, base_now, base_now_minus_bias, max_target_error):
    points = []
    for row in rows:
        try:
            joint = int(float(row.get("target_joint", "")))
        except Exception:
            continue
        if joint < 0 or joint > 3:
            continue

        command_deg = to_float(row.get("target_deg", ""))
        actual_deg = to_float(row.get(f"j{joint}_actual_mean", ""))
        if not (math.isfinite(command_deg) and math.isfinite(actual_deg)):
            continue
        if max_target_error is not None and abs(command_deg - actual_deg) > max_target_error:
            continue

        rec = {
            "target_joint": joint,
            "command_deg": command_deg,
            "actual_deg": actual_deg,
            "target_error_deg": command_deg - actual_deg,
            "rows": row.get("rows", ""),
        }
        for j in range(4):
            rec[f"j{j}_actual"] = to_float(row.get(f"j{j}_actual_mean", ""))
            rec[f"j{j}_target"] = to_float(row.get(f"j{j}_target_mean", ""))
        for m in range(5):
            now = row_motor(row, m, "now")
            bias = row_motor(row, m, "bias")
            now_minus_bias = now - bias if math.isfinite(now) and math.isfinite(bias) else math.nan
            rec[f"m{m}_now"] = now
            rec[f"m{m}_bias"] = bias
            rec[f"m{m}_now_minus_bias"] = now_minus_bias
            rec[f"m{m}_delta_from_zero"] = now - base_now[m] if math.isfinite(now) and math.isfinite(base_now[m]) else math.nan
            rec[f"m{m}_delta_now_minus_bias_from_zero"] = (
                now_minus_bias - base_now_minus_bias[m]
                if math.isfinite(now_minus_bias) and math.isfinite(base_now_minus_bias[m])
                else math.nan
            )
        points.append(rec)

    points.sort(key=lambda r: (r["target_joint"], r["actual_deg"], r["command_deg"]))
    return points


def write_points(path, points):
    fields = [
        "target_joint",
        "command_deg",
        "actual_deg",
        "target_error_deg",
        "rows",
    ]
    fields += [f"j{j}_{name}" for j in range(4) for name in ("actual", "target")]
    fields += [f"m{m}_{name}" for m in range(5) for name in (
        "now",
        "bias",
        "now_minus_bias",
        "delta_from_zero",
        "delta_now_minus_bias_from_zero",
    )]
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(points)


def write_report(path, source_path, rows, filtered_rows, zero_rows, base_now, base_now_minus_bias, points):
    by_joint = defaultdict(list)
    for point in points:
        by_joint[point["target_joint"]].append(point)

    lines = [
        f"source: {source_path}",
        f"samples_total: {len(rows)}",
        f"samples_after_filters: {len(filtered_rows)}",
        f"zero_samples_used: {len(zero_rows)}",
        "",
        "base_motor_now_at_zero:",
    ]
    for m, value in enumerate(base_now):
        lines.append(f"  M{m}: {value:.3f}")
    lines.append("")
    lines.append("base_motor_now_minus_bias_at_zero:")
    for m, value in enumerate(base_now_minus_bias):
        lines.append(f"  M{m}: {value:.3f}")
    lines.append("")
    lines.append("per_joint_points:")
    for joint in range(4):
        values = by_joint[joint]
        if not values:
            lines.append(f"  J{joint}: 0 points")
            continue
        actuals = [v["actual_deg"] for v in values]
        lines.append(
            f"  J{joint}: {len(values)} points, actual_deg {min(actuals):.3f}..{max(actuals):.3f}")
    lines.append("")
    lines.append("runtime_use:")
    lines.append("  motor_now_target = base_motor_now_at_zero + sum(interp(points_for_joint, target_joint_angle))")
    lines.append("  then start angle PID around that feedforward target")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Build measured joint-angle to motor-position feedforward points from averaged collection samples.")
    parser.add_argument("samples_csv", help="CSV from summarize_feedforward_collect.py")
    parser.add_argument("--zero-tol", type=float, default=0.25)
    parser.add_argument("--max-target-error", type=float, default=None)
    parser.add_argument("--max-bias-abs", type=float, default=None)
    parser.add_argument("--max-load-abs", type=float, default=None)
    parser.add_argument("--out-prefix", default="")
    args = parser.parse_args()

    rows = load_rows(args.samples_csv)
    filtered = [row for row in rows if sample_ok(row, args.max_bias_abs, args.max_load_abs)]
    base_now, base_now_minus_bias, zero_rows = find_base_now(filtered, args.zero_tol)
    if not zero_rows:
        print("No zero-pose samples found.")
        return 1

    points = build_points(filtered, base_now, base_now_minus_bias, args.max_target_error)
    if not points:
        print("No usable empirical points found.")
        return 1

    prefix = args.out_prefix or os.path.splitext(args.samples_csv)[0] + "_empirical"
    os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
    write_points(prefix + "_points.csv", points)
    write_report(prefix + "_report.txt", args.samples_csv, rows, filtered, zero_rows, base_now, base_now_minus_bias, points)
    print(f"Saved empirical points to {prefix}_points.csv")
    print(f"Saved report to {prefix}_report.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
