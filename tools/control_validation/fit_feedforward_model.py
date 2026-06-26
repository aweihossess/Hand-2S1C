#!/usr/bin/env python3
import argparse
import csv
import math
import os

import numpy as np


def to_float(value, default=math.nan):
    try:
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def load_samples(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def row_value(row, mean_name, raw_name):
    value = to_float(row.get(mean_name, ""))
    if math.isfinite(value):
        return value
    return to_float(row.get(raw_name, ""))


def sample_ok(row, max_load_abs, max_bias_abs):
    if max_load_abs is not None:
        for m in range(5):
            load = row_value(row, f"m{m}_load_mean", f"m{m}_load")
            if math.isfinite(load) and abs(load) > max_load_abs:
                return False
    if max_bias_abs is not None:
        for m in range(5):
            bias = row_value(row, f"m{m}_bias_mean", f"m{m}_bias")
            if math.isfinite(bias) and abs(bias) > max_bias_abs:
                return False
    return True


def motor_field_name(motor_index, fit_target):
    if fit_target in ("cmd_minus_bias", "now_minus_bias"):
        return None
    return f"m{motor_index}_{fit_target}_mean"


def motor_target_value(row, motor_index, fit_target):
    if fit_target == "cmd_minus_bias":
        cmd = row_value(row, f"m{motor_index}_cmd_mean", f"m{motor_index}_cmd")
        bias = row_value(row, f"m{motor_index}_bias_mean", f"m{motor_index}_bias")
        if math.isfinite(cmd) and math.isfinite(bias):
            return cmd - bias
        return math.nan
    if fit_target == "now_minus_bias":
        now = row_value(row, f"m{motor_index}_now_mean", f"m{motor_index}_now")
        bias = row_value(row, f"m{motor_index}_bias_mean", f"m{motor_index}_bias")
        if math.isfinite(now) and math.isfinite(bias):
            return now - bias
        return math.nan
    return row_value(row, motor_field_name(motor_index, fit_target), f"m{motor_index}_{fit_target}")


def build_xy(rows, fit_target):
    x_rows = []
    y_rows = []
    kept = []
    for row in rows:
        q = [row_value(row, f"j{j}_actual_mean", f"j{j}_actual") for j in range(4)]
        y = [motor_target_value(row, m, fit_target) for m in range(5)]
        if all(math.isfinite(v) for v in q + y):
            x_rows.append([1.0] + q)
            y_rows.append(y)
            kept.append(row)
    return np.array(x_rows, dtype=float), np.array(y_rows, dtype=float), kept


def fit_linear(x, y):
    coeff, *_ = np.linalg.lstsq(x, y, rcond=None)
    pred = x @ coeff
    err = pred - y
    rmse = np.sqrt(np.mean(err * err, axis=0))
    return coeff, rmse


def write_coeff_csv(path, coeff, rmse, sample_count):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = ["motor", "intercept", "j0", "j1", "j2", "j3", "rmse_counts", "samples"]
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for m in range(5):
            writer.writerow({
                "motor": f"M{m}",
                "intercept": coeff[0, m],
                "j0": coeff[1, m],
                "j1": coeff[2, m],
                "j2": coeff[3, m],
                "j3": coeff[4, m],
                "rmse_counts": rmse[m],
                "samples": sample_count,
            })


def write_report(path, coeff, rmse, rows, kept, fit_target):
    lines = []
    lines.append(f"samples_total: {len(rows)}")
    lines.append(f"samples_used: {len(kept)}")
    lines.append("")
    lines.append(f"fit_target: {fit_target}")
    lines.append("model: motor_target ~= intercept + j0*c0 + j1*c1 + j2*c2 + j3*c3")
    lines.append("units: motor counts / deg for joint coefficients")
    lines.append("")
    for m in range(5):
        lines.append(
            f"M{m}: intercept={coeff[0, m]:.2f}, "
            f"J0={coeff[1, m]:.2f}, J1={coeff[2, m]:.2f}, "
            f"J2={coeff[3, m]:.2f}, J3={coeff[4, m]:.2f}, "
            f"rmse={rmse[m]:.1f}")
    lines.append("")
    lines.append("C++ row-major matrix suggestion, rows=M0..M4, cols=J0..J3:")
    lines.append("static const float kEmpiricalMotorPerDeg[5][4] = {")
    for m in range(5):
        suffix = "," if m < 4 else ""
        lines.append(
            f"    {{{coeff[1, m]: .3f}f, {coeff[2, m]: .3f}f, "
            f"{coeff[3, m]: .3f}f, {coeff[4, m]: .3f}f}}{suffix}")
    lines.append("};")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Fit empirical motor feedforward from averaged collection samples.")
    parser.add_argument("samples_csv")
    parser.add_argument("--max-load-abs", type=float, default=None)
    parser.add_argument("--max-bias-abs", type=float, default=None)
    parser.add_argument(
        "--fit-target",
        choices=["now", "now_minus_bias", "solver", "map", "cmd", "cmd_minus_bias"],
        default="now",
        help="Which motor field to fit. Works with raw control CSV rows or averaged sample CSV rows.")
    parser.add_argument("--out-prefix", default="")
    args = parser.parse_args()

    rows = load_samples(args.samples_csv)
    filtered = [r for r in rows if sample_ok(r, args.max_load_abs, args.max_bias_abs)]
    x, y, kept = build_xy(filtered, args.fit_target)
    if len(kept) < 5:
        print(f"Not enough usable samples: {len(kept)}")
        return 1
    coeff, rmse = fit_linear(x, y)

    prefix = args.out_prefix or os.path.splitext(args.samples_csv)[0] + "_fit"
    write_coeff_csv(prefix + "_coeff.csv", coeff, rmse, len(kept))
    write_report(prefix + "_report.txt", coeff, rmse, rows, kept, args.fit_target)
    print(f"Saved fit report to {prefix}_report.txt")
    print(f"Saved coefficients to {prefix}_coeff.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
