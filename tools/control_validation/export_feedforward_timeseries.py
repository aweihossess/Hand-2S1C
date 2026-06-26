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


def to_int_like(value):
    number = to_float(value)
    if not math.isfinite(number):
        return ""
    return int(round(number))


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def export_rows(rows):
    out = []
    for row in rows:
        rec = {
            "t_rel": row.get("t_rel", ""),
            "j0_actual": row.get("j0_actual", ""),
            "j1_actual": row.get("j1_actual", ""),
            "j2_actual": row.get("j2_actual", ""),
            "j3_actual": row.get("j3_actual", ""),
        }
        for j in range(4):
            rec[f"j{j}_target"] = row.get(f"j{j}_target", "")
        for m in range(5):
            now = to_float(row.get(f"m{m}_now", ""))
            bias = to_float(row.get(f"m{m}_bias", ""))
            rec[f"m{m}_now"] = row.get(f"m{m}_now", "")
            rec[f"m{m}_bias"] = row.get(f"m{m}_bias", "")
            rec[f"m{m}_now_minus_bias"] = "" if not (math.isfinite(now) and math.isfinite(bias)) else int(round(now - bias))
            rec[f"m{m}_load"] = row.get(f"m{m}_load", "")
            rec[f"m{m}_solver"] = row.get(f"m{m}_solver", "")
            rec[f"m{m}_cmd"] = row.get(f"m{m}_cmd", "")
        out.append(rec)
    return out


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = ["t_rel"]
    fields.extend([f"j{j}_actual" for j in range(4)])
    fields.extend([f"j{j}_target" for j in range(4)])
    for m in range(5):
        fields.extend([
            f"m{m}_now",
            f"m{m}_bias",
            f"m{m}_now_minus_bias",
            f"m{m}_load",
            f"m{m}_solver",
            f"m{m}_cmd",
        ])
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="Export raw MCP control rows as time-aligned feedforward samples.")
    parser.add_argument("csv_path")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    out_rows = export_rows(rows)
    out_path = args.out or os.path.splitext(args.csv_path)[0] + "_ff_timeseries.csv"
    write_csv(out_path, out_rows)
    print(f"Saved {len(out_rows)} rows to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
