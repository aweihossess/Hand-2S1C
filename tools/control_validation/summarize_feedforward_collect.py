#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics


def to_float(value, default=math.nan):
    try:
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def mean(values):
    vals = [v for v in values if math.isfinite(v)]
    return statistics.fmean(vals) if vals else math.nan


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def summarize(rows, joint, target_step_min, avg_tail_sec):
    field = f"j{joint}_target"
    other_fields = [f"j{j}_target" for j in range(4) if j != joint]
    segments = []
    current = []
    current_target = math.nan

    for row in rows:
        target = to_float(row.get(field, ""))
        if not math.isfinite(target):
            continue
        if not current or not math.isclose(target, current_target, abs_tol=1.0e-6):
            if current:
                segments.append((current_target, current))
            current = [row]
            current_target = target
        else:
            current.append(row)
    if current:
        segments.append((current_target, current))

    out = []
    last_kept_target = math.nan
    for idx, (target, seg) in enumerate(segments):
        if all(math.isclose(target, 0.0, abs_tol=1.0e-6) for _ in [0]):
            other_nonzero = any(
                abs(to_float(r.get(other, ""))) > target_step_min
                for r in seg
                for other in other_fields)
            if other_nonzero:
                continue
        if math.isfinite(last_kept_target) and abs(target - last_kept_target) < target_step_min:
            continue
        t_vals = [to_float(r.get("t_rel", "")) for r in seg]
        finite_t = [t for t in t_vals if math.isfinite(t)]
        if not finite_t:
            continue
        t_end = max(finite_t)
        tail = [r for r in seg if to_float(r.get("t_rel", "")) >= t_end - avg_tail_sec]
        if not tail:
            tail = seg

        rec = {
            "segment": idx,
            "target_joint": joint,
            "target_deg": target,
            "request_deg": target,
            "rows": len(tail),
            "t_start": min(to_float(r.get("t_rel", "")) for r in tail),
            "t_end": t_end,
        }
        for j in range(4):
            rec[f"j{j}_actual_mean"] = mean(to_float(r.get(f"j{j}_actual", "")) for r in tail)
            rec[f"j{j}_target_mean"] = mean(to_float(r.get(f"j{j}_target", "")) for r in tail)
        for m in range(5):
            rec[f"m{m}_now_mean"] = mean(to_float(r.get(f"m{m}_now", "")) for r in tail)
            rec[f"m{m}_map_mean"] = mean(to_float(r.get(f"m{m}_map", "")) for r in tail)
            rec[f"m{m}_solver_mean"] = mean(to_float(r.get(f"m{m}_solver", "")) for r in tail)
            rec[f"m{m}_cmd_mean"] = mean(to_float(r.get(f"m{m}_cmd", "")) for r in tail)
            rec[f"m{m}_load_mean"] = mean(to_float(r.get(f"m{m}_load", "")) for r in tail)
            rec[f"m{m}_bias_mean"] = mean(to_float(r.get(f"m{m}_bias", "")) for r in tail)
        out.append(rec)
        last_kept_target = target
    return out


def summarize_collect_metadata(rows, joint, avg_tail_sec):
    segments = []
    current = []
    current_key = None

    for row in rows:
        try:
            collect_joint = int(float(row.get("collect_joint", "")))
        except Exception:
            continue
        if collect_joint != joint:
            continue
        point = row.get("collect_point", "")
        request = row.get("collect_request_deg", "")
        if point == "" or request == "":
            continue
        key = (point, request)
        if current and key != current_key:
            segments.append((current_key, current))
            current = [row]
            current_key = key
        else:
            current.append(row)
            current_key = key
    if current:
        segments.append((current_key, current))

    out = []
    for idx, ((point, request), seg) in enumerate(segments):
        t_vals = [to_float(r.get("t_rel", "")) for r in seg]
        finite_t = [t for t in t_vals if math.isfinite(t)]
        if not finite_t:
            continue
        t_end = max(finite_t)
        tail = [r for r in seg if to_float(r.get("t_rel", "")) >= t_end - avg_tail_sec]
        if not tail:
            tail = seg
        effective_target = mean(to_float(r.get(f"j{joint}_target", "")) for r in tail)
        rec = {
            "segment": idx,
            "target_joint": joint,
            "target_deg": effective_target,
            "request_deg": to_float(request),
            "rows": len(tail),
            "t_start": min(to_float(r.get("t_rel", "")) for r in tail),
            "t_end": t_end,
        }
        for j in range(4):
            rec[f"j{j}_actual_mean"] = mean(to_float(r.get(f"j{j}_actual", "")) for r in tail)
            rec[f"j{j}_target_mean"] = mean(to_float(r.get(f"j{j}_target", "")) for r in tail)
        for m in range(5):
            rec[f"m{m}_now_mean"] = mean(to_float(r.get(f"m{m}_now", "")) for r in tail)
            rec[f"m{m}_map_mean"] = mean(to_float(r.get(f"m{m}_map", "")) for r in tail)
            rec[f"m{m}_solver_mean"] = mean(to_float(r.get(f"m{m}_solver", "")) for r in tail)
            rec[f"m{m}_cmd_mean"] = mean(to_float(r.get(f"m{m}_cmd", "")) for r in tail)
            rec[f"m{m}_load_mean"] = mean(to_float(r.get(f"m{m}_load", "")) for r in tail)
            rec[f"m{m}_bias_mean"] = mean(to_float(r.get(f"m{m}_bias", "")) for r in tail)
        out.append(rec)
    return out


def main():
    parser = argparse.ArgumentParser(description="Summarize slow feedforward collection CSV into averaged samples.")
    parser.add_argument("csv_path")
    parser.add_argument("--joint", type=int, default=3)
    parser.add_argument("--all-joints", action="store_true")
    parser.add_argument("--avg-tail-sec", type=float, default=1.0)
    parser.add_argument("--target-step-min", type=float, default=0.05)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    samples = []
    joints = range(4) if args.all_joints else [args.joint]
    has_collect_metadata = any(row.get("collect_joint", "") != "" for row in rows)
    for joint in joints:
        if has_collect_metadata:
            samples.extend(summarize_collect_metadata(rows, joint, args.avg_tail_sec))
        else:
            samples.extend(summarize(rows, joint, args.target_step_min, args.avg_tail_sec))
    if not samples:
        print("No samples found.")
        return 1

    out_path = args.out or os.path.splitext(args.csv_path)[0] + "_ff_samples.csv"
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
        writer.writeheader()
        writer.writerows(samples)
    print(f"Saved {len(samples)} samples to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
