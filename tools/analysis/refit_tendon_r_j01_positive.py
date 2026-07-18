from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


DEFAULT_FILES = [
    "linearity_20260717_014216.csv",
    "linearity_20260717_015536.csv",
    "linearity_20260717_020423.csv",
    "linearity_20260717_105037.csv",
    "linearity_20260717_120136.csv",
    "linearity_20260717_122631.csv",
    "linearity_20260717_125748.csv",
    "linearity_20260717_131726.csv",
    "linearity_20260717_134547.csv",
    "linearity_20260717_145844.csv",
    "linearity_20260718_015256.csv",
    "linearity_20260718_021551.csv",
]

JOINT_COLUMNS = [f"joint_j{i:02d}_deg" for i in range(4)]
JOINT_VALID_COLUMNS = [f"joint_j{i:02d}_valid" for i in range(4)]
TENSION_COLUMNS = [f"tension_m{i:02d}_n" for i in range(5)]
SERVO_COLUMNS = [f"servo_m{i:02d}_abs" for i in range(5)]
DELTA_JOINT_COLUMNS = [f"delta_joint_j{i:02d}_deg" for i in range(4)]
DELTA_TENSION_COLUMNS = [f"delta_tension_m{i:02d}_n" for i in range(5)]
DELTA_SERVO_COLUMNS = [f"delta_servo_m{i:02d}_abs" for i in range(5)]

COUNTS_PER_MM = 200.0
COMPLIANCE_MM_PER_N = np.array(
    [1.0 / 16.25, 1.0 / 16.25, 1.0 / 20.3125, 1.0 / 21.5313, 1.0 / 23.1563],
    dtype=float,
)

# 122631 has a persistently invalid M02 force channel (about +621..670 N).
# 131726 was explicitly excluded as a whole by the experimenter.
FIT_FILES = [
    "linearity_20260717_014216.csv",
    "linearity_20260717_015536.csv",
    "linearity_20260717_105037.csv",
    "linearity_20260717_120136.csv",
    "linearity_20260717_125748.csv",
    "linearity_20260717_134547.csv",
    "linearity_20260717_145844.csv",
    "linearity_20260718_015256.csv",
    "linearity_20260718_021551.csv",
]

READ_COLUMNS = [
    "phase",
    "motor_j",
    "delta_counts",
    "actual_delta_counts",
    "repeat_id",
    "trial_index",
    "sequence_step",
    "is_baseline",
    "baseline_sample_index",
    "settle_elapsed_s",
    "stable_hold_s",
    "sample_index",
    *JOINT_COLUMNS,
    *JOINT_VALID_COLUMNS,
    *TENSION_COLUMNS,
    *SERVO_COLUMNS,
    *DELTA_JOINT_COLUMNS,
    *DELTA_TENSION_COLUMNS,
    *DELTA_SERVO_COLUMNS,
]


def load_csv(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, usecols=lambda name: name in READ_COLUMNS, low_memory=False)
    for column in frame.columns:
        if column not in {"phase", "motor_j"}:
            frame[column] = pd.to_numeric(frame[column], errors="coerce")
    return frame


def summarize(path: Path, frame: pd.DataFrame) -> dict[str, object]:
    joint_valid = np.ones(len(frame), dtype=bool)
    for column in JOINT_VALID_COLUMNS:
        joint_valid &= frame[column].fillna(0).to_numpy() == 1
    j01 = frame["joint_j01_deg"]
    tensions = frame[TENSION_COLUMNS]
    phase_counts = frame["phase"].fillna("<blank>").value_counts().to_dict()
    return {
        "file": path.name,
        "rows": int(len(frame)),
        "phase_counts": {str(k): int(v) for k, v in phase_counts.items()},
        "all_joints_valid_rows": int(joint_valid.sum()),
        "j01_min_valid_deg": float(j01[joint_valid].min()),
        "j01_max_valid_deg": float(j01[joint_valid].max()),
        "j01_nonpositive_valid_rows": int(((j01 <= 0) & joint_valid).sum()),
        "positive_tension_rows": int((tensions.gt(0).any(axis=1)).sum()),
        "tension_min_n": {column: float(tensions[column].min()) for column in TENSION_COLUMNS},
        "tension_max_n": {column: float(tensions[column].max()) for column in TENSION_COLUMNS},
    }


def endpoint_rows(frame: pd.DataFrame) -> pd.DataFrame:
    mask = frame["phase"].eq("perturb_post_hold")
    for column in JOINT_VALID_COLUMNS:
        mask &= frame[column].eq(1)
    mask &= frame["joint_j01_deg"].gt(0)
    # Positive relative force means slack/invalid for the current sign convention.
    mask &= frame[TENSION_COLUMNS].le(0).all(axis=1)
    mask &= frame[DELTA_JOINT_COLUMNS + DELTA_TENSION_COLUMNS + DELTA_SERVO_COLUMNS].notna().all(axis=1)
    return frame.loc[mask].copy()


def design_and_output(frame: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    x = np.deg2rad(frame[DELTA_JOINT_COLUMNS].to_numpy(dtype=float))
    delta_force = frame[DELTA_TENSION_COLUMNS].to_numpy(dtype=float)
    delta_length = frame[DELTA_SERVO_COLUMNS].to_numpy(dtype=float) / COUNTS_PER_MM
    y = delta_length - delta_force * COMPLIANCE_MM_PER_N[np.newaxis, :]
    return x, y


def fit_ols(frames: dict[str, pd.DataFrame]) -> dict[str, object]:
    selected = {name: endpoint_rows(frame) for name, frame in frames.items()}
    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    dataset_parts: list[str] = []
    for name, frame in selected.items():
        x, y = design_and_output(frame)
        x_parts.append(x)
        y_parts.append(y)
        dataset_parts.extend([name] * len(frame))
    x_all = np.vstack(x_parts)
    y_all = np.vstack(y_parts)
    design = np.column_stack([x_all, np.ones(len(x_all))])
    coefficients, _, rank, singular_values = np.linalg.lstsq(design, y_all, rcond=None)
    r_matrix = coefficients[:4, :].T
    e_global = coefficients[4, :]
    prediction = design @ coefficients
    residual = y_all - prediction
    rmse = np.sqrt(np.mean(residual**2, axis=0))
    nrmse = rmse / np.maximum(np.ptp(y_all, axis=0), 1e-12)

    per_dataset: dict[str, object] = {}
    start = 0
    for name, frame in selected.items():
        count = len(frame)
        x = x_all[start : start + count]
        y = y_all[start : start + count]
        # Pose-specific e with the pooled R fixed.
        e_pose = np.mean(y - x @ r_matrix.T, axis=0)
        pred_pose = x @ r_matrix.T + e_pose
        pose_rmse = np.sqrt(np.mean((y - pred_pose) ** 2, axis=0))
        original = frames[name]
        post_hold_total = int(original["phase"].eq("perturb_post_hold").sum())
        per_dataset[name] = {
            "post_hold_total": post_hold_total,
            "selected_rows": count,
            "excluded_rows": post_hold_total - count,
            "j01_min_all_valid_deg": float(
                original.loc[original["joint_j01_valid"].eq(1), "joint_j01_deg"].min()
            ),
            "j01_max_all_valid_deg": float(
                original.loc[original["joint_j01_valid"].eq(1), "joint_j01_deg"].max()
            ),
            "e_mm": e_pose.tolist(),
            "rmse_mm": pose_rmse.tolist(),
        }
        start += count

    return {
        "model": "delta_L_counts/200 = K*delta_F + R*delta_theta_rad + e",
        "K_mm_per_N": COMPLIANCE_MM_PER_N.tolist(),
        "files": list(frames),
        "total_selected_rows": int(len(x_all)),
        "rank": int(rank),
        "singular_values": singular_values.tolist(),
        "condition_number": float(singular_values[0] / singular_values[-1]),
        "R_mm_per_rad": r_matrix.tolist(),
        "e_global_mm": e_global.tolist(),
        "rmse_mm": rmse.tolist(),
        "nrmse": nrmse.tolist(),
        "per_dataset": per_dataset,
    }


def compare_old_fit_variants(frames: dict[str, pd.DataFrame]) -> list[dict[str, object]]:
    target = np.array(
        [
            [4.2455, 1.6900, 1.7266, -0.2066],
            [-4.0940, 5.1808, 0.9432, 0.6716],
            [-0.6760, -3.9189, 3.4368, -0.5300],
            [1.3932, -0.4444, -3.4777, 4.4329],
            [-0.4759, -4.0033, -3.9076, -3.8918],
        ]
    )
    results: list[dict[str, object]] = []
    for phase in ("perturb", "perturb_post_hold"):
        for tension_filter in (False, True):
            for force_sign in (-1.0, 0.0, 1.0):
                xs: list[np.ndarray] = []
                ys: list[np.ndarray] = []
                for frame in frames.values():
                    mask = frame["phase"].eq(phase)
                    for column in JOINT_VALID_COLUMNS:
                        mask &= frame[column].eq(1)
                    mask &= frame["joint_j01_deg"].gt(0)
                    if tension_filter:
                        mask &= frame[TENSION_COLUMNS].le(0).all(axis=1)
                    part = frame.loc[mask]
                    x = np.deg2rad(part[DELTA_JOINT_COLUMNS].to_numpy(dtype=float))
                    df = part[DELTA_TENSION_COLUMNS].to_numpy(dtype=float)
                    dl = part[DELTA_SERVO_COLUMNS].to_numpy(dtype=float) / COUNTS_PER_MM
                    # force_sign=-1: signed sensor force; +1: positive physical
                    # tension magnitude; 0: omit the compliance correction.
                    y = dl + force_sign * df * COMPLIANCE_MM_PER_N[np.newaxis, :]
                    xs.append(x)
                    ys.append(y)
                x_all = np.vstack(xs)
                y_all = np.vstack(ys)
                design = np.column_stack([x_all, np.ones(len(x_all))])
                coef, *_ = np.linalg.lstsq(design, y_all, rcond=None)
                r_matrix = coef[:4, :].T
                results.append(
                    {
                        "phase": phase,
                        "tension_filter": tension_filter,
                        "force_sign": force_sign,
                        "rows": len(x_all),
                        "distance_to_previous_R": float(np.linalg.norm(r_matrix - target)),
                        "R": r_matrix.tolist(),
                    }
                )
    return sorted(results, key=lambda item: item["distance_to_previous_R"])


STRUCTURAL_COLUMNS = {
    0: [0, 1],
    1: [0, 1],
    2: [0, 1, 2],
    3: [0, 1, 3],
    4: [1, 2, 3],
}


def _weighted_lstsq(design: np.ndarray, output: np.ndarray, robust: bool) -> np.ndarray:
    weights = np.ones(len(output), dtype=float)
    coefficient = np.zeros(design.shape[1], dtype=float)
    for _ in range(30 if robust else 1):
        root_weight = np.sqrt(weights)
        coefficient, *_ = np.linalg.lstsq(
            design * root_weight[:, np.newaxis],
            output * root_weight,
            rcond=None,
        )
        if not robust:
            break
        residual = output - design @ coefficient
        center = np.median(residual)
        scale = 1.4826 * np.median(np.abs(residual - center))
        if scale < 1e-9:
            break
        normalized = np.abs(residual - center) / (1.345 * scale)
        new_weights = np.ones_like(weights)
        outlier = normalized > 1.0
        new_weights[outlier] = 1.0 / normalized[outlier]
        if np.max(np.abs(new_weights - weights)) < 1e-6:
            weights = new_weights
            break
        weights = new_weights
    return coefficient


def fit_with_pose_intercepts(
    frames: dict[str, pd.DataFrame],
    *,
    structured: bool,
    robust: bool,
) -> dict[str, object]:
    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    dataset_index: list[int] = []
    selection: dict[str, dict[str, int]] = {}
    names = list(frames)
    for dataset_id, (name, original) in enumerate(frames.items()):
        frame = endpoint_rows(original)
        x, _ = design_and_output(frame)
        delta_force_signed = frame[DELTA_TENSION_COLUMNS].to_numpy(dtype=float)
        delta_length = frame[DELTA_SERVO_COLUMNS].to_numpy(dtype=float) / COUNTS_PER_MM
        # Sensor tension is negative when taut, while F_phys is the positive
        # tensile-force magnitude: delta_F_phys = -delta_F_sensor.
        y = delta_length + delta_force_signed * COMPLIANCE_MM_PER_N[np.newaxis, :]
        x_parts.append(x)
        y_parts.append(y)
        dataset_index.extend([dataset_id] * len(frame))
        selection[name] = {
            "post_hold_total": int(original["phase"].eq("perturb_post_hold").sum()),
            "selected": int(len(frame)),
        }
    x_all = np.vstack(x_parts)
    y_all = np.vstack(y_parts)
    dataset_index_array = np.asarray(dataset_index, dtype=int)
    dummies = np.zeros((len(x_all), len(names)), dtype=float)
    dummies[np.arange(len(x_all)), dataset_index_array] = 1.0

    r_matrix = np.zeros((5, 4), dtype=float)
    e_by_dataset = np.zeros((len(names), 5), dtype=float)
    residual = np.zeros_like(y_all)
    ranks: list[int] = []
    for motor in range(5):
        joint_columns = STRUCTURAL_COLUMNS[motor] if structured else [0, 1, 2, 3]
        design = np.column_stack([x_all[:, joint_columns], dummies])
        coefficient = _weighted_lstsq(design, y_all[:, motor], robust)
        r_matrix[motor, joint_columns] = coefficient[: len(joint_columns)]
        e_by_dataset[:, motor] = coefficient[len(joint_columns) :]
        residual[:, motor] = y_all[:, motor] - design @ coefficient
        ranks.append(int(np.linalg.matrix_rank(design)))

    rmse = np.sqrt(np.mean(residual**2, axis=0))
    nrmse = rmse / np.maximum(np.ptp(y_all, axis=0), 1e-12)
    return {
        "structured": structured,
        "robust_huber": robust,
        "files": names,
        "selection": selection,
        "rows": int(len(x_all)),
        "ranks": ranks,
        "R_mm_per_rad": r_matrix.tolist(),
        "e_by_dataset_mm": {
            name: e_by_dataset[index].tolist() for index, name in enumerate(names)
        },
        "rmse_mm": rmse.tolist(),
        "nrmse": nrmse.tolist(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-data", type=Path, default=Path("run_data"))
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--variants", action="store_true")
    args = parser.parse_args()

    summaries: list[dict[str, object]] = []
    for name in DEFAULT_FILES:
        path = args.run_data / name
        if not path.exists():
            summaries.append({"file": name, "error": "missing"})
            continue
        frame = load_csv(path)
        summaries.append(summarize(path, frame))
    if args.summary_only:
        print(json.dumps(summaries, ensure_ascii=False, indent=2))
        return

    fit_frames: dict[str, pd.DataFrame] = {}
    for name in FIT_FILES:
        path = args.run_data / name
        fit_frames[name] = load_csv(path)
    if args.variants:
        old_names = FIT_FILES[:-2]
        print(
            json.dumps(
                compare_old_fit_variants({name: fit_frames[name] for name in old_names}),
                ensure_ascii=False,
                indent=2,
            )
        )
        return
    result = {
        "dataset_review": summaries,
        "fit": fit_ols(fit_frames),
        "pose_intercept_fits": {
            "unconstrained_ols": fit_with_pose_intercepts(
                fit_frames, structured=False, robust=False
            ),
            "unconstrained_huber": fit_with_pose_intercepts(
                fit_frames, structured=False, robust=True
            ),
            "structured_ols": fit_with_pose_intercepts(
                fit_frames, structured=True, robust=False
            ),
            "structured_huber": fit_with_pose_intercepts(
                fit_frames, structured=True, robust=True
            ),
            "old_only_structured_huber": fit_with_pose_intercepts(
                {name: fit_frames[name] for name in FIT_FILES[:-2]},
                structured=True,
                robust=True,
            ),
        },
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
