"""Sequence generation and offline analysis for multi-input identification."""

from __future__ import annotations

import csv
import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


JOINTS = (0, 1, 2, 3)
MOTORS = (0, 1, 2, 3, 4)


class MultiInputAnalysisError(ValueError):
    pass


@dataclass(frozen=True)
class InputIndependence:
    rank: int
    condition_number: float
    max_abs_correlation: float
    singular_values: tuple[float, ...]


@dataclass(frozen=True)
class MultiInputAnalysisResult:
    markdown: str
    markdown_path: Path
    output_dir: Path
    matrix_deg_per_count: tuple[tuple[float, ...], ...]
    endpoint_rows: int
    train_rows: int
    validation_rows: int
    test_rows: int
    rank: int
    max_abs_correlation: float


def parse_number_vector(text: str, length: int, *, name: str) -> list[float]:
    normalized = str(text).replace(",", ";").replace("，", ";").replace("；", ";")
    parts = [part.strip() for part in normalized.split(";") if part.strip()]
    if len(parts) != length:
        raise ValueError(f"{name}需要 {length} 个数，用分号隔开；当前为 {len(parts)} 个")
    try:
        values = [float(part) for part in parts]
    except ValueError as exc:
        raise ValueError(f"{name}包含非数字") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name}必须都是有限数")
    return values


def parse_split_ratios(text: str) -> tuple[float, float, float]:
    values = parse_number_vector(text, 3, name="训练/验证/测试比例")
    if any(value <= 0.0 for value in values):
        raise ValueError("训练/验证/测试比例都必须大于 0")
    total = sum(values)
    return tuple(value / total for value in values)  # type: ignore[return-value]


def split_labels(count: int, ratios: Iterable[float]) -> list[str]:
    if count < 3:
        raise ValueError("样本数至少为 3，才能划分训练/验证/测试集")
    train_ratio, validation_ratio, _ = tuple(ratios)
    train_count = max(1, int(round(count * train_ratio)))
    validation_count = max(1, int(round(count * validation_ratio)))
    if train_count + validation_count >= count:
        overflow = train_count + validation_count - (count - 1)
        if validation_count > overflow:
            validation_count -= overflow
        else:
            train_count = max(1, train_count - (overflow - validation_count + 1))
            validation_count = 1
    test_count = count - train_count - validation_count
    return ["train"] * train_count + ["validation"] * validation_count + ["test"] * test_count


def _lfsr_bit(state: int) -> tuple[int, int]:
    # 16-bit maximal-length taps: x^16 + x^14 + x^13 + x^11 + 1.
    output = state & 1
    feedback = ((state >> 0) ^ (state >> 2) ^ (state >> 3) ^ (state >> 5)) & 1
    state = ((state >> 1) | (feedback << 15)) & 0xFFFF
    return state or 0xACE1, output


def build_multi_input_sequence(
    mode: str,
    amplitudes: Iterable[float],
    count: int,
    seed: int,
    *,
    step_period_s: float = 1.0,
    frequencies_hz: Iterable[float] | None = None,
) -> np.ndarray:
    """Return an N x 5 matrix of independent relative motor-position commands."""

    mode_key = str(mode).strip().lower()
    amplitudes_array = np.asarray(tuple(amplitudes), dtype=float)
    if amplitudes_array.shape != (len(MOTORS),):
        raise ValueError("幅值必须包含 M00-M04 共 5 路")
    if count < 10:
        raise ValueError("联合辨识样本数至少为 10")
    if np.any(~np.isfinite(amplitudes_array)) or np.any(amplitudes_array <= 0.0):
        raise ValueError("五路幅值都必须为有限正数")

    rng = random.Random(int(seed))
    if mode_key == "random":
        levels = (-1.0, -0.5, 0.0, 0.5, 1.0)
        sequence = np.asarray(
            [[rng.choice(levels) for _ in MOTORS] for _ in range(count)],
            dtype=float,
        )
        for row_index in range(count):
            if not np.any(sequence[row_index]):
                motor = rng.randrange(len(MOTORS))
                sequence[row_index, motor] = rng.choice((-1.0, 1.0))
        sequence *= amplitudes_array
    elif mode_key == "prbs":
        states = [((int(seed) + 0x1F3D * (motor + 1)) & 0xFFFF) or (motor + 1) for motor in MOTORS]
        sequence = np.zeros((count, len(MOTORS)), dtype=float)
        for row_index in range(count):
            for motor in MOTORS:
                states[motor], bit = _lfsr_bit(states[motor])
                sequence[row_index, motor] = amplitudes_array[motor] if bit else -amplitudes_array[motor]
    elif mode_key == "multisine":
        frequencies = np.asarray(tuple(frequencies_hz or ()), dtype=float)
        if frequencies.shape != (len(MOTORS),):
            raise ValueError("多正弦模式需要 M00-M04 共 5 个频率")
        if step_period_s <= 0.0 or np.any(frequencies <= 0.0):
            raise ValueError("采样周期和五路频率都必须大于 0")
        phases = np.asarray([rng.uniform(0.0, 2.0 * math.pi) for _ in MOTORS])
        sample_times = np.arange(count, dtype=float)[:, None] * float(step_period_s)
        sequence = amplitudes_array[None, :] * np.sin(
            2.0 * math.pi * sample_times * frequencies[None, :] + phases[None, :]
        )
    else:
        raise ValueError("激励模式必须是 random、prbs 或 multisine")

    rounded = np.rint(sequence).astype(int)
    for row_index in range(count):
        if not np.any(rounded[row_index]):
            motor = row_index % len(MOTORS)
            rounded[row_index, motor] = max(1, int(round(amplitudes_array[motor])))
    return rounded


def input_independence_metrics(sequence: np.ndarray) -> InputIndependence:
    matrix = np.asarray(sequence, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != len(MOTORS):
        raise ValueError("输入序列必须为 N x 5")
    centered = matrix - np.mean(matrix, axis=0, keepdims=True)
    rank = int(np.linalg.matrix_rank(centered))
    singular_values = np.linalg.svd(centered, compute_uv=False)
    smallest = float(singular_values[-1]) if len(singular_values) else 0.0
    condition = float(singular_values[0] / smallest) if smallest > 1e-12 else math.inf
    std = np.std(centered, axis=0)
    if np.any(std <= 1e-12):
        max_correlation = 1.0
    else:
        correlation = np.corrcoef(centered, rowvar=False)
        upper = np.abs(correlation[np.triu_indices(len(MOTORS), 1)])
        max_correlation = float(np.max(upper)) if upper.size else 0.0
    return InputIndependence(
        rank=rank,
        condition_number=condition,
        max_abs_correlation=max_correlation,
        singular_values=tuple(float(value) for value in singular_values),
    )


def _float_value(row: dict[str, str], key: str) -> float:
    text = str(row.get(key, "")).strip()
    if not text:
        raise MultiInputAnalysisError(f"CSV 字段 {key} 存在空值")
    try:
        value = float(text)
    except ValueError as exc:
        raise MultiInputAnalysisError(f"CSV 字段 {key} 包含非数字: {text}") from exc
    if not math.isfinite(value):
        raise MultiInputAnalysisError(f"CSV 字段 {key} 包含非有限数")
    return value


def _fit_ridge(inputs: np.ndarray, outputs: np.ndarray, lambda_id: float) -> np.ndarray:
    # inputs: N x 5; outputs: N x 4. Return 4 x 5.
    u = inputs.T
    theta = outputs.T
    regularizer = float(lambda_id) * np.eye(len(MOTORS))
    return theta @ u.T @ np.linalg.inv(u @ u.T + regularizer)


def _split_metrics(j_matrix: np.ndarray, inputs: np.ndarray, outputs: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    prediction = inputs @ j_matrix.T
    residual = outputs - prediction
    rmse = np.sqrt(np.mean(residual * residual, axis=0))
    spans = np.ptp(outputs, axis=0)
    nrmse = np.divide(rmse, spans, out=np.full_like(rmse, np.nan), where=spans > 1e-12)
    nonzero = np.abs(outputs) > 1e-9
    sign_correct = np.divide(
        np.sum((np.sign(prediction) == np.sign(outputs)) & nonzero, axis=0),
        np.sum(nonzero, axis=0),
        out=np.full(len(JOINTS), np.nan),
        where=np.sum(nonzero, axis=0) > 0,
    )
    return rmse, nrmse, sign_correct


def analyze_multi_input_csv(csv_path: Path, lambda_id: float, output_root: Path) -> MultiInputAnalysisResult:
    csv_path = Path(csv_path)
    if not csv_path.is_file():
        raise MultiInputAnalysisError(f"CSV 文件不存在: {csv_path}")
    if not math.isfinite(lambda_id) or lambda_id < 0.0:
        raise MultiInputAnalysisError("岭回归 λ 必须是非负有限数")

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as fp:
        reader = csv.DictReader(fp)
        fieldnames = set(reader.fieldnames or [])
        required = {"phase", "dataset_split"}
        required.update(f"delta_servo_m{motor:02d}_abs" for motor in MOTORS)
        required.update(f"delta_joint_j{joint:02d}_deg" for joint in JOINTS)
        missing = sorted(required - fieldnames)
        if missing:
            raise MultiInputAnalysisError("CSV 缺少联合辨识字段: " + ", ".join(missing))
        rows = [dict(row) for row in reader if str(row.get("phase", "")).strip().lower() == "excitation"]

    if len(rows) < 10:
        raise MultiInputAnalysisError(f"有效最终点只有 {len(rows)} 个，至少需要 10 个")

    inputs = np.asarray(
        [[_float_value(row, f"delta_servo_m{motor:02d}_abs") for motor in MOTORS] for row in rows],
        dtype=float,
    )
    outputs = np.asarray(
        [[_float_value(row, f"delta_joint_j{joint:02d}_deg") for joint in JOINTS] for row in rows],
        dtype=float,
    )
    split = np.asarray([str(row.get("dataset_split", "")).strip().lower() for row in rows])
    masks = {name: split == name for name in ("train", "validation", "test")}
    counts = {name: int(np.sum(mask)) for name, mask in masks.items()}
    if any(count <= 0 for count in counts.values()):
        raise MultiInputAnalysisError(
            "训练/验证/测试集必须都有最终点；当前 "
            + ", ".join(f"{name}={count}" for name, count in counts.items())
        )

    independence = input_independence_metrics(inputs[masks["train"]])
    if independence.rank < len(MOTORS):
        raise MultiInputAnalysisError(f"训练集输入矩阵秩为 {independence.rank}，不足 5，无法辨识完整 4x5 矩阵")
    j_matrix = _fit_ridge(inputs[masks["train"]], outputs[masks["train"]], lambda_id)

    metrics = {
        name: _split_metrics(j_matrix, inputs[mask], outputs[mask])
        for name, mask in masks.items()
    }
    mode = str(rows[0].get("excitation_mode", "")).strip() or "unknown"
    seed = str(rows[0].get("random_seed", "")).strip() or "--"
    condition_text = f"{independence.condition_number:.2f}" if math.isfinite(independence.condition_number) else "∞"

    lines = [
        "# 多输入联合辨识结果",
        "",
        f"- 数据：`{csv_path.name}`",
        f"- 激励：{mode}，seed={seed}，最终点 N={len(rows)}",
        f"- 划分：训练 {counts['train']} / 验证 {counts['validation']} / 测试 {counts['test']}",
        f"- 岭回归：λ={lambda_id:g}",
        f"- 训练输入：rank={independence.rank}/5，条件数={condition_text}，最大通道相关系数 |r|={independence.max_abs_correlation:.3f}",
        "",
        "## 局部影响矩阵",
        "",
        r"定义：$\Delta\theta \approx J\Delta u$。下表单位为 **度/100 counts**，行为 J00-J03，列为 M00-M04。",
        "",
        "|关节\\电机|M00|M01|M02|M03|M04|",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for joint in JOINTS:
        values = "|".join(f"{100.0 * j_matrix[joint, motor]:.4f}" for motor in MOTORS)
        lines.append(f"|J{joint:02d}|{values}|")

    lines.extend(
        [
            "",
            "程序使用的度/count矩阵：",
            "",
            "```text",
            "J = [",
        ]
    )
    for joint in JOINTS:
        comma = "," if joint < JOINTS[-1] else ""
        lines.append("  [" + ", ".join(f"{value:.9f}" for value in j_matrix[joint]) + "]" + comma)
    lines.extend(["]", "```", "", "## 独立数据集预测误差", ""])
    lines.append("|数据集|指标|J00|J01|J02|J03|")
    lines.append("|---|---|---:|---:|---:|---:|")
    labels = {"train": "训练", "validation": "验证", "test": "测试"}
    for name in ("train", "validation", "test"):
        rmse, nrmse, sign_correct = metrics[name]
        lines.append(f"|{labels[name]}|RMSE (°)|" + "|".join(f"{value:.4f}" for value in rmse) + "|")
        lines.append(
            f"|{labels[name]}|NRMSE|"
            + "|".join("--" if not math.isfinite(value) else f"{100.0 * value:.2f}%" for value in nrmse)
            + "|"
        )
        lines.append(
            f"|{labels[name]}|方向正确率|"
            + "|".join("--" if not math.isfinite(value) else f"{100.0 * value:.1f}%" for value in sign_correct)
            + "|"
        )

    validation_nrmse = metrics["validation"][1]
    test_nrmse = metrics["test"][1]
    finite_holdout = np.concatenate(
        [validation_nrmse[np.isfinite(validation_nrmse)], test_nrmse[np.isfinite(test_nrmse)]]
    )
    mean_holdout = float(np.mean(finite_holdout)) if finite_holdout.size else math.nan
    lines.extend(["", "## 结论", ""])
    if independence.max_abs_correlation >= 0.90:
        lines.append("- 输入通道相关性过高，矩阵元素可能不稳定；建议增加样本或更换 seed/频率。")
    else:
        lines.append("- 训练输入满秩且未出现长期固定比例，可辨识完整 4×5 矩阵。")
    if math.isfinite(mean_holdout):
        if mean_holdout <= 0.10:
            assessment = "当前工作点的联合线性预测较好"
        elif mean_holdout <= 0.20:
            assessment = "当前工作点可近似线性，但仍有明显残差"
        else:
            assessment = "独立数据误差偏大，应考虑方向双模型、分段线性或非线性模型"
        lines.append(f"- 验证/测试平均 NRMSE={100.0 * mean_holdout:.2f}%：{assessment}。")
    lines.append("- 模型评价以验证集和测试集为准，不以训练误差单独下结论。")

    markdown = "\n".join(lines) + "\n"
    output_dir = Path(output_root) / csv_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    markdown_path = output_dir / f"{csv_path.stem}_multi_input.md"
    markdown_path.write_text(markdown, encoding="utf-8")
    return MultiInputAnalysisResult(
        markdown=markdown,
        markdown_path=markdown_path,
        output_dir=output_dir,
        matrix_deg_per_count=tuple(tuple(float(value) for value in row) for row in j_matrix),
        endpoint_rows=len(rows),
        train_rows=counts["train"],
        validation_rows=counts["validation"],
        test_rows=counts["test"],
        rank=independence.rank,
        max_abs_correlation=independence.max_abs_correlation,
    )


def run_selftest() -> None:
    for mode in ("random", "prbs", "multisine"):
        sequence = build_multi_input_sequence(
            mode,
            [200] * 5,
            120,
            20260716,
            step_period_s=2.0,
            frequencies_hz=[0.037, 0.053, 0.071, 0.089, 0.113],
        )
        metrics = input_independence_metrics(sequence)
        if sequence.shape != (120, 5) or metrics.rank != 5:
            raise AssertionError((mode, sequence.shape, metrics))
    labels = split_labels(101, (0.6, 0.2, 0.2))
    if len(labels) != 101 or set(labels) != {"train", "validation", "test"}:
        raise AssertionError(labels)
    true_j = np.asarray(
        [
            [0.012, -0.008, 0.003, 0.001, -0.004],
            [0.004, 0.010, -0.006, 0.002, -0.003],
            [-0.002, 0.001, 0.008, -0.005, 0.004],
            [0.001, -0.004, 0.002, 0.011, -0.009],
        ]
    )
    sequence = build_multi_input_sequence("random", [200] * 5, 60, 17)
    outputs = sequence @ true_j.T
    labels = split_labels(60, (0.6, 0.2, 0.2))
    fieldnames = (
        ["phase", "dataset_split", "excitation_mode", "random_seed"]
        + [f"delta_servo_m{motor:02d}_abs" for motor in MOTORS]
        + [f"delta_joint_j{joint:02d}_deg" for joint in JOINTS]
    )
    with tempfile.TemporaryDirectory() as temp_dir:
        csv_path = Path(temp_dir) / "synthetic.csv"
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            writer.writeheader()
            for row_index in range(len(sequence)):
                row: dict[str, object] = {
                    "phase": "excitation",
                    "dataset_split": labels[row_index],
                    "excitation_mode": "random",
                    "random_seed": 17,
                }
                row.update(
                    {f"delta_servo_m{motor:02d}_abs": sequence[row_index, motor] for motor in MOTORS}
                )
                row.update(
                    {f"delta_joint_j{joint:02d}_deg": outputs[row_index, joint] for joint in JOINTS}
                )
                writer.writerow(row)
        result = analyze_multi_input_csv(csv_path, 0.0, Path(temp_dir) / "result")
        if not np.allclose(np.asarray(result.matrix_deg_per_count), true_j, atol=1e-10):
            raise AssertionError(result.matrix_deg_per_count)


if __name__ == "__main__":
    run_selftest()
    print("multi input analysis selftest: OK")
