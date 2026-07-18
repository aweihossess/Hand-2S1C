"""Offline analysis for tendon-drive quasi-static linearity experiments."""

from __future__ import annotations

import csv
import html
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


JOINTS = (0, 1, 2, 3)
MOTORS = (0, 1, 2, 3, 4)
MIN_RESPONSE_SPAN_DEG = 0.10
MAX_LINEAR_NRMSE = 0.10
MAX_HYSTERESIS_RATIO = 0.15
MAX_DIRECTION_GAP = 0.25
MAX_QUADRATIC_IMPROVEMENT = 0.20
MAX_STATE_SLOPE_VARIATION = 0.25


class LinearityAnalysisError(ValueError):
    pass


@dataclass(frozen=True)
class ExperimentSeries:
    state: str
    repeat: str
    motor: int
    x_abs: list[float]
    command_delta: list[float]
    joint_deg: dict[int, list[float]]


@dataclass(frozen=True)
class LinearityAnalysisResult:
    kind: str
    markdown: str
    markdown_path: Path
    plot_path: Path
    output_dir: Path
    valid_series: int
    rejected_series: int


def _float_or_none(value: object) -> Optional[float]:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        value_float = float(text)
    except ValueError:
        return None
    return value_float if math.isfinite(value_float) else None


def _int_or_zero(value: object) -> int:
    number = _float_or_none(value)
    return int(round(number)) if number is not None else 0


def _motor_index(value: object) -> Optional[int]:
    text = str(value or "").strip().upper()
    if text.startswith("M"):
        text = text[1:]
    try:
        motor = int(text)
    except ValueError:
        return None
    return motor if motor in MOTORS else None


def load_linearity_series(csv_path: Path) -> list[ExperimentSeries]:
    csv_path = Path(csv_path)
    if not csv_path.is_file():
        raise LinearityAnalysisError(f"CSV 文件不存在: {csv_path}")

    try:
        with open(csv_path, "r", newline="", encoding="utf-8-sig") as fp:
            reader = csv.DictReader(fp)
            fieldnames = set(reader.fieldnames or [])
            required = {"phase", "state_i", "motor_j", "repeat_id", "sequence_step", "delta_counts"}
            missing = sorted(required - fieldnames)
            if missing:
                raise LinearityAnalysisError("CSV 缺少线性实验字段: " + ", ".join(missing))
            rows = [dict(row) for row in reader if str(row.get("phase", "")).strip().lower() == "perturb"]
    except UnicodeError as exc:
        raise LinearityAnalysisError(f"CSV 编码无法读取: {exc}") from exc

    if not rows:
        raise LinearityAnalysisError("CSV 中没有 phase=perturb 的实验记录。")

    groups: dict[tuple[str, str, int], list[dict[str, str]]] = {}
    for row in rows:
        motor = _motor_index(row.get("motor_j"))
        if motor is None:
            continue
        state = str(row.get("state_i") or row.get("state_label") or "1").strip()
        repeat = str(row.get("repeat_id") or "1").strip()
        groups.setdefault((state, repeat, motor), []).append(row)

    series_list: list[ExperimentSeries] = []
    for (state, repeat, motor), group_rows in sorted(groups.items()):
        group_rows.sort(key=lambda row: (_int_or_zero(row.get("sequence_step")), _int_or_zero(row.get("trial_index"))))
        x_values: list[float] = []
        command_values: list[float] = []
        joint_values: dict[int, list[float]] = {joint: [] for joint in JOINTS}
        valid = True
        for row in group_rows:
            x_value = _float_or_none(row.get(f"servo_m{motor:02d}_abs"))
            if x_value is None:
                x_value = _float_or_none(row.get(f"delta_servo_m{motor:02d}_abs"))
            command_delta = _float_or_none(row.get("actual_delta_counts"))
            if command_delta is None:
                command_delta = _float_or_none(row.get("delta_counts"))
            current_joints: dict[int, float] = {}
            for joint in JOINTS:
                angle = _float_or_none(row.get(f"joint_j{joint:02d}_deg"))
                if angle is None:
                    angle = _float_or_none(row.get(f"delta_joint_j{joint:02d}_deg"))
                if angle is not None:
                    current_joints[joint] = angle
            if x_value is None or command_delta is None or len(current_joints) != len(JOINTS):
                valid = False
                break
            x_values.append(x_value)
            command_values.append(command_delta)
            for joint in JOINTS:
                joint_values[joint].append(current_joints[joint])
        if valid and len(x_values) >= 3:
            series_list.append(
                ExperimentSeries(
                    state=state,
                    repeat=repeat,
                    motor=motor,
                    x_abs=x_values,
                    command_delta=command_values,
                    joint_deg=joint_values,
                )
            )

    if not series_list:
        raise LinearityAnalysisError("没有包含实际舵机 ABS 和 J00-J03 角度的完整扰动序列。")
    return series_list


def _series_excitation_valid(series: ExperimentSeries) -> bool:
    actual_span = max(series.x_abs) - min(series.x_abs)
    command_span = max(series.command_delta) - min(series.command_delta)
    required_span = max(5.0, 0.5 * command_span)
    return actual_span >= required_span


def _split_valid_series(series_list: list[ExperimentSeries]) -> tuple[list[ExperimentSeries], list[ExperimentSeries]]:
    valid = [series for series in series_list if _series_excitation_valid(series)]
    rejected = [series for series in series_list if not _series_excitation_valid(series)]
    if not valid:
        raise LinearityAnalysisError(
            "所有序列的实际 Servo ABS 变化都不足。请检查电机是否真正执行扰动，"
            "不能用 target ABS 代替实际 ABS 计算影响系数。"
        )
    return valid, rejected


def _mean(values: list[float]) -> Optional[float]:
    return statistics.fmean(values) if values else None


def _stdev(values: list[float]) -> Optional[float]:
    return statistics.stdev(values) if len(values) >= 2 else None


def _median(values: list[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def _fmt(value: Optional[float], digits: int = 6) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}"


def _segment_slopes(series: ExperimentSeries, joint: int) -> tuple[list[float], list[float]]:
    upward: list[float] = []
    downward: list[float] = []
    y_values = series.joint_deg[joint]
    for index in range(len(series.x_abs) - 1):
        dx = series.x_abs[index + 1] - series.x_abs[index]
        if abs(dx) < 0.5:
            continue
        slope = (y_values[index + 1] - y_values[index]) / dx
        (upward if dx > 0 else downward).append(slope)
    return upward, downward


def _duplicate_hysteresis(series: ExperimentSeries, joint: int) -> list[float]:
    by_command: dict[float, list[float]] = {}
    for command, angle in zip(series.command_delta, series.joint_deg[joint]):
        by_command.setdefault(round(command, 6), []).append(angle)
    return [max(values) - min(values) for values in by_command.values() if len(values) >= 2]


def _linear_fit(x_values: list[float], y_values: list[float]) -> tuple[float, float, float, float]:
    x_mean = statistics.fmean(x_values)
    y_mean = statistics.fmean(y_values)
    denominator = sum((x - x_mean) ** 2 for x in x_values)
    if denominator <= 1e-12:
        raise LinearityAnalysisError("实际 Servo ABS 没有足够变化，无法拟合。")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_values, y_values)) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [y - (intercept + slope * x) for x, y in zip(x_values, y_values)]
    ss_res = sum(value * value for value in residuals)
    ss_tot = sum((y - y_mean) ** 2 for y in y_values)
    rmse = math.sqrt(ss_res / len(y_values))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return slope, intercept, rmse, r_squared


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> Optional[list[float]]:
    augmented = [list(row) + [vector[index]] for index, row in enumerate(matrix)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * reference
                for current, reference in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][3] for row in range(3)]


def _quadratic_rmse(x_values: list[float], y_values: list[float]) -> Optional[float]:
    x_origin = statistics.fmean(x_values)
    x = [value - x_origin for value in x_values]
    sums = [sum(value ** power for value in x) for power in range(5)]
    matrix = [
        [sums[0], sums[1], sums[2]],
        [sums[1], sums[2], sums[3]],
        [sums[2], sums[3], sums[4]],
    ]
    vector = [
        sum(y_values),
        sum(value * y for value, y in zip(x, y_values)),
        sum(value * value * y for value, y in zip(x, y_values)),
    ]
    coefficients = _solve_3x3(matrix, vector)
    if coefficients is None:
        return None
    residual_sum = sum(
        (y - (coefficients[0] + coefficients[1] * value + coefficients[2] * value * value)) ** 2
        for value, y in zip(x, y_values)
    )
    return math.sqrt(residual_sum / len(y_values))


def _rgb_mix(start: tuple[int, int, int], end: tuple[int, int, int], ratio: float) -> str:
    ratio = max(0.0, min(1.0, ratio))
    rgb = tuple(round(a + (b - a) * ratio) for a, b in zip(start, end))
    return "#" + "".join(f"{value:02x}" for value in rgb)


def _cell_color(value: Optional[float], scale: float, diverging: bool) -> str:
    if value is None or not math.isfinite(value):
        return "#e5e7eb"
    if diverging:
        ratio = min(1.0, abs(value) / max(scale, 1e-12))
        end = (220, 38, 38) if value >= 0 else (37, 99, 235)
        return _rgb_mix((255, 255, 255), end, ratio)
    ratio = min(1.0, max(0.0, value) / max(scale, 1e-12))
    return _rgb_mix((255, 255, 255), (8, 145, 178), ratio)


def _write_heatmap_svg(
    path: Path,
    title: str,
    panels: list[tuple[str, dict[tuple[int, int], Optional[float]], bool, int]],
) -> None:
    cell_w, cell_h = 112, 48
    label_w, panel_gap = 78, 34
    panel_w = label_w + cell_w * len(MOTORS)
    width = 40 + len(panels) * panel_w + max(0, len(panels) - 1) * panel_gap + 40
    height = 112 + cell_h * (len(JOINTS) + 1) + 50
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="32" y="38" font-family="Arial, sans-serif" font-size="22" font-weight="700" fill="#111827">{html.escape(title)}</text>',
    ]
    for panel_index, (panel_title, values, diverging, digits) in enumerate(panels):
        origin_x = 40 + panel_index * (panel_w + panel_gap)
        origin_y = 82
        finite_values = [abs(value) if diverging else value for value in values.values() if value is not None and math.isfinite(value)]
        scale = max(finite_values) if finite_values else 1.0
        pieces.append(
            f'<text x="{origin_x}" y="{origin_y}" font-family="Arial, sans-serif" font-size="16" font-weight="700" fill="#374151">{html.escape(panel_title)}</text>'
        )
        grid_y = origin_y + 22
        for motor in MOTORS:
            x = origin_x + label_w + motor * cell_w
            pieces.append(
                f'<text x="{x + cell_w / 2}" y="{grid_y + 20}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#374151">M{motor:02d}</text>'
            )
        for joint in JOINTS:
            y = grid_y + cell_h * (joint + 1)
            pieces.append(
                f'<text x="{origin_x + label_w - 12}" y="{y + 30}" text-anchor="end" font-family="Arial, sans-serif" font-size="13" fill="#374151">J{joint:02d}</text>'
            )
            for motor in MOTORS:
                value = values.get((motor, joint))
                x = origin_x + label_w + motor * cell_w
                fill = _cell_color(value, scale, diverging)
                text_value = "--" if value is None or not math.isfinite(value) else f"{value:.{digits}f}"
                pieces.extend(
                    [
                        f'<rect x="{x}" y="{y}" width="{cell_w - 2}" height="{cell_h - 2}" rx="4" fill="{fill}" stroke="#d1d5db"/>',
                        f'<text x="{x + (cell_w - 2) / 2}" y="{y + 29}" text-anchor="middle" font-family="Consolas, monospace" font-size="12" fill="#111827">{html.escape(text_value)}</text>',
                    ]
                )
    pieces.extend(
        [
            f'<text x="32" y="{height - 22}" font-family="Arial, sans-serif" font-size="12" fill="#6b7280">Input uses measured servo ABS; gray means insufficient data.</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(pieces), encoding="utf-8")


def _report_preamble(title: str, csv_path: Path, valid_count: int, rejected_count: int) -> list[str]:
    lines = [
        f"# {title}",
        "",
        f"- 数据文件：`{csv_path}`",
        f"- 有效扰动序列：{valid_count}",
        f"- 激励不足序列：{rejected_count}",
        "- 输入量采用实测 Servo ABS，不采用目标位置代替。",
        "",
    ]
    if rejected_count:
        lines.extend(
            [
                "> 注意：部分序列的实际 ABS 变化不足，已从计算中排除。请检查电机目标是否真正执行。",
                "",
            ]
        )
    return lines


def _impact_report(
    csv_path: Path,
    valid: list[ExperimentSeries],
    rejected_count: int,
    plot_path: Path,
) -> str:
    upward: dict[tuple[int, int], list[float]] = {(motor, joint): [] for motor in MOTORS for joint in JOINTS}
    downward: dict[tuple[int, int], list[float]] = {(motor, joint): [] for motor in MOTORS for joint in JOINTS}
    for series in valid:
        for joint in JOINTS:
            up_values, down_values = _segment_slopes(series, joint)
            upward[(series.motor, joint)].extend(up_values)
            downward[(series.motor, joint)].extend(down_values)

    up_mean = {key: _mean(values) for key, values in upward.items()}
    down_mean = {key: _mean(values) for key, values in downward.items()}
    if not any(value is not None for value in up_mean.values()) or not any(value is not None for value in down_mean.values()):
        raise LinearityAnalysisError("缺少完整的正向或反向实际位置变化，无法计算双向影响系数。")

    _write_heatmap_svg(
        plot_path,
        "Step 3 - Local impact coefficients (deg/count)",
        [
            ("ABS increasing", up_mean, True, 5),
            ("ABS decreasing", down_mean, True, 5),
        ],
    )
    lines = _report_preamble("步骤3：局部影响系数", csv_path, len(valid), rejected_count)
    lines.extend(
        [
            "分段系数定义为 `g = Δθ / Δp_actual`，单位为 deg/count。正向和反向分别统计，避免符号及回差被平均掩盖。",
            "",
            "| 电机 | 关节 | ABS增大均值 | 标准差 | n | ABS减小均值 | 标准差 | n | 方向差异 |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for motor in MOTORS:
        for joint in JOINTS:
            up = upward[(motor, joint)]
            down = downward[(motor, joint)]
            up_value, down_value = _mean(up), _mean(down)
            denominator = max(abs(up_value or 0.0), abs(down_value or 0.0), 1e-9)
            gap = abs((up_value or 0.0) - (down_value or 0.0)) / denominator if up_value is not None and down_value is not None else None
            lines.append(
                f"| M{motor:02d} | J{joint:02d} | {_fmt(up_value)} | {_fmt(_stdev(up))} | {len(up)} "
                f"| {_fmt(down_value)} | {_fmt(_stdev(down))} | {len(down)} | {_fmt(gap, 3)} |"
            )
    lines.extend(["", f"图表：`{plot_path.name}`", ""])
    return "\n".join(lines)


def _hysteresis_report(
    csv_path: Path,
    valid: list[ExperimentSeries],
    rejected_count: int,
    plot_path: Path,
) -> str:
    hysteresis: dict[tuple[int, int], list[float]] = {(motor, joint): [] for motor in MOTORS for joint in JOINTS}
    for series in valid:
        for joint in JOINTS:
            hysteresis[(series.motor, joint)].extend(_duplicate_hysteresis(series, joint))
    maxima = {key: (max(values) if values else None) for key, values in hysteresis.items()}
    if not any(value is not None for value in maxima.values()):
        raise LinearityAnalysisError("扰动序列中没有重复访问相同 delta 的位置，无法计算回差。")

    _write_heatmap_svg(
        plot_path,
        "Step 4 - Path hysteresis (deg)",
        [("Maximum repeated-position angle range", maxima, False, 3)],
    )
    lines = _report_preamble("步骤4：回差与重复性", csv_path, len(valid), rejected_count)
    lines.extend(
        [
            "对同一序列中重复出现的 `0、+50、-50 counts` 等位置，计算关节角度的极差。数值越大，路径相关回差越明显。",
            "",
            "| 电机 | 关节 | 平均回差(deg) | 最大回差(deg) | 比较次数 |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for motor in MOTORS:
        for joint in JOINTS:
            values = hysteresis[(motor, joint)]
            lines.append(
                f"| M{motor:02d} | J{joint:02d} | {_fmt(_mean(values), 3)} "
                f"| {_fmt(max(values) if values else None, 3)} | {len(values)} |"
            )
    lines.extend(["", f"图表：`{plot_path.name}`", ""])
    return "\n".join(lines)


def _linearity_report(
    csv_path: Path,
    valid: list[ExperimentSeries],
    rejected_count: int,
    plot_path: Path,
) -> str:
    metrics: dict[tuple[int, int], list[dict[str, Optional[float]]]] = {
        (motor, joint): [] for motor in MOTORS for joint in JOINTS
    }
    state_slopes: dict[tuple[int, int, str], list[float]] = {}
    for series in valid:
        for joint in JOINTS:
            y_values = series.joint_deg[joint]
            slope, _intercept, rmse, r_squared = _linear_fit(series.x_abs, y_values)
            y_span = max(y_values) - min(y_values)
            nrmse = rmse / y_span if y_span >= MIN_RESPONSE_SPAN_DEG else None
            quadratic_rmse = _quadratic_rmse(series.x_abs, y_values)
            quadratic_improvement = (
                max(0.0, (rmse - quadratic_rmse) / rmse)
                if quadratic_rmse is not None and rmse > 1e-12
                else 0.0
            )
            hysteresis_values = _duplicate_hysteresis(series, joint)
            hysteresis_ratio = (
                max(hysteresis_values) / y_span
                if hysteresis_values and y_span >= MIN_RESPONSE_SPAN_DEG
                else None
            )
            up, down = _segment_slopes(series, joint)
            up_mean, down_mean = _mean(up), _mean(down)
            direction_gap = None
            if up_mean is not None and down_mean is not None:
                direction_gap = abs(up_mean - down_mean) / max(abs(up_mean), abs(down_mean), 1e-9)
            metrics[(series.motor, joint)].append(
                {
                    "slope": slope,
                    "span": y_span,
                    "nrmse": nrmse,
                    "r2": r_squared,
                    "hysteresis": hysteresis_ratio,
                    "direction": direction_gap,
                    "quadratic": quadratic_improvement,
                }
            )
            state_slopes.setdefault((series.motor, joint, series.state), []).append(slope)

    nrmse_matrix: dict[tuple[int, int], Optional[float]] = {}
    direction_matrix: dict[tuple[int, int], Optional[float]] = {}
    conclusions: dict[tuple[int, int], str] = {}
    summary_rows: list[str] = []
    valid_cells = 0
    linear_cells = 0
    for motor in MOTORS:
        for joint in JOINTS:
            cell_metrics = metrics[(motor, joint)]
            spans = [value["span"] for value in cell_metrics if value["span"] is not None]
            nrmse_values = [value["nrmse"] for value in cell_metrics if value["nrmse"] is not None]
            r2_values = [value["r2"] for value in cell_metrics if value["r2"] is not None]
            hysteresis_values = [value["hysteresis"] for value in cell_metrics if value["hysteresis"] is not None]
            direction_values = [value["direction"] for value in cell_metrics if value["direction"] is not None]
            quadratic_values = [value["quadratic"] for value in cell_metrics if value["quadratic"] is not None]
            span = _median(spans)
            nrmse = _median(nrmse_values)
            r_squared = _median(r2_values)
            hysteresis_ratio = _median(hysteresis_values)
            direction_gap = _median(direction_values)
            quadratic_improvement = _median(quadratic_values)
            slopes_by_state = [
                statistics.fmean(values)
                for (candidate_motor, candidate_joint, _state), values in state_slopes.items()
                if candidate_motor == motor and candidate_joint == joint and values
            ]
            state_variation = None
            if len(slopes_by_state) >= 2:
                state_variation = (max(slopes_by_state) - min(slopes_by_state)) / max(
                    max(abs(value) for value in slopes_by_state),
                    1e-9,
                )
            nrmse_matrix[(motor, joint)] = nrmse
            direction_matrix[(motor, joint)] = direction_gap

            reasons: list[str] = []
            if span is None or span < MIN_RESPONSE_SPAN_DEG:
                conclusion = "响应不足/弱耦合"
            else:
                valid_cells += 1
                if nrmse is None or nrmse > MAX_LINEAR_NRMSE:
                    reasons.append("线性残差较大")
                if hysteresis_ratio is None or hysteresis_ratio > MAX_HYSTERESIS_RATIO:
                    reasons.append("回差较大")
                if direction_gap is None or direction_gap > MAX_DIRECTION_GAP:
                    reasons.append("正反向斜率不一致")
                if quadratic_improvement is not None and quadratic_improvement > MAX_QUADRATIC_IMPROVEMENT:
                    reasons.append("二次项改善明显")
                if state_variation is not None and state_variation > MAX_STATE_SLOPE_VARIATION:
                    reasons.append("不同姿态斜率变化明显")
                if reasons:
                    conclusion = "；".join(reasons)
                else:
                    conclusion = "局部近似线性"
                    linear_cells += 1
            conclusions[(motor, joint)] = conclusion
            summary_rows.append(
                f"| M{motor:02d} | J{joint:02d} | {_fmt(span, 3)} | {_fmt(nrmse, 3)} | {_fmt(r_squared, 3)} "
                f"| {_fmt(hysteresis_ratio, 3)} | {_fmt(direction_gap, 3)} | {_fmt(quadratic_improvement, 3)} "
                f"| {_fmt(state_variation, 3)} | {conclusion} |"
            )

    _write_heatmap_svg(
        plot_path,
        "Step 5 - Local linearity metrics",
        [
            ("Linear fit NRMSE", nrmse_matrix, False, 3),
            ("Direction slope gap", direction_matrix, False, 3),
        ],
    )
    lines = _report_preamble("步骤5：局部线性判断", csv_path, len(valid), rejected_count)
    lines.extend(
        [
            "判定范围仅限本次初始姿态、张力状态及扰动幅度。默认工程阈值如下：",
            "",
            f"- 线性拟合 NRMSE ≤ {MAX_LINEAR_NRMSE:.2f}",
            f"- 重复位置回差/输出跨度 ≤ {MAX_HYSTERESIS_RATIO:.2f}",
            f"- 正反向斜率相对差异 ≤ {MAX_DIRECTION_GAP:.2f}",
            f"- 二次模型相对改善 ≤ {MAX_QUADRATIC_IMPROVEMENT:.2f}",
            f"- 不同初始姿态的斜率相对变化 ≤ {MAX_STATE_SLOPE_VARIATION:.2f}",
            f"- 关节响应跨度 < {MIN_RESPONSE_SPAN_DEG:.2f} deg 时标记为响应不足，不参与线性结论",
            "",
            "| 电机 | 关节 | 响应跨度(deg) | NRMSE | R² | 回差比 | 方向差异 | 二次改善 | 姿态变化 | 结论 |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
            *summary_rows,
            "",
            f"在 {valid_cells} 个具有足够响应的电机-关节通道中，{linear_cells} 个满足上述局部近似线性阈值。",
            "",
            "> 本 CSV 为单电机扰动实验，尚未验证多输入叠加性，因此不能单独据此认定完整五输入四输出系统为全局线性系统。",
            "",
            f"图表：`{plot_path.name}`",
            "",
        ]
    )
    return "\n".join(lines)


def analyze_linearity_csv(csv_path: Path, kind: str, output_root: Path) -> LinearityAnalysisResult:
    kind = kind.strip().lower()
    if kind not in {"impact", "hysteresis", "linearity"}:
        raise LinearityAnalysisError(f"未知分析类型: {kind}")
    csv_path = Path(csv_path).resolve()
    series_list = load_linearity_series(csv_path)
    valid, rejected = _split_valid_series(series_list)
    output_dir = Path(output_root).resolve() / csv_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    names = {
        "impact": ("step3_impact_coefficients.md", "step3_impact_coefficients.svg"),
        "hysteresis": ("step4_hysteresis.md", "step4_hysteresis.svg"),
        "linearity": ("step5_local_linearity.md", "step5_local_linearity.svg"),
    }
    markdown_name, plot_name = names[kind]
    markdown_path = output_dir / markdown_name
    plot_path = output_dir / plot_name
    if kind == "impact":
        markdown = _impact_report(csv_path, valid, len(rejected), plot_path)
    elif kind == "hysteresis":
        markdown = _hysteresis_report(csv_path, valid, len(rejected), plot_path)
    else:
        markdown = _linearity_report(csv_path, valid, len(rejected), plot_path)
    markdown_path.write_text(markdown, encoding="utf-8")
    return LinearityAnalysisResult(
        kind=kind,
        markdown=markdown,
        markdown_path=markdown_path,
        plot_path=plot_path,
        output_dir=output_dir,
        valid_series=len(valid),
        rejected_series=len(rejected),
    )
