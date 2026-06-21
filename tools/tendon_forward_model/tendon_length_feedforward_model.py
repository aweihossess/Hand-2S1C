#!/usr/bin/env python3
"""Unified offline tendon-length feedforward model for the five-tendon finger.

This is the main feedforward entry point for the offline model folder. It keeps
all five tendon outputs in one place and leaves the older per-motor scripts as
debug/validation tools.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional

from m00_id1_right_mcp_tendon_model import (
    M00Id1RightMcpGeometry,
    compute_m00_id1_right_mcp_tendon_length,
    _frange,
)
from m01_id2_left_mcp_tendon_model import (
    M01Id2LeftMcpGeometry,
    compute_m01_id2_left_mcp_tendon_length,
)
from m02_id3_pip_flex_tendon_model import (
    M02Id3PipFlexGeometry,
    compute_m02_id3_pip_flex_tendon_delta,
)
from m03_id4_dip_flex_tendon_model import (
    M03Id4DipFlexGeometry,
    compute_m03_id4_dip_flex_tendon_delta,
)
from m04_id5_return_tendon_model import (
    M04Id5ReturnGeometry,
    compute_m04_id5_return_tendon_delta,
)


DEFAULT_MCP_WRAP_MODE = "direct-if-clear-ccw"


@dataclass(frozen=True)
class JointAnglesDeg:
    theta1: float = 0.0  # J00 / MCP-AA
    theta2: float = 0.0  # J01 / MCP-FE
    theta3: float = 0.0  # J02 / PIP-FE
    theta4: float = 0.0  # J03 / DIP/distal-FE


@dataclass(frozen=True)
class TendonFeedforwardConfig:
    # Shared obstacle padding, normally tendon radius / clearance margin.
    obstacle_padding_mm: float = 0.0

    # M00/M01 wrap side. Physical tests suggested direct-if-clear-ccw for the
    # MCP flexion tendons; keep it configurable for comparison.
    mcp_wrap_mode: str = DEFAULT_MCP_WRAP_MODE

    # Current hardware convention:
    #   motor_abs increases -> tendon shortens.
    # Therefore:
    #   motor_delta_counts = -length_delta_mm * counts_per_mm.
    counts_per_mm: float = 1.0


@dataclass(frozen=True)
class TendonFeedforwardRow:
    motor_name: str
    servo_id: int
    code_channel: str
    role: str
    length_delta_mm: float
    shortening_delta_mm: float
    motor_abs_delta_counts: float
    model_status: str
    component_mcp_mm: float = 0.0
    component_pip_mm: float = 0.0
    component_dip_mm: float = 0.0
    component_theta1_mm: float = 0.0
    raw_length_mm: float = 0.0
    raw_zero_length_mm: float = 0.0


@dataclass(frozen=True)
class TendonFeedforwardResult:
    theta1_deg: float
    theta2_deg: float
    theta3_deg: float
    theta4_deg: float
    obstacle_padding_mm: float
    mcp_wrap_mode: str
    counts_per_mm: float
    tendons: List[TendonFeedforwardRow]


@dataclass(frozen=True)
class TendonFeedforwardCsvRow:
    theta1_deg: float
    theta2_deg: float
    theta3_deg: float
    theta4_deg: float
    m00_id1_length_delta_mm: float
    m01_id2_length_delta_mm: float
    m02_id3_length_delta_mm: float
    m03_id4_length_delta_mm: float
    m04_id5_length_delta_mm: float
    m00_id1_motor_abs_delta_counts: float
    m01_id2_motor_abs_delta_counts: float
    m02_id3_motor_abs_delta_counts: float
    m03_id4_motor_abs_delta_counts: float
    m04_id5_motor_abs_delta_counts: float


def _make_m00_geom(config: TendonFeedforwardConfig) -> M00Id1RightMcpGeometry:
    base = M00Id1RightMcpGeometry()
    return M00Id1RightMcpGeometry(
        cylinder_radius=base.cylinder_radius + config.obstacle_padding_mm,
    )


def _make_m01_geom(config: TendonFeedforwardConfig) -> M01Id2LeftMcpGeometry:
    base = M01Id2LeftMcpGeometry()
    return M01Id2LeftMcpGeometry(
        cylinder_radius=base.cylinder_radius + config.obstacle_padding_mm,
    )


def _make_m02_geom(config: TendonFeedforwardConfig) -> M02Id3PipFlexGeometry:
    base = M02Id3PipFlexGeometry()
    pad = config.obstacle_padding_mm
    return M02Id3PipFlexGeometry(
        cylinder_radius=base.cylinder_radius + pad,
        keepout_x_min=base.keepout_x_min - pad,
        keepout_x_max=base.keepout_x_max + pad,
        keepout_y_min=base.keepout_y_min - pad,
        keepout_y_max=base.keepout_y_max + pad,
        keepout_left_corner_radius=base.keepout_left_corner_radius + pad,
    )


def _make_m03_geom(config: TendonFeedforwardConfig) -> M03Id4DipFlexGeometry:
    base = M03Id4DipFlexGeometry()
    pad = config.obstacle_padding_mm
    return M03Id4DipFlexGeometry(
        cylinder_radius=base.cylinder_radius + pad,
        keepout_x_min=base.keepout_x_min - pad,
        keepout_x_max=base.keepout_x_max + pad,
        keepout_y_min=base.keepout_y_min - pad,
        keepout_y_max=base.keepout_y_max + pad,
        keepout_left_corner_radius=base.keepout_left_corner_radius + pad,
    )


def _make_m04_geom(config: TendonFeedforwardConfig) -> M04Id5ReturnGeometry:
    base = M04Id5ReturnGeometry()
    pad = config.obstacle_padding_mm
    return M04Id5ReturnGeometry(
        keepout_y_min=base.keepout_y_min - pad,
        keepout_y_max=base.keepout_y_max + pad,
        keepout_x_right=base.keepout_x_right + pad,
        keepout_right_corner_radius=base.keepout_right_corner_radius + pad,
    )


def _motor_delta_counts(length_delta_mm: float, config: TendonFeedforwardConfig) -> float:
    return -length_delta_mm * config.counts_per_mm


def _row(
    motor_name: str,
    servo_id: int,
    code_channel: str,
    role: str,
    length_delta_mm: float,
    config: TendonFeedforwardConfig,
    model_status: str,
    component_mcp_mm: float = 0.0,
    component_pip_mm: float = 0.0,
    component_dip_mm: float = 0.0,
    component_theta1_mm: float = 0.0,
    raw_length_mm: float = 0.0,
    raw_zero_length_mm: float = 0.0,
) -> TendonFeedforwardRow:
    return TendonFeedforwardRow(
        motor_name=motor_name,
        servo_id=servo_id,
        code_channel=code_channel,
        role=role,
        length_delta_mm=length_delta_mm,
        shortening_delta_mm=-length_delta_mm,
        motor_abs_delta_counts=_motor_delta_counts(length_delta_mm, config),
        model_status=model_status,
        component_mcp_mm=component_mcp_mm,
        component_pip_mm=component_pip_mm,
        component_dip_mm=component_dip_mm,
        component_theta1_mm=component_theta1_mm,
        raw_length_mm=raw_length_mm,
        raw_zero_length_mm=raw_zero_length_mm,
    )


def compute_tendon_length_feedforward(
    angles: JointAnglesDeg,
    config: TendonFeedforwardConfig = TendonFeedforwardConfig(),
) -> TendonFeedforwardResult:
    """Compute all five tendon length deltas for one target pose."""

    m00_geom = _make_m00_geom(config)
    m01_geom = _make_m01_geom(config)
    m02_geom = _make_m02_geom(config)
    m03_geom = _make_m03_geom(config)
    m04_geom = _make_m04_geom(config)

    m00 = compute_m00_id1_right_mcp_tendon_length(
        angles.theta1,
        angles.theta2,
        m00_geom,
        config.mcp_wrap_mode,
    )
    m00_zero = compute_m00_id1_right_mcp_tendon_length(
        0.0,
        0.0,
        m00_geom,
        config.mcp_wrap_mode,
    )
    m00_delta = m00.length_mm - m00_zero.length_mm

    m01 = compute_m01_id2_left_mcp_tendon_length(
        angles.theta1,
        angles.theta2,
        m01_geom,
        config.mcp_wrap_mode,
    )
    m01_zero = compute_m01_id2_left_mcp_tendon_length(
        0.0,
        0.0,
        m01_geom,
        config.mcp_wrap_mode,
    )
    m01_delta = m01.length_mm - m01_zero.length_mm

    m02 = compute_m02_id3_pip_flex_tendon_delta(
        angles.theta1,
        angles.theta2,
        angles.theta3,
        m02_geom,
    )

    m03 = compute_m03_id4_dip_flex_tendon_delta(
        angles.theta1,
        angles.theta2,
        angles.theta3,
        angles.theta4,
        m03_geom,
    )

    m04 = compute_m04_id5_return_tendon_delta(
        angles.theta1,
        angles.theta2,
        angles.theta3,
        angles.theta4,
        m04_geom,
    )

    tendons = [
        _row(
            "M00",
            1,
            "M00",
            "MCP-AA right swing + MCP-FE flexion",
            m00_delta,
            config,
            m00.path_type,
            component_mcp_mm=m00_delta,
            raw_length_mm=m00.length_mm,
            raw_zero_length_mm=m00_zero.length_mm,
        ),
        _row(
            "M01",
            2,
            "M01",
            "MCP-AA left swing + MCP-FE flexion",
            m01_delta,
            config,
            m01.path_type,
            component_mcp_mm=m01_delta,
            raw_length_mm=m01.length_mm,
            raw_zero_length_mm=m01_zero.length_mm,
        ),
        _row(
            "M02",
            3,
            "M02",
            "PIP-FE flexion",
            m02.length_delta_mm,
            config,
            m02.model_status,
            component_mcp_mm=m02.mcp_routing_delta_mm,
            component_pip_mm=m02.pip_flexion_delta_mm,
            raw_length_mm=m02.mcp_routing_length_mm,
            raw_zero_length_mm=m02.mcp_routing_zero_length_mm,
        ),
        _row(
            "M03",
            4,
            "M03",
            "DIP/distal-FE flexion",
            m03.length_delta_mm,
            config,
            m03.model_status,
            component_mcp_mm=m03.mcp_routing_delta_mm,
            component_pip_mm=m03.pip_flexion_delta_mm,
            component_dip_mm=m03.dip_flexion_delta_mm,
            raw_length_mm=m03.mcp_routing_length_mm,
            raw_zero_length_mm=m03.mcp_routing_zero_length_mm,
        ),
        _row(
            "M04",
            5,
            "M04",
            "MCP/PIP/DIP common return",
            m04.length_delta_mm,
            config,
            m04.model_status,
            component_mcp_mm=m04.mcp_fe_routing_delta_mm,
            component_pip_mm=m04.pip_return_delta_mm,
            component_dip_mm=m04.dip_return_delta_mm,
            component_theta1_mm=m04.theta1_return_delta_mm,
            raw_length_mm=m04.mcp_fe_routing_length_mm,
            raw_zero_length_mm=m04.mcp_fe_routing_zero_length_mm,
        ),
    ]

    return TendonFeedforwardResult(
        theta1_deg=angles.theta1,
        theta2_deg=angles.theta2,
        theta3_deg=angles.theta3,
        theta4_deg=angles.theta4,
        obstacle_padding_mm=config.obstacle_padding_mm,
        mcp_wrap_mode=config.mcp_wrap_mode,
        counts_per_mm=config.counts_per_mm,
        tendons=tendons,
    )


def _to_csv_row(result: TendonFeedforwardResult) -> TendonFeedforwardCsvRow:
    by_name = {row.motor_name: row for row in result.tendons}
    return TendonFeedforwardCsvRow(
        theta1_deg=result.theta1_deg,
        theta2_deg=result.theta2_deg,
        theta3_deg=result.theta3_deg,
        theta4_deg=result.theta4_deg,
        m00_id1_length_delta_mm=by_name["M00"].length_delta_mm,
        m01_id2_length_delta_mm=by_name["M01"].length_delta_mm,
        m02_id3_length_delta_mm=by_name["M02"].length_delta_mm,
        m03_id4_length_delta_mm=by_name["M03"].length_delta_mm,
        m04_id5_length_delta_mm=by_name["M04"].length_delta_mm,
        m00_id1_motor_abs_delta_counts=by_name["M00"].motor_abs_delta_counts,
        m01_id2_motor_abs_delta_counts=by_name["M01"].motor_abs_delta_counts,
        m02_id3_motor_abs_delta_counts=by_name["M02"].motor_abs_delta_counts,
        m03_id4_motor_abs_delta_counts=by_name["M03"].motor_abs_delta_counts,
        m04_id5_motor_abs_delta_counts=by_name["M04"].motor_abs_delta_counts,
    )


def _write_grid_csv(path: Path, rows: Iterable[TendonFeedforwardCsvRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def _grid_values(single_value: float, min_value: Optional[float], max_value: Optional[float], step: float) -> List[float]:
    if min_value is None and max_value is None:
        return [single_value]
    if min_value is None or max_value is None:
        raise ValueError("grid min and max must be provided together")
    return list(_frange(min_value, max_value, step))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified five-tendon length feedforward model."
    )
    parser.add_argument("--theta1", type=float, default=0.0, help="J00/MCP-AA angle in degrees.")
    parser.add_argument("--theta2", type=float, default=0.0, help="J01/MCP-FE angle in degrees.")
    parser.add_argument("--theta3", type=float, default=0.0, help="J02/PIP-FE angle in degrees.")
    parser.add_argument("--theta4", type=float, default=0.0, help="J03/DIP/distal-FE angle in degrees.")
    parser.add_argument("--obstacle-padding", type=float, default=0.0, help="Shared obstacle clearance in mm.")
    parser.add_argument(
        "--mcp-wrap-mode",
        choices=("shortest", "cw", "ccw", "direct-if-clear-cw", "direct-if-clear-ccw"),
        default=DEFAULT_MCP_WRAP_MODE,
        help="M00/M01 cylinder avoidance mode.",
    )
    parser.add_argument("--counts-per-mm", type=float, default=1.0, help="Counts per mm for motor feedforward preview.")
    parser.add_argument("--json", action="store_true", help="Print JSON instead of a readable summary.")

    parser.add_argument("--grid", action="store_true", help="Export a theta grid to CSV.")
    parser.add_argument("--theta1-min", type=float, default=None)
    parser.add_argument("--theta1-max", type=float, default=None)
    parser.add_argument("--theta1-step", type=float, default=1.0)
    parser.add_argument("--theta2-min", type=float, default=None)
    parser.add_argument("--theta2-max", type=float, default=None)
    parser.add_argument("--theta2-step", type=float, default=1.0)
    parser.add_argument("--theta3-min", type=float, default=None)
    parser.add_argument("--theta3-max", type=float, default=None)
    parser.add_argument("--theta3-step", type=float, default=1.0)
    parser.add_argument("--theta4-min", type=float, default=None)
    parser.add_argument("--theta4-max", type=float, default=None)
    parser.add_argument("--theta4-step", type=float, default=1.0)
    parser.add_argument("--out", type=Path, default=Path("run_data/tendon_length_feedforward_grid.csv"))
    return parser


def _print_readable(result: TendonFeedforwardResult) -> None:
    print(
        "theta="
        f"({result.theta1_deg:.3f}, {result.theta2_deg:.3f}, "
        f"{result.theta3_deg:.3f}, {result.theta4_deg:.3f}) deg"
    )
    print(f"obstacle_padding_mm={result.obstacle_padding_mm:.3f}")
    print(f"mcp_wrap_mode={result.mcp_wrap_mode}")
    print(f"counts_per_mm={result.counts_per_mm:.6f}")
    print("note=length_delta_mm > 0 means tendon lengthens; motor_abs_delta_counts uses motor_abs increases -> tendon shortens")
    for row in result.tendons:
        print(
            f"{row.motor_name}/ID{row.servo_id} "
            f"length_delta_mm={row.length_delta_mm:.6f} "
            f"shortening_delta_mm={row.shortening_delta_mm:.6f} "
            f"motor_abs_delta_counts={row.motor_abs_delta_counts:.3f} "
            f"components(theta1/mcp/pip/dip)="
            f"{row.component_theta1_mm:.6f}/"
            f"{row.component_mcp_mm:.6f}/"
            f"{row.component_pip_mm:.6f}/"
            f"{row.component_dip_mm:.6f} "
            f"status={row.model_status}"
        )


def main() -> int:
    args = build_arg_parser().parse_args()
    config = TendonFeedforwardConfig(
        obstacle_padding_mm=args.obstacle_padding,
        mcp_wrap_mode=args.mcp_wrap_mode,
        counts_per_mm=args.counts_per_mm,
    )

    if args.grid:
        theta1_values = _grid_values(args.theta1, args.theta1_min, args.theta1_max, args.theta1_step)
        theta2_values = _grid_values(args.theta2, args.theta2_min, args.theta2_max, args.theta2_step)
        theta3_values = _grid_values(args.theta3, args.theta3_min, args.theta3_max, args.theta3_step)
        theta4_values = _grid_values(args.theta4, args.theta4_min, args.theta4_max, args.theta4_step)
        rows = [
            _to_csv_row(compute_tendon_length_feedforward(
                JointAnglesDeg(theta1, theta2, theta3, theta4),
                config,
            ))
            for theta1 in theta1_values
            for theta2 in theta2_values
            for theta3 in theta3_values
            for theta4 in theta4_values
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_tendon_length_feedforward(
        JointAnglesDeg(args.theta1, args.theta2, args.theta3, args.theta4),
        config,
    )
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    _print_readable(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
