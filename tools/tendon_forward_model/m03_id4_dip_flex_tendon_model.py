#!/usr/bin/env python3
"""Offline M03/ID4 distal-flexion tendon-length feedforward estimator.

The model is intentionally kept standalone and is not connected to the firmware
control path.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from m00_id1_right_mcp_tendon_model import _dist3, _frange, _to_mcp_obstacle_frame
from m02_id3_pip_flex_tendon_model import _compute_shortest_xy_path, _path_points_to_text


Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


DEFAULT_M03_L2_MM = math.hypot(8.10, 0.18)
DEFAULT_M03_THETA_O_DEG = -math.degrees(math.atan2(0.18, 8.10))


@dataclass(frozen=True)
class M03Id4DipFlexGeometry:
    # Fixed anchor for the distal flexion tendon MCP-routing component, mm.
    anchor_x: float = -6.93
    anchor_y: float = 2.75
    anchor_z: float = -4.00

    # MCP routing geometry, mm and degrees.
    l1: float = 13.00
    l2: float = DEFAULT_M03_L2_MM
    l3: float = 1.50
    theta_o_deg: float = DEFAULT_M03_THETA_O_DEG

    # Same infinite-Z cylinder keepout used by the M02 routing estimate.
    cylinder_x: float = 13.00
    cylinder_y: float = 0.00
    cylinder_radius: float = 3.50

    # Same origin keepout approximation used by M02.
    keepout_x_min: float = -5.00
    keepout_x_max: float = 5.00
    keepout_y_min: float = -5.00
    keepout_y_max: float = 3.70
    keepout_left_corner_radius: float = 2.00
    obstacles_follow_theta1: bool = True

    # Visibility graph resolution for obstacle-boundary samples.
    circle_samples: int = 96
    keepout_arc_samples: int = 16
    keepout_edge_samples: int = 20

    # Physical meaning:
    #   theta3 increases by 1 rad -> tendon length changes by -3.50 mm.
    #   theta4 increases by 1 rad -> tendon length changes by -4.80 mm.
    pip_flex_length_slope_mm_per_rad: float = -3.50
    dip_flex_length_slope_mm_per_rad: float = -4.80


@dataclass(frozen=True)
class M03TendonLengthResult:
    theta1_deg: float
    theta2_deg: float
    theta3_deg: float
    theta4_deg: float
    length_delta_mm: float
    mcp_routing_delta_mm: float
    mcp_routing_length_mm: float
    mcp_routing_zero_length_mm: float
    mcp_routing_straight_length_mm: float
    mcp_routing_extra_length_mm: float
    mcp_routing_xy_path_length_mm: float
    mcp_routing_z_delta_mm: float
    pip_flexion_delta_mm: float
    dip_flexion_delta_mm: float
    d_x: float
    d_y: float
    d_z: float
    xy_path_points: str
    model_status: str


def compute_m03_mcp_routing_d(theta1_deg: float, theta2_deg: float, geom: M03Id4DipFlexGeometry) -> Vec3:
    """Compute the MCP routing moving point.

    M03 runs over the upper side of the MCP-FE axis, so positive MCP-FE uses
    the opposite theta2 sign from the lower-side M01/M02 routing models.
    """

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(-theta2_deg + geom.theta_o_deg)
    projected = geom.l1 + geom.l2 * math.cos(theta2)

    x = math.cos(theta1) * projected - geom.l3 * math.sin(theta1)
    y = -geom.l2 * math.sin(theta2)
    z_m02 = math.sin(theta1) * projected + geom.l3 * math.cos(theta1)
    z = -z_m02
    return (x, y, z)


def compute_m03_mcp_routing_length_mm(
    theta1_deg: float,
    theta2_deg: float,
    geom: M03Id4DipFlexGeometry,
) -> Tuple[float, float, float, float, Vec3, List[Vec2]]:
    """Return physical MCP routing length and diagnostics.

    Returns:
        length_mm, straight_length_mm, xy_path_length_mm, z_delta_mm, d, xy_path
    """

    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    d = compute_m03_mcp_routing_d(theta1_deg, theta2_deg, geom)
    anchor_path = _to_mcp_obstacle_frame(anchor, theta1_deg, geom.obstacles_follow_theta1)
    d_path = _to_mcp_obstacle_frame(d, theta1_deg, geom.obstacles_follow_theta1)
    start_xy = (anchor_path[0], anchor_path[1])
    goal_xy = (d_path[0], d_path[1])
    xy_length, xy_path = _compute_shortest_xy_path(start_xy, goal_xy, geom)
    z_delta = d_path[2] - anchor_path[2]
    length = math.sqrt(xy_length * xy_length + z_delta * z_delta)
    straight = _dist3(anchor, d)
    return length, straight, xy_length, z_delta, d, xy_path


def compute_mcp_routing_delta_mm(
    theta1_deg: float,
    theta2_deg: float,
    geom: M03Id4DipFlexGeometry,
) -> Tuple[float, float, float, float, float, float, Vec3, List[Vec2]]:
    length, straight, xy_len, z_delta, d, path = compute_m03_mcp_routing_length_mm(theta1_deg, theta2_deg, geom)
    zero_length, _, _, _, _, _ = compute_m03_mcp_routing_length_mm(0.0, 0.0, geom)
    theta1_zero_length, _, _, _, _, _ = compute_m03_mcp_routing_length_mm(theta1_deg, 0.0, geom)
    theta1_component = theta1_zero_length - zero_length
    theta2_component = length - theta1_zero_length
    # M03 crosses the upper side of the MCP-FE axis, so its MCP-FE contribution
    # has the opposite sign from lower-side flexion tendons.
    signed_delta = theta1_component - theta2_component
    return signed_delta, length, zero_length, straight, length - straight, xy_len, z_delta, d, path


def compute_pip_flexion_delta_mm(theta3_deg: float, geom: M03Id4DipFlexGeometry) -> float:
    """Return the PIP-FE contribution to M03 tendon length."""

    return geom.pip_flex_length_slope_mm_per_rad * math.radians(theta3_deg)


def compute_dip_flexion_delta_mm(theta4_deg: float, geom: M03Id4DipFlexGeometry) -> float:
    """Return the distal/DIP-FE contribution to M03 tendon length."""

    return geom.dip_flex_length_slope_mm_per_rad * math.radians(theta4_deg)


def compute_m03_id4_dip_flex_tendon_delta(
    theta1_deg: float,
    theta2_deg: float,
    theta3_deg: float,
    theta4_deg: float,
    geom: M03Id4DipFlexGeometry = M03Id4DipFlexGeometry(),
) -> M03TendonLengthResult:
    """Compute M03/ID4 tendon length delta in mm.

    Current model:

        delta_L =
            mcp_routing_delta(theta1, theta2)
          + pip_flexion_delta(theta3)
          + dip_flexion_delta(theta4)

    Negative delta_L means the physical tendon gets shorter, so with the current
    hardware convention it should correspond to increasing motor_abs after
    conversion to motor counts.
    """

    (
        mcp_delta,
        mcp_length,
        mcp_zero,
        mcp_straight,
        mcp_extra,
        mcp_xy,
        mcp_z_delta,
        d,
        path,
    ) = compute_mcp_routing_delta_mm(theta1_deg, theta2_deg, geom)
    pip_delta = compute_pip_flexion_delta_mm(theta3_deg, geom)
    dip_delta = compute_dip_flexion_delta_mm(theta4_deg, geom)
    return M03TendonLengthResult(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        theta3_deg=theta3_deg,
        theta4_deg=theta4_deg,
        length_delta_mm=mcp_delta + pip_delta + dip_delta,
        mcp_routing_delta_mm=mcp_delta,
        mcp_routing_length_mm=mcp_length,
        mcp_routing_zero_length_mm=mcp_zero,
        mcp_routing_straight_length_mm=mcp_straight,
        mcp_routing_extra_length_mm=mcp_extra,
        mcp_routing_xy_path_length_mm=mcp_xy,
        mcp_routing_z_delta_mm=mcp_z_delta,
        pip_flexion_delta_mm=pip_delta,
        dip_flexion_delta_mm=dip_delta,
        d_x=d[0],
        d_y=d[1],
        d_z=d[2],
        xy_path_points=_path_points_to_text(path),
        model_status="mirrored_mcp_routing_plus_pip_and_dip_flexion",
    )


def _write_grid_csv(path: Path, rows: Iterable[M03TendonLengthResult]) -> None:
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
        description="Offline M03/ID4 distal flexion tendon length model."
    )
    parser.add_argument("--theta1", type=float, default=0.0, help="J00/MCP-AA angle in degrees.")
    parser.add_argument("--theta2", type=float, default=0.0, help="J01/MCP-FE angle in degrees.")
    parser.add_argument("--theta3", type=float, default=0.0, help="J02/PIP-FE angle in degrees.")
    parser.add_argument("--theta4", type=float, default=0.0, help="J03/DIP/distal-FE angle in degrees.")
    parser.add_argument(
        "--obstacle-padding",
        type=float,
        default=0.0,
        help="Extra clearance in mm added to the cylinder and origin keepout, e.g. tendon radius.",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON instead of a readable summary.")

    parser.add_argument("--grid", action="store_true", help="Export a theta1/theta2/theta3/theta4 grid to CSV.")
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
    parser.add_argument("--out", type=Path, default=Path("run_data/m03_id4_dip_flex_tendon_delta_grid.csv"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    base_geom = M03Id4DipFlexGeometry()
    padding = args.obstacle_padding
    geom = M03Id4DipFlexGeometry(
        cylinder_radius=base_geom.cylinder_radius + padding,
        keepout_x_min=base_geom.keepout_x_min - padding,
        keepout_x_max=base_geom.keepout_x_max + padding,
        keepout_y_min=base_geom.keepout_y_min - padding,
        keepout_y_max=base_geom.keepout_y_max + padding,
        keepout_left_corner_radius=base_geom.keepout_left_corner_radius + padding,
    )

    if args.grid:
        theta1_values = _grid_values(args.theta1, args.theta1_min, args.theta1_max, args.theta1_step)
        theta2_values = _grid_values(args.theta2, args.theta2_min, args.theta2_max, args.theta2_step)
        theta3_values = _grid_values(args.theta3, args.theta3_min, args.theta3_max, args.theta3_step)
        theta4_values = _grid_values(args.theta4, args.theta4_min, args.theta4_max, args.theta4_step)
        rows = [
            compute_m03_id4_dip_flex_tendon_delta(theta1, theta2, theta3, theta4, geom)
            for theta1 in theta1_values
            for theta2 in theta2_values
            for theta3 in theta3_values
            for theta4 in theta4_values
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_m03_id4_dip_flex_tendon_delta(args.theta1, args.theta2, args.theta3, args.theta4, geom)
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    print(f"theta1_deg={result.theta1_deg:.3f}")
    print(f"theta2_deg={result.theta2_deg:.3f}")
    print(f"theta3_deg={result.theta3_deg:.3f}")
    print(f"theta4_deg={result.theta4_deg:.3f}")
    print(f"model_status={result.model_status}")
    print(f"D=({result.d_x:.3f}, {result.d_y:.3f}, {result.d_z:.3f}) mm")
    print(f"mcp_routing_length_mm={result.mcp_routing_length_mm:.6f}")
    print(f"mcp_routing_zero_length_mm={result.mcp_routing_zero_length_mm:.6f}")
    print(f"mcp_routing_delta_mm={result.mcp_routing_delta_mm:.6f}")
    print(f"mcp_routing_straight_length_mm={result.mcp_routing_straight_length_mm:.6f}")
    print(f"mcp_routing_extra_length_mm={result.mcp_routing_extra_length_mm:.6f}")
    print(f"mcp_routing_xy_path_length_mm={result.mcp_routing_xy_path_length_mm:.6f}")
    print(f"mcp_routing_z_delta_mm={result.mcp_routing_z_delta_mm:.6f}")
    print(f"pip_flexion_delta_mm={result.pip_flexion_delta_mm:.6f}")
    print(f"dip_flexion_delta_mm={result.dip_flexion_delta_mm:.6f}")
    print(f"length_delta_mm={result.length_delta_mm:.6f}")
    print("note=negative length_delta means tendon shortens; motor_abs should increase after counts conversion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
