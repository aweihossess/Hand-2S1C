#!/usr/bin/env python3
"""Offline M01/ID2 left-MCP tendon-length feedforward estimator.

This tool is intentionally standalone. It does not modify or call the firmware
control code. The model computes the tendon length from A2 to D2 while avoiding
an infinite cylinder whose axis is parallel to Z.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

from m00_id1_right_mcp_tendon_model import (
    DEFAULT_L2_MM,
    DEFAULT_THETA_O_DEG,
    _best_xy_path_around_cylinder,
    _dist3,
    _frange,
    _to_mcp_obstacle_frame,
)


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class M01Id2LeftMcpGeometry:
    # A2 fixed anchor, mm. The sign follows the supplied A2D2 expression:
    # z component contains "+ z1", so A2 is interpreted as (x1, y1, -z1).
    a2_x: float = -3.66
    a2_y: float = -2.25
    a2_z: float = -8.58

    # Same link/tendon geometry as ID1.
    l1: float = 13.00
    l2: float = DEFAULT_L2_MM
    l3: float = 4.50
    theta_o_deg: float = DEFAULT_THETA_O_DEG

    # Same infinite cylinder obstacle as ID1. At zero pose it uses the listed
    # coordinates; by default it follows theta1 with the MCP-AA link frame.
    cylinder_x: float = 13.00
    cylinder_y: float = 0.00
    cylinder_z: float = 5.00
    cylinder_radius: float = 5.90
    cylinder_follows_theta1: bool = True


@dataclass(frozen=True)
class LeftTendonLengthResult:
    theta1_deg: float
    theta2_deg: float
    length_mm: float
    straight_length_mm: float
    extra_length_mm: float
    xy_path_length_mm: float
    z_delta_mm: float
    path_type: str
    blocked_by_cylinder: bool
    d2_x: float
    d2_y: float
    d2_z: float
    tangent_start_x: Optional[float] = None
    tangent_start_y: Optional[float] = None
    tangent_end_x: Optional[float] = None
    tangent_end_y: Optional[float] = None
    arc_angle_deg: Optional[float] = None


def compute_d2(theta1_deg: float, theta2_deg: float, geom: M01Id2LeftMcpGeometry) -> Vec3:
    """Compute the moving D2 point in mm.

    The returned point is absolute. It matches the supplied A2D2 vector if
    A2 = (x1, y1, -z1).
    """

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg + geom.theta_o_deg)
    projected = geom.l1 + geom.l2 * math.cos(theta2)

    x = math.cos(theta1) * projected + geom.l3 * math.sin(theta1)
    y = -geom.l2 * math.sin(theta2)
    z = math.sin(theta1) * projected - geom.l3 * math.cos(theta1)
    return (x, y, z)


def compute_m01_id2_left_mcp_tendon_length(
    theta1_deg: float,
    theta2_deg: float,
    geom: M01Id2LeftMcpGeometry = M01Id2LeftMcpGeometry(),
    wrap_mode: str = "shortest",
) -> LeftTendonLengthResult:
    """Compute M01/ID2 left-MCP tendon length in mm for J00/J01 angles."""

    a2 = (geom.a2_x, geom.a2_y, geom.a2_z)
    d2 = compute_d2(theta1_deg, theta2_deg, geom)
    a2_path = _to_mcp_obstacle_frame(a2, theta1_deg, geom.cylinder_follows_theta1)
    d2_path = _to_mcp_obstacle_frame(d2, theta1_deg, geom.cylinder_follows_theta1)
    p_xy = (a2_path[0], a2_path[1])
    q_xy = (d2_path[0], d2_path[1])
    cylinder_xy = (geom.cylinder_x, geom.cylinder_y)

    straight_length = _dist3(a2, d2)
    path_type, xy_length, wrap = _best_xy_path_around_cylinder(
        p_xy,
        q_xy,
        cylinder_xy,
        geom.cylinder_radius,
        wrap_mode,
    )

    z_delta = d2_path[2] - a2_path[2]
    length = math.sqrt(xy_length * xy_length + z_delta * z_delta)
    blocked = path_type != "direct"

    if wrap is None:
        return LeftTendonLengthResult(
            theta1_deg=theta1_deg,
            theta2_deg=theta2_deg,
            length_mm=length,
            straight_length_mm=straight_length,
            extra_length_mm=length - straight_length,
            xy_path_length_mm=xy_length,
            z_delta_mm=z_delta,
            path_type=path_type,
            blocked_by_cylinder=blocked,
            d2_x=d2[0],
            d2_y=d2[1],
            d2_z=d2[2],
        )

    return LeftTendonLengthResult(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        length_mm=length,
        straight_length_mm=straight_length,
        extra_length_mm=length - straight_length,
        xy_path_length_mm=xy_length,
        z_delta_mm=z_delta,
        path_type=path_type,
        blocked_by_cylinder=blocked,
        d2_x=d2[0],
        d2_y=d2[1],
        d2_z=d2[2],
        tangent_start_x=wrap.tangent_start[0],
        tangent_start_y=wrap.tangent_start[1],
        tangent_end_x=wrap.tangent_end[0],
        tangent_end_y=wrap.tangent_end[1],
        arc_angle_deg=math.degrees(wrap.arc_angle_rad),
    )


def _write_grid_csv(path: Path, rows: Iterable[LeftTendonLengthResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline M01/ID2 left-MCP tendon length model with infinite-cylinder avoidance."
    )
    parser.add_argument("--theta1", type=float, default=0.0, help="J00/MCP-AA angle in degrees.")
    parser.add_argument("--theta2", type=float, default=0.0, help="J01/MCP-FE angle in degrees.")
    parser.add_argument("--radius-padding", type=float, default=0.0, help="Extra obstacle radius in mm, e.g. tendon radius.")
    parser.add_argument(
        "--wrap-mode",
        choices=("shortest", "cw", "ccw", "direct-if-clear-cw", "direct-if-clear-ccw"),
        default="shortest",
        help="Cylinder avoidance mode. Use cw/ccw to force a fixed side.",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON instead of a readable summary.")

    parser.add_argument("--grid", action="store_true", help="Export a theta1/theta2 grid to CSV.")
    parser.add_argument("--theta1-min", type=float, default=-20.0)
    parser.add_argument("--theta1-max", type=float, default=30.0)
    parser.add_argument("--theta1-step", type=float, default=1.0)
    parser.add_argument("--theta2-min", type=float, default=-20.0)
    parser.add_argument("--theta2-max", type=float, default=90.0)
    parser.add_argument("--theta2-step", type=float, default=1.0)
    parser.add_argument("--out", type=Path, default=Path("run_data/m01_id2_left_mcp_tendon_length_grid.csv"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    base_geom = M01Id2LeftMcpGeometry()
    geom = M01Id2LeftMcpGeometry(
        cylinder_radius=base_geom.cylinder_radius + args.radius_padding
    )

    if args.grid:
        rows = [
            compute_m01_id2_left_mcp_tendon_length(theta1, theta2, geom, args.wrap_mode)
            for theta1 in _frange(args.theta1_min, args.theta1_max, args.theta1_step)
            for theta2 in _frange(args.theta2_min, args.theta2_max, args.theta2_step)
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_m01_id2_left_mcp_tendon_length(args.theta1, args.theta2, geom, args.wrap_mode)
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    print(f"theta1_deg={result.theta1_deg:.3f}")
    print(f"theta2_deg={result.theta2_deg:.3f}")
    print(f"path_type={result.path_type}")
    print(f"blocked_by_cylinder={int(result.blocked_by_cylinder)}")
    print(f"D2=({result.d2_x:.3f}, {result.d2_y:.3f}, {result.d2_z:.3f}) mm")
    print(f"straight_length_mm={result.straight_length_mm:.6f}")
    print(f"curve_length_mm={result.length_mm:.6f}")
    print(f"extra_length_mm={result.extra_length_mm:.6f}")
    print(f"xy_path_length_mm={result.xy_path_length_mm:.6f}")
    print(f"z_delta_mm={result.z_delta_mm:.6f}")
    if result.arc_angle_deg is not None:
        print(f"arc_angle_deg={result.arc_angle_deg:.6f}")
        print(f"tangent_start=({result.tangent_start_x:.3f}, {result.tangent_start_y:.3f}) mm")
        print(f"tangent_end=({result.tangent_end_x:.3f}, {result.tangent_end_y:.3f}) mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
