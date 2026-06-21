#!/usr/bin/env python3
"""Offline M00/ID1 right-MCP tendon-length feedforward estimator.

This tool is intentionally standalone. It does not modify or call the firmware
control code. The model computes the tendon length from A1 to D1 while avoiding
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


Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]

DEFAULT_L2_MM = math.hypot(8.10, 2.00)
DEFAULT_THETA_O_DEG = -math.degrees(math.atan2(2.00, 8.10))


@dataclass(frozen=True)
class M00Id1RightMcpGeometry:
    # A1 fixed anchor, mm.
    a1_x: float = -3.66
    a1_y: float = -2.25
    a1_z: float = 8.58

    # Link/tendon geometry from the previous MCP model, mm and degrees.
    l1: float = 13.00
    l2: float = DEFAULT_L2_MM
    l3: float = 4.50
    theta_o_deg: float = DEFAULT_THETA_O_DEG

    # Infinite cylinder obstacle at the zero MCP-AA pose. When
    # cylinder_follows_theta1 is true, the cylinder is interpreted in the local
    # MCP-AA frame and moves with theta1.
    cylinder_x: float = 13.00
    cylinder_y: float = 0.00
    cylinder_z: float = 5.00
    cylinder_radius: float = 5.90
    cylinder_follows_theta1: bool = True


@dataclass(frozen=True)
class TendonLengthResult:
    theta1_deg: float
    theta2_deg: float
    length_mm: float
    straight_length_mm: float
    extra_length_mm: float
    xy_path_length_mm: float
    z_delta_mm: float
    path_type: str
    blocked_by_cylinder: bool
    d1_x: float
    d1_y: float
    d1_z: float
    tangent_start_x: Optional[float] = None
    tangent_start_y: Optional[float] = None
    tangent_end_x: Optional[float] = None
    tangent_end_y: Optional[float] = None
    arc_angle_deg: Optional[float] = None


def _sub2(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def _dot2(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _norm2(a: Vec2) -> float:
    return math.hypot(a[0], a[1])


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _rotate_mcp_y(point: Vec3, angle_deg: float) -> Vec3:
    """Rotate with the same Y-axis convention used by the MCP-AA model."""

    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    x, y, z = point
    return (
        c * x - s * z,
        y,
        s * x + c * z,
    )


def _to_mcp_obstacle_frame(point: Vec3, theta1_deg: float, follows_theta1: bool) -> Vec3:
    if not follows_theta1:
        return point
    return _rotate_mcp_y(point, -theta1_deg)


def _positive_angle(angle_rad: float) -> float:
    tau = 2.0 * math.pi
    value = angle_rad % tau
    if value < 0.0:
        value += tau
    return value


def _ccw_delta(start_rad: float, end_rad: float) -> float:
    return _positive_angle(end_rad - start_rad)


def _point_on_circle(center: Vec2, radius: float, angle_rad: float) -> Vec2:
    return (
        center[0] + radius * math.cos(angle_rad),
        center[1] + radius * math.sin(angle_rad),
    )


def compute_d1(theta1_deg: float, theta2_deg: float, geom: M00Id1RightMcpGeometry) -> Vec3:
    """Compute the moving D1 point in mm.

    The returned point is absolute. The A1D1 vector is D1 - A1 and matches the
    formula supplied by the user.
    """

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg + geom.theta_o_deg)
    projected = geom.l1 + geom.l2 * math.cos(theta2)

    x = math.cos(theta1) * projected - geom.l3 * math.sin(theta1)
    y = -geom.l2 * math.sin(theta2)
    z = math.sin(theta1) * projected + geom.l3 * math.cos(theta1)
    return (x, y, z)


def _segment_min_distance_to_point(p: Vec2, q: Vec2, c: Vec2) -> float:
    vx = q[0] - p[0]
    vy = q[1] - p[1]
    denom = vx * vx + vy * vy
    if denom <= 1.0e-12:
        return _norm2(_sub2(p, c))
    t = ((c[0] - p[0]) * vx + (c[1] - p[1]) * vy) / denom
    t = max(0.0, min(1.0, t))
    closest = (p[0] + t * vx, p[1] + t * vy)
    return _norm2(_sub2(closest, c))


def _straight_path_clear(p: Vec2, q: Vec2, c: Vec2, radius: float) -> bool:
    if _norm2(_sub2(p, c)) < radius or _norm2(_sub2(q, c)) < radius:
        return False
    return _segment_min_distance_to_point(p, q, c) >= radius


@dataclass(frozen=True)
class _WrapCandidate:
    path_type: str
    xy_length: float
    tangent_start: Vec2
    tangent_end: Vec2
    arc_angle_rad: float


def _wrap_candidate(p: Vec2, q: Vec2, c: Vec2, radius: float, ccw: bool) -> _WrapCandidate:
    p_rel = _sub2(p, c)
    q_rel = _sub2(q, c)
    dp = _norm2(p_rel)
    dq = _norm2(q_rel)
    if dp <= radius or dq <= radius:
        raise ValueError("A1 or D1 projects inside/on the cylinder obstacle")

    p_angle = math.atan2(p_rel[1], p_rel[0])
    q_angle = math.atan2(q_rel[1], q_rel[0])
    p_offset = math.acos(radius / dp)
    q_offset = math.acos(radius / dq)

    if ccw:
        start_angle = p_angle + p_offset
        end_angle = q_angle - q_offset
        arc_angle = _ccw_delta(start_angle, end_angle)
        path_type = "wrap_ccw"
    else:
        start_angle = p_angle - p_offset
        end_angle = q_angle + q_offset
        arc_angle = _ccw_delta(end_angle, start_angle)
        path_type = "wrap_cw"

    tangent_start = _point_on_circle(c, radius, start_angle)
    tangent_end = _point_on_circle(c, radius, end_angle)
    tangent_len_p = math.sqrt(max(0.0, dp * dp - radius * radius))
    tangent_len_q = math.sqrt(max(0.0, dq * dq - radius * radius))
    xy_length = tangent_len_p + radius * arc_angle + tangent_len_q

    return _WrapCandidate(
        path_type=path_type,
        xy_length=xy_length,
        tangent_start=tangent_start,
        tangent_end=tangent_end,
        arc_angle_rad=arc_angle,
    )


def _best_xy_path_around_cylinder(
    p: Vec2,
    q: Vec2,
    c: Vec2,
    radius: float,
    wrap_mode: str = "shortest",
) -> Tuple[str, float, Optional[_WrapCandidate]]:
    if wrap_mode not in ("shortest", "cw", "ccw", "direct-if-clear-cw", "direct-if-clear-ccw"):
        raise ValueError(f"unknown wrap_mode: {wrap_mode}")

    if wrap_mode == "shortest" and _straight_path_clear(p, q, c, radius):
        return ("direct", _norm2(_sub2(q, p)), None)

    if wrap_mode == "direct-if-clear-cw" and _straight_path_clear(p, q, c, radius):
        return ("direct", _norm2(_sub2(q, p)), None)

    if wrap_mode == "direct-if-clear-ccw" and _straight_path_clear(p, q, c, radius):
        return ("direct", _norm2(_sub2(q, p)), None)

    if wrap_mode in ("cw", "direct-if-clear-cw"):
        cw = _wrap_candidate(p, q, c, radius, ccw=False)
        return (cw.path_type, cw.xy_length, cw)

    if wrap_mode in ("ccw", "direct-if-clear-ccw"):
        ccw = _wrap_candidate(p, q, c, radius, ccw=True)
        return (ccw.path_type, ccw.xy_length, ccw)

    ccw = _wrap_candidate(p, q, c, radius, ccw=True)
    cw = _wrap_candidate(p, q, c, radius, ccw=False)
    best = ccw if ccw.xy_length <= cw.xy_length else cw
    return (best.path_type, best.xy_length, best)


def compute_m00_id1_right_mcp_tendon_length(
    theta1_deg: float,
    theta2_deg: float,
    geom: M00Id1RightMcpGeometry = M00Id1RightMcpGeometry(),
    wrap_mode: str = "shortest",
) -> TendonLengthResult:
    """Compute M00/ID1 right-MCP tendon length in mm for J00/J01 angles."""

    a1 = (geom.a1_x, geom.a1_y, geom.a1_z)
    d1 = compute_d1(theta1_deg, theta2_deg, geom)
    a1_path = _to_mcp_obstacle_frame(a1, theta1_deg, geom.cylinder_follows_theta1)
    d1_path = _to_mcp_obstacle_frame(d1, theta1_deg, geom.cylinder_follows_theta1)
    p_xy = (a1_path[0], a1_path[1])
    q_xy = (d1_path[0], d1_path[1])
    cylinder_xy = (geom.cylinder_x, geom.cylinder_y)

    straight_length = _dist3(a1, d1)
    path_type, xy_length, wrap = _best_xy_path_around_cylinder(
        p_xy,
        q_xy,
        cylinder_xy,
        geom.cylinder_radius,
        wrap_mode,
    )

    z_delta = d1_path[2] - a1_path[2]
    length = math.sqrt(xy_length * xy_length + z_delta * z_delta)
    blocked = path_type != "direct"

    if wrap is None:
        return TendonLengthResult(
            theta1_deg=theta1_deg,
            theta2_deg=theta2_deg,
            length_mm=length,
            straight_length_mm=straight_length,
            extra_length_mm=length - straight_length,
            xy_path_length_mm=xy_length,
            z_delta_mm=z_delta,
            path_type=path_type,
            blocked_by_cylinder=blocked,
            d1_x=d1[0],
            d1_y=d1[1],
            d1_z=d1[2],
        )

    return TendonLengthResult(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        length_mm=length,
        straight_length_mm=straight_length,
        extra_length_mm=length - straight_length,
        xy_path_length_mm=xy_length,
        z_delta_mm=z_delta,
        path_type=path_type,
        blocked_by_cylinder=blocked,
        d1_x=d1[0],
        d1_y=d1[1],
        d1_z=d1[2],
        tangent_start_x=wrap.tangent_start[0],
        tangent_start_y=wrap.tangent_start[1],
        tangent_end_x=wrap.tangent_end[0],
        tangent_end_y=wrap.tangent_end[1],
        arc_angle_deg=math.degrees(wrap.arc_angle_rad),
    )


def _frange(start: float, stop: float, step: float) -> Iterable[float]:
    if step <= 0:
        raise ValueError("step must be positive")
    value = start
    eps = step * 1.0e-9
    while value <= stop + eps:
        yield round(value, 10)
        value += step


def _write_grid_csv(path: Path, rows: Iterable[TendonLengthResult]) -> None:
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
        description="Offline M00/ID1 right-MCP tendon length model with infinite-cylinder avoidance."
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
    parser.add_argument("--out", type=Path, default=Path("run_data/m00_id1_right_mcp_tendon_length_grid.csv"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    base_geom = M00Id1RightMcpGeometry()
    geom = M00Id1RightMcpGeometry(
        cylinder_radius=base_geom.cylinder_radius + args.radius_padding
    )

    if args.grid:
        rows = [
            compute_m00_id1_right_mcp_tendon_length(theta1, theta2, geom, args.wrap_mode)
            for theta1 in _frange(args.theta1_min, args.theta1_max, args.theta1_step)
            for theta2 in _frange(args.theta2_min, args.theta2_max, args.theta2_step)
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_m00_id1_right_mcp_tendon_length(args.theta1, args.theta2, geom, args.wrap_mode)
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    print(f"theta1_deg={result.theta1_deg:.3f}")
    print(f"theta2_deg={result.theta2_deg:.3f}")
    print(f"path_type={result.path_type}")
    print(f"blocked_by_cylinder={int(result.blocked_by_cylinder)}")
    print(f"D1=({result.d1_x:.3f}, {result.d1_y:.3f}, {result.d1_z:.3f}) mm")
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
