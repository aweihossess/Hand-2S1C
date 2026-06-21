#!/usr/bin/env python3
"""Offline M04/ID5 return-tendon length feedforward estimator.

This model is standalone and is not connected to the firmware control path.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from m00_id1_right_mcp_tendon_model import _dist3, _frange


Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


DEFAULT_THETA2_ZERO_OFFSET_DEG = -math.degrees(math.atan2(9.00, 8.10))


@dataclass(frozen=True)
class M04Id5ReturnGeometry:
    # Fixed/moving geometry for the MCP-AA return contribution, mm.
    theta1_anchor_x: float = -6.95
    theta1_anchor_y: float = 5.50
    theta1_anchor_z: float = 0.00
    theta1_circle_x: float = 0.00
    theta1_circle_y: float = 5.50
    theta1_circle_z: float = 0.00
    theta1_circle_radius: float = 7.00

    # Fixed anchor for the MCP-FE return routing component, mm.
    anchor_x: float = 3.50
    anchor_y: float = 5.50
    anchor_z: float = 0.00

    # Moving point lies on this circle in the XY plane.
    circle_x: float = 13.00
    circle_y: float = 0.00
    circle_z: float = 0.00
    circle_radius: float = 12.11
    theta2_zero_offset_deg: float = DEFAULT_THETA2_ZERO_OFFSET_DEG

    # Infinite-Z keepout:
    #   x < keepout_x_right
    #   keepout_y_min < y < keepout_y_max
    # with the right-side intersections rounded by radius keepout_right_corner_radius.
    keepout_x_right: float = 17.00
    keepout_y_min: float = -5.00
    keepout_y_max: float = 3.70
    keepout_right_corner_radius: float = 3.00
    mcp_fe_keepout_follows_theta2: bool = True

    # The keepout extends to x=-infinity. This finite value is only used to
    # sample boundary nodes; the top and bottom boundary samples are not closed
    # into a fake left-side bypass.
    keepout_sample_x_left: float = -40.00

    keepout_edge_samples: int = 48
    keepout_arc_samples: int = 24

    # Linear terms. Negative means the physical tendon gets shorter as the
    # corresponding flexion angle increases.
    pip_return_slope_mm_per_rad: float = -9.90
    dip_return_slope_mm_per_rad: float = -6.45


@dataclass(frozen=True)
class M04TendonLengthResult:
    theta1_deg: float
    theta2_deg: float
    theta3_deg: float
    theta4_deg: float
    length_delta_mm: float
    theta1_return_delta_mm: float
    theta1_return_length_mm: float
    theta1_return_zero_length_mm: float
    theta1_b_x: float
    theta1_b_y: float
    theta1_b_z: float
    mcp_fe_routing_delta_mm: float
    mcp_fe_routing_length_mm: float
    mcp_fe_routing_zero_length_mm: float
    mcp_fe_routing_straight_length_mm: float
    mcp_fe_routing_extra_length_mm: float
    mcp_fe_routing_xy_path_length_mm: float
    mcp_fe_routing_z_delta_mm: float
    pip_return_delta_mm: float
    dip_return_delta_mm: float
    d_x: float
    d_y: float
    d_z: float
    xy_path_points: str
    model_status: str


def _dist2(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _rotate2_about(point: Vec2, center: Vec2, angle_deg: float) -> Vec2:
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    return (
        center[0] + c * dx - s * dy,
        center[1] + s * dx + c * dy,
    )


def _to_m04_theta2_keepout_frame(point: Vec3, theta2_deg: float, geom: M04Id5ReturnGeometry) -> Vec3:
    if not geom.mcp_fe_keepout_follows_theta2:
        return point
    x, y = _rotate2_about((point[0], point[1]), (geom.circle_x, geom.circle_y), -theta2_deg)
    return (x, y, point[2])


def _sample_line(a: Vec2, b: Vec2, count: int, include_end: bool = False) -> List[Vec2]:
    count = max(1, count)
    denom = count if include_end else count + 1
    points = []
    for i in range(1, count + (1 if include_end else 0)):
        t = i / denom
        points.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return points


def _sample_arc(center: Vec2, radius: float, start_rad: float, end_rad: float, count: int) -> List[Vec2]:
    count = max(2, count)
    return [
        (
            center[0] + radius * math.cos(start_rad + (end_rad - start_rad) * i / count),
            center[1] + radius * math.sin(start_rad + (end_rad - start_rad) * i / count),
        )
        for i in range(count + 1)
    ]


def _right_rounded_keepout_contains(p: Vec2, geom: M04Id5ReturnGeometry, eps: float = 1.0e-6) -> bool:
    x, y = p
    xr = geom.keepout_x_right
    y_min = geom.keepout_y_min
    y_max = geom.keepout_y_max
    r = geom.keepout_right_corner_radius

    if y < y_min + eps or y > y_max - eps:
        return False
    if x <= xr - r:
        return True
    if x > xr - eps:
        return False

    top_center = (xr - r, y_max - r)
    bottom_center = (xr - r, y_min + r)
    if y >= y_max - r:
        return _dist2(p, top_center) < r - eps
    if y <= y_min + r:
        return _dist2(p, bottom_center) < r - eps
    return x < xr - eps


def _segment_clear(a: Vec2, b: Vec2, geom: M04Id5ReturnGeometry) -> bool:
    length = _dist2(a, b)
    steps = max(2, int(math.ceil(length / 0.15)))
    for i in range(1, steps):
        t = i / steps
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if _right_rounded_keepout_contains(p, geom):
            return False
    return True


def _sample_right_rounded_keepout_boundary(geom: M04Id5ReturnGeometry, start: Vec2, goal: Vec2) -> List[Vec2]:
    xr = geom.keepout_x_right
    y_min = geom.keepout_y_min
    y_max = geom.keepout_y_max
    r = geom.keepout_right_corner_radius
    x_left = min(geom.keepout_sample_x_left, start[0] - 20.0, goal[0] - 20.0)
    edge_n = max(4, geom.keepout_edge_samples)
    arc_n = max(4, geom.keepout_arc_samples)

    top_center = (xr - r, y_max - r)
    bottom_center = (xr - r, y_min + r)

    points: List[Vec2] = []
    points.extend(_sample_line((x_left, y_max), (xr - r, y_max), edge_n, include_end=True))
    points.extend(_sample_arc(top_center, r, math.pi / 2.0, 0.0, arc_n)[1:])
    points.extend(_sample_line((xr, y_max - r), (xr, y_min + r), edge_n, include_end=True))
    points.extend(_sample_arc(bottom_center, r, 0.0, -math.pi / 2.0, arc_n)[1:])
    points.extend(_sample_line((xr - r, y_min), (x_left, y_min), edge_n, include_end=True))

    deduped: List[Vec2] = []
    for p in points:
        if not deduped or _dist2(deduped[-1], p) > 1.0e-6:
            deduped.append(p)
    return deduped


def _compute_shortest_xy_path(start: Vec2, goal: Vec2, geom: M04Id5ReturnGeometry) -> Tuple[float, List[Vec2]]:
    if _right_rounded_keepout_contains(start, geom) or _right_rounded_keepout_contains(goal, geom):
        raise ValueError("start or goal is inside the return keepout")

    if _segment_clear(start, goal, geom):
        return _dist2(start, goal), [start, goal]

    keepout_nodes = _sample_right_rounded_keepout_boundary(geom, start, goal)
    nodes = [start, goal] + keepout_nodes
    n = len(nodes)
    adjacency: List[List[Tuple[int, float]]] = [[] for _ in range(n)]

    def add_edge(i: int, j: int, weight: Optional[float] = None) -> None:
        if weight is None:
            weight = _dist2(nodes[i], nodes[j])
        adjacency[i].append((j, weight))
        adjacency[j].append((i, weight))

    for i in range(n):
        for j in range(i + 1, n):
            if _segment_clear(nodes[i], nodes[j], geom):
                add_edge(i, j)

    # Boundary edges are a chain around the right side. Do not close the chain
    # across the sampled left end, because the real keepout extends to x=-infinity.
    keep_start = 2
    keep_count = len(keepout_nodes)
    for k in range(keep_count - 1):
        i = keep_start + k
        j = keep_start + k + 1
        add_edge(i, j, _dist2(nodes[i], nodes[j]))

    dist = [float("inf")] * n
    prev = [-1] * n
    dist[0] = 0.0
    heap: List[Tuple[float, int]] = [(0.0, 0)]
    while heap:
        current_dist, i = heapq.heappop(heap)
        if current_dist != dist[i]:
            continue
        if i == 1:
            break
        for j, w in adjacency[i]:
            nd = current_dist + w
            if nd < dist[j]:
                dist[j] = nd
                prev[j] = i
                heapq.heappush(heap, (nd, j))

    if not math.isfinite(dist[1]):
        raise ValueError("no collision-free XY path found")

    path_idx = []
    cur = 1
    while cur >= 0:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return dist[1], [nodes[i] for i in path_idx]


def _path_points_to_text(points: List[Vec2]) -> str:
    return ";".join(f"{x:.3f},{y:.3f}" for x, y in points)


def compute_m04_mcp_fe_point(theta2_deg: float, geom: M04Id5ReturnGeometry) -> Vec3:
    theta = math.radians(theta2_deg + geom.theta2_zero_offset_deg)
    return (
        geom.circle_x + geom.circle_radius * math.cos(theta),
        geom.circle_y + geom.circle_radius * math.sin(theta),
        geom.circle_z,
    )


def compute_m04_mcp_fe_routing_length_mm(
    theta2_deg: float,
    geom: M04Id5ReturnGeometry,
) -> Tuple[float, float, float, float, Vec3, List[Vec2]]:
    """Return MCP-FE return routing length and diagnostics."""

    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    d = compute_m04_mcp_fe_point(theta2_deg, geom)
    anchor_path = _to_m04_theta2_keepout_frame(anchor, theta2_deg, geom)
    d_path = _to_m04_theta2_keepout_frame(d, theta2_deg, geom)
    start_xy = (anchor_path[0], anchor_path[1])
    goal_xy = (d_path[0], d_path[1])
    xy_length, xy_path = _compute_shortest_xy_path(start_xy, goal_xy, geom)
    z_delta = d_path[2] - anchor_path[2]
    length = math.sqrt(xy_length * xy_length + z_delta * z_delta)
    straight = _dist3(anchor, d)
    return length, straight, xy_length, z_delta, d, xy_path


def compute_mcp_fe_routing_delta_mm(
    theta2_deg: float,
    geom: M04Id5ReturnGeometry,
) -> Tuple[float, float, float, float, float, float, Vec3, List[Vec2]]:
    length, straight, xy_len, z_delta, d, path = compute_m04_mcp_fe_routing_length_mm(theta2_deg, geom)
    zero_length, _, _, _, _, _ = compute_m04_mcp_fe_routing_length_mm(0.0, geom)
    # M04 crosses the upper side of the MCP-FE axis, so positive MCP-FE should
    # lengthen this tendon. The geometric lower-side routing term is therefore
    # sign-flipped relative to the raw length delta.
    return zero_length - length, length, zero_length, straight, length - straight, xy_len, z_delta, d, path


def compute_m04_theta1_return_b(theta1_deg: float, geom: M04Id5ReturnGeometry) -> Vec3:
    """Compute point B on the MCP-AA return circle.

    theta1=0 is the closest B point to A, so B starts on the negative-X side of
    the circle from its center.
    """

    theta = math.radians(theta1_deg)
    return (
        geom.theta1_circle_x - geom.theta1_circle_radius * math.cos(theta),
        geom.theta1_circle_y + geom.theta1_circle_radius * math.sin(theta),
        geom.theta1_circle_z,
    )


def compute_theta1_return_length_mm(theta1_deg: float, geom: M04Id5ReturnGeometry) -> Tuple[float, Vec3]:
    anchor = (geom.theta1_anchor_x, geom.theta1_anchor_y, geom.theta1_anchor_z)
    b = compute_m04_theta1_return_b(theta1_deg, geom)
    return _dist3(anchor, b), b


def compute_theta1_return_delta_mm(theta1_deg: float, geom: M04Id5ReturnGeometry) -> Tuple[float, float, float, Vec3]:
    length, b = compute_theta1_return_length_mm(theta1_deg, geom)
    zero_length, _ = compute_theta1_return_length_mm(0.0, geom)
    return length - zero_length, length, zero_length, b


def compute_pip_return_delta_mm(theta3_deg: float, geom: M04Id5ReturnGeometry) -> float:
    return geom.pip_return_slope_mm_per_rad * math.radians(theta3_deg)


def compute_dip_return_delta_mm(theta4_deg: float, geom: M04Id5ReturnGeometry) -> float:
    return geom.dip_return_slope_mm_per_rad * math.radians(theta4_deg)


def compute_m04_id5_return_tendon_delta(
    theta1_deg: float,
    theta2_deg: float,
    theta3_deg: float,
    theta4_deg: float,
    geom: M04Id5ReturnGeometry = M04Id5ReturnGeometry(),
) -> M04TendonLengthResult:
    """Compute M04/ID5 return tendon length delta in mm."""

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
    ) = compute_mcp_fe_routing_delta_mm(theta2_deg, geom)
    theta1_delta, theta1_length, theta1_zero, theta1_b = compute_theta1_return_delta_mm(theta1_deg, geom)
    pip_delta = compute_pip_return_delta_mm(theta3_deg, geom)
    dip_delta = compute_dip_return_delta_mm(theta4_deg, geom)
    return M04TendonLengthResult(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        theta3_deg=theta3_deg,
        theta4_deg=theta4_deg,
        length_delta_mm=theta1_delta + mcp_delta + pip_delta + dip_delta,
        theta1_return_delta_mm=theta1_delta,
        theta1_return_length_mm=theta1_length,
        theta1_return_zero_length_mm=theta1_zero,
        theta1_b_x=theta1_b[0],
        theta1_b_y=theta1_b[1],
        theta1_b_z=theta1_b[2],
        mcp_fe_routing_delta_mm=mcp_delta,
        mcp_fe_routing_length_mm=mcp_length,
        mcp_fe_routing_zero_length_mm=mcp_zero,
        mcp_fe_routing_straight_length_mm=mcp_straight,
        mcp_fe_routing_extra_length_mm=mcp_extra,
        mcp_fe_routing_xy_path_length_mm=mcp_xy,
        mcp_fe_routing_z_delta_mm=mcp_z_delta,
        pip_return_delta_mm=pip_delta,
        dip_return_delta_mm=dip_delta,
        d_x=d[0],
        d_y=d[1],
        d_z=d[2],
        xy_path_points=_path_points_to_text(path),
        model_status="theta1_ab_plus_mcp_fe_routing_plus_pip_dip_return",
    )


def _write_grid_csv(path: Path, rows: Iterable[M04TendonLengthResult]) -> None:
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
        description="Offline M04/ID5 return tendon length model."
    )
    parser.add_argument("--theta1", type=float, default=0.0, help="J00/MCP-AA angle in degrees.")
    parser.add_argument("--theta2", type=float, default=0.0, help="J01/MCP-FE angle in degrees.")
    parser.add_argument("--theta3", type=float, default=0.0, help="J02/PIP-FE angle in degrees.")
    parser.add_argument("--theta4", type=float, default=0.0, help="J03/DIP/distal-FE angle in degrees.")
    parser.add_argument(
        "--obstacle-padding",
        type=float,
        default=0.0,
        help="Extra clearance in mm added to the rounded keepout, e.g. tendon radius.",
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
    parser.add_argument("--out", type=Path, default=Path("run_data/m04_id5_return_tendon_delta_grid.csv"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    base_geom = M04Id5ReturnGeometry()
    padding = args.obstacle_padding
    geom = M04Id5ReturnGeometry(
        keepout_y_min=base_geom.keepout_y_min - padding,
        keepout_y_max=base_geom.keepout_y_max + padding,
        keepout_x_right=base_geom.keepout_x_right + padding,
        keepout_right_corner_radius=base_geom.keepout_right_corner_radius + padding,
    )

    if args.grid:
        theta1_values = _grid_values(args.theta1, args.theta1_min, args.theta1_max, args.theta1_step)
        theta2_values = _grid_values(args.theta2, args.theta2_min, args.theta2_max, args.theta2_step)
        theta3_values = _grid_values(args.theta3, args.theta3_min, args.theta3_max, args.theta3_step)
        theta4_values = _grid_values(args.theta4, args.theta4_min, args.theta4_max, args.theta4_step)
        rows = [
            compute_m04_id5_return_tendon_delta(theta1, theta2, theta3, theta4, geom)
            for theta1 in theta1_values
            for theta2 in theta2_values
            for theta3 in theta3_values
            for theta4 in theta4_values
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_m04_id5_return_tendon_delta(args.theta1, args.theta2, args.theta3, args.theta4, geom)
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    print(f"theta1_deg={result.theta1_deg:.3f}")
    print(f"theta2_deg={result.theta2_deg:.3f}")
    print(f"theta3_deg={result.theta3_deg:.3f}")
    print(f"theta4_deg={result.theta4_deg:.3f}")
    print(f"model_status={result.model_status}")
    print(f"D=({result.d_x:.3f}, {result.d_y:.3f}, {result.d_z:.3f}) mm")
    print(f"theta1_return_length_mm={result.theta1_return_length_mm:.6f}")
    print(f"theta1_return_zero_length_mm={result.theta1_return_zero_length_mm:.6f}")
    print(f"theta1_return_delta_mm={result.theta1_return_delta_mm:.6f}")
    print(f"theta1_B=({result.theta1_b_x:.3f}, {result.theta1_b_y:.3f}, {result.theta1_b_z:.3f}) mm")
    print(f"mcp_fe_routing_length_mm={result.mcp_fe_routing_length_mm:.6f}")
    print(f"mcp_fe_routing_zero_length_mm={result.mcp_fe_routing_zero_length_mm:.6f}")
    print(f"mcp_fe_routing_delta_mm={result.mcp_fe_routing_delta_mm:.6f}")
    print(f"mcp_fe_routing_straight_length_mm={result.mcp_fe_routing_straight_length_mm:.6f}")
    print(f"mcp_fe_routing_extra_length_mm={result.mcp_fe_routing_extra_length_mm:.6f}")
    print(f"mcp_fe_routing_xy_path_length_mm={result.mcp_fe_routing_xy_path_length_mm:.6f}")
    print(f"mcp_fe_routing_z_delta_mm={result.mcp_fe_routing_z_delta_mm:.6f}")
    print(f"pip_return_delta_mm={result.pip_return_delta_mm:.6f}")
    print(f"dip_return_delta_mm={result.dip_return_delta_mm:.6f}")
    print(f"length_delta_mm={result.length_delta_mm:.6f}")
    print("note=positive length_delta means the return tendon gets longer; motor_abs should decrease after counts conversion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
