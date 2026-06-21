#!/usr/bin/env python3
"""Offline M02/ID3 PIP-flexion tendon-length feedforward estimator.

The model is intentionally kept standalone and is not connected to the firmware
control path.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import heapq
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from m00_id1_right_mcp_tendon_model import _dist3, _frange, _to_mcp_obstacle_frame


Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


DEFAULT_M02_L2_MM = math.hypot(8.10, 0.18)
DEFAULT_M02_THETA_O_DEG = -math.degrees(math.atan2(0.18, 8.10))


@dataclass(frozen=True)
class M02Id3PipFlexGeometry:
    # A fixed anchor for the MCP routing component, mm.
    anchor_x: float = -6.66
    anchor_y: float = 2.75
    anchor_z: float = 4.00

    # MCP routing geometry, mm and degrees.
    l1: float = 13.00
    l2: float = DEFAULT_M02_L2_MM
    l3: float = 1.50
    theta_o_deg: float = DEFAULT_M02_THETA_O_DEG

    # First obstacle: same center as MCP obstacle, smaller radius.
    cylinder_x: float = 13.00
    cylinder_y: float = 0.00
    cylinder_radius: float = 3.50

    # Second obstacle: infinite-Z keepout near the origin.
    # Approximation: rectangle x=[-5, 5], y=[-5, 3.7] with both left corners
    # rounded by radius 2.0 mm.
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

    # PIP flexion tendon moment arm / length slope.
    # Physical meaning:
    #   theta3 increases by 1 rad -> tendon length changes by -6 mm.
    pip_flex_length_slope_mm_per_rad: float = -6.0


@dataclass(frozen=True)
class M02TendonLengthResult:
    theta1_deg: float
    theta2_deg: float
    theta3_deg: float
    length_delta_mm: float
    mcp_routing_delta_mm: float
    mcp_routing_length_mm: float
    mcp_routing_zero_length_mm: float
    mcp_routing_straight_length_mm: float
    mcp_routing_extra_length_mm: float
    mcp_routing_xy_path_length_mm: float
    mcp_routing_z_delta_mm: float
    pip_flexion_delta_mm: float
    d_x: float
    d_y: float
    d_z: float
    xy_path_points: str
    model_status: str


def _dist2(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _segment_distance_to_point(a: Vec2, b: Vec2, c: Vec2) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    denom = vx * vx + vy * vy
    if denom <= 1.0e-12:
        return _dist2(a, c)
    t = ((c[0] - a[0]) * vx + (c[1] - a[1]) * vy) / denom
    t = max(0.0, min(1.0, t))
    p = (a[0] + t * vx, a[1] + t * vy)
    return _dist2(p, c)


def _left_rounded_keepout_contains(p: Vec2, geom: M02Id3PipFlexGeometry, eps: float = 1.0e-6) -> bool:
    x, y = p
    x_min = geom.keepout_x_min
    x_max = geom.keepout_x_max
    y_min = geom.keepout_y_min
    y_max = geom.keepout_y_max
    r = geom.keepout_left_corner_radius

    if x < x_min + eps or x > x_max - eps or y < y_min + eps or y > y_max - eps:
        return False

    corner_x = x_min + r
    bottom_center = (corner_x, y_min + r)
    top_center = (corner_x, y_max - r)

    if x >= corner_x:
        return True
    if bottom_center[1] <= y <= top_center[1]:
        return True
    if y < bottom_center[1]:
        return _dist2(p, bottom_center) < r - eps
    return _dist2(p, top_center) < r - eps


def _circle_contains(p: Vec2, center: Vec2, radius: float, eps: float = 1.0e-6) -> bool:
    return _dist2(p, center) < radius - eps


def _segment_clear(a: Vec2, b: Vec2, geom: M02Id3PipFlexGeometry) -> bool:
    circle_center = (geom.cylinder_x, geom.cylinder_y)
    if _segment_distance_to_point(a, b, circle_center) < geom.cylinder_radius - 1.0e-6:
        return False

    length = _dist2(a, b)
    steps = max(2, int(math.ceil(length / 0.20)))
    for i in range(1, steps):
        t = i / steps
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if _left_rounded_keepout_contains(p, geom):
            return False
    return True


def _sample_circle_boundary(center: Vec2, radius: float, count: int) -> List[Vec2]:
    count = max(12, count)
    return [
        (center[0] + radius * math.cos(2.0 * math.pi * i / count),
         center[1] + radius * math.sin(2.0 * math.pi * i / count))
        for i in range(count)
    ]


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
        (center[0] + radius * math.cos(start_rad + (end_rad - start_rad) * i / count),
         center[1] + radius * math.sin(start_rad + (end_rad - start_rad) * i / count))
        for i in range(count + 1)
    ]


def _sample_keepout_boundary(geom: M02Id3PipFlexGeometry) -> List[Vec2]:
    x_min = geom.keepout_x_min
    x_max = geom.keepout_x_max
    y_min = geom.keepout_y_min
    y_max = geom.keepout_y_max
    r = geom.keepout_left_corner_radius
    edge_n = max(4, geom.keepout_edge_samples)
    arc_n = max(4, geom.keepout_arc_samples)

    corner_x = x_min + r
    bottom_center = (corner_x, y_min + r)
    top_center = (corner_x, y_max - r)

    # Clockwise loop around the left-rounded rectangle.
    points: List[Vec2] = []
    points.extend(_sample_line((corner_x, y_min), (x_max, y_min), edge_n, include_end=True))
    points.extend(_sample_line((x_max, y_min), (x_max, y_max), edge_n, include_end=True))
    points.extend(_sample_line((x_max, y_max), (corner_x, y_max), edge_n, include_end=True))
    points.extend(_sample_arc(top_center, r, math.pi / 2.0, math.pi, arc_n)[1:])
    points.extend(_sample_line((x_min, y_max - r), (x_min, y_min + r), edge_n, include_end=True))
    points.extend(_sample_arc(bottom_center, r, math.pi, 3.0 * math.pi / 2.0, arc_n)[1:])

    # Deduplicate adjacent equal points.
    deduped: List[Vec2] = []
    for p in points:
        if not deduped or _dist2(deduped[-1], p) > 1.0e-6:
            deduped.append(p)
    return deduped


def _compute_shortest_xy_path(start: Vec2, goal: Vec2, geom: M02Id3PipFlexGeometry) -> Tuple[float, List[Vec2]]:
    circle_center = (geom.cylinder_x, geom.cylinder_y)
    if _circle_contains(start, circle_center, geom.cylinder_radius) or _circle_contains(goal, circle_center, geom.cylinder_radius):
        raise ValueError("start or goal is inside the cylinder keepout")
    if _left_rounded_keepout_contains(start, geom) or _left_rounded_keepout_contains(goal, geom):
        raise ValueError("start or goal is inside the origin keepout")

    if _segment_clear(start, goal, geom):
        return _dist2(start, goal), [start, goal]

    circle_nodes = _sample_circle_boundary(circle_center, geom.cylinder_radius, geom.circle_samples)
    keepout_nodes = _sample_keepout_boundary(geom)
    nodes = [start, goal] + circle_nodes + keepout_nodes
    n = len(nodes)
    adjacency: List[List[Tuple[int, float]]] = [[] for _ in range(n)]

    def add_edge(i: int, j: int, weight: Optional[float] = None) -> None:
        if weight is None:
            weight = _dist2(nodes[i], nodes[j])
        adjacency[i].append((j, weight))
        adjacency[j].append((i, weight))

    # Line-of-sight edges.
    for i in range(n):
        for j in range(i + 1, n):
            if _segment_clear(nodes[i], nodes[j], geom):
                add_edge(i, j)

    # Boundary edges for the circular cylinder.
    circle_start = 2
    circle_count = len(circle_nodes)
    arc_weight = geom.cylinder_radius * (2.0 * math.pi / circle_count)
    for k in range(circle_count):
        add_edge(circle_start + k, circle_start + ((k + 1) % circle_count), arc_weight)

    # Boundary edges for the left-rounded keepout polygon.
    keep_start = circle_start + circle_count
    keep_count = len(keepout_nodes)
    for k in range(keep_count):
        i = keep_start + k
        j = keep_start + ((k + 1) % keep_count)
        add_edge(i, j, _dist2(nodes[i], nodes[j]))

    # Dijkstra.
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


def compute_m02_mcp_routing_d(theta1_deg: float, theta2_deg: float, geom: M02Id3PipFlexGeometry) -> Vec3:
    """Compute the MCP routing moving point.

    M02 runs under the MCP-FE axis in the current hardware, so positive MCP-FE
    uses the negative geometric theta2 direction and should shorten this tendon.
    """

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(-theta2_deg + geom.theta_o_deg)
    projected = geom.l1 + geom.l2 * math.cos(theta2)

    x = math.cos(theta1) * projected - geom.l3 * math.sin(theta1)
    y = -geom.l2 * math.sin(theta2)
    z = math.sin(theta1) * projected + geom.l3 * math.cos(theta1)
    return (x, y, z)


def compute_m02_mcp_routing_length_mm(
    theta1_deg: float,
    theta2_deg: float,
    geom: M02Id3PipFlexGeometry,
) -> Tuple[float, float, float, float, Vec3, List[Vec2]]:
    """Return physical routing length and diagnostics.

    Returns:
        length_mm, straight_length_mm, xy_path_length_mm, z_delta_mm, d, xy_path
    """

    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    d = compute_m02_mcp_routing_d(theta1_deg, theta2_deg, geom)
    anchor_path = _to_mcp_obstacle_frame(anchor, theta1_deg, geom.obstacles_follow_theta1)
    d_path = _to_mcp_obstacle_frame(d, theta1_deg, geom.obstacles_follow_theta1)
    start_xy = (anchor_path[0], anchor_path[1])
    goal_xy = (d_path[0], d_path[1])
    xy_length, xy_path = _compute_shortest_xy_path(start_xy, goal_xy, geom)
    z_delta = d_path[2] - anchor_path[2]
    length = math.sqrt(xy_length * xy_length + z_delta * z_delta)
    straight = _dist3(anchor, d)
    return length, straight, xy_length, z_delta, d, xy_path


def compute_mcp_routing_delta_mm(theta1_deg: float, theta2_deg: float, geom: M02Id3PipFlexGeometry) -> Tuple[float, float, float, float, float, Vec3, List[Vec2]]:
    length, straight, xy_len, z_delta, d, path = compute_m02_mcp_routing_length_mm(theta1_deg, theta2_deg, geom)
    zero_length, _, _, _, _, _ = compute_m02_mcp_routing_length_mm(0.0, 0.0, geom)
    return length - zero_length, length, zero_length, straight, xy_len, z_delta, d, path


def compute_pip_flexion_delta_mm(theta3_deg: float, geom: M02Id3PipFlexGeometry) -> float:
    """Return the PIP-FE contribution to M02 tendon length."""

    theta3_rad = math.radians(theta3_deg)
    return geom.pip_flex_length_slope_mm_per_rad * theta3_rad


def compute_m02_id3_pip_flex_tendon_delta(
    theta1_deg: float,
    theta2_deg: float,
    theta3_deg: float,
    geom: M02Id3PipFlexGeometry = M02Id3PipFlexGeometry(),
) -> M02TendonLengthResult:
    """Compute M02/ID3 tendon length delta in mm.

    The current model is:

        delta_L = mcp_routing_delta(theta1, theta2) + pip_flexion_delta(theta3)

    with:

        pip_flexion_delta(theta3) = -6 * theta3_rad

    Negative delta_L means the physical tendon gets shorter, so with the current
    hardware convention it should correspond to increasing motor_abs after
    conversion to motor counts.
    """

    (
        mcp_delta,
        mcp_length,
        mcp_zero,
        mcp_straight,
        mcp_xy,
        mcp_z_delta,
        d,
        path,
    ) = compute_mcp_routing_delta_mm(theta1_deg, theta2_deg, geom)
    pip_delta = compute_pip_flexion_delta_mm(theta3_deg, geom)
    return M02TendonLengthResult(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        theta3_deg=theta3_deg,
        length_delta_mm=mcp_delta + pip_delta,
        mcp_routing_delta_mm=mcp_delta,
        mcp_routing_length_mm=mcp_length,
        mcp_routing_zero_length_mm=mcp_zero,
        mcp_routing_straight_length_mm=mcp_straight,
        mcp_routing_extra_length_mm=mcp_length - mcp_straight,
        mcp_routing_xy_path_length_mm=mcp_xy,
        mcp_routing_z_delta_mm=mcp_z_delta,
        pip_flexion_delta_mm=pip_delta,
        d_x=d[0],
        d_y=d[1],
        d_z=d[2],
        xy_path_points=_path_points_to_text(path),
        model_status="mcp_routing_plus_pip_flexion",
    )


def _write_grid_csv(path: Path, rows: Iterable[M02TendonLengthResult]) -> None:
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
        description="Offline M02/ID3 PIP flexion tendon length model."
    )
    parser.add_argument("--theta1", type=float, default=0.0, help="J00/MCP-AA angle in degrees.")
    parser.add_argument("--theta2", type=float, default=0.0, help="J01/MCP-FE angle in degrees.")
    parser.add_argument("--theta3", type=float, default=0.0, help="J02/PIP-FE angle in degrees.")
    parser.add_argument(
        "--obstacle-padding",
        type=float,
        default=0.0,
        help="Extra clearance in mm added to the cylinder and origin keepout, e.g. tendon radius.",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON instead of a readable summary.")

    parser.add_argument("--grid", action="store_true", help="Export a theta1/theta2/theta3 grid to CSV.")
    parser.add_argument("--theta1-min", type=float, default=-20.0)
    parser.add_argument("--theta1-max", type=float, default=30.0)
    parser.add_argument("--theta1-step", type=float, default=1.0)
    parser.add_argument("--theta2-min", type=float, default=None)
    parser.add_argument("--theta2-max", type=float, default=None)
    parser.add_argument("--theta2-step", type=float, default=1.0)
    parser.add_argument("--theta3-min", type=float, default=0.0)
    parser.add_argument("--theta3-max", type=float, default=90.0)
    parser.add_argument("--theta3-step", type=float, default=1.0)
    parser.add_argument("--out", type=Path, default=Path("run_data/m02_id3_pip_flex_tendon_delta_grid.csv"))
    return parser


def _grid_values(single_value: float, min_value: Optional[float], max_value: Optional[float], step: float) -> List[float]:
    if min_value is None and max_value is None:
        return [single_value]
    if min_value is None or max_value is None:
        raise ValueError("grid min and max must be provided together")
    return list(_frange(min_value, max_value, step))


def main() -> int:
    args = build_arg_parser().parse_args()
    base_geom = M02Id3PipFlexGeometry()
    padding = args.obstacle_padding
    geom = M02Id3PipFlexGeometry(
        cylinder_radius=base_geom.cylinder_radius + padding,
        keepout_x_min=base_geom.keepout_x_min - padding,
        keepout_x_max=base_geom.keepout_x_max + padding,
        keepout_y_min=base_geom.keepout_y_min - padding,
        keepout_y_max=base_geom.keepout_y_max + padding,
        keepout_left_corner_radius=base_geom.keepout_left_corner_radius + padding,
    )

    if args.grid:
        theta2_values = _grid_values(args.theta2, args.theta2_min, args.theta2_max, args.theta2_step)
        rows = [
            compute_m02_id3_pip_flex_tendon_delta(theta1, theta2, theta3, geom)
            for theta1 in _frange(args.theta1_min, args.theta1_max, args.theta1_step)
            for theta2 in theta2_values
            for theta3 in _frange(args.theta3_min, args.theta3_max, args.theta3_step)
        ]
        _write_grid_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
        return 0

    result = compute_m02_id3_pip_flex_tendon_delta(args.theta1, args.theta2, args.theta3, geom)
    if args.json:
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0

    print(f"theta1_deg={result.theta1_deg:.3f}")
    print(f"theta2_deg={result.theta2_deg:.3f}")
    print(f"theta3_deg={result.theta3_deg:.3f}")
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
    print(f"length_delta_mm={result.length_delta_mm:.6f}")
    print("note=negative length_delta means tendon shortens; motor_abs should increase after counts conversion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
