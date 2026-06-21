#!/usr/bin/env python3
"""Generate 0.1-degree tendon feedforward lookup tables.

The generated tables are decomposed for MCU use:

- M00/M01/M02/M03: 2D tables over theta1/theta2.
- M04 theta1: 1D table over theta1.
- M04 theta2: 1D table over theta2.
- theta3/theta4 terms are linear and stored as slopes in metadata.

This avoids a full 4D table while preserving 0.1-degree resolution for the
nonlinear parts.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Tuple

import numpy as np

from m00_id1_right_mcp_tendon_model import (
    M00Id1RightMcpGeometry,
    _dist3,
    _to_mcp_obstacle_frame,
    compute_d1,
    compute_m00_id1_right_mcp_tendon_length,
)
from m01_id2_left_mcp_tendon_model import (
    M01Id2LeftMcpGeometry,
    compute_d2,
    compute_m01_id2_left_mcp_tendon_length,
)
from m02_id3_pip_flex_tendon_model import (
    M02Id3PipFlexGeometry,
    _circle_contains as m02_circle_contains,
    _compute_shortest_xy_path as m02_reference_shortest_xy_path,
    _dist2 as m02_dist2,
    _left_rounded_keepout_contains as m02_keepout_contains,
    _sample_circle_boundary as m02_sample_circle_boundary,
    _sample_keepout_boundary as m02_sample_keepout_boundary,
    _segment_clear as m02_segment_clear,
    compute_m02_mcp_routing_d,
)
from m03_id4_dip_flex_tendon_model import (
    M03Id4DipFlexGeometry,
    compute_m03_mcp_routing_d,
)
from m04_id5_return_tendon_model import (
    M04Id5ReturnGeometry,
    _compute_shortest_xy_path as m04_reference_shortest_xy_path,
    _dist2 as m04_dist2,
    _right_rounded_keepout_contains as m04_keepout_contains,
    _sample_right_rounded_keepout_boundary as m04_sample_keepout_boundary,
    _segment_clear as m04_segment_clear,
    _to_m04_theta2_keepout_frame,
    compute_m04_mcp_fe_point,
    compute_theta1_return_delta_mm,
)
from tendon_length_feedforward_model import DEFAULT_MCP_WRAP_MODE


Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class LutConfig:
    theta1_min_deg: float = -20.0
    theta1_max_deg: float = 30.0
    theta2_min_deg: float = 0.0
    theta2_max_deg: float = 90.0
    step_deg: float = 0.1
    obstacle_padding_mm: float = 0.0
    mcp_wrap_mode: str = DEFAULT_MCP_WRAP_MODE
    quantization_scale_mm: float = 1000.0
    invalid_policy: str = "straight"


@dataclass(frozen=True)
class LutMetadata:
    version: int
    description: str
    theta1_min_deg: float
    theta1_max_deg: float
    theta2_min_deg: float
    theta2_max_deg: float
    step_deg: float
    theta1_count: int
    theta2_count: int
    obstacle_padding_mm: float
    mcp_wrap_mode: str
    quantization_scale_mm: float
    invalid_policy: str
    table_units: str
    int16_units: str
    motor_conversion: str
    m00_invalid_count: int
    m01_invalid_count: int
    m02_invalid_count: int
    m03_invalid_count: int
    pip_m02_slope_mm_per_rad: float
    pip_m03_slope_mm_per_rad: float
    dip_m03_slope_mm_per_rad: float
    pip_m04_slope_mm_per_rad: float
    dip_m04_slope_mm_per_rad: float


class FixedStartVisibilitySolver:
    """Fast shortest-path query for a fixed start and many moving goals."""

    def __init__(
        self,
        start: Vec2,
        boundary_nodes: List[Vec2],
        segment_clear: Callable[[Vec2, Vec2], bool],
        contains: Callable[[Vec2], bool],
        boundary_edges: Iterable[Tuple[int, int, float]],
        dist2: Callable[[Vec2, Vec2], float],
    ) -> None:
        self.start = start
        self.boundary_nodes = boundary_nodes
        self.segment_clear = segment_clear
        self.contains = contains
        self.dist2 = dist2
        self.node_count = len(boundary_nodes)
        self.adjacency: List[List[Tuple[int, float]]] = [[] for _ in range(self.node_count)]

        for i in range(self.node_count):
            for j in range(i + 1, self.node_count):
                if segment_clear(boundary_nodes[i], boundary_nodes[j]):
                    self._add_edge(i, j, dist2(boundary_nodes[i], boundary_nodes[j]))

        for i, j, weight in boundary_edges:
            self._add_edge(i, j, weight)

        self.dist_from_start = self._compute_dist_from_start()

    def _add_edge(self, i: int, j: int, weight: float) -> None:
        self.adjacency[i].append((j, weight))
        self.adjacency[j].append((i, weight))

    def _compute_dist_from_start(self) -> List[float]:
        dist = [float("inf")] * self.node_count
        heap: List[Tuple[float, int]] = []
        for i, node in enumerate(self.boundary_nodes):
            if self.segment_clear(self.start, node):
                dist[i] = self.dist2(self.start, node)
                heapq.heappush(heap, (dist[i], i))

        while heap:
            current_dist, i = heapq.heappop(heap)
            if current_dist != dist[i]:
                continue
            for j, w in self.adjacency[i]:
                nd = current_dist + w
                if nd < dist[j]:
                    dist[j] = nd
                    heapq.heappush(heap, (nd, j))
        return dist

    def path_length(self, goal: Vec2) -> float:
        if self.contains(goal):
            raise ValueError("goal is inside keepout")
        if self.segment_clear(self.start, goal):
            return self.dist2(self.start, goal)

        best = float("inf")
        for i, node in enumerate(self.boundary_nodes):
            start_to_node = self.dist_from_start[i]
            if not math.isfinite(start_to_node):
                continue
            if self.segment_clear(node, goal):
                best = min(best, start_to_node + self.dist2(node, goal))
        if not math.isfinite(best):
            raise ValueError("no collision-free path found")
        return best


def _axis_values(min_deg: float, max_deg: float, step_deg: float) -> np.ndarray:
    count = int(round((max_deg - min_deg) / step_deg)) + 1
    return np.round(min_deg + np.arange(count, dtype=np.float64) * step_deg, 10)


def _make_m00_geom(config: LutConfig) -> M00Id1RightMcpGeometry:
    base = M00Id1RightMcpGeometry()
    return M00Id1RightMcpGeometry(cylinder_radius=base.cylinder_radius + config.obstacle_padding_mm)


def _make_m01_geom(config: LutConfig) -> M01Id2LeftMcpGeometry:
    base = M01Id2LeftMcpGeometry()
    return M01Id2LeftMcpGeometry(cylinder_radius=base.cylinder_radius + config.obstacle_padding_mm)


def _make_m02_geom(config: LutConfig) -> M02Id3PipFlexGeometry:
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


def _make_m03_geom(config: LutConfig) -> M03Id4DipFlexGeometry:
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


def _make_m04_geom(config: LutConfig) -> M04Id5ReturnGeometry:
    base = M04Id5ReturnGeometry()
    pad = config.obstacle_padding_mm
    return M04Id5ReturnGeometry(
        keepout_y_min=base.keepout_y_min - pad,
        keepout_y_max=base.keepout_y_max + pad,
        keepout_x_right=base.keepout_x_right + pad,
        keepout_right_corner_radius=base.keepout_right_corner_radius + pad,
    )


def _build_m02_solver(geom: M02Id3PipFlexGeometry) -> FixedStartVisibilitySolver:
    circle_center = (geom.cylinder_x, geom.cylinder_y)
    circle_nodes = m02_sample_circle_boundary(circle_center, geom.cylinder_radius, geom.circle_samples)
    keepout_nodes = m02_sample_keepout_boundary(geom)
    nodes = circle_nodes + keepout_nodes
    circle_count = len(circle_nodes)
    keep_start = circle_count
    keep_count = len(keepout_nodes)

    edges: List[Tuple[int, int, float]] = []
    arc_weight = geom.cylinder_radius * (2.0 * math.pi / circle_count)
    for k in range(circle_count):
        edges.append((k, (k + 1) % circle_count, arc_weight))
    for k in range(keep_count):
        i = keep_start + k
        j = keep_start + ((k + 1) % keep_count)
        edges.append((i, j, m02_dist2(nodes[i], nodes[j])))

    def contains(p: Vec2) -> bool:
        return (
            m02_circle_contains(p, circle_center, geom.cylinder_radius)
            or m02_keepout_contains(p, geom)
        )

    start = (geom.anchor_x, geom.anchor_y)
    return FixedStartVisibilitySolver(
        start=start,
        boundary_nodes=nodes,
        segment_clear=lambda a, b: m02_segment_clear(a, b, geom),
        contains=contains,
        boundary_edges=edges,
        dist2=m02_dist2,
    )


def _build_m02_solver_for_theta1(
    geom: M02Id3PipFlexGeometry | M03Id4DipFlexGeometry,
    theta1: float,
) -> FixedStartVisibilitySolver:
    if not geom.obstacles_follow_theta1:
        return _build_m02_solver(geom)

    start_3d = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    start_path = _to_mcp_obstacle_frame(start_3d, theta1, True)
    circle_center = (geom.cylinder_x, geom.cylinder_y)
    circle_nodes = m02_sample_circle_boundary(circle_center, geom.cylinder_radius, geom.circle_samples)
    keepout_nodes = m02_sample_keepout_boundary(geom)
    nodes = circle_nodes + keepout_nodes
    circle_count = len(circle_nodes)
    keep_start = circle_count
    keep_count = len(keepout_nodes)

    edges: List[Tuple[int, int, float]] = []
    arc_weight = geom.cylinder_radius * (2.0 * math.pi / circle_count)
    for k in range(circle_count):
        edges.append((k, (k + 1) % circle_count, arc_weight))
    for k in range(keep_count):
        i = keep_start + k
        j = keep_start + ((k + 1) % keep_count)
        edges.append((i, j, m02_dist2(nodes[i], nodes[j])))

    def contains(p: Vec2) -> bool:
        return (
            m02_circle_contains(p, circle_center, geom.cylinder_radius)
            or m02_keepout_contains(p, geom)
        )

    return FixedStartVisibilitySolver(
        start=(start_path[0], start_path[1]),
        boundary_nodes=nodes,
        segment_clear=lambda a, b: m02_segment_clear(a, b, geom),
        contains=contains,
        boundary_edges=edges,
        dist2=m02_dist2,
    )


def _build_m04_solver(geom: M04Id5ReturnGeometry) -> FixedStartVisibilitySolver:
    start = (geom.anchor_x, geom.anchor_y)
    zero_goal_3d = compute_m04_mcp_fe_point(0.0, geom)
    zero_goal = (zero_goal_3d[0], zero_goal_3d[1])
    nodes = m04_sample_keepout_boundary(geom, start, zero_goal)

    edges: List[Tuple[int, int, float]] = []
    for k in range(len(nodes) - 1):
        edges.append((k, k + 1, m04_dist2(nodes[k], nodes[k + 1])))

    return FixedStartVisibilitySolver(
        start=start,
        boundary_nodes=nodes,
        segment_clear=lambda a, b: m04_segment_clear(a, b, geom),
        contains=lambda p: m04_keepout_contains(p, geom),
        boundary_edges=edges,
        dist2=m04_dist2,
    )


def _build_m04_solver_for_theta2(
    geom: M04Id5ReturnGeometry,
    theta2: float,
) -> FixedStartVisibilitySolver:
    if not geom.mcp_fe_keepout_follows_theta2:
        return _build_m04_solver(geom)

    start_3d = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    start_path = _to_m04_theta2_keepout_frame(start_3d, theta2, geom)
    zero_goal_3d = compute_m04_mcp_fe_point(0.0, geom)
    zero_goal_path = _to_m04_theta2_keepout_frame(zero_goal_3d, theta2, geom)
    nodes = m04_sample_keepout_boundary(
        geom,
        (start_path[0], start_path[1]),
        (zero_goal_path[0], zero_goal_path[1]),
    )

    edges: List[Tuple[int, int, float]] = []
    for k in range(len(nodes) - 1):
        edges.append((k, k + 1, m04_dist2(nodes[k], nodes[k + 1])))

    return FixedStartVisibilitySolver(
        start=(start_path[0], start_path[1]),
        boundary_nodes=nodes,
        segment_clear=lambda a, b: m04_segment_clear(a, b, geom),
        contains=lambda p: m04_keepout_contains(p, geom),
        boundary_edges=edges,
        dist2=m04_dist2,
    )


def _m02_mcp_routing_delta_fast(theta1: float, theta2: float, geom: M02Id3PipFlexGeometry, solver: FixedStartVisibilitySolver, zero_len: float) -> float:
    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    anchor_path = _to_mcp_obstacle_frame(anchor, theta1, geom.obstacles_follow_theta1)
    d = compute_m02_mcp_routing_d(theta1, theta2, geom)
    d_path = _to_mcp_obstacle_frame(d, theta1, geom.obstacles_follow_theta1)
    xy = solver.path_length((d_path[0], d_path[1]))
    dz = d_path[2] - anchor_path[2]
    return math.sqrt(xy * xy + dz * dz) - zero_len


def _m02_mcp_routing_straight_delta(theta1: float, theta2: float, geom: M02Id3PipFlexGeometry, zero_len: float) -> float:
    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    d = compute_m02_mcp_routing_d(theta1, theta2, geom)
    return _dist3(anchor, d) - zero_len


def _m02_mcp_routing_delta_safe(
    theta1: float,
    theta2: float,
    geom: M02Id3PipFlexGeometry,
    solver: FixedStartVisibilitySolver,
    zero_len: float,
    invalid_policy: str,
) -> Tuple[float, bool]:
    try:
        return _m02_mcp_routing_delta_fast(theta1, theta2, geom, solver, zero_len), False
    except ValueError:
        if invalid_policy == "raise":
            raise
        if invalid_policy == "straight":
            return _m02_mcp_routing_straight_delta(theta1, theta2, geom, zero_len), True
        if invalid_policy == "nan":
            return float("nan"), True
        raise ValueError(f"unknown invalid_policy: {invalid_policy}")


def _m03_mcp_routing_raw_length_fast(theta1: float, theta2: float, geom: M03Id4DipFlexGeometry, solver: FixedStartVisibilitySolver) -> float:
    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    anchor_path = _to_mcp_obstacle_frame(anchor, theta1, geom.obstacles_follow_theta1)
    d = compute_m03_mcp_routing_d(theta1, theta2, geom)
    d_path = _to_mcp_obstacle_frame(d, theta1, geom.obstacles_follow_theta1)
    xy = solver.path_length((d_path[0], d_path[1]))
    dz = d_path[2] - anchor_path[2]
    return math.sqrt(xy * xy + dz * dz)


def _m03_mcp_routing_raw_straight_length(theta1: float, theta2: float, geom: M03Id4DipFlexGeometry) -> float:
    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    d = compute_m03_mcp_routing_d(theta1, theta2, geom)
    return _dist3(anchor, d)


def _m03_mcp_routing_raw_length_safe(
    theta1: float,
    theta2: float,
    geom: M03Id4DipFlexGeometry,
    solver: FixedStartVisibilitySolver,
    invalid_policy: str,
) -> Tuple[float, bool]:
    try:
        return _m03_mcp_routing_raw_length_fast(theta1, theta2, geom, solver), False
    except ValueError:
        if invalid_policy == "raise":
            raise
        if invalid_policy == "straight":
            return _m03_mcp_routing_raw_straight_length(theta1, theta2, geom), True
        if invalid_policy == "nan":
            return float("nan"), True
        raise ValueError(f"unknown invalid_policy: {invalid_policy}")


def _m03_mcp_routing_delta_fast(
    theta1: float,
    theta2: float,
    geom: M03Id4DipFlexGeometry,
    solver: FixedStartVisibilitySolver,
    zero_len: float,
    theta1_zero_len: float,
) -> float:
    raw_len = _m03_mcp_routing_raw_length_fast(theta1, theta2, geom, solver)
    theta1_component = theta1_zero_len - zero_len
    theta2_component = raw_len - theta1_zero_len
    return theta1_component - theta2_component


def _m03_mcp_routing_straight_delta(
    theta1: float,
    theta2: float,
    geom: M03Id4DipFlexGeometry,
    zero_len: float,
    theta1_zero_len: float,
) -> float:
    raw_len = _m03_mcp_routing_raw_straight_length(theta1, theta2, geom)
    theta1_component = theta1_zero_len - zero_len
    theta2_component = raw_len - theta1_zero_len
    return theta1_component - theta2_component


def _m03_mcp_routing_delta_safe(
    theta1: float,
    theta2: float,
    geom: M03Id4DipFlexGeometry,
    solver: FixedStartVisibilitySolver,
    zero_len: float,
    theta1_zero_len: float,
    invalid_policy: str,
) -> Tuple[float, bool]:
    try:
        return _m03_mcp_routing_delta_fast(theta1, theta2, geom, solver, zero_len, theta1_zero_len), False
    except ValueError:
        if invalid_policy == "raise":
            raise
        if invalid_policy == "straight":
            return _m03_mcp_routing_straight_delta(theta1, theta2, geom, zero_len, theta1_zero_len), True
        if invalid_policy == "nan":
            return float("nan"), True
        raise ValueError(f"unknown invalid_policy: {invalid_policy}")


def _m04_theta2_delta_fast(theta2: float, geom: M04Id5ReturnGeometry, solver: FixedStartVisibilitySolver, zero_len: float) -> float:
    d = compute_m04_mcp_fe_point(theta2, geom)
    anchor = (geom.anchor_x, geom.anchor_y, geom.anchor_z)
    anchor_path = _to_m04_theta2_keepout_frame(anchor, theta2, geom)
    d_path = _to_m04_theta2_keepout_frame(d, theta2, geom)
    xy = solver.path_length((d_path[0], d_path[1]))
    dz = d_path[2] - anchor_path[2]
    return zero_len - math.sqrt(xy * xy + dz * dz)


def _m00_straight_length(theta1: float, theta2: float, geom: M00Id1RightMcpGeometry) -> float:
    anchor = (geom.a1_x, geom.a1_y, geom.a1_z)
    return _dist3(anchor, compute_d1(theta1, theta2, geom))


def _m01_straight_length(theta1: float, theta2: float, geom: M01Id2LeftMcpGeometry) -> float:
    anchor = (geom.a2_x, geom.a2_y, geom.a2_z)
    return _dist3(anchor, compute_d2(theta1, theta2, geom))


def _m00_delta_safe(
    theta1: float,
    theta2: float,
    geom: M00Id1RightMcpGeometry,
    zero_len: float,
    wrap_mode: str,
    invalid_policy: str,
) -> Tuple[float, bool]:
    try:
        return compute_m00_id1_right_mcp_tendon_length(theta1, theta2, geom, wrap_mode).length_mm - zero_len, False
    except ValueError:
        if invalid_policy == "raise":
            raise
        if invalid_policy == "straight":
            return _m00_straight_length(theta1, theta2, geom) - zero_len, True
        if invalid_policy == "nan":
            return float("nan"), True
        raise ValueError(f"unknown invalid_policy: {invalid_policy}")


def _m01_delta_safe(
    theta1: float,
    theta2: float,
    geom: M01Id2LeftMcpGeometry,
    zero_len: float,
    wrap_mode: str,
    invalid_policy: str,
) -> Tuple[float, bool]:
    try:
        return compute_m01_id2_left_mcp_tendon_length(theta1, theta2, geom, wrap_mode).length_mm - zero_len, False
    except ValueError:
        if invalid_policy == "raise":
            raise
        if invalid_policy == "straight":
            return _m01_straight_length(theta1, theta2, geom) - zero_len, True
        if invalid_policy == "nan":
            return float("nan"), True
        raise ValueError(f"unknown invalid_policy: {invalid_policy}")


def _progress(label: str, index: int, total: int, start_time: float) -> None:
    if index == 0 or index == total or index % max(1, total // 20) == 0:
        elapsed = time.perf_counter() - start_time
        rate = index / elapsed if elapsed > 0.0 and index > 0 else 0.0
        remaining = (total - index) / rate if rate > 0.0 else 0.0
        print(f"{label}: {index}/{total} elapsed={elapsed:.1f}s eta={remaining:.1f}s")


def _compute_tables(config: LutConfig) -> Tuple[np.ndarray, dict]:
    theta1_values = _axis_values(config.theta1_min_deg, config.theta1_max_deg, config.step_deg)
    theta2_values = _axis_values(config.theta2_min_deg, config.theta2_max_deg, config.step_deg)
    t1_count = len(theta1_values)
    t2_count = len(theta2_values)

    m00 = np.empty((t1_count, t2_count), dtype=np.float32)
    m01 = np.empty((t1_count, t2_count), dtype=np.float32)
    m02 = np.empty((t1_count, t2_count), dtype=np.float32)
    m03 = np.empty((t1_count, t2_count), dtype=np.float32)
    m00_invalid = np.zeros((t1_count, t2_count), dtype=np.uint8)
    m01_invalid = np.zeros((t1_count, t2_count), dtype=np.uint8)
    m02_invalid = np.zeros((t1_count, t2_count), dtype=np.uint8)
    m03_invalid = np.zeros((t1_count, t2_count), dtype=np.uint8)
    m04_theta1 = np.empty((t1_count,), dtype=np.float32)
    m04_theta2 = np.empty((t2_count,), dtype=np.float32)

    m00_geom = _make_m00_geom(config)
    m01_geom = _make_m01_geom(config)
    m02_geom = _make_m02_geom(config)
    m03_geom = _make_m03_geom(config)
    m04_geom = _make_m04_geom(config)

    m00_zero = compute_m00_id1_right_mcp_tendon_length(0.0, 0.0, m00_geom, config.mcp_wrap_mode).length_mm
    m01_zero = compute_m01_id2_left_mcp_tendon_length(0.0, 0.0, m01_geom, config.mcp_wrap_mode).length_mm

    print("building optimized M02/M03 visibility solvers")
    m02_solver = _build_m02_solver_for_theta1(m02_geom, 0.0)
    m03_solver = _build_m02_solver_for_theta1(m03_geom, 0.0)
    m02_zero_d = compute_m02_mcp_routing_d(0.0, 0.0, m02_geom)
    m03_zero_d = compute_m03_mcp_routing_d(0.0, 0.0, m03_geom)
    m02_zero_anchor = (m02_geom.anchor_x, m02_geom.anchor_y, m02_geom.anchor_z)
    m03_zero_anchor = (m03_geom.anchor_x, m03_geom.anchor_y, m03_geom.anchor_z)
    m02_zero_anchor_path = _to_mcp_obstacle_frame(m02_zero_anchor, 0.0, m02_geom.obstacles_follow_theta1)
    m03_zero_anchor_path = _to_mcp_obstacle_frame(m03_zero_anchor, 0.0, m03_geom.obstacles_follow_theta1)
    m02_zero_d_path = _to_mcp_obstacle_frame(m02_zero_d, 0.0, m02_geom.obstacles_follow_theta1)
    m03_zero_d_path = _to_mcp_obstacle_frame(m03_zero_d, 0.0, m03_geom.obstacles_follow_theta1)
    m02_zero_xy = m02_solver.path_length((m02_zero_d_path[0], m02_zero_d_path[1]))
    m03_zero_xy = m03_solver.path_length((m03_zero_d_path[0], m03_zero_d_path[1]))
    m02_zero_len = math.sqrt(m02_zero_xy * m02_zero_xy + (m02_zero_d_path[2] - m02_zero_anchor_path[2]) ** 2)
    m03_zero_len = math.sqrt(m03_zero_xy * m03_zero_xy + (m03_zero_d_path[2] - m03_zero_anchor_path[2]) ** 2)

    print("building optimized M04 theta2 visibility solver")
    m04_solver = _build_m04_solver_for_theta2(m04_geom, 0.0)
    m04_zero_d = compute_m04_mcp_fe_point(0.0, m04_geom)
    m04_zero_anchor = (m04_geom.anchor_x, m04_geom.anchor_y, m04_geom.anchor_z)
    m04_zero_anchor_path = _to_m04_theta2_keepout_frame(m04_zero_anchor, 0.0, m04_geom)
    m04_zero_d_path = _to_m04_theta2_keepout_frame(m04_zero_d, 0.0, m04_geom)
    m04_zero_xy = m04_solver.path_length((m04_zero_d_path[0], m04_zero_d_path[1]))
    m04_zero_len = math.sqrt(m04_zero_xy * m04_zero_xy + (m04_zero_d_path[2] - m04_zero_anchor_path[2]) ** 2)

    print("computing M00/M01/M02/M03 2D tables")
    start_time = time.perf_counter()
    total = t1_count * t2_count
    done = 0
    for i, theta1 in enumerate(theta1_values):
        if m02_geom.obstacles_follow_theta1:
            m02_solver = _build_m02_solver_for_theta1(m02_geom, float(theta1))
        if m03_geom.obstacles_follow_theta1:
            m03_solver = _build_m02_solver_for_theta1(m03_geom, float(theta1))
        m03_theta1_zero_len, m03_theta1_zero_invalid = _m03_mcp_routing_raw_length_safe(
            float(theta1),
            0.0,
            m03_geom,
            m03_solver,
            config.invalid_policy,
        )
        for j, theta2 in enumerate(theta2_values):
            m00_value, m00_was_invalid = _m00_delta_safe(
                float(theta1), float(theta2), m00_geom, m00_zero, config.mcp_wrap_mode, config.invalid_policy
            )
            m01_value, m01_was_invalid = _m01_delta_safe(
                float(theta1), float(theta2), m01_geom, m01_zero, config.mcp_wrap_mode, config.invalid_policy
            )
            m00[i, j] = m00_value
            m01[i, j] = m01_value
            m00_invalid[i, j] = 1 if m00_was_invalid else 0
            m01_invalid[i, j] = 1 if m01_was_invalid else 0
            m02_value, m02_was_invalid = _m02_mcp_routing_delta_safe(
                float(theta1), float(theta2), m02_geom, m02_solver, m02_zero_len, config.invalid_policy
            )
            m03_value, m03_was_invalid = _m03_mcp_routing_delta_safe(
                float(theta1), float(theta2), m03_geom, m03_solver, m03_zero_len, m03_theta1_zero_len, config.invalid_policy
            )
            m02[i, j] = m02_value
            m03[i, j] = m03_value
            m02_invalid[i, j] = 1 if m02_was_invalid else 0
            m03_invalid[i, j] = 1 if (m03_theta1_zero_invalid or m03_was_invalid) else 0
            done += 1
        _progress("2D tables", done, total, start_time)

    print("computing M04 1D tables")
    for i, theta1 in enumerate(theta1_values):
        m04_theta1[i] = compute_theta1_return_delta_mm(float(theta1), m04_geom)[0]
    for j, theta2 in enumerate(theta2_values):
        if m04_geom.mcp_fe_keepout_follows_theta2:
            m04_solver = _build_m04_solver_for_theta2(m04_geom, float(theta2))
        m04_theta2[j] = _m04_theta2_delta_fast(float(theta2), m04_geom, m04_solver, m04_zero_len)

    arrays = {
        "theta1_deg": theta1_values.astype(np.float32),
        "theta2_deg": theta2_values.astype(np.float32),
        "m00_id1_mcp_delta_mm": m00,
        "m01_id2_mcp_delta_mm": m01,
        "m02_id3_mcp_routing_delta_mm": m02,
        "m03_id4_mcp_routing_delta_mm": m03,
        "m00_id1_invalid_mask": m00_invalid,
        "m01_id2_invalid_mask": m01_invalid,
        "m02_id3_invalid_mask": m02_invalid,
        "m03_id4_invalid_mask": m03_invalid,
        "m04_theta1_return_delta_mm": m04_theta1,
        "m04_theta2_return_delta_mm": m04_theta2,
    }
    return arrays, {
        "m02_zero_len": m02_zero_len,
        "m03_zero_len": m03_zero_len,
        "m04_theta2_zero_len": m04_zero_len,
    }


def _quantize(arr: np.ndarray, scale: float) -> np.ndarray:
    arr = np.nan_to_num(arr, nan=0.0)
    quantized = np.rint(arr.astype(np.float64) * scale)
    if np.nanmin(quantized) < -32768 or np.nanmax(quantized) > 32767:
        raise ValueError("quantized table does not fit int16")
    return quantized.astype(np.int16)


def _write_csv_2d(path: Path, theta1_values: np.ndarray, theta2_values: np.ndarray, tables: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "theta1_deg",
            "theta2_deg",
            "m00_id1_mcp_delta_mm",
            "m01_id2_mcp_delta_mm",
            "m02_id3_mcp_routing_delta_mm",
            "m03_id4_mcp_routing_delta_mm",
            "m00_id1_invalid_mask",
            "m01_id2_invalid_mask",
            "m02_id3_invalid_mask",
            "m03_id4_invalid_mask",
        ])
        for i, theta1 in enumerate(theta1_values):
            for j, theta2 in enumerate(theta2_values):
                writer.writerow([
                    f"{float(theta1):.1f}",
                    f"{float(theta2):.1f}",
                    f"{float(tables['m00_id1_mcp_delta_mm'][i, j]):.6f}",
                    f"{float(tables['m01_id2_mcp_delta_mm'][i, j]):.6f}",
                    f"{float(tables['m02_id3_mcp_routing_delta_mm'][i, j]):.6f}",
                    f"{float(tables['m03_id4_mcp_routing_delta_mm'][i, j]):.6f}",
                    int(tables["m00_id1_invalid_mask"][i, j]),
                    int(tables["m01_id2_invalid_mask"][i, j]),
                    int(tables["m02_id3_invalid_mask"][i, j]),
                    int(tables["m03_id4_invalid_mask"][i, j]),
                ])


def _write_csv_1d(path: Path, theta1_values: np.ndarray, theta2_values: np.ndarray, tables: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    max_len = max(len(theta1_values), len(theta2_values))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "theta1_deg",
            "m04_theta1_return_delta_mm",
            "theta2_deg",
            "m04_theta2_return_delta_mm",
        ])
        for i in range(max_len):
            writer.writerow([
                f"{float(theta1_values[i]):.1f}" if i < len(theta1_values) else "",
                f"{float(tables['m04_theta1_return_delta_mm'][i]):.6f}" if i < len(theta1_values) else "",
                f"{float(theta2_values[i]):.1f}" if i < len(theta2_values) else "",
                f"{float(tables['m04_theta2_return_delta_mm'][i]):.6f}" if i < len(theta2_values) else "",
            ])


def _write_outputs(output_dir: Path, config: LutConfig, arrays: dict, diagnostics: dict, write_csv: bool) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    step_tag = f"{config.step_deg:g}".replace(".", "p")
    metadata = LutMetadata(
        version=1,
        description="0.1-degree decomposed tendon feedforward LUT generated from offline geometry model.",
        theta1_min_deg=config.theta1_min_deg,
        theta1_max_deg=config.theta1_max_deg,
        theta2_min_deg=config.theta2_min_deg,
        theta2_max_deg=config.theta2_max_deg,
        step_deg=config.step_deg,
        theta1_count=int(len(arrays["theta1_deg"])),
        theta2_count=int(len(arrays["theta2_deg"])),
        obstacle_padding_mm=config.obstacle_padding_mm,
        mcp_wrap_mode=config.mcp_wrap_mode,
        quantization_scale_mm=config.quantization_scale_mm,
        invalid_policy=config.invalid_policy,
        table_units="float32 mm in .npz",
        int16_units="round(mm * quantization_scale_mm)",
        motor_conversion="motor_abs_delta_counts = -length_delta_mm * counts_per_mm",
        m00_invalid_count=int(np.sum(arrays["m00_id1_invalid_mask"])),
        m01_invalid_count=int(np.sum(arrays["m01_id2_invalid_mask"])),
        m02_invalid_count=int(np.sum(arrays["m02_id3_invalid_mask"])),
        m03_invalid_count=int(np.sum(arrays["m03_id4_invalid_mask"])),
        pip_m02_slope_mm_per_rad=M02Id3PipFlexGeometry().pip_flex_length_slope_mm_per_rad,
        pip_m03_slope_mm_per_rad=M03Id4DipFlexGeometry().pip_flex_length_slope_mm_per_rad,
        dip_m03_slope_mm_per_rad=M03Id4DipFlexGeometry().dip_flex_length_slope_mm_per_rad,
        pip_m04_slope_mm_per_rad=M04Id5ReturnGeometry().pip_return_slope_mm_per_rad,
        dip_m04_slope_mm_per_rad=M04Id5ReturnGeometry().dip_return_slope_mm_per_rad,
    )
    metadata_dict = asdict(metadata)
    metadata_dict["diagnostics"] = diagnostics

    float_path = output_dir / f"tendon_lut_{step_tag}deg_float32.npz"
    np.savez_compressed(float_path, **arrays)

    int_arrays = {
        key: value
        for key, value in arrays.items()
        if key in ("theta1_deg", "theta2_deg")
    }
    for key, value in arrays.items():
        if key not in int_arrays:
            if key.endswith("_mask"):
                int_arrays[key] = value.astype(np.uint8)
            else:
                int_arrays[key.replace("_mm", "_i16")] = _quantize(value, config.quantization_scale_mm)
    int_path = output_dir / f"tendon_lut_{step_tag}deg_int16.npz"
    np.savez_compressed(int_path, **int_arrays)

    metadata_path = output_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata_dict, ensure_ascii=False, indent=2), encoding="utf-8")

    if write_csv:
        _write_csv_2d(output_dir / f"tendon_lut_{step_tag}deg_2d_tables.csv", arrays["theta1_deg"], arrays["theta2_deg"], arrays)
        _write_csv_1d(output_dir / f"tendon_lut_{step_tag}deg_1d_tables.csv", arrays["theta1_deg"], arrays["theta2_deg"], arrays)

    print(f"wrote {float_path}")
    print(f"wrote {int_path}")
    print(f"wrote {metadata_path}")
    if write_csv:
        print(f"wrote {output_dir / f'tendon_lut_{step_tag}deg_2d_tables.csv'}")
        print(f"wrote {output_dir / f'tendon_lut_{step_tag}deg_1d_tables.csv'}")


def _self_check() -> None:
    config = LutConfig(theta1_min_deg=-1.0, theta1_max_deg=1.0, theta2_min_deg=0.0, theta2_max_deg=2.0, step_deg=0.5)
    arrays, diagnostics = _compute_tables(config)
    output_dir = Path("run_data/tendon_lut_self_check")
    _write_outputs(output_dir, config, arrays, diagnostics, write_csv=True)
    print("self-check complete")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate decomposed 0.1-degree tendon feedforward LUTs.")
    parser.add_argument("--theta1-min", type=float, default=-20.0)
    parser.add_argument("--theta1-max", type=float, default=30.0)
    parser.add_argument("--theta2-min", type=float, default=0.0)
    parser.add_argument("--theta2-max", type=float, default=90.0)
    parser.add_argument("--step", type=float, default=0.1)
    parser.add_argument("--obstacle-padding", type=float, default=0.0)
    parser.add_argument(
        "--mcp-wrap-mode",
        choices=("shortest", "cw", "ccw", "direct-if-clear-cw", "direct-if-clear-ccw"),
        default=DEFAULT_MCP_WRAP_MODE,
    )
    parser.add_argument("--quantization-scale-mm", type=float, default=1000.0)
    parser.add_argument(
        "--invalid-policy",
        choices=("straight", "nan", "raise"),
        default="straight",
        help="How to fill M00/M01 points where the infinite-cylinder model is geometrically invalid.",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("tools/tendon_forward_model/generated_lut_0p1deg"))
    parser.add_argument("--csv", action="store_true", help="Also write readable CSV files.")
    parser.add_argument("--self-check", action="store_true", help="Generate a tiny table to verify the pipeline.")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.self_check:
        _self_check()
        return 0

    config = LutConfig(
        theta1_min_deg=args.theta1_min,
        theta1_max_deg=args.theta1_max,
        theta2_min_deg=args.theta2_min,
        theta2_max_deg=args.theta2_max,
        step_deg=args.step,
        obstacle_padding_mm=args.obstacle_padding,
        mcp_wrap_mode=args.mcp_wrap_mode,
        quantization_scale_mm=args.quantization_scale_mm,
        invalid_policy=args.invalid_policy,
    )
    arrays, diagnostics = _compute_tables(config)
    _write_outputs(args.out_dir, config, arrays, diagnostics, write_csv=args.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
