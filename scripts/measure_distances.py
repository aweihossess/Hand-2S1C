#!/usr/bin/env python3
"""Measure three point-to-point 3D distances.

Coordinate system:
  - Origin: O = (0, 0, 0)
  - Right-handed axes: +X right, +Y forward, +Z up
  - Unit: user-defined, but all points must use the same unit

The script can either read one JSON file once or watch it repeatedly.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_PAIRS = [
    ("P1", "Q1"),
    ("P2", "Q2"),
    ("P3", "Q3"),
]


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float

    @staticmethod
    def from_json(value: Any, name: str) -> "Point3":
        if isinstance(value, dict):
            try:
                return Point3(float(value["x"]), float(value["y"]), float(value["z"]))
            except KeyError as exc:
                raise ValueError(f"{name} missing key {exc.args[0]!r}") from exc
        if isinstance(value, list) and len(value) == 3:
            return Point3(float(value[0]), float(value[1]), float(value[2]))
        raise ValueError(f"{name} must be [x, y, z] or {{\"x\":..., \"y\":..., \"z\":...}}")

    def distance_to(self, other: "Point3") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def load_points(path: Path) -> tuple[str, dict[str, Point3], list[tuple[str, str]]]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    unit = str(data.get("unit", "unit"))
    raw_points = data.get("points")
    if not isinstance(raw_points, dict):
        raise ValueError("JSON must contain a 'points' object")

    points = {
        name: Point3.from_json(value, f"points.{name}")
        for name, value in raw_points.items()
    }

    raw_pairs = data.get("pairs", DEFAULT_PAIRS)
    pairs: list[tuple[str, str]] = []
    for idx, pair in enumerate(raw_pairs):
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError(f"pairs[{idx}] must be [fromPointName, toPointName]")
        a, b = str(pair[0]), str(pair[1])
        if a not in points:
            raise ValueError(f"pairs[{idx}] references unknown point {a!r}")
        if b not in points:
            raise ValueError(f"pairs[{idx}] references unknown point {b!r}")
        pairs.append((a, b))

    return unit, points, pairs


def format_report(path: Path, unit: str, points: dict[str, Point3], pairs: list[tuple[str, str]]) -> str:
    lines = [
        "3D Distance Measurement",
        f"source: {path}",
        "coordinate: right-handed, O=(0,0,0), +X right, +Y forward, +Z up",
        "",
    ]

    width = max(len(a) + len(b) + 3 for a, b in pairs)
    for a, b in pairs:
        distance = points[a].distance_to(points[b])
        label = f"{a}->{b}"
        lines.append(f"{label:<{width}}  {distance:12.6f} {unit}")

    return "\n".join(lines)


def clear_screen() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def run_once(path: Path) -> int:
    unit, points, pairs = load_points(path)
    print(format_report(path, unit, points, pairs))
    return 0


def run_watch(path: Path, interval: float, no_clear: bool) -> int:
    while True:
        try:
            unit, points, pairs = load_points(path)
            if not no_clear:
                clear_screen()
            print(format_report(path, unit, points, pairs))
            print("")
            print(f"watching every {interval:.2f}s, press Ctrl+C to stop")
        except Exception as exc:
            if not no_clear:
                clear_screen()
            print(f"Error: {exc}", file=sys.stderr)
            print(f"Fix {path} and the watcher will retry.", file=sys.stderr)

        try:
            time.sleep(interval)
        except KeyboardInterrupt:
            print("\nstopped")
            return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure three 3D point-to-point distances.")
    parser.add_argument(
        "file",
        nargs="?",
        default="scripts/distance_points.json",
        help="JSON file containing points and pairs. Default: scripts/distance_points.json",
    )
    parser.add_argument("--watch", action="store_true", help="Refresh repeatedly for real-time measurement.")
    parser.add_argument("--interval", type=float, default=0.2, help="Watch refresh interval in seconds.")
    parser.add_argument("--no-clear", action="store_true", help="Do not clear the terminal in watch mode.")
    args = parser.parse_args()

    path = Path(args.file)
    if not path.exists():
        print(f"Missing input file: {path}", file=sys.stderr)
        print("Copy scripts/distance_points.example.json to scripts/distance_points.json first.", file=sys.stderr)
        return 2

    if args.watch:
        return run_watch(path, max(args.interval, 0.02), args.no_clear)
    return run_once(path)


if __name__ == "__main__":
    raise SystemExit(main())
