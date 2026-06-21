#!/usr/bin/env python3
"""Export decomposed tendon feedforward LUTs to ESP32 firmware C++ files.

The generated C++ module is intentionally read-only and standalone. It gives
firmware a small lookup API, but it does not connect the model to the control
loop by itself.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

import numpy as np


DEFAULT_LUT = Path("tools/tendon_forward_model/generated_lut_0p2deg/tendon_lut_0p2deg_int16.npz")
DEFAULT_METADATA = Path("tools/tendon_forward_model/generated_lut_0p2deg/metadata.json")
DEFAULT_OUT_DIR = Path("ESP32-P4/ServoBoardMain/src/control")


TABLE_KEYS = {
    "m00": "m00_id1_mcp_delta_i16",
    "m01": "m01_id2_mcp_delta_i16",
    "m02": "m02_id3_mcp_routing_delta_i16",
    "m03": "m03_id4_mcp_routing_delta_i16",
    "m04_theta1": "m04_theta1_return_delta_i16",
    "m04_theta2": "m04_theta2_return_delta_i16",
}


def _as_float_literal(value: float) -> str:
    text = f"{float(value):.9g}"
    if "e" not in text and "." not in text:
        text += ".0"
    return f"{text}f"


def _format_i16_array(name: str, array: np.ndarray, values_per_line: int = 16) -> str:
    flat = array.astype(np.int16, copy=False).ravel()
    lines = [f"static const int16_t {name}[{flat.size}] = {{"]
    for start in range(0, flat.size, values_per_line):
        chunk = flat[start : start + values_per_line]
        suffix = "," if start + values_per_line < flat.size else ""
        lines.append("    " + ", ".join(str(int(v)) for v in chunk) + suffix)
    lines.append("};")
    return "\n".join(lines)


def _require_keys(files: Iterable[str], required: Iterable[str]) -> None:
    available = set(files)
    missing = [key for key in required if key not in available]
    if missing:
        raise KeyError(f"LUT file is missing required arrays: {', '.join(missing)}")


def _load_metadata(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _write_header(path: Path, metadata: dict) -> None:
    theta1_min = metadata.get("theta1_min_deg", -20.0)
    theta1_max = metadata.get("theta1_max_deg", 30.0)
    theta2_min = metadata.get("theta2_min_deg", 0.0)
    theta2_max = metadata.get("theta2_max_deg", 90.0)
    step = metadata.get("step_deg", 0.2)
    theta1_count = metadata.get("theta1_count", 251)
    theta2_count = metadata.get("theta2_count", 451)
    scale = metadata.get("quantization_scale_mm", 1000.0)
    m00_invalid_count = metadata.get("m00_invalid_count", 0)
    m01_invalid_count = metadata.get("m01_invalid_count", 0)
    m02_invalid_count = metadata.get("m02_invalid_count", 0)
    m03_invalid_count = metadata.get("m03_invalid_count", 0)

    path.write_text(
        f"""#ifndef TENDON_FEEDFORWARD_LUT_H
#define TENDON_FEEDFORWARD_LUT_H

#include <stdint.h>

namespace TendonFeedforwardLut {{

// Generated from tools/tendon_forward_model/generated_lut_0p2deg.
// The table stores physical tendon length deltas relative to the zero pose.
// Sign convention:
//   lengthDeltaMm > 0 means the tendon lengthens.
//   lengthDeltaMm < 0 means the tendon shortens.
// Hardware conversion is normally:
//   motor_abs_delta_counts = -lengthDeltaMm * counts_per_mm
// because motor_abs increasing tightens/shortens the tendon.
static const uint8_t kTendonCount = 5;
static const float kTheta1MinDeg = {_as_float_literal(theta1_min)};
static const float kTheta1MaxDeg = {_as_float_literal(theta1_max)};
static const float kTheta2MinDeg = {_as_float_literal(theta2_min)};
static const float kTheta2MaxDeg = {_as_float_literal(theta2_max)};
static const float kStepDeg = {_as_float_literal(step)};
static const uint16_t kTheta1Count = {int(theta1_count)};
static const uint16_t kTheta2Count = {int(theta2_count)};
static const float kQuantizationScaleMm = {_as_float_literal(scale)};

// Offline geometry warnings. These entries were filled with straight-line
// fallback during table generation because the current keepout model marked
// those poses as geometrically invalid.
static const uint32_t kM00InvalidCount = {int(m00_invalid_count)};
static const uint32_t kM01InvalidCount = {int(m01_invalid_count)};
static const uint32_t kM02InvalidCount = {int(m02_invalid_count)};
static const uint32_t kM03InvalidCount = {int(m03_invalid_count)};

enum TendonIndex : uint8_t {{
    kM00Id1 = 0,
    kM01Id2 = 1,
    kM02Id3 = 2,
    kM03Id4 = 3,
    kM04Id5 = 4,
}};

struct LengthDeltaResult {{
    float lengthDeltaMm[kTendonCount];
    bool inputClamped;
}};

bool computeLengthDeltaMm(float theta1Deg,
                          float theta2Deg,
                          float theta3Deg,
                          float theta4Deg,
                          float outLengthDeltaMm[kTendonCount],
                          bool* inputClamped = nullptr);

LengthDeltaResult computeLengthDeltaMm(float theta1Deg,
                                       float theta2Deg,
                                       float theta3Deg,
                                       float theta4Deg);

float lookupM00Id1DeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped = nullptr);
float lookupM01Id2DeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped = nullptr);
float lookupM02McpRoutingDeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped = nullptr);
float lookupM03McpRoutingDeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped = nullptr);
float lookupM04Theta1DeltaMm(float theta1Deg, bool* inputClamped = nullptr);
float lookupM04Theta2DeltaMm(float theta2Deg, bool* inputClamped = nullptr);

}} // namespace TendonFeedforwardLut

#endif // TENDON_FEEDFORWARD_LUT_H
""",
        encoding="utf-8",
    )


def _write_source(path: Path, header_name: str, arrays: dict, metadata: dict) -> None:
    theta1_count = int(metadata.get("theta1_count", arrays[TABLE_KEYS["m00"]].shape[0]))
    theta2_count = int(metadata.get("theta2_count", arrays[TABLE_KEYS["m00"]].shape[1]))
    scale = float(metadata.get("quantization_scale_mm", 1000.0))
    step = float(metadata.get("step_deg", 0.2))
    theta1_min = float(metadata.get("theta1_min_deg", -20.0))
    theta2_min = float(metadata.get("theta2_min_deg", 0.0))
    m02_slope = float(metadata.get("pip_m02_slope_mm_per_rad", -6.0))
    m03_pip_slope = float(metadata.get("pip_m03_slope_mm_per_rad", -3.5))
    m03_dip_slope = float(metadata.get("dip_m03_slope_mm_per_rad", -4.8))
    m04_pip_slope = float(metadata.get("pip_m04_slope_mm_per_rad", -9.9))
    m04_dip_slope = float(metadata.get("dip_m04_slope_mm_per_rad", -6.45))

    source = f"""#include \"{header_name}\"

#include <math.h>

namespace TendonFeedforwardLut {{
namespace {{

static const float kDegToRad = 0.017453292519943295f;
static const float kTheta1Min = {_as_float_literal(theta1_min)};
static const float kTheta2Min = {_as_float_literal(theta2_min)};
static const float kAxisStep = {_as_float_literal(step)};
static const float kInvScale = {_as_float_literal(1.0 / scale)};
static const float kM02PipSlopeMmPerRad = {_as_float_literal(m02_slope)};
static const float kM03PipSlopeMmPerRad = {_as_float_literal(m03_pip_slope)};
static const float kM03DipSlopeMmPerRad = {_as_float_literal(m03_dip_slope)};
static const float kM04PipSlopeMmPerRad = {_as_float_literal(m04_pip_slope)};
static const float kM04DipSlopeMmPerRad = {_as_float_literal(m04_dip_slope)};

// clang-format off
{_format_i16_array("kM00Id1McpDeltaI16", arrays[TABLE_KEYS["m00"]])}

{_format_i16_array("kM01Id2McpDeltaI16", arrays[TABLE_KEYS["m01"]])}

{_format_i16_array("kM02Id3McpRoutingDeltaI16", arrays[TABLE_KEYS["m02"]])}

{_format_i16_array("kM03Id4McpRoutingDeltaI16", arrays[TABLE_KEYS["m03"]])}

{_format_i16_array("kM04Theta1ReturnDeltaI16", arrays[TABLE_KEYS["m04_theta1"]])}

{_format_i16_array("kM04Theta2ReturnDeltaI16", arrays[TABLE_KEYS["m04_theta2"]])}
// clang-format on

float i16ToMm(int16_t value)
{{
    return (float)value * kInvScale;
}}

float lookup1D(const int16_t* table,
               uint16_t count,
               float minDeg,
               float stepDeg,
               float xDeg,
               bool* inputClamped)
{{
    if (!table || count == 0) return 0.0f;
    if (!isfinite(xDeg)) xDeg = minDeg;

    const float maxDeg = minDeg + stepDeg * (float)(count - 1);
    bool clamped = false;
    if (xDeg < minDeg) {{
        xDeg = minDeg;
        clamped = true;
    }} else if (xDeg > maxDeg) {{
        xDeg = maxDeg;
        clamped = true;
    }}
    if (inputClamped) *inputClamped = clamped;

    const float position = (xDeg - minDeg) / stepDeg;
    uint16_t i0 = (uint16_t)floorf(position);
    if (i0 >= count - 1) return i16ToMm(table[count - 1]);

    const uint16_t i1 = i0 + 1;
    const float t = position - (float)i0;
    const float v0 = i16ToMm(table[i0]);
    const float v1 = i16ToMm(table[i1]);
    return v0 + (v1 - v0) * t;
}}

float lookup2D(const int16_t* table,
               uint16_t rows,
               uint16_t cols,
               float theta1Deg,
               float theta2Deg,
               bool* inputClamped)
{{
    if (!table || rows == 0 || cols == 0) return 0.0f;
    if (!isfinite(theta1Deg)) theta1Deg = kTheta1Min;
    if (!isfinite(theta2Deg)) theta2Deg = kTheta2Min;

    const float theta1Max = kTheta1Min + kAxisStep * (float)(rows - 1);
    const float theta2Max = kTheta2Min + kAxisStep * (float)(cols - 1);
    bool clamped = false;
    if (theta1Deg < kTheta1Min) {{
        theta1Deg = kTheta1Min;
        clamped = true;
    }} else if (theta1Deg > theta1Max) {{
        theta1Deg = theta1Max;
        clamped = true;
    }}
    if (theta2Deg < kTheta2Min) {{
        theta2Deg = kTheta2Min;
        clamped = true;
    }} else if (theta2Deg > theta2Max) {{
        theta2Deg = theta2Max;
        clamped = true;
    }}
    if (inputClamped) *inputClamped = clamped;

    const float rowPosition = (theta1Deg - kTheta1Min) / kAxisStep;
    const float colPosition = (theta2Deg - kTheta2Min) / kAxisStep;
    uint16_t r0 = (uint16_t)floorf(rowPosition);
    uint16_t c0 = (uint16_t)floorf(colPosition);
    if (r0 >= rows - 1) r0 = rows - 2;
    if (c0 >= cols - 1) c0 = cols - 2;
    const uint16_t r1 = r0 + 1;
    const uint16_t c1 = c0 + 1;
    const float tr = rowPosition - (float)r0;
    const float tc = colPosition - (float)c0;

    const uint32_t row0 = (uint32_t)r0 * cols;
    const uint32_t row1 = (uint32_t)r1 * cols;
    const float v00 = i16ToMm(table[row0 + c0]);
    const float v01 = i16ToMm(table[row0 + c1]);
    const float v10 = i16ToMm(table[row1 + c0]);
    const float v11 = i16ToMm(table[row1 + c1]);
    const float v0 = v00 + (v01 - v00) * tc;
    const float v1 = v10 + (v11 - v10) * tc;
    return v0 + (v1 - v0) * tr;
}}

}} // namespace

float lookupM00Id1DeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped)
{{
    return lookup2D(kM00Id1McpDeltaI16, {theta1_count}, {theta2_count}, theta1Deg, theta2Deg, inputClamped);
}}

float lookupM01Id2DeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped)
{{
    return lookup2D(kM01Id2McpDeltaI16, {theta1_count}, {theta2_count}, theta1Deg, theta2Deg, inputClamped);
}}

float lookupM02McpRoutingDeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped)
{{
    return lookup2D(kM02Id3McpRoutingDeltaI16, {theta1_count}, {theta2_count}, theta1Deg, theta2Deg, inputClamped);
}}

float lookupM03McpRoutingDeltaMm(float theta1Deg, float theta2Deg, bool* inputClamped)
{{
    return lookup2D(kM03Id4McpRoutingDeltaI16, {theta1_count}, {theta2_count}, theta1Deg, theta2Deg, inputClamped);
}}

float lookupM04Theta1DeltaMm(float theta1Deg, bool* inputClamped)
{{
    return lookup1D(kM04Theta1ReturnDeltaI16, {theta1_count}, kTheta1Min, kAxisStep, theta1Deg, inputClamped);
}}

float lookupM04Theta2DeltaMm(float theta2Deg, bool* inputClamped)
{{
    return lookup1D(kM04Theta2ReturnDeltaI16, {theta2_count}, kTheta2Min, kAxisStep, theta2Deg, inputClamped);
}}

bool computeLengthDeltaMm(float theta1Deg,
                          float theta2Deg,
                          float theta3Deg,
                          float theta4Deg,
                          float outLengthDeltaMm[kTendonCount],
                          bool* inputClamped)
{{
    if (!outLengthDeltaMm) return false;

    bool clamped = false;
    bool lookupClamped = false;
    const float m02McpRouting = lookupM02McpRoutingDeltaMm(theta1Deg, theta2Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    lookupClamped = false;
    const float m03McpRouting = lookupM03McpRoutingDeltaMm(theta1Deg, theta2Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    const float theta3Rad = isfinite(theta3Deg) ? theta3Deg * kDegToRad : 0.0f;
    const float theta4Rad = isfinite(theta4Deg) ? theta4Deg * kDegToRad : 0.0f;

    lookupClamped = false;
    outLengthDeltaMm[kM00Id1] = lookupM00Id1DeltaMm(theta1Deg, theta2Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    lookupClamped = false;
    outLengthDeltaMm[kM01Id2] = lookupM01Id2DeltaMm(theta1Deg, theta2Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    outLengthDeltaMm[kM02Id3] = m02McpRouting + kM02PipSlopeMmPerRad * theta3Rad;
    outLengthDeltaMm[kM03Id4] = m03McpRouting +
        kM03PipSlopeMmPerRad * theta3Rad +
        kM03DipSlopeMmPerRad * theta4Rad;
    lookupClamped = false;
    const float m04Theta1 = lookupM04Theta1DeltaMm(theta1Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    lookupClamped = false;
    const float m04Theta2 = lookupM04Theta2DeltaMm(theta2Deg, &lookupClamped);
    clamped = clamped || lookupClamped;
    outLengthDeltaMm[kM04Id5] = m04Theta1 + m04Theta2 +
        kM04PipSlopeMmPerRad * theta3Rad +
        kM04DipSlopeMmPerRad * theta4Rad;

    if (inputClamped) *inputClamped = clamped;
    return true;
}}

LengthDeltaResult computeLengthDeltaMm(float theta1Deg,
                                       float theta2Deg,
                                       float theta3Deg,
                                       float theta4Deg)
{{
    LengthDeltaResult result = {{{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, false}};
    computeLengthDeltaMm(theta1Deg,
                         theta2Deg,
                         theta3Deg,
                         theta4Deg,
                         result.lengthDeltaMm,
                         &result.inputClamped);
    return result;
}}

}} // namespace TendonFeedforwardLut
"""
    path.write_text(source, encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export tendon LUT NPZ to firmware C++ files.")
    parser.add_argument("--lut", type=Path, default=DEFAULT_LUT)
    parser.add_argument("--metadata", type=Path, default=DEFAULT_METADATA)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--header-name", default="TendonFeedforwardLut.h")
    parser.add_argument("--source-name", default="TendonFeedforwardLut.cpp")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    arrays = np.load(args.lut)
    _require_keys(arrays.files, TABLE_KEYS.values())
    metadata = _load_metadata(args.metadata)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    header_path = out_dir / args.header_name
    source_path = out_dir / args.source_name
    _write_header(header_path, metadata)
    _write_source(source_path, args.header_name, arrays, metadata)
    print(f"wrote {header_path}")
    print(f"wrote {source_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
