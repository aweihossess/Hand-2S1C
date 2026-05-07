"""
MCP 双绳长正向几何：已知关节角求绳长（解析式，单位与常数一致，视为 mm）。

约定（与上位机滑块一致）：
- θ1 = AA（外展/内收），度
- θ2 = FE（屈曲），度

|A1D1|、|A1D2| 为两路绳长；相对零位取 θ1=θ2=0 时的长度 L_ref，控制用 ΔL = L - L_ref。
"""

from __future__ import annotations

import math
from typing import Tuple

# 合并常数后的解析式系数（与 CAD/推导一致）
_C1 = 589.64
_C2_BASE = 189.34
_C2_COS = 282.36
_C2_COS2 = 117.94
_C3 = 117.94
_C4 = 97.94


def rope_lengths_mm(theta1_aa_rad: float, theta2_fe_rad: float) -> Tuple[float, float]:
    """
    输入弧度：θ1 为 AA，θ2 为 FE。
    返回 (L1, L2)，与公式中 |A1D1|、|A1D2| 对应。
    """
    c1 = math.cos(theta1_aa_rad)
    s1 = math.sin(theta1_aa_rad)
    c2 = math.cos(theta2_fe_rad)
    s2 = math.sin(theta2_fe_rad)
    s1sq = s1 * s1
    c1sq = c1 * c1
    inner = _C2_BASE + _C2_COS * c2 + _C2_COS2 * c2 * c2
    common = _C1 * c1sq + inner * s1sq + _C3 * s2 * s2
    cross = _C4 * (c2 - 1.0) * s1 * c1
    i1 = max(0.0, common + cross)
    i2 = max(0.0, common - cross)
    return math.sqrt(i1), math.sqrt(i2)


def rope_reference_mm() -> Tuple[float, float]:
    """零位 θ1=θ2=0 时的两路绳长。"""
    return rope_lengths_mm(0.0, 0.0)


def delta_rope_lengths_mm(aa_deg: float, fe_deg: float) -> Tuple[float, float]:
    """
    相对零位绳长变化 ΔL1、ΔL2（mm）。
    aa_deg → θ1，fe_deg → θ2。
    """
    t1 = math.radians(float(aa_deg))
    t2 = math.radians(float(fe_deg))
    l1, l2 = rope_lengths_mm(t1, t2)
    lr1, lr2 = rope_reference_mm()
    return l1 - lr1, l2 - lr2
