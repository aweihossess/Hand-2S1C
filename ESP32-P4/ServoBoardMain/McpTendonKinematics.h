#ifndef MCP_TENDON_KINEMATICS_H
#define MCP_TENDON_KINEMATICS_H

// MCP 双腱正向运动学（解析绳长）。
// 约定与上位机一致：θ1 = AA（外展/内收），θ2 = FE（屈伸），单位均为【度】。
// 式中三角项使用 θ2 + 35°（与推导一致）。

/**
 * MCP 电机 M00 对应腱的解析绳长（mm），由 θ1(AA)、θ2(FE)（度）计算。
 * （几何式对应原推导 |A1D1|。）
 */
float mcpRopeLengthM00_mm(float theta1Deg, float theta2Deg);

/**
 * MCP 电机 M01 对应腱的解析绳长（mm），由 θ1(AA)、θ2(FE)（度）计算。
 * （几何式对应原推导 |A2D2|。）
 */
float mcpRopeLengthM01_mm(float theta1Deg, float theta2Deg);

#endif
