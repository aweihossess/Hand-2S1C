#ifndef MCP_ROPE_TO_PULSE_H
#define MCP_ROPE_TO_PULSE_H

#include <stdint.h>

// MCP 腱绳空间(mm) → 舵机多圈指令(脉冲计数) 线性换算。
// 控制链（关节模式 + MCP 绳控生效时）：
//   AngleSolver → mcpTendonPdStep（得到指令绳长 u_cmd mm）
//   → 本模块 mcpRopeMmToMotorAbsPulses / mcpDuoRopeMmToMotorAbsPulses
//   → 多圈脉冲 → ServoBusManager::setTarget（M00/M01）。
// 与 desk_2S1C mcp_zero_config 一致：机构零位记录 mech_zero_pulses，
// 该姿态下名义绳长为 ref_rope_mm（通常取运动学 x(0,0)），则
//   P_cmd = mech_zero_pulses + sign * k_counts_per_mm * (rope_cmd_mm - ref_rope_mm)
// sign=+1 为默认收绳方向；若某路电机接线反向可设 sign=-1。

/** 与协议 CMD_MOTOR_POS_ABS / clampServoPos 一致的多圈绝对位置限幅 */
#define MCP_MOTOR_ABS_PULSE_MAX 30719
#define MCP_MOTOR_ABS_PULSE_MIN (-30719)

/**
 * 绳长变化 ΔL(mm) → 脉冲增量（四舍五入为整数）。
 */
int32_t mcpRopeDeltaMmToPulseDelta(float deltaRopeMm, float kCountsPerMm);

/**
 * 指令绳长 rope_cmd(mm) → 多圈绝对脉冲 P_cmd。
 * @param rope_cmd_mm       期望腱绳长度（与 McpTendonKinematics 输出同单位）
 * @param mech_zero_pulses  机构零位时该路电机多圈位置
 * @param ref_rope_mm       机构零位时该路名义绳长（常用 x(θ_aa=0,θ_fe=0)）
 * @param k_counts_per_mm   mm → 脉冲，正值
 * @param direction_sign    +1 或 -1
 */
int32_t mcpRopeMmToMotorAbsPulses(float rope_cmd_mm,
                                  int32_t mech_zero_pulses,
                                  float ref_rope_mm,
                                  float k_counts_per_mm,
                                  int8_t direction_sign);

/**
 * MCP 双路腱：两路「指令绳长 mm」一次换为 M00/M01 多圈绝对脉冲（即最终下发给电机的量纲）。
 */
void mcpDuoRopeMmToMotorAbsPulses(const float rope_cmd_mm[2],
                                 const float ref_rope_mm[2],
                                 const int32_t mech_zero_pulses[2],
                                 const float k_counts_per_mm[2],
                                 const int8_t direction_sign[2],
                                 int32_t out_motor_abs_pulses[2]);

#endif
