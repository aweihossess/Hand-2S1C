#ifndef MCP_TENDON_PD_CONTROL_H
#define MCP_TENDON_PD_CONTROL_H

#include <stdint.h>

// MCP 双腱绳长 PD + 模型前馈（与 McpTendonKinematics 一致，θ1=AA、θ2=FE，度）。
// u_cmd,i = s_i * x_i(q_ref) + Kp_i * e_i + Kd_i * de_i/dt ，e_i = x_i(q_ref) - x_i(q_fb)
// 舵机多圈脉冲由 McpRopeToPulse：P_cmd = P0 + sign * k * (u_cmd - L_ref)，见 mcp_zero_config。

void mcpTendonPdReset(void);

/**
 * @param q_aa_ref_deg / q_fe_ref_deg  参考角（与磁编零位坐标一致）
 * @param q_aa_fb_deg / q_fe_fb_deg    反馈角 q_fb
 * @param dt_sec  控制周期（秒）
 * @param mech_zero_abs  M00/M01 在机构零位（θ=0）标定处的多圈绝对位置
 * @param k_counts_per_mm  绳长 mm → 脉冲比例（与 JSON mcp_rope_k_counts_per_mm 一致）
 * @param out_m0_abs / out_m1_abs  输出 M00/M01 多圈绝对命令（已限幅）
 */
void mcpTendonPdStep(float q_aa_ref_deg,
                      float q_fe_ref_deg,
                      float q_aa_fb_deg,
                      float q_fe_fb_deg,
                      float dt_sec,
                      const int32_t mech_zero_abs[2],
                      const float k_counts_per_mm[2],
                      int32_t* out_m0_abs,
                      int32_t* out_m1_abs);

#endif
