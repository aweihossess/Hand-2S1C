#include "McpTendonPdControl.h"

#include "McpTendonKinematics.h"
#include "McpRopeToPulse.h"

#include <math.h>
#include <string.h>

// 可调增益（绳长单位 mm；de/dt 为 mm/s）
namespace {
constexpr float kFeedforwardS[2] = {1.0f, 1.0f};
constexpr float kKp[2] = {1.0f, 1.0f};
constexpr float kKd[2] = {0.05f, 0.05f};
// 某路收绳方向与脉冲增大方向相反时置 -1
constexpr int8_t kRopeToPulseSign[2] = {1, 1};

float s_prevE[2];
uint8_t s_hasPrevE = 0;
} // namespace

void mcpTendonPdReset(void)
{
    memset(s_prevE, 0, sizeof(s_prevE));
    s_hasPrevE = 0;
}

void mcpTendonPdStep(float q_aa_ref_deg,
                     float q_fe_ref_deg,
                     float q_aa_fb_deg,
                     float q_fe_fb_deg,
                     float dt_sec,
                     const int32_t mech_zero_abs[2],
                     const float k_counts_per_mm[2],
                     int32_t* out_m0_abs,
                     int32_t* out_m1_abs)
{
    if (!mech_zero_abs || !k_counts_per_mm || !out_m0_abs || !out_m1_abs) {
        return;
    }
    if (!isfinite(dt_sec) || dt_sec <= 0.0f) {
        dt_sec = 0.01f;
    }

    const float xr0 = mcpRopeLengthM00_mm(q_aa_ref_deg, q_fe_ref_deg);
    const float xr1 = mcpRopeLengthM01_mm(q_aa_ref_deg, q_fe_ref_deg);
    const float xf0 = mcpRopeLengthM00_mm(q_aa_fb_deg, q_fe_fb_deg);
    const float xf1 = mcpRopeLengthM01_mm(q_aa_fb_deg, q_fe_fb_deg);

    const float e0 = xr0 - xf0;
    const float e1 = xr1 - xf1;

    float de0_dt = 0.0f;
    float de1_dt = 0.0f;
    if (s_hasPrevE) {
        de0_dt = (e0 - s_prevE[0]) / dt_sec;
        de1_dt = (e1 - s_prevE[1]) / dt_sec;
    }
    s_prevE[0] = e0;
    s_prevE[1] = e1;
    s_hasPrevE = 1;

    const float u0 = kFeedforwardS[0] * xr0 + kKp[0] * e0 + kKd[0] * de0_dt;
    const float u1 = kFeedforwardS[1] * xr1 + kKp[1] * e1 + kKd[1] * de1_dt;

    // 名义零位绳长（θ_aa=θ_fe=0）；机构标定应在该姿态附近完成以便 mech_zero 与 L_ref 对齐
    const float Lref0 = mcpRopeLengthM00_mm(0.0f, 0.0f);
    const float Lref1 = mcpRopeLengthM01_mm(0.0f, 0.0f);

    const float k0 = (k_counts_per_mm[0] > 1e-6f) ? k_counts_per_mm[0] : 80.0f;
    const float k1 = (k_counts_per_mm[1] > 1e-6f) ? k_counts_per_mm[1] : 80.0f;

    const float kArr[2] = {k0, k1};
    const float uCmdMm[2] = {u0, u1};
    const float LrefArr[2] = {Lref0, Lref1};
    int32_t pulses[2];
    mcpDuoRopeMmToMotorAbsPulses(
        uCmdMm,
        LrefArr,
        mech_zero_abs,
        kArr,
        kRopeToPulseSign,
        pulses);
    *out_m0_abs = pulses[0];
    *out_m1_abs = pulses[1];
}
