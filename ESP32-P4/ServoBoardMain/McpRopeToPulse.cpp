#include "McpRopeToPulse.h"

#include <math.h>

int32_t mcpRopeDeltaMmToPulseDelta(float deltaRopeMm, float kCountsPerMm)
{
    if (!isfinite(deltaRopeMm) || !isfinite(kCountsPerMm)) {
        return 0;
    }
    if (kCountsPerMm <= 1e-9f) {
        return 0;
    }
    const float dp = deltaRopeMm * kCountsPerMm;
    if (!isfinite(dp)) {
        return 0;
    }
    return (int32_t)lroundf(dp);
}

int32_t mcpRopeMmToMotorAbsPulses(float rope_cmd_mm,
                                  int32_t mech_zero_pulses,
                                  float ref_rope_mm,
                                  float kCountsPerMm,
                                  int8_t direction_sign)
{
    if (!isfinite(rope_cmd_mm)) {
        rope_cmd_mm = ref_rope_mm;
    }
    if (!isfinite(ref_rope_mm)) {
        ref_rope_mm = 0.0f;
    }
    float k = kCountsPerMm;
    if (!isfinite(k) || k <= 1e-9f) {
        k = 80.0f;
    }

    int8_t sgn = (direction_sign < 0) ? -1 : 1;
    const float deltaL = rope_cmd_mm - ref_rope_mm;
    const float deltaP = (float)sgn * k * deltaL;
    if (!isfinite(deltaP)) {
        return mech_zero_pulses;
    }

    float sum = (float)mech_zero_pulses + deltaP;
    if (!isfinite(sum)) {
        return mech_zero_pulses;
    }

    int32_t p = (int32_t)lroundf(sum);
    if (p > MCP_MOTOR_ABS_PULSE_MAX) {
        return MCP_MOTOR_ABS_PULSE_MAX;
    }
    if (p < MCP_MOTOR_ABS_PULSE_MIN) {
        return MCP_MOTOR_ABS_PULSE_MIN;
    }
    return p;
}

void mcpDuoRopeMmToMotorAbsPulses(const float rope_cmd_mm[2],
                                 const float ref_rope_mm[2],
                                 const int32_t mech_zero_pulses[2],
                                 const float k_counts_per_mm[2],
                                 const int8_t direction_sign[2],
                                 int32_t out_motor_abs_pulses[2])
{
    if (!rope_cmd_mm || !ref_rope_mm || !mech_zero_pulses || !k_counts_per_mm || !direction_sign ||
        !out_motor_abs_pulses) {
        return;
    }
    for (int i = 0; i < 2; i++) {
        out_motor_abs_pulses[i] = mcpRopeMmToMotorAbsPulses(
            rope_cmd_mm[i],
            mech_zero_pulses[i],
            ref_rope_mm[i],
            k_counts_per_mm[i],
            direction_sign[i]);
    }
}
