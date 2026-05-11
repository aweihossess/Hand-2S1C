#include "McpTendonKinematics.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

// θ2 中 cos/sin 内的常值偏置（度 → 弧度）
constexpr float kTheta2OffsetDeg = 35.0f;

static float degToRad(float deg)
{
    return deg * (float)(M_PI / 180.0);
}

// 两式公共项系数（与推导式一致）
constexpr float kB0 = 405.97f;
constexpr float kB_cos_t2p = 282.36f;
constexpr float kB_sin_t2p = -74.50f;
constexpr float kC_sin1_a1 = -256.02f;
constexpr float kC_sin1_a2 = 256.02f;
constexpr float kD_cos1 = 17.94f;
constexpr float kE_cross_a1 = -186.36f;
constexpr float kE_cross_a2 = 186.36f;
constexpr float kF_cross = 79.50f;

static float sqrtPositive(float x)
{
    if (!isfinite(x) || x <= 0.0f) {
        return 0.0f;
    }
    return sqrtf(x);
}

} // namespace

float mcpRopeLengthM00_mm(float theta1Deg, float theta2Deg)
{
    const float t1 = degToRad(theta1Deg);
    const float t2p = degToRad(theta2Deg + kTheta2OffsetDeg);
    const float c2p = cosf(t2p);
    const float s2p = sinf(t2p);
    const float s1 = sinf(t1);
    const float c1 = cosf(t1);

    const float inner = kB0 + kB_cos_t2p * c2p + kB_sin_t2p * s2p + kC_sin1_a1 * s1 +
                        kD_cos1 * c1 + kE_cross_a1 * s1 * c2p + kF_cross * c1 * c2p;

    return sqrtPositive(inner);
}

float mcpRopeLengthM01_mm(float theta1Deg, float theta2Deg)
{
    const float t1 = degToRad(theta1Deg);
    const float t2p = degToRad(theta2Deg + kTheta2OffsetDeg);
    const float c2p = cosf(t2p);
    const float s2p = sinf(t2p);
    const float s1 = sinf(t1);
    const float c1 = cosf(t1);

    const float inner = kB0 + kB_cos_t2p * c2p + kB_sin_t2p * s2p + kC_sin1_a2 * s1 +
                        kD_cos1 * c1 + kE_cross_a2 * s1 * c2p + kF_cross * c1 * c2p;

    return sqrtPositive(inner);
}
