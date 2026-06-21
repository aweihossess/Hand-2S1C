#ifndef TENDON_FEEDFORWARD_LUT_H
#define TENDON_FEEDFORWARD_LUT_H

#include <stdint.h>

namespace TendonFeedforwardLut {

// Generated from tools/tendon_forward_model/generated_lut_0p2deg.
// The table stores physical tendon length deltas relative to the zero pose.
// Sign convention:
//   lengthDeltaMm > 0 means the tendon lengthens.
//   lengthDeltaMm < 0 means the tendon shortens.
// Hardware conversion is normally:
//   motor_abs_delta_counts = -lengthDeltaMm * counts_per_mm
// because motor_abs increasing tightens/shortens the tendon.
static const uint8_t kTendonCount = 5;
static const float kTheta1MinDeg = -20.0f;
static const float kTheta1MaxDeg = 30.0f;
static const float kTheta2MinDeg = 0.0f;
static const float kTheta2MaxDeg = 90.0f;
static const float kStepDeg = 0.2f;
static const uint16_t kTheta1Count = 251;
static const uint16_t kTheta2Count = 451;
static const float kQuantizationScaleMm = 1000.0f;

// Offline geometry warnings. These entries were filled with straight-line
// fallback during table generation because the current keepout model marked
// those poses as geometrically invalid.
static const uint32_t kM00InvalidCount = 0;
static const uint32_t kM01InvalidCount = 0;
static const uint32_t kM02InvalidCount = 18942;
static const uint32_t kM03InvalidCount = 27962;

enum TendonIndex : uint8_t {
    kM00Id1 = 0,
    kM01Id2 = 1,
    kM02Id3 = 2,
    kM03Id4 = 3,
    kM04Id5 = 4,
};

struct LengthDeltaResult {
    float lengthDeltaMm[kTendonCount];
    bool inputClamped;
};

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

} // namespace TendonFeedforwardLut

#endif // TENDON_FEEDFORWARD_LUT_H
