#ifndef CONTROL_SOLVER_CONFIG_H
#define CONTROL_SOLVER_CONFIG_H

#include "../shared/TaskSharedData.h"

// MCP tendon model config.
// Physical motor placement is swapped:
// index 0 maps to M00/model R tendon, index 1 maps to M01/model L tendon.
// Motor Abs is the servo position relative to SW Zero Ofs, so 0 means the
// mechanical zero captured by Set Servo Zero.

static const float kTendonLengthToPulse[JOINT_COUNT] = {
    -160.0f, -160.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// Small length-feedback correction in mm per mm of tendon length error.
// Feedforward remains the primary command path.
static const float kTendonLengthKp[JOINT_COUNT] = {
    10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float kTendonLengthKd[JOINT_COUNT] = {
    0.05f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// Motor position loop. Output is a per-cycle Motor Abs step.
static const float kTendonMotorKp[JOINT_COUNT] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float kTendonMotorKd[JOINT_COUNT] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float kTendonMotorOutputLimit[JOINT_COUNT] = {
    20.0f, 20.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f,
    4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f,
    4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f
};

#endif // CONTROL_SOLVER_CONFIG_H
