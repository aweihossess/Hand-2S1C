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

// Angle feedback correction for the MCP pair.
// Rows are motors/tendons: 0=M00/model R, 1=M01/model L.
// Columns are joints: 0=J00/MCP-AA, 1=J01/MCP-FE.
// Units:
//   P: motor counts / deg
//   I: motor counts / (deg*s)
//   D: motor counts / (deg/s)
static const float kMcpAngleKp[2][2] = {
    {-20.0f, 30.0f},
    { 20.0f, 30.0f}
};

static const float kMcpAngleKi[2][2] = {
    {-1.0f, 20.0f},
    { 1.0f, 20.0f}
};

static const float kMcpAngleKd[2][2] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};

static const float kMcpAngleIntegralLimitDegSec[2] = {
    30.0f, 60.0f
};

static const float kMcpAngleFeedbackLimitCounts[2] = {
    1200.0f, 1200.0f
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
