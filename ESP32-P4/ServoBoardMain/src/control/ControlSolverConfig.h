#ifndef CONTROL_SOLVER_CONFIG_H
#define CONTROL_SOLVER_CONFIG_H

#include "../shared/TaskSharedData.h"

// Five-tendon finger feedforward config.
// Tendon indices:
//   0=M00/ID1 MCP-AA right swing + MCP-FE flexion
//   1=M01/ID2 MCP-AA left swing  + MCP-FE flexion
//   2=M02/ID3 PIP-FE flexion
//   3=M03/ID4 DIP/distal-FE flexion
//   4=M04/ID5 common return tendon
// Motor Abs is the servo position relative to SW Zero Ofs, so 0 means the
// mechanical zero captured by Set Servo Zero.

// Empirical five-tendon feedforward generated from the tight-tendon collection:
//   run_data/mcp_control_20260625_215252_tension_on_fit_now_minus_bias_report.txt
// The model target is `now - tension_bias`, so runtime tension bias can still
// add preload independently. Values are motor counts per degree, rows M00..M04,
// columns J00..J03.
static const bool kUseEmpiricalMcpFeedforwardCounts = true;
// 4+1 mode: M00..M03 control the four joint DOFs, M04 is reserved as a
// preload-only return tendon and receives only runtime tension bias.
static const bool kMcpReturnTendonPreloadOnly = true;
static const float kEmpiricalMcpMotorPerDeg[5][4] = {
    { 19.525f,  11.619f,   0.848f,  -8.994f},
    {-13.384f,  13.533f,  -7.042f,   0.826f},
    { -5.835f,  -9.028f,  38.401f, -14.173f},
    { 24.708f,  -7.853f, -18.443f,  29.339f},
    {  5.597f, -30.330f, -54.012f, -30.450f}
};

static const float kTendonLengthToPulse[JOINT_COUNT] = {
    -160.0f, -160.0f, -160.0f, -160.0f, -160.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// MCP feedforward geometry, in mm and degrees.
// These parameters define the A1D1/A2D2 tendon length model:
// A1D1 = [cos(t1)*(L1 + L2*cos(t2 + To)) - L3*sin(t1) - X1,
//         -L2*sin(t2 + To) - Y1,
//         sin(t1)*(L1 + L2*cos(t2 + To)) + L3*cos(t1) - Z1]
// A2D2 = [cos(t1)*(L1 + L2*cos(t2 + To)) + L3*sin(t1) - X1,
//         -L2*sin(t2 + To) - Y1,
//         sin(t1)*(L1 + L2*cos(t2 + To)) - L3*cos(t1) + Z1]
// AC   = [cos(t1)*(L1 + L4*cos(t2 + T4)) - X3,
//         -L4*sin(t2 + T4) - Y3,
//         sin(t1)*(L1 + L4*cos(t2 + T4)) - Z3]
static const float kMcpGeometryX1Mm = -3.66f;
static const float kMcpGeometryY1Mm = 4.25f;
static const float kMcpGeometryZ1Mm = 8.58f;
static const float kMcpGeometryL1Mm = 13.00f;
static const float kMcpGeometryL2Mm = 10.08f;
static const float kMcpGeometryL3Mm = 4.5f;
static const float kMcpThetaOffsetDeg = -36.5f;
static const float kMcpGeometryX3Mm = -7.06f;
static const float kMcpGeometryY3Mm = -5.08f;
static const float kMcpGeometryZ3Mm = 0.0f;
static const float kMcpGeometryL4Mm = 9.27f;
static const float kMcpTheta4Deg = 29.0546f;

// Angle feedback correction for the five-tendon actuator set.
// Rows are motors/tendons M00..M04.
// Columns are joints: 0=J00/MCP-AA, 1=J01/MCP-FE, 2=J02/PIP-FE, 3=J03/DIP-FE.
// Keep these gains conservative while the new five-tendon length model is
// being validated. The LUT feedforward supplies the nominal motor target, and
// this matrix adds a small encoder-error correction in motor counts.
// Units:
//   P: motor counts / deg
//   I: motor counts / (deg*s)
//   D: motor counts / (deg/s)
static const float kMcpAngleKp[5][4] = {
    { 30.0f,  20.0f,   0.0f,   0.0f},
    {-30.0f,  20.0f,   0.0f,   0.0f},
    {  0.0f,   -20.0f,  30.0f,   0.0f},
    { 10.0f, -17.5f, -30.0f,  24.0f},
    {  0.0f,   0.0f,   0.0f,   0.0f}
};

static const float kMcpAngleKi[5][4] = {
    { 0.90f,  0.60f,   0.00f,   0.00f},
    {-0.90f,  0.60f,   0.00f,   0.00f},
    { 0.00f, -0.60f,   0.90f,   0.00f},
    { 0.30f, -0.525f, -0.90f,   2.40f},
    { 0.00f,  0.00f,   0.00f,   0.00f}
};

static const float kMcpAngleKd[5][4] = {
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.02f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

static const float kMcpAngleIntegralLimitDegSec[4] = {
    30.0f, 60.0f, 60.0f, 60.0f
};

static const float kMcpAngleFeedbackLimitCounts[5] = {
    2400.0f, 2400.0f, 2400.0f, 2400.0f, 2400.0f
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
