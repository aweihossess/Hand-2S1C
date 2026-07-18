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

// Identified local tendon model used by the runtime feedforward:
//
//   delta_L_ff_counts / 200 = R * (q_ref - q_entry)_rad
//
// This is the feedforward branch.  It depends on the commanded reference
// displacement from the pose captured when joint control starts; it does not
// use the tracking error.  Delta-F is intentionally not included because it
// is not known at command time, and fitted residual e is not repeatedly
// injected into the controller.
// Units of R: mm/rad. Rows are M00..M04 and columns are J00..J03.
// Fit revision 2026-07-18: 1695 stable endpoints from nine experiments;
// requires all four encoders valid, J01 > 0 deg and all five tension readings
// <= 0 N.  The fit uses the physical tendon sparsity pattern, a separate e for
// each initial pose and Huber robust regression.
static const float kMcpCountsPerMm = 200.0f;
// Finite-increment limits for the 10 ms control loop.  A new joint target is
// approached as a sequence of small reference and motor-position steps rather
// than one large absolute move.
static const float kMcpReferenceMaxStepDeg = 0.25f;
static const float kFittedMcpR[5][4] = {
    { 3.6109f,  3.4232f,  0.0000f,  0.0000f},
    {-2.9767f,  4.0643f,  0.0000f,  0.0000f},
    {-1.9980f, -1.3476f,  2.9110f,  0.0000f},
    { 2.5066f, -1.5569f,  0.0000f,  3.6078f},
    { 0.0000f, -3.8985f, -4.3955f, -3.3443f}
};

// Orthogonal task-space projector Q = R * pinv(R), evaluated from the fitted
// matrix above.  Q keeps only actuator displacements that can be produced by
// the four joint coordinates.  The one-dimensional internal-tension motion
// is deliberately added after this projection, so Q must never be applied to
// the host-provided tension bias alpha*n.
static const float kMcpTaskProjector[5][5] = {
    { 0.83682277f, -0.15979540f, -0.24728505f, -0.15180796f, -0.16376903f},
    {-0.15979540f,  0.84351634f, -0.24216010f, -0.14866176f, -0.16037494f},
    {-0.24728505f, -0.24216010f,  0.62525472f, -0.23005563f, -0.24818189f},
    {-0.15180796f, -0.14866176f, -0.23005563f,  0.85876915f, -0.15235854f},
    {-0.16376903f, -0.16037494f, -0.24818189f, -0.15235854f,  0.83563702f}
};

static const float kTendonLengthToPulse[JOINT_COUNT] = {
    -160.0f, -160.0f, -160.0f, -160.0f, -160.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// PI angle feedback correction for the five-tendon actuator set:
//
//   delta_L_P_counts = 200 * Kp * P * (q_ref - q_feedback)_rad
//   delta_L_I_counts = 200 * Ki * P * integral(q_ref - q_feedback)_rad_s
//
// P is the fixed physical tendon/joint coupling matrix below.  Kp is a
// separate dimensionless runtime scale, so tuning Kp never changes the
// coupling directions or sparsity of P.
// Rows: M00..M04. Columns: J00..J03. Unit of P: mm/rad.
static const float kMcpAnglePBase[5][4] = {
    { 4.3f,  5.9f,  0.0f,  0.0f},
    {-4.3f,  5.9f,  0.0f,  0.0f},
    {-1.2f, -3.0f,  6.0f,  0.0f},
    { 1.2f, -3.0f,  0.0f,  4.8f},
    { 0.0f, -6.2f, -6.0f, -6.2f}
};

// Safe initial value.  It can be changed online with "kp <value>" without
// rebuilding the firmware.  Start low and increase after small-angle tests.
static const float kDefaultMcpAngleKpScale = 0.10f;

// Ki is in 1/s.  Integration starts only after the finite-step reference has
// reached the requested target.  Errors inside the deadband are not
// integrated.  There is intentionally no separate I-state or I-output limit;
// the combined projected PI vector is uniformly scaled when any one channel
// reaches the actuator-output boundary.  Uniform scaling preserves range(R).
static const float kDefaultMcpAngleKiScale = 0.005f;
static const float kMcpAngleIntegralDeadbandDeg = 0.20f;

static const float kMcpAngleFeedbackLimitCounts[5] = {
    2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f
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
    20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 4096.0f, 4096.0f,
    4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f,
    4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f, 4096.0f
};

#endif // CONTROL_SOLVER_CONFIG_H
