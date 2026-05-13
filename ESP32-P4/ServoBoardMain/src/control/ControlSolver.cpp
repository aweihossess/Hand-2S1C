#include "ControlSolver.h"
#include "ControlSolverConfig.h"

#include <math.h>
#include <string.h>

static const float kDefaultQRefMaxStepDeg = 2.0f;
static const float kDefaultQFbLpfAlpha = 0.25f;
static const float kDefaultQFbMinDeg = -360.0f;
static const float kDefaultQFbMaxDeg = 360.0f;
static const float kDegToRad = 0.017453292519943295f;
static const float kControlPeriodSec = 0.01f;
static const float kDefaultTendonMotorOutputLimit = 4096.0f;
static const float kMcpMotorAbsLimitCounts = 2000.0f;

static const uint8_t kMcpLTendonIndex = 0;
static const uint8_t kMcpRTendonIndex = 1;
static const uint8_t kMcpTheta1Joint = 0;
static const uint8_t kMcpTheta2Joint = 1;

static const float kMcpX1 = -3.66f;
static const float kMcpY1 = -3.43f;
static const float kMcpZ1 = 8.58f;
static const float kMcpL1 = 13.00f;
static const float kMcpL2 = 10.854f;
static const float kMcpL3 = 4.5f;
static const float kMcpThetaOffsetRad = 0.6109f;

static const int8_t kJointMotorDirection[JOINT_COUNT] = {
    -1, -1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1
};

static int16_t clampCommand(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

static float clampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

ControlSolver::ControlSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_tendonPrevLengthError, 0, sizeof(_tendonPrevLengthError));
    memset(_tendonKp, 0, sizeof(_tendonKp));
    memset(_tendonKd, 0, sizeof(_tendonKd));
    memset(_targetTendonLength, 0, sizeof(_targetTendonLength));
    memset(_actualTendonLength, 0, sizeof(_actualTendonLength));
    memset(_tendonZeroLength, 0, sizeof(_tendonZeroLength));
    memset(_tendonLengthToPulse, 0, sizeof(_tendonLengthToPulse));
    memset(_tendonPrevMotorError, 0, sizeof(_tendonPrevMotorError));
    memset(_tendonMotorKp, 0, sizeof(_tendonMotorKp));
    memset(_tendonMotorKd, 0, sizeof(_tendonMotorKd));
    memset(_tendonMotorOutputLimit, 0, sizeof(_tendonMotorOutputLimit));
    memset(_mappedMotorTarget, 0, sizeof(_mappedMotorTarget));
    memset(_qRef, 0, sizeof(_qRef));
    memset(_qRefMaxStepDeg, 0, sizeof(_qRefMaxStepDeg));
    memset(_qFbLpfStage1, 0, sizeof(_qFbLpfStage1));
    memset(_qFbFiltered, 0, sizeof(_qFbFiltered));
    memset(_qFbLpfAlpha, 0, sizeof(_qFbLpfAlpha));
    memset(_qFbMinDeg, 0, sizeof(_qFbMinDeg));
    memset(_qFbMaxDeg, 0, sizeof(_qFbMaxDeg));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_pids, 0, sizeof(_pids));
}

void ControlSolver::begin()
{
    int16_t zeros[JOINT_COUNT];
    float ratios[JOINT_COUNT];
    int8_t dirs[JOINT_COUNT];

    for (int i = 0; i < JOINT_COUNT; i++) {
        zeros[i] = 2048;
        ratios[i] = 1.0f;
        dirs[i] = kJointMotorDirection[i];

        _qRefMaxStepDeg[i] = kDefaultQRefMaxStepDeg;
        _qFbLpfAlpha[i] = kDefaultQFbLpfAlpha;
        _qFbMinDeg[i] = kDefaultQFbMinDeg;
        _qFbMaxDeg[i] = kDefaultQFbMaxDeg;
        _tendonLengthToPulse[i] = kTendonLengthToPulse[i];
        _tendonKp[i] = kTendonLengthKp[i];
        _tendonKd[i] = kTendonLengthKd[i];
        _tendonMotorKp[i] = kTendonMotorKp[i];
        _tendonMotorKd[i] = kTendonMotorKd[i];
        _tendonMotorOutputLimit[i] = kTendonMotorOutputLimit[i];
    }

    init(zeros, ratios, dirs);

    float pidConfigs[2][PID_PARAMETER_NUM] = {
        {16.5f, 0.0f, 0.5f, 0.0f, 100.0f, 600.0f},
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}
    };
    setPIDParams(pidConfigs);
}

void ControlSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions)
{
    for (int i = 0; i < JOINT_COUNT; i++) {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    _initialized = true;
}

void ControlSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++) {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

bool ControlSolver::compute(float* targetDegs,
                            float* magActualDegs,
                            const int32_t* absolutePosition,
                            const int32_t* motorZeroAbs,
                            int16_t* outServoPulses)
{
    return computeDualLoopPid(targetDegs, magActualDegs, absolutePosition, motorZeroAbs, outServoPulses);
}

bool ControlSolver::computeTendonFeedforward(float* targetDegs,
                                             float* magActualDegs,
                                             const int32_t* absolutePosition,
                                             const int32_t* motorZeroAbs,
                                             int16_t* outServoPulses)
{
    if (!targetDegs || !magActualDegs || !absolutePosition || !motorZeroAbs || !outServoPulses) {
        return false;
    }

    float qRef[JOINT_COUNT];
    float qFb[JOINT_COUNT];
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        qRef[i] = updateTargetReference(i, targetDegs[i]);
        qFb[i] = updateFeedbackFilter(i, magActualDegs[i]);
    }

    if (!computeDualLoopPid(qRef, qFb, absolutePosition, motorZeroAbs, outServoPulses)) {
        return false;
    }

    if (JOINT_COUNT > kMcpTheta2Joint) {
        _targetTendonLength[kMcpLTendonIndex] =
            computeMcpLTendonLength(qRef[kMcpTheta1Joint], qRef[kMcpTheta2Joint]);
        _actualTendonLength[kMcpLTendonIndex] =
            computeMcpLTendonLength(qFb[kMcpTheta1Joint], qFb[kMcpTheta2Joint]);
        _targetTendonLength[kMcpRTendonIndex] =
            computeMcpRTendonLength(qRef[kMcpTheta1Joint], qRef[kMcpTheta2Joint]);
        _actualTendonLength[kMcpRTendonIndex] =
            computeMcpRTendonLength(qFb[kMcpTheta1Joint], qFb[kMcpTheta2Joint]);

        outServoPulses[kMcpLTendonIndex] =
            computeTendonCascadeOutput(kMcpLTendonIndex, absolutePosition[kMcpLTendonIndex]);
        outServoPulses[kMcpRTendonIndex] =
            computeTendonCascadeOutput(kMcpRTendonIndex, absolutePosition[kMcpRTendonIndex]);
    }

    return true;
}

float ControlSolver::updateTargetReference(uint8_t jointIndex, float targetDeg)
{
    if (jointIndex >= JOINT_COUNT) return 0.0f;
    if (!isfinite(targetDeg)) targetDeg = 0.0f;

    if (!_qRefInitialized[jointIndex]) {
        _qRef[jointIndex] = targetDeg;
        _qRefInitialized[jointIndex] = true;
        return _qRef[jointIndex];
    }

    float maxStep = _qRefMaxStepDeg[jointIndex];
    if (!isfinite(maxStep) || maxStep < 0.0f) maxStep = kDefaultQRefMaxStepDeg;

    const float delta = targetDeg - _qRef[jointIndex];
    if (delta > maxStep) {
        _qRef[jointIndex] += maxStep;
    } else if (delta < -maxStep) {
        _qRef[jointIndex] -= maxStep;
    } else {
        _qRef[jointIndex] = targetDeg;
    }
    return _qRef[jointIndex];
}

float ControlSolver::updateFeedbackFilter(uint8_t jointIndex, float feedbackDeg)
{
    if (jointIndex >= JOINT_COUNT) return 0.0f;
    if (!isfinite(feedbackDeg)) {
        feedbackDeg = _qFbInitialized[jointIndex] ? _qFbFiltered[jointIndex] : 0.0f;
    }

    if (!_qFbInitialized[jointIndex]) {
        _qFbLpfStage1[jointIndex] = feedbackDeg;
        _qFbFiltered[jointIndex] = feedbackDeg;
        _qFbInitialized[jointIndex] = true;
    } else {
        float alpha = _qFbLpfAlpha[jointIndex];
        if (!isfinite(alpha)) alpha = kDefaultQFbLpfAlpha;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        _qFbLpfStage1[jointIndex] += alpha * (feedbackDeg - _qFbLpfStage1[jointIndex]);
        _qFbFiltered[jointIndex] += alpha * (_qFbLpfStage1[jointIndex] - _qFbFiltered[jointIndex]);
    }

    _qFbFiltered[jointIndex] = clampFeedbackDeg(jointIndex, _qFbFiltered[jointIndex]);
    return _qFbFiltered[jointIndex];
}

float ControlSolver::clampFeedbackDeg(uint8_t jointIndex, float feedbackDeg) const
{
    if (jointIndex >= JOINT_COUNT) return 0.0f;

    float minDeg = _qFbMinDeg[jointIndex];
    float maxDeg = _qFbMaxDeg[jointIndex];
    if (!isfinite(minDeg)) minDeg = kDefaultQFbMinDeg;
    if (!isfinite(maxDeg)) maxDeg = kDefaultQFbMaxDeg;
    if (minDeg > maxDeg) {
        const float tmp = minDeg;
        minDeg = maxDeg;
        maxDeg = tmp;
    }

    if (feedbackDeg < minDeg) return minDeg;
    if (feedbackDeg > maxDeg) return maxDeg;
    return feedbackDeg;
}

float ControlSolver::computeMcpLTendonLength(float theta1Deg, float theta2Deg) const
{
    if (!isfinite(theta1Deg)) theta1Deg = 0.0f;
    if (!isfinite(theta2Deg)) theta2Deg = 0.0f;

    const float theta1 = theta1Deg * kDegToRad;
    const float theta2 = theta2Deg * kDegToRad + kMcpThetaOffsetRad;
    const float projected = kMcpL1 + kMcpL2 * cosf(theta2);

    const float dx = cosf(theta1) * projected - kMcpL3 * sinf(theta1) - kMcpX1;
    const float dy = -kMcpL2 * sinf(theta2) - kMcpY1;
    const float dz = sinf(theta1) * projected + kMcpL3 * cosf(theta1) - kMcpZ1;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float ControlSolver::computeMcpRTendonLength(float theta1Deg, float theta2Deg) const
{
    if (!isfinite(theta1Deg)) theta1Deg = 0.0f;
    if (!isfinite(theta2Deg)) theta2Deg = 0.0f;

    const float theta1 = theta1Deg * kDegToRad;
    const float theta2 = theta2Deg * kDegToRad + kMcpThetaOffsetRad;
    const float projected = kMcpL1 + kMcpL2 * cosf(theta2);

    const float dx = cosf(theta1) * projected + kMcpL3 * sinf(theta1) - kMcpX1;
    const float dy = -kMcpL2 * sinf(theta2) - kMcpY1;
    const float dz = sinf(theta1) * projected - kMcpL3 * cosf(theta1) + kMcpZ1;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float ControlSolver::getTendonModelZeroLength(uint8_t tendonIndex) const
{
    if (tendonIndex == kMcpLTendonIndex) return computeMcpLTendonLength(0.0f, 0.0f);
    if (tendonIndex == kMcpRTendonIndex) return computeMcpRTendonLength(0.0f, 0.0f);
    return 0.0f;
}

int16_t ControlSolver::computeTendonCascadeOutput(uint8_t tendonIndex, int32_t actualMotorAbs)
{
    if (tendonIndex >= JOINT_COUNT) return 0;

    if (!_tendonControllerInitialized[tendonIndex]) {
        _tendonZeroLength[tendonIndex] = getTendonModelZeroLength(tendonIndex);
        _tendonPrevLengthError[tendonIndex] = 0.0f;
        _tendonPrevMotorError[tendonIndex] = 0.0f;
        _tendonControllerInitialized[tendonIndex] = true;
    }

    const float lengthToPulse = _tendonLengthToPulse[tendonIndex];
    if (!isfinite(lengthToPulse) || fabsf(lengthToPulse) < 1.0e-6f) {
        return clampCommand((float)actualMotorAbs);
    }

    const float lengthError = _targetTendonLength[tendonIndex] - _actualTendonLength[tendonIndex];
    const float lengthErrorDelta =
        (lengthError - _tendonPrevLengthError[tendonIndex]) / kControlPeriodSec;
    float feedbackCorrectionMm =
        _tendonKp[tendonIndex] * lengthError +
        _tendonKd[tendonIndex] * lengthErrorDelta;
    feedbackCorrectionMm = clampFloat(feedbackCorrectionMm, -0.2f, 0.2f);
    _tendonPrevLengthError[tendonIndex] = lengthError;

    const float feedforwardMm =
        _targetTendonLength[tendonIndex] - _tendonZeroLength[tendonIndex];
    float motorAbsTarget =
        (feedforwardMm + feedbackCorrectionMm) * lengthToPulse;
    motorAbsTarget = clampFloat(motorAbsTarget, -kMcpMotorAbsLimitCounts, kMcpMotorAbsLimitCounts);
    _mappedMotorTarget[tendonIndex] = motorAbsTarget;

    _tendonPrevMotorError[tendonIndex] = motorAbsTarget - (float)actualMotorAbs;
    return clampCommand(motorAbsTarget);
}

bool ControlSolver::computeDualLoopPid(float* targetDegs,
                                       float* magActualDegs,
                                       const int32_t* absolutePosition,
                                       const int32_t* motorZeroAbs,
                                       int16_t* outServoPulses)
{
    if (!targetDegs || !magActualDegs || !absolutePosition || !outServoPulses) {
        return false;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        const float motorOrigin = motorZeroAbs ? (float)motorZeroAbs[i] : (float)absolutePosition[i];
        const float motorDirection = (_directions[i] < 0) ? -1.0f : 1.0f;
        const float loop2Target = motorOrigin + motorDirection * _pids[i][0].Output;
        f_PID_Calculate(&_pids[i][1], loop2Target, (float)absolutePosition[i]);

        outServoPulses[i] = clampCommand((float)absolutePosition[i] + _pids[i][1].Output);
    }
    return true;
}

float ControlSolver::getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const
{
    if (jointIndex >= JOINT_COUNT || loopIndex > 1) return 0.0f;
    return _pids[jointIndex][loopIndex].Output;
}

float ControlSolver::getTargetTendonLength(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _targetTendonLength[jointIndex] : 0.0f;
}

float ControlSolver::getActualTendonLength(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _actualTendonLength[jointIndex] : 0.0f;
}

float ControlSolver::getMappedMotorTarget(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _mappedMotorTarget[jointIndex] : 0.0f;
}

void ControlSolver::resetAll()
{
    memset(_tendonPrevLengthError, 0, sizeof(_tendonPrevLengthError));
    memset(_tendonPrevMotorError, 0, sizeof(_tendonPrevMotorError));
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    begin();
}
