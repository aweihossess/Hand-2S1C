#include "ControlSolver.h"
#include "ControlSolverConfig.h"
#include "TendonFeedforwardLut.h"

#include <math.h>
#include <string.h>

static const float kDefaultQRefMaxStepDeg = 2.0f;
static const float kDefaultQFbLpfAlpha = 0.25f;
static const float kDefaultQFbMinDeg = -360.0f;
static const float kDefaultQFbMaxDeg = 360.0f;
static const float kDegToRad = 0.017453292519943295f;
static const float kControlPeriodSec = 0.01f;
static const float kDefaultTendonMotorOutputLimit = 4096.0f;
static const float kMcpMotorAbsLimitCounts = 6400.0f;
static const bool kDefaultEnableTendonLengthFeedforward = true;

static const uint8_t kMcpControlledMotorCount = 5;
static const uint8_t kMcpTheta1Joint = 0;
static const uint8_t kMcpTheta2Joint = 1;
static const uint8_t kMcpTheta3Joint = 2;
static const uint8_t kMcpTheta4Joint = 3;
static const uint8_t kMcpControlledJointCount = 4;

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

static void computeEmpiricalMcpMotorCounts(const float* qDeg, float outCounts[kMcpControlledMotorCount])
{
    if (!qDeg || !outCounts) return;
    for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
        float value = 0.0f;
        for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
            const float q = isfinite(qDeg[jointIndex]) ? qDeg[jointIndex] : 0.0f;
            value += kEmpiricalMcpMotorPerDeg[tendonIndex][jointIndex] * q;
        }
        outCounts[tendonIndex] = value;
    }
}

ControlSolver::ControlSolver() :
    _entryPoseInitialized(false),
    _tendonLengthFeedforwardEnabled(kDefaultEnableTendonLengthFeedforward),
    _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_tendonPrevLengthError, 0, sizeof(_tendonPrevLengthError));
    memset(_tendonKp, 0, sizeof(_tendonKp));
    memset(_tendonKd, 0, sizeof(_tendonKd));
    memset(_targetTendonLength, 0, sizeof(_targetTendonLength));
    memset(_actualTendonLength, 0, sizeof(_actualTendonLength));
    memset(_tendonFirstLength, 0, sizeof(_tendonFirstLength));
    memset(_tendonLengthToPulse, 0, sizeof(_tendonLengthToPulse));
    memset(_tendonPrevMotorError, 0, sizeof(_tendonPrevMotorError));
    memset(_jointErrorIntegral, 0, sizeof(_jointErrorIntegral));
    memset(_jointPrevError, 0, sizeof(_jointPrevError));
    memset(_tendonMotorKp, 0, sizeof(_tendonMotorKp));
    memset(_tendonMotorKd, 0, sizeof(_tendonMotorKd));
    memset(_tendonMotorOutputLimit, 0, sizeof(_tendonMotorOutputLimit));
    memset(_mappedMotorTarget, 0, sizeof(_mappedMotorTarget));
    memset(_qEntry, 0, sizeof(_qEntry));
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
        _tendonKp[i] = 0.0f;
        _tendonKd[i] = 0.0f;
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
        qFb[i] = updateFeedbackFilter(i, magActualDegs[i]);
    }
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        qRef[i] = updateTargetReferenceFromFeedback(i, targetDegs[i], qFb[i]);
    }

    if (!computeDualLoopPid(qRef, qFb, absolutePosition, motorZeroAbs, outServoPulses)) {
        return false;
    }

    if (JOINT_COUNT > kMcpTheta4Joint) {
        for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
            _targetTendonLength[tendonIndex] = 0.0f;
            _actualTendonLength[tendonIndex] = 0.0f;
        }

        if (_tendonLengthFeedforwardEnabled) {
            float targetLengthDelta[TendonFeedforwardLut::kTendonCount] = {0.0f};
            float actualLengthDelta[TendonFeedforwardLut::kTendonCount] = {0.0f};
            if (kUseEmpiricalMcpFeedforwardCounts) {
                computeEmpiricalMcpMotorCounts(qRef, targetLengthDelta);
                computeEmpiricalMcpMotorCounts(qFb, actualLengthDelta);
            } else {
                bool targetClamped = false;
                bool actualClamped = false;
                TendonFeedforwardLut::computeLengthDeltaMm(
                    qRef[kMcpTheta1Joint],
                    qRef[kMcpTheta2Joint],
                    qRef[kMcpTheta3Joint],
                    qRef[kMcpTheta4Joint],
                    targetLengthDelta,
                    &targetClamped);
                TendonFeedforwardLut::computeLengthDeltaMm(
                    qFb[kMcpTheta1Joint],
                    qFb[kMcpTheta2Joint],
                    qFb[kMcpTheta3Joint],
                    qFb[kMcpTheta4Joint],
                    actualLengthDelta,
                    &actualClamped);
            }

            for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
                _targetTendonLength[tendonIndex] = targetLengthDelta[tendonIndex];
                _actualTendonLength[tendonIndex] = actualLengthDelta[tendonIndex];
            }
        }

        if (!_entryPoseInitialized) {
            for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
                _qEntry[jointIndex] = qFb[jointIndex];
            }
            for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
                _tendonFirstLength[tendonIndex] = _actualTendonLength[tendonIndex];
                _tendonPrevMotorError[tendonIndex] = 0.0f;
                _tendonControllerInitialized[tendonIndex] = true;
            }
            _entryPoseInitialized = true;
        }

        for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
            const float targetRelativeDeg = qRef[jointIndex] - _qEntry[jointIndex];
            const float actualRelativeDeg = qFb[jointIndex] - _qEntry[jointIndex];
            const float error = targetRelativeDeg - actualRelativeDeg;
            _jointErrorIntegral[jointIndex] += error * kControlPeriodSec;
            _jointErrorIntegral[jointIndex] = clampFloat(
                _jointErrorIntegral[jointIndex],
                -kMcpAngleIntegralLimitDegSec[jointIndex],
                kMcpAngleIntegralLimitDegSec[jointIndex]);
        }

        for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
            outServoPulses[tendonIndex] =
                computeTendonCascadeOutput(tendonIndex, absolutePosition[tendonIndex]);
        }
        for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
            const float targetRelativeDeg = qRef[jointIndex] - _qEntry[jointIndex];
            const float actualRelativeDeg = qFb[jointIndex] - _qEntry[jointIndex];
            _jointPrevError[jointIndex] = targetRelativeDeg - actualRelativeDeg;
        }
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

float ControlSolver::updateTargetReferenceFromFeedback(uint8_t jointIndex, float targetDeg, float feedbackDeg)
{
    if (jointIndex >= JOINT_COUNT) return 0.0f;
    if (!isfinite(targetDeg)) targetDeg = 0.0f;
    if (!isfinite(feedbackDeg)) feedbackDeg = targetDeg;

    if (!_qRefInitialized[jointIndex]) {
        _qRef[jointIndex] = feedbackDeg;
        _qRefInitialized[jointIndex] = true;
    }

    return updateTargetReference(jointIndex, targetDeg);
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
    const float theta2 = (theta2Deg + kMcpThetaOffsetDeg) * kDegToRad;
    const float projected = kMcpGeometryL1Mm + kMcpGeometryL2Mm * cosf(theta2);

    const float dx = cosf(theta1) * projected -
        kMcpGeometryL3Mm * sinf(theta1) - kMcpGeometryX1Mm;
    const float dy = -kMcpGeometryL2Mm * sinf(theta2) - kMcpGeometryY1Mm;
    const float dz = sinf(theta1) * projected +
        kMcpGeometryL3Mm * cosf(theta1) - kMcpGeometryZ1Mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float ControlSolver::computeMcpRTendonLength(float theta1Deg, float theta2Deg) const
{
    if (!isfinite(theta1Deg)) theta1Deg = 0.0f;
    if (!isfinite(theta2Deg)) theta2Deg = 0.0f;

    const float theta1 = theta1Deg * kDegToRad;
    const float theta2 = (theta2Deg + kMcpThetaOffsetDeg) * kDegToRad;
    const float projected = kMcpGeometryL1Mm + kMcpGeometryL2Mm * cosf(theta2);

    const float dx = cosf(theta1) * projected +
        kMcpGeometryL3Mm * sinf(theta1) - kMcpGeometryX1Mm;
    const float dy = -kMcpGeometryL2Mm * sinf(theta2) - kMcpGeometryY1Mm;
    const float dz = sinf(theta1) * projected -
        kMcpGeometryL3Mm * cosf(theta1) + kMcpGeometryZ1Mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float ControlSolver::computeMcpCTendonLength(float theta1Deg, float theta2Deg) const
{
    if (!isfinite(theta1Deg)) theta1Deg = 0.0f;
    if (!isfinite(theta2Deg)) theta2Deg = 0.0f;

    const float theta1 = theta1Deg * kDegToRad;
    const float theta2 = (theta2Deg + kMcpTheta4Deg) * kDegToRad;
    const float projected = kMcpGeometryL1Mm + kMcpGeometryL4Mm * cosf(theta2);

    const float dx = cosf(theta1) * projected - kMcpGeometryX3Mm;
    const float dy = -kMcpGeometryL4Mm * sinf(theta2) - kMcpGeometryY3Mm;
    const float dz = sinf(theta1) * projected - kMcpGeometryZ3Mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

int16_t ControlSolver::computeTendonCascadeOutput(uint8_t tendonIndex, int32_t actualMotorAbs)
{
    if (tendonIndex >= JOINT_COUNT) return 0;

    if (!_tendonControllerInitialized[tendonIndex]) {
        _tendonFirstLength[tendonIndex] = _actualTendonLength[tendonIndex];
        _tendonPrevMotorError[tendonIndex] = 0.0f;
        _tendonControllerInitialized[tendonIndex] = true;
    }

    const float lengthToPulse = _tendonLengthToPulse[tendonIndex];
    if (!isfinite(lengthToPulse) || fabsf(lengthToPulse) < 1.0e-6f) {
        return clampCommand((float)actualMotorAbs);
    }

    float feedforwardCounts = 0.0f;
    if (_tendonLengthFeedforwardEnabled) {
        if (kUseEmpiricalMcpFeedforwardCounts && tendonIndex < kMcpControlledMotorCount) {
            feedforwardCounts = _targetTendonLength[tendonIndex] - _tendonFirstLength[tendonIndex];
        } else {
            const float feedforwardMm =
                _targetTendonLength[tendonIndex] - _tendonFirstLength[tendonIndex];
            feedforwardCounts = feedforwardMm * lengthToPulse;
        }
    }

    float angleFeedbackCounts = 0.0f;
    for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
        const float targetRelativeDeg = _qRef[jointIndex] - _qEntry[jointIndex];
        const float actualRelativeDeg = _qFbFiltered[jointIndex] - _qEntry[jointIndex];
        const float error = targetRelativeDeg - actualRelativeDeg;
        const float errorDelta = (error - _jointPrevError[jointIndex]) / kControlPeriodSec;
        angleFeedbackCounts +=
            kMcpAngleKp[tendonIndex][jointIndex] * error +
            kMcpAngleKi[tendonIndex][jointIndex] * _jointErrorIntegral[jointIndex] +
            kMcpAngleKd[tendonIndex][jointIndex] * errorDelta;
    }
    angleFeedbackCounts = clampFloat(angleFeedbackCounts,
                                     -kMcpAngleFeedbackLimitCounts[tendonIndex],
                                     kMcpAngleFeedbackLimitCounts[tendonIndex]);

    const float baseMotorTarget = _tendonLengthFeedforwardEnabled
        ? feedforwardCounts
        : (float)actualMotorAbs;
    float motorAbsTarget = baseMotorTarget + angleFeedbackCounts;
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

float ControlSolver::getTendonFirstLength(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _tendonFirstLength[jointIndex] : 0.0f;
}

float ControlSolver::getEntryJointDeg(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _qEntry[jointIndex] : 0.0f;
}

float ControlSolver::getMappedMotorTarget(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _mappedMotorTarget[jointIndex] : 0.0f;
}

void ControlSolver::setTendonLengthFeedforwardEnabled(bool enabled)
{
    if (_tendonLengthFeedforwardEnabled == enabled) {
        return;
    }
    _tendonLengthFeedforwardEnabled = enabled;
    memset(_jointErrorIntegral, 0, sizeof(_jointErrorIntegral));
    memset(_jointPrevError, 0, sizeof(_jointPrevError));
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    memset(_qEntry, 0, sizeof(_qEntry));
    _entryPoseInitialized = false;
}

bool ControlSolver::isTendonLengthFeedforwardEnabled() const
{
    return _tendonLengthFeedforwardEnabled;
}

void ControlSolver::resetAll()
{
    memset(_tendonPrevLengthError, 0, sizeof(_tendonPrevLengthError));
    memset(_tendonPrevMotorError, 0, sizeof(_tendonPrevMotorError));
    memset(_jointErrorIntegral, 0, sizeof(_jointErrorIntegral));
    memset(_jointPrevError, 0, sizeof(_jointPrevError));
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    memset(_qEntry, 0, sizeof(_qEntry));
    _entryPoseInitialized = false;
    _tendonLengthFeedforwardEnabled = kDefaultEnableTendonLengthFeedforward;
    begin();
}
