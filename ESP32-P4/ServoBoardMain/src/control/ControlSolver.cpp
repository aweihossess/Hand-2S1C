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
// The joint-mode safety envelope is relative to the motor positions captured
// when the controller starts.  Keep only the physical/protocol absolute limit
// here so a valid pre-tensioned pose above the old fixed +/-9600 range is not
// pulled back toward zero by the solver.
static const float kMcpMotorAbsLimitCounts = 30719.0f;
static const bool kDefaultEnableTendonLengthFeedforward = true;

static const uint8_t kMcpControlledMotorCount = 5;
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

ControlSolver::ControlSolver() :
    _entryPoseInitialized(false),
    _tendonLengthFeedforwardEnabled(kDefaultEnableTendonLengthFeedforward),
    _mcpAngleKpScale(kDefaultMcpAngleKpScale),
    _mcpAngleKiScale(kDefaultMcpAngleKiScale),
    _mcpFeedforwardRBlend(0.0f),
    _mcpFeedbackPBlend(0.0f),
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
    memset(_tendonMotorKp, 0, sizeof(_tendonMotorKp));
    memset(_tendonMotorKd, 0, sizeof(_tendonMotorKd));
    memset(_tendonMotorOutputLimit, 0, sizeof(_tendonMotorOutputLimit));
    memset(_mappedMotorTarget, 0, sizeof(_mappedMotorTarget));
    memset(_tendonFeedforwardCounts, 0, sizeof(_tendonFeedforwardCounts));
    memset(_tendonAnglePCounts, 0, sizeof(_tendonAnglePCounts));
    memset(_tendonAngleICounts, 0, sizeof(_tendonAngleICounts));
    memset(_tendonAngleDCounts, 0, sizeof(_tendonAngleDCounts));
    memset(_tendonAngleFeedbackCounts, 0, sizeof(_tendonAngleFeedbackCounts));
    memset(_qEntry, 0, sizeof(_qEntry));
    memset(_motorEntryAbs, 0, sizeof(_motorEntryAbs));
    memset(_qRef, 0, sizeof(_qRef));
    memset(_qRefMaxStepDeg, 0, sizeof(_qRefMaxStepDeg));
    memset(_qFbLpfStage1, 0, sizeof(_qFbLpfStage1));
    memset(_qFbFiltered, 0, sizeof(_qFbFiltered));
    memset(_qFbVelocityDegPerSec, 0, sizeof(_qFbVelocityDegPerSec));
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

        _qRefMaxStepDeg[i] =
            (i < kMcpControlledJointCount) ? kMcpReferenceMaxStepDeg : kDefaultQRefMaxStepDeg;
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
                                             const int32_t* tensionBiasCounts,
                                             int16_t* outServoPulses)
{
    if (!targetDegs || !magActualDegs || !absolutePosition || !motorZeroAbs ||
        !tensionBiasCounts || !outServoPulses) {
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

    if (JOINT_COUNT >= kMcpControlledJointCount) {
        for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
            _targetTendonLength[tendonIndex] = 0.0f;
            _actualTendonLength[tendonIndex] = 0.0f;
            _tendonFirstLength[tendonIndex] = 0.0f;
        }

        if (!_entryPoseInitialized) {
            for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
                _qEntry[jointIndex] = qFb[jointIndex];
            }
            for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
                _motorEntryAbs[tendonIndex] = absolutePosition[tendonIndex];
                _tendonPrevMotorError[tendonIndex] = 0.0f;
                _tendonControllerInitialized[tendonIndex] = true;
            }
            _entryPoseInitialized = true;
        }

        // Integrate only after the finite-step reference has reached the final
        // requested target.  This keeps I from fighting the reference ramp.
        bool feedbackValid = true;
        bool referenceSettled = true;
        for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
            if (!isfinite(magActualDegs[jointIndex])) {
                feedbackValid = false;
            }
            if (fabsf(targetDegs[jointIndex] - qRef[jointIndex]) > 0.001f) {
                referenceSettled = false;
            }
        }
        if (_mcpAngleKiScale > 0.0f && feedbackValid && referenceSettled) {
            for (uint8_t jointIndex = 0; jointIndex < kMcpControlledJointCount; jointIndex++) {
                float errorDeg = qRef[jointIndex] - qFb[jointIndex];
                if (fabsf(errorDeg) <= kMcpAngleIntegralDeadbandDeg) {
                    errorDeg = 0.0f;
                }
                _jointErrorIntegral[jointIndex] += errorDeg * kControlPeriodSec;
                if (!isfinite(_jointErrorIntegral[jointIndex])) {
                    _jointErrorIntegral[jointIndex] = 0.0f;
                }
            }
        }

        // Build the five-channel feedforward and PI task vectors first, then
        // project both through Q=R*pinv(R).  The null-space tension bias is not
        // part of this calculation; it is added later to the absolute target.
        prepareProjectedMcpTaskCounts();

        for (uint8_t tendonIndex = 0; tendonIndex < kMcpControlledMotorCount; tendonIndex++) {
            outServoPulses[tendonIndex] =
                computeTendonCascadeOutput(
                    tendonIndex,
                    absolutePosition[tendonIndex],
                    tensionBiasCounts[tendonIndex]
                );
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
        _qFbVelocityDegPerSec[jointIndex] = 0.0f;
        _qFbInitialized[jointIndex] = true;
    } else {
        const float previousFilteredDeg = _qFbFiltered[jointIndex];
        float alpha = _qFbLpfAlpha[jointIndex];
        if (!isfinite(alpha)) alpha = kDefaultQFbLpfAlpha;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        _qFbLpfStage1[jointIndex] += alpha * (feedbackDeg - _qFbLpfStage1[jointIndex]);
        _qFbFiltered[jointIndex] += alpha * (_qFbLpfStage1[jointIndex] - _qFbFiltered[jointIndex]);
        _qFbFiltered[jointIndex] = clampFeedbackDeg(jointIndex, _qFbFiltered[jointIndex]);
        _qFbVelocityDegPerSec[jointIndex] =
            (_qFbFiltered[jointIndex] - previousFilteredDeg) / kControlPeriodSec;
        return _qFbFiltered[jointIndex];
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

void ControlSolver::prepareProjectedMcpTaskCounts()
{
    float rawFeedforwardCounts[kMcpControlledMotorCount] = {0.0f};
    float rawAnglePCounts[kMcpControlledMotorCount] = {0.0f};
    float rawAngleICounts[kMcpControlledMotorCount] = {0.0f};

    // First form the unprojected actuator-space task vectors.  Feedforward is
    // already in range(R), while P feedback can contain a component along the
    // one-dimensional internal-tension direction.
    for (uint8_t sourceTendon = 0;
         sourceTendon < kMcpControlledMotorCount;
         sourceTendon++) {
        if (_tendonLengthFeedforwardEnabled) {
            for (uint8_t jointIndex = 0;
                 jointIndex < kMcpControlledJointCount;
                 jointIndex++) {
                const float referenceDeltaDeg =
                    _qRef[jointIndex] - _qEntry[jointIndex];
                const float referenceDeltaRad = referenceDeltaDeg * kDegToRad;
                const float rElement =
                    (1.0f - _mcpFeedforwardRBlend) *
                        kFittedMcpR[sourceTendon][jointIndex] +
                    _mcpFeedforwardRBlend *
                        kLocalIdentifiedMcpMap[sourceTendon][jointIndex];
                rawFeedforwardCounts[sourceTendon] +=
                    kMcpCountsPerMm *
                    rElement *
                    referenceDeltaRad;
            }
        }

        for (uint8_t jointIndex = 0;
             jointIndex < kMcpControlledJointCount;
             jointIndex++) {
            const float errorDeg = _qRef[jointIndex] - _qFbFiltered[jointIndex];
            const float errorRad = errorDeg * kDegToRad;
            const float pElement =
                (1.0f - _mcpFeedbackPBlend) *
                    kMcpAnglePBase[sourceTendon][jointIndex] +
                _mcpFeedbackPBlend *
                    kLocalIdentifiedMcpMap[sourceTendon][jointIndex];
            rawAnglePCounts[sourceTendon] +=
                kMcpCountsPerMm *
                _mcpAngleKpScale *
                pElement *
                errorRad;

            const float integralRadSec =
                _jointErrorIntegral[jointIndex] * kDegToRad;
            rawAngleICounts[sourceTendon] +=
                kMcpCountsPerMm *
                _mcpAngleKiScale *
                pElement *
                integralRadSec;
        }
    }

    float projectedFeedforwardCounts[kMcpControlledMotorCount] = {0.0f};
    float projectedAnglePCounts[kMcpControlledMotorCount] = {0.0f};
    float projectedAngleICounts[kMcpControlledMotorCount] = {0.0f};

    // Q removes any task-controller contribution in null(R^T).  The explicit
    // alpha*n bias is intentionally absent from this projection.
    for (uint8_t tendonIndex = 0;
         tendonIndex < kMcpControlledMotorCount;
         tendonIndex++) {
        for (uint8_t sourceTendon = 0;
             sourceTendon < kMcpControlledMotorCount;
             sourceTendon++) {
            const float qElement =
                kMcpTaskProjector[tendonIndex][sourceTendon];
            projectedFeedforwardCounts[tendonIndex] +=
                qElement * rawFeedforwardCounts[sourceTendon];
            projectedAnglePCounts[tendonIndex] +=
                qElement * rawAnglePCounts[sourceTendon];
            projectedAngleICounts[tendonIndex] +=
                qElement * rawAngleICounts[sourceTendon];
        }
    }

    // A separate per-motor clamp would bend the five-channel vector out of
    // range(R) and recreate a null-space component.  Use one common scale for
    // all five PI channels instead; this preserves the projected direction
    // while respecting every motor's existing PI count limit.
    float piScale = 1.0f;
    for (uint8_t tendonIndex = 0;
         tendonIndex < kMcpControlledMotorCount;
         tendonIndex++) {
        const float combinedPiCounts =
            projectedAnglePCounts[tendonIndex] +
            projectedAngleICounts[tendonIndex];
        const float piLimitCounts =
            kMcpAngleFeedbackLimitCounts[tendonIndex];
        if (piLimitCounts > 0.0f &&
            fabsf(combinedPiCounts) > piLimitCounts) {
            const float channelScale =
                piLimitCounts / fabsf(combinedPiCounts);
            if (channelScale < piScale) {
                piScale = channelScale;
            }
        }
    }

    for (uint8_t tendonIndex = 0;
         tendonIndex < kMcpControlledMotorCount;
         tendonIndex++) {
        const float limitedAnglePCounts =
            piScale * projectedAnglePCounts[tendonIndex];
        const float limitedAngleICounts =
            piScale * projectedAngleICounts[tendonIndex];
        _tendonFeedforwardCounts[tendonIndex] =
            projectedFeedforwardCounts[tendonIndex];
        _tendonAnglePCounts[tendonIndex] = limitedAnglePCounts;
        _tendonAngleICounts[tendonIndex] = limitedAngleICounts;
        _tendonAngleDCounts[tendonIndex] = 0.0f;
        _tendonAngleFeedbackCounts[tendonIndex] =
            limitedAnglePCounts + limitedAngleICounts;
        _targetTendonLength[tendonIndex] =
            projectedFeedforwardCounts[tendonIndex] / kMcpCountsPerMm;
        _actualTendonLength[tendonIndex] = 0.0f;
        _tendonFirstLength[tendonIndex] = 0.0f;
    }
}

int16_t ControlSolver::computeTendonCascadeOutput(uint8_t tendonIndex,
                                                  int32_t actualMotorAbs,
                                                  int32_t tensionBiasCounts)
{
    if (tendonIndex >= JOINT_COUNT) return 0;

    if (!_tendonControllerInitialized[tendonIndex]) {
        _tendonPrevMotorError[tendonIndex] = 0.0f;
        _tendonControllerInitialized[tendonIndex] = true;
    }

    const float feedforwardCounts = _tendonFeedforwardCounts[tendonIndex];
    const float angleFeedbackCounts =
        _tendonAngleFeedbackCounts[tendonIndex];

    // Keep every position contribution in one absolute target.  In particular,
    // the null-space tension bias must be included before finite differencing.
    // Adding it after this stage would re-apply the full bias on every 10 ms
    // cycle and turn a fixed co-contraction position into a runaway rate input.
    const float requestedMotorAbs = (float)_motorEntryAbs[tendonIndex] +
                                    feedforwardCounts +
                                    angleFeedbackCounts +
                                    (float)tensionBiasCounts;
    const float requestedMotorDeltaCounts = requestedMotorAbs - (float)actualMotorAbs;
    float maxMotorStepCounts = _tendonMotorOutputLimit[tendonIndex];
    if (!isfinite(maxMotorStepCounts) || maxMotorStepCounts <= 0.0f) {
        maxMotorStepCounts = kDefaultTendonMotorOutputLimit;
    }
    const float limitedMotorDeltaCounts = clampFloat(
        requestedMotorDeltaCounts,
        -maxMotorStepCounts,
        maxMotorStepCounts
    );
    float motorAbsTarget = (float)actualMotorAbs + limitedMotorDeltaCounts;
    motorAbsTarget = clampFloat(motorAbsTarget, -kMcpMotorAbsLimitCounts, kMcpMotorAbsLimitCounts);
    _mappedMotorTarget[tendonIndex] = motorAbsTarget;

    _tendonPrevMotorError[tendonIndex] = limitedMotorDeltaCounts;
    return clampCommand(motorAbsTarget);
}

void ControlSolver::setMcpAngleKpScale(float scale)
{
    if (!isfinite(scale)) {
        return;
    }
    _mcpAngleKpScale = clampFloat(scale, 0.0f, 5.0f);
}

float ControlSolver::getMcpAngleKpScale() const
{
    return _mcpAngleKpScale;
}

void ControlSolver::setMcpAngleKiScale(float scale)
{
    if (!isfinite(scale)) {
        return;
    }
    const float nextScale = clampFloat(scale, 0.0f, 1.0f);
    if (fabsf(nextScale - _mcpAngleKiScale) > 0.0000001f) {
        _mcpAngleKiScale = nextScale;
        resetAngleIntegral();
    }
}

float ControlSolver::getMcpAngleKiScale() const
{
    return _mcpAngleKiScale;
}

void ControlSolver::setMcpFeedforwardRBlend(float blend)
{
    if (!isfinite(blend)) return;
    _mcpFeedforwardRBlend = clampFloat(blend, 0.0f, 1.0f);
}

float ControlSolver::getMcpFeedforwardRBlend() const
{
    return _mcpFeedforwardRBlend;
}

void ControlSolver::setMcpFeedbackPBlend(float blend)
{
    if (!isfinite(blend)) return;
    _mcpFeedbackPBlend = clampFloat(blend, 0.0f, 1.0f);
}

float ControlSolver::getMcpFeedbackPBlend() const
{
    return _mcpFeedbackPBlend;
}

void ControlSolver::resetAngleIntegral()
{
    memset(_jointErrorIntegral, 0, sizeof(_jointErrorIntegral));
    memset(_tendonAngleICounts, 0, sizeof(_tendonAngleICounts));
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

float ControlSolver::getReferenceDeg(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _qRef[jointIndex] : 0.0f;
}

float ControlSolver::getFilteredFeedbackDeg(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _qFbFiltered[jointIndex] : 0.0f;
}

float ControlSolver::getFeedbackVelocityDegPerSec(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _qFbVelocityDegPerSec[jointIndex] : 0.0f;
}

float ControlSolver::getJointErrorIntegralDegSec(uint8_t jointIndex) const
{
    return jointIndex < JOINT_COUNT ? _jointErrorIntegral[jointIndex] : 0.0f;
}

float ControlSolver::getTendonFeedforwardCounts(uint8_t tendonIndex) const
{
    return tendonIndex < JOINT_COUNT ? _tendonFeedforwardCounts[tendonIndex] : 0.0f;
}

float ControlSolver::getTendonAnglePCounts(uint8_t tendonIndex) const
{
    return tendonIndex < JOINT_COUNT ? _tendonAnglePCounts[tendonIndex] : 0.0f;
}

float ControlSolver::getTendonAngleICounts(uint8_t tendonIndex) const
{
    return tendonIndex < JOINT_COUNT ? _tendonAngleICounts[tendonIndex] : 0.0f;
}

float ControlSolver::getTendonAngleDCounts(uint8_t tendonIndex) const
{
    return tendonIndex < JOINT_COUNT ? _tendonAngleDCounts[tendonIndex] : 0.0f;
}

float ControlSolver::getTendonAngleFeedbackCounts(uint8_t tendonIndex) const
{
    return tendonIndex < JOINT_COUNT ? _tendonAngleFeedbackCounts[tendonIndex] : 0.0f;
}

void ControlSolver::setTendonLengthFeedforwardEnabled(bool enabled)
{
    if (_tendonLengthFeedforwardEnabled == enabled) {
        return;
    }
    _tendonLengthFeedforwardEnabled = enabled;
    memset(_jointErrorIntegral, 0, sizeof(_jointErrorIntegral));
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    memset(_qFbVelocityDegPerSec, 0, sizeof(_qFbVelocityDegPerSec));
    memset(_qEntry, 0, sizeof(_qEntry));
    memset(_motorEntryAbs, 0, sizeof(_motorEntryAbs));
    memset(_tendonFeedforwardCounts, 0, sizeof(_tendonFeedforwardCounts));
    memset(_tendonAnglePCounts, 0, sizeof(_tendonAnglePCounts));
    memset(_tendonAngleICounts, 0, sizeof(_tendonAngleICounts));
    memset(_tendonAngleDCounts, 0, sizeof(_tendonAngleDCounts));
    memset(_tendonAngleFeedbackCounts, 0, sizeof(_tendonAngleFeedbackCounts));
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
    memset(_tendonControllerInitialized, 0, sizeof(_tendonControllerInitialized));
    memset(_qRefInitialized, 0, sizeof(_qRefInitialized));
    memset(_qFbInitialized, 0, sizeof(_qFbInitialized));
    memset(_qFbVelocityDegPerSec, 0, sizeof(_qFbVelocityDegPerSec));
    memset(_qEntry, 0, sizeof(_qEntry));
    memset(_motorEntryAbs, 0, sizeof(_motorEntryAbs));
    memset(_tendonFeedforwardCounts, 0, sizeof(_tendonFeedforwardCounts));
    memset(_tendonAnglePCounts, 0, sizeof(_tendonAnglePCounts));
    memset(_tendonAngleICounts, 0, sizeof(_tendonAngleICounts));
    memset(_tendonAngleDCounts, 0, sizeof(_tendonAngleDCounts));
    memset(_tendonAngleFeedbackCounts, 0, sizeof(_tendonAngleFeedbackCounts));
    _entryPoseInitialized = false;
    _tendonLengthFeedforwardEnabled = kDefaultEnableTendonLengthFeedforward;
    begin();
}
