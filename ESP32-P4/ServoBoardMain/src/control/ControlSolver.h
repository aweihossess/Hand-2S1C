#ifndef CONTROL_SOLVER_H
#define CONTROL_SOLVER_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"
#include "pid.h"

// Pure calculation module. It does not access queues, the state machine, or
// hardware buses directly.
class ControlSolver {
public:
    ControlSolver();

    void begin();
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);

    // Legacy dual-loop calculation for joints outside the five-tendon group.
    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 const int32_t* motorZeroAbs,
                 int16_t* outServoPulses);

    // Five-tendon controller: identified R feedforward and actuator-space PI
    // are projected through Q=R*pinv(R), then the independent alpha*n tension
    // bias is added before finite differencing.
    bool computeTendonFeedforward(float* targetDegs,
                                  float* magActualDegs,
                                  const int32_t* absolutePosition,
                                  const int32_t* motorZeroAbs,
                                  const int32_t* tensionBiasCounts,
                                  int16_t* outServoPulses);

    float getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const;
    float getTargetTendonLength(uint8_t jointIndex) const;
    float getActualTendonLength(uint8_t jointIndex) const;
    float getTendonFirstLength(uint8_t jointIndex) const;
    float getEntryJointDeg(uint8_t jointIndex) const;
    float getMappedMotorTarget(uint8_t jointIndex) const;
    float getReferenceDeg(uint8_t jointIndex) const;
    float getFilteredFeedbackDeg(uint8_t jointIndex) const;
    float getFeedbackVelocityDegPerSec(uint8_t jointIndex) const;
    float getJointErrorIntegralDegSec(uint8_t jointIndex) const;
    float getTendonFeedforwardCounts(uint8_t tendonIndex) const;
    float getTendonAnglePCounts(uint8_t tendonIndex) const;
    float getTendonAngleICounts(uint8_t tendonIndex) const;
    float getTendonAngleDCounts(uint8_t tendonIndex) const;
    float getTendonAngleFeedbackCounts(uint8_t tendonIndex) const;

    void setTendonLengthFeedforwardEnabled(bool enabled);
    bool isTendonLengthFeedforwardEnabled() const;
    void setMcpAngleKpScale(float scale);
    float getMcpAngleKpScale() const;
    void setMcpAngleKiScale(float scale);
    float getMcpAngleKiScale() const;
    void resetAngleIntegral();
    void resetAll();

private:
    float updateTargetReference(uint8_t jointIndex, float targetDeg);
    float updateTargetReferenceFromFeedback(uint8_t jointIndex, float targetDeg, float feedbackDeg);
    float updateFeedbackFilter(uint8_t jointIndex, float feedbackDeg);
    float clampFeedbackDeg(uint8_t jointIndex, float feedbackDeg) const;
    void prepareProjectedMcpTaskCounts();
    int16_t computeTendonCascadeOutput(uint8_t tendonIndex,
                                       int32_t actualMotorAbs,
                                       int32_t tensionBiasCounts);
    bool computeDualLoopPid(float* targetDegs,
                            float* magActualDegs,
                            const int32_t* absolutePosition,
                            const int32_t* motorZeroAbs,
                            int16_t* outServoPulses);

    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    float _tendonPrevLengthError[JOINT_COUNT];
    float _tendonKp[JOINT_COUNT];
    float _tendonKd[JOINT_COUNT];
    float _targetTendonLength[JOINT_COUNT];
    float _actualTendonLength[JOINT_COUNT];
    float _tendonFirstLength[JOINT_COUNT];
    float _tendonLengthToPulse[JOINT_COUNT];
    float _tendonPrevMotorError[JOINT_COUNT];
    float _jointErrorIntegral[JOINT_COUNT];
    float _tendonMotorKp[JOINT_COUNT];
    float _tendonMotorKd[JOINT_COUNT];
    float _tendonMotorOutputLimit[JOINT_COUNT];
    float _mappedMotorTarget[JOINT_COUNT];
    float _tendonFeedforwardCounts[JOINT_COUNT];
    float _tendonAnglePCounts[JOINT_COUNT];
    float _tendonAngleICounts[JOINT_COUNT];
    float _tendonAngleDCounts[JOINT_COUNT];
    float _tendonAngleFeedbackCounts[JOINT_COUNT];
    float _qEntry[JOINT_COUNT];
    int32_t _motorEntryAbs[JOINT_COUNT];
    float _qRef[JOINT_COUNT];
    float _qRefMaxStepDeg[JOINT_COUNT];
    float _qFbLpfStage1[JOINT_COUNT];
    float _qFbFiltered[JOINT_COUNT];
    float _qFbVelocityDegPerSec[JOINT_COUNT];
    float _qFbLpfAlpha[JOINT_COUNT];
    float _qFbMinDeg[JOINT_COUNT];
    float _qFbMaxDeg[JOINT_COUNT];
    bool _qRefInitialized[JOINT_COUNT];
    bool _qFbInitialized[JOINT_COUNT];
    bool _tendonControllerInitialized[JOINT_COUNT];
    bool _entryPoseInitialized;
    bool _tendonLengthFeedforwardEnabled;
    float _mcpAngleKpScale;
    float _mcpAngleKiScale;
    PID_Info_TypeDef _pids[JOINT_COUNT][2];
    bool _initialized;
};

#endif // CONTROL_SOLVER_H
