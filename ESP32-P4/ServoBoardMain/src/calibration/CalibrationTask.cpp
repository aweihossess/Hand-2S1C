#include "CalibrationTask.h"

#include <string.h>

#include "JointCalibrationProfile.h"

/*
 * CalibrationTask 模块职责说明：
 * 1) 管理每个关节的标定参数与自动/手动标定结果。
 * 2) 实现单关节自动标定主流程（通过负载收紧判断机械限位）。
 * 3) 测试阶段支持手工注入标定结果，但本模块不作为常驻任务。
 *
 * 源数据流转约束如下：
 * - 编码器原始值：来自 canRxQueue（RemoteSensorData_t）。
 * - 舵机运动与反馈：通过 ServoCommunicationTask 的队列或快照交互。
 * - 标定相关生命周期及 UI 状态由 StateMachineTask 统一管理。
 */

// 关节映射、关节16副舵机信息（外部定义）
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem joint16SecondaryMotor;

// 全局：每个关节的标定参数、标定结果存储
JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// g_encoderDirection：编码器安装方向表，+1正向、-1反向。所有原始角度需先经此变换，确保正负方向统一。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, -1, -1,
    1,  1, -1, -1, 1
};

// 手动回退编码器Offset表。自动标定失败或不可用时，控制任务用作零位偏移。
int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM] = {0};

// Manual servo absolute zero table in motorMap channel order.
// Joint-angle control uses these lower-controller-owned values as the motor
// command origin. Fill them with servo absolute positions measured at the same
// mechanical zero pose as kManualZeroRaw.
int32_t g_motorZeroAbsManual[SERVO_TOTAL_NUM] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0
};

// 手动零位 raw（来自磁编码器原始反馈，范围 0~16383）。
// 这是给现场调试预留的固定参数位：将每个关节在“机械零位”时的 raw 填到这里。
// -1 表示该关节未配置，默认回退为 0。
static const int32_t kManualZeroRaw[ENCODER_TOTAL_NUM] = {
    4500, 1960, 6700, 13000,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1, -1
};

// Joint16（通常为手爪/负载较大关节）相关常数与双舵机默认约束
static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffsetDefault = 0;
static const int32_t kJoint16TensionBiasDefault = 0;

// Joint16双舵机约束配置结构体（主：bus3-id17，副：bus3-id18，偏置为0）
JointDualServoConstraint g_joint16DualServoConstraint = {
    1, // enabled
    kJoint16Index,
    3, 17,
    3, 18,
    kJoint16SecondaryOffsetDefault,
    kJoint16TensionBiasDefault
};

// 手工机械限位原始表（测试/开发用）
// 填入Mag编码器原始值，方向变换在下方实现
// -1表示该关节未配置该端点。0为有效raw
static const uint8_t kTestJointCount = ENCODER_TOTAL_NUM;
static const int32_t kManualEncoderMinRaw[kTestJointCount] = {
    11000, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    5766, 6586, 6248, 12130,
    -1, -1, -1, -1, -1
};
static const int32_t kManualEncoderMaxRaw[kTestJointCount] = {
    4800, 12636, 9174, 12822,
    12440, 9738, 3500, 1232,
    3339, 2425, 12643, 6464,
    9061, 3422, 2408, 1457,
    -1, 6542, 6480, 10115, 6535
};

// 编码器计数一圈的模
static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;

/**
 * 编码器原始值方向归一化
 * @param rawValue 原始编码器值
 * @param direction +1/-1
 * @return 归一化方向后[0, modulo)区间
 */
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction) {
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

/**
 * 校验手填的原始编码器数值是否合法
 */
static bool isManualEncoderRawValid(int32_t rawValue) {
    return rawValue >= 0 && rawValue < kEncoderModulo;
}

/**
 * 编码器计数取模归一化（标准环绕处理）
 */
static int32_t wrapEncoderCount(int32_t value) {
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

/**
 * 计算起点到终点的正向环绕差值（如机械限位区间长度）
 */
static int32_t forwardEncoderDelta(int32_t from, int32_t to) {
    return wrapEncoderCount(to - from);
}

/**
 * 编码器计数转角度
 */
static float encoderCountToDeg(int32_t count) {
    return (float)count * 360.0f / (float)kEncoderModulo;
}

/**
 * 角度转编码器计数
 */
static int32_t degToEncoderCount(float deg) {
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

/**
 * 舵机绝对位置安全限幅，规避协议/硬件保护
 */
static int16_t clampToServoPos(int32_t value) {
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

/**
 * 获取指定关节的双舵机约束。
 * @return 成功（仅对joint16有效）则返回true且out填充配置，否则false。
 */
bool getJointDualServoConstraint(uint8_t jointIndex, JointDualServoConstraint* out) {
    if (!out) {
        return false;
    }
    if (g_joint16DualServoConstraint.enabled != 0 &&
        g_joint16DualServoConstraint.jointIndex == jointIndex) {
        *out = g_joint16DualServoConstraint;
        return true;
    }
    return false;
}

/**
 * 根据主舵机目标推导副舵机目标（用于拮抗结构）
 * secondary = -primary + secondaryOffset + tensionBias
 */
bool computeSecondaryServoTargetForJoint(uint8_t jointIndex,
                                         int32_t primaryTarget,
                                         int16_t* outSecondaryTarget) {
    if (!outSecondaryTarget) {
        return false;
    }
    JointDualServoConstraint cfg;
    if (!getJointDualServoConstraint(jointIndex, &cfg)) {
        return false;
    }
    // 按拮抗算法计算副舵机目标，并限幅
    const int32_t secondaryTarget = -primaryTarget + cfg.secondaryOffset + cfg.tensionBias;
    *outSecondaryTarget = clampToServoPos(secondaryTarget);
    return true;
}

/**
 * 计算目标与当前编码器差值，并通过环绕方式归一到最短路径
 */
static int32_t wrapEncoderDelta(int32_t target, int32_t actual) {
    int32_t delta = target - actual;
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

/**
 * 从CAN最新快照读取关节编码器值，并变换统一方向
 * @return 成功则outEncoder更新并返回true，失败返回false
 */
static bool readLatestEncoderRaw(TaskSharedData_t* sharedData, uint8_t jointIndex, int32_t* outEncoder) {
    if (!sharedData || !outEncoder || jointIndex >= ENCODER_TOTAL_NUM) return false;
    RemoteSensorData_t sensorData;
    if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE) return false;
    if (!sensorData.isValid) return false;
    if (sensorData.errorFlags[jointIndex]) return false;
    *outEncoder = orientEncoderRaw(sensorData.encoderValues[jointIndex], g_encoderDirection[jointIndex]);
    return true;
}

/**
 * 根据bus/id查找motorMap数组对应通道（用于反馈数组下标）
 */
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

/**
 * 读取最新舵机反馈绝对位置（角度），不直接操作舵机总线。
 */
static bool readLatestServoAbs(TaskSharedData_t* sharedData, uint8_t bus, uint8_t id, int32_t* outAbs)
{
    if (!sharedData || !outAbs) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }
    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue || xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE) {
        return false;
    }
    if (servoData.onlineStatus[ch] == 0) {
        return false;
    }
    *outAbs = servoData.servoAngles[ch];
    return true;
}

/**
 * 读取最新舵机负载（用于判断是否碰到机械限位边界）
 */
static bool readLatestServoLoad(TaskSharedData_t* sharedData, uint8_t bus, uint8_t id, int16_t* outLoad)
{
    if (!sharedData || !outLoad) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }
    ServoTelemetryData_t telemetry;
    QueueHandle_t queue = sharedData->servoTelemetrySnapshotQueue ? sharedData->servoTelemetrySnapshotQueue : sharedData->servoTelemetryQueue;
    if (!queue || xQueuePeek(queue, &telemetry, 0) != pdTRUE) {
        return false;
    }
    if (telemetry.onlineStatus[ch] == 0) {
        return false;
    }
    *outLoad = telemetry.load[ch];
    return true;
}

/**
 * 将舵机控制目标追加到控制批次（用于Batch控制下发）
 */
static bool appendServoTarget(ServoTargetBatch_t* batch,
                              uint8_t busIndex,
                              uint8_t servoId,
                              int16_t position,
                              uint16_t speed,
                              uint8_t acc)
{
    // 校准任务通过主队列下发批量目标，队列由ServoCommunicationTask执行。
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX || busIndex >= NUM_BUSES) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.busIndex = busIndex;
    cmd.servoId = servoId;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

/**
 * 批量下发关节舵机目标（兼容双舵机情况）
 */
static bool queueJointTarget(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             int16_t primaryTarget,
                             const JointCalibrationConfig& cfg)
{
    // 只有owner为CALIBRATION时允许下发目标，且目标带上校准标识
    if (!sharedData || !sharedData->servoTargetQueue || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.timestamp = millis();
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    appendServoTarget(&batch,
                      jointMap[jointIndex].busIndex,
                      jointMap[jointIndex].servoID,
                      primaryTarget,
                      cfg.tightenSpeed,
                      cfg.tightenAcc);

    // 若关节为双舵机结构，推导副舵机目标一同下发
    JointDualServoConstraint dualCfg;
    int16_t secondaryTarget = 0;
    if (getJointDualServoConstraint(jointIndex, &dualCfg) &&
        computeSecondaryServoTargetForJoint(jointIndex, primaryTarget, &secondaryTarget)) {
        appendServoTarget(&batch,
                          dualCfg.secondaryBusIndex,
                          dualCfg.secondaryServoID,
                          secondaryTarget,
                          cfg.tightenSpeed,
                          cfg.tightenAcc);
    }

    if (batch.count == 0) {
        return false;
    }
    if (sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
        return false;
    }
    return xQueueOverwrite(sharedData->servoTargetQueue, &batch) == pdTRUE;
}

/**
 * 标定动作完成后，回退关节至angleReserved，避免长停在机械限位。
 * 尝试3秒内达到预留角度，过期/反馈失效直接失败。
 * @return 执行成功true，超时/无反馈false（不影响已保存的主标定结果）
 */
static bool moveJointToReserved(TaskSharedData_t* sharedData,
                                uint8_t jointIndex,
                                const JointCalibrationConfig& cfg,
                                int32_t offset) {
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) return false;

    const int32_t targetEncoder = offset + degToEncoderCount(cfg.angleReserved);
    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    const uint32_t beginMs = millis();

    while (millis() - beginMs < 3000) {
        int32_t currentEncoder = 0;
        if (!readLatestEncoderRaw(sharedData, jointIndex, &currentEncoder)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int32_t encoderErr = wrapEncoderDelta(targetEncoder, currentEncoder);
        if (abs((int)encoderErr) <= 6) {
            return true;
        }

        int32_t currentServoPos = 0;
        if (!readLatestServoAbs(sharedData, bus, servoId, &currentServoPos)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int32_t step = encoderErr / 8;
        if (step == 0) step = (encoderErr > 0) ? 1 : -1;

        int32_t stepLimit = (int32_t)abs((int)cfg.tightenStep) * 3;
        if (stepLimit < 1) stepLimit = 1;
        if (step > stepLimit) step = stepLimit;
        if (step < -stepLimit) step = -stepLimit;

        const int16_t targetPos = clampToServoPos(currentServoPos + step);
        queueJointTarget(sharedData, jointIndex, targetPos, cfg);
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));
    }

    return false;
}

/**
 * 装载默认标定参数（通常从profile表填充到用户配置）
 */
void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count) {
    loadJointCalibrationProfile(cfg, count);
}

/**
 * 测试模式下，加载手工min/max限位表以模拟机械范围
 * 操作步骤：
 * 1) 装载应用场景默认配置
 * 2) 按方向预处理手工minraw/maxraw表
 * 3) 推导angleScope/offset/angleMax等关节参数
 * 4) 可用结果写入手动回退表
 * 5) 标定主流程保持IDLE, 不自动运行/写回
 */
void initManualCalibrationForTest(void) {
    initDefaultCalibrationConfig(g_jointCalibConfig, ENCODER_TOTAL_NUM);
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_encoderOffsetManual, 0, sizeof(g_encoderOffsetManual));

    for (uint8_t i = 0; i < kTestJointCount; i++) {
        JointCalibrationConfig& cfg = g_jointCalibConfig[i];
        JointCalibrationResult& out = g_jointCalibResult[i];
        const int32_t rawEncoderMin = kManualEncoderMinRaw[i];
        const int32_t rawEncoderMax = kManualEncoderMaxRaw[i];
        const int32_t rawZero = kManualZeroRaw[i];

        // 先校验表项（均为CAN原始值），之后变换方向并计算机械区间
        const bool rawValid =
            isManualEncoderRawValid(rawEncoderMin) &&
            isManualEncoderRawValid(rawEncoderMax);
        const int32_t encoderMin = rawValid
            ? orientEncoderRaw((uint16_t)rawEncoderMin, g_encoderDirection[i])
            : 0;
        const int32_t encoderMax = rawValid
            ? orientEncoderRaw((uint16_t)rawEncoderMax, g_encoderDirection[i])
            : 0;
        const int32_t derivedAngleScopeCount = rawValid
            ? forwardEncoderDelta(encoderMin, encoderMax)
            : 0;
        const float derivedAngleScopeDeg = encoderCountToDeg(derivedAngleScopeCount);
        const int32_t bottomReservedCount = degToEncoderCount(cfg.bottomReserved);
        const int32_t topReservedCount = degToEncoderCount(cfg.topReserved);
        const float usableAngleMax =
            derivedAngleScopeDeg - cfg.bottomReserved - cfg.topReserved;

        // 保留机械区间推导（用于角度限幅），但映射零位统一由手填 kManualZeroRaw 驱动。
        const bool scopeValid =
            rawValid &&
            derivedAngleScopeCount > 0 &&
            (bottomReservedCount + topReservedCount) < derivedAngleScopeCount &&
            usableAngleMax > 0.0f;
        if (scopeValid) {
            cfg.angleScope = derivedAngleScopeDeg;
        }

        out.success = false;
        out.encoderMin = scopeValid ? encoderMin : 0;
        out.encoderMax = scopeValid ? encoderMax : 0;
        out.offset = 0;
        out.angleMax = scopeValid ? usableAngleMax : 0.0f;

        // 将“机械零位 raw”转为统一方向后的 offset，供 ControlTask 直接做 current-zero 映射。
        if (isManualEncoderRawValid(rawZero)) {
            g_encoderOffsetManual[i] = orientEncoderRaw((uint16_t)rawZero, g_encoderDirection[i]);
        } else {
            g_encoderOffsetManual[i] = 0;
        }
    }

    // 双舵机参数同步刷新（测试场景下手动同步）
    g_joint16DualServoConstraint.enabled = 1;
    g_joint16DualServoConstraint.jointIndex = kJoint16Index;
    g_joint16DualServoConstraint.primaryBusIndex = jointMap[kJoint16Index].busIndex;
    g_joint16DualServoConstraint.primaryServoID = jointMap[kJoint16Index].servoID;
    g_joint16DualServoConstraint.secondaryBusIndex = joint16SecondaryMotor.busIndex;
    g_joint16DualServoConstraint.secondaryServoID = joint16SecondaryMotor.servoID;
    g_joint16DualServoConstraint.secondaryOffset = kJoint16SecondaryOffsetDefault;
    g_joint16DualServoConstraint.tensionBias = kJoint16TensionBiasDefault;

}

/**
 * 自动单关节标定主流程：
 * 逻辑流程：
 * - 舵机按step逐步收紧运动，直到负载先低后高超阈值即判为碰到机械限位
 * - 记录碰撞一刻编码器值作为max
 * - 推导angleScope/offset/angleMax等参数
 * - 最后回退至预留角度
 * 典型失败（false）原因：
 * - 参数非法、队列不可用、采集失败、超时未能触发阈值
 */
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out) {
    if (!sharedData || !out || jointIndex >= ENCODER_TOTAL_NUM) return false;

    out->success = false;
    out->encoderMin = 0;
    out->encoderMax = 0;
    out->offset = 0;
    out->angleMax = 0.0f;

    // 取当前舵机物理/编码器状态
    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    int32_t currentServoPos = 0;
    if (!readLatestServoAbs(sharedData, bus, servoId, &currentServoPos)) {
        return false;
    }
    int16_t commandPos = clampToServoPos(currentServoPos);
    int32_t encoderMin = 0;
    if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMin)) {
        return false;
    }

    // seenBelowThreshold：标识“先低于阈值再高于”逻辑（即真正碰撞到了机械限位）
    bool seenBelowThreshold = false;
    const uint32_t searchBeginMs = millis();

    while (millis() - searchBeginMs < cfg.maxSearchMs) {
        int32_t tightenStep = (int32_t)cfg.tightenStep;
        if (tightenStep == 0) tightenStep = 1;
        commandPos = clampToServoPos((int32_t)commandPos + tightenStep);
        if (!queueJointTarget(sharedData, jointIndex, commandPos, cfg)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));

        int16_t load = 0;
        if (!readLatestServoLoad(sharedData, bus, servoId, &load)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (load < cfg.loadThreshold) {
            seenBelowThreshold = true;
        }

        // 只关心先低于后高于阈值此瞬间
        if (seenBelowThreshold && load >= cfg.loadThreshold) {
            int32_t encoderMax = 0;
            if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMax)) {
                return false;
            }

            const int32_t derivedAngleScopeCount = forwardEncoderDelta(encoderMin, encoderMax);
            const int32_t bottomReservedCount = degToEncoderCount(cfg.bottomReserved);
            const int32_t topReservedCount = degToEncoderCount(cfg.topReserved);
            const float usableAngleMax =
                encoderCountToDeg(derivedAngleScopeCount) - cfg.bottomReserved - cfg.topReserved;
            // 机械区间、预留角度区间、实际可用运动范围校验
            if (derivedAngleScopeCount <= 0 ||
                (bottomReservedCount + topReservedCount) >= derivedAngleScopeCount ||
                usableAngleMax <= 0.0f) {
                return false;
            }

            out->encoderMin = encoderMin;
            out->encoderMax = encoderMax;
            out->offset = wrapEncoderCount(encoderMin + bottomReservedCount);
            out->angleMax = usableAngleMax;
            out->success = true;

            moveJointToReserved(sharedData, jointIndex, cfg, out->offset);
            return true;
        }
    }

    return false;
}
