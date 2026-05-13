#include "ControlTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/CalibrationTask.h"
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "ControlSolver.h"

#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

static ControlSolver g_controlSolver;

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;

static const uint8_t kDebugJointIndices[] = {0, 1, 2, 3};
static const uint8_t kDebugJointCount = (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

static const uint32_t kCanBusOfflineTimeoutMs = 300;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;
static const float kMagCountLpfAlpha = 0.25f;
static const int32_t kJointZeroHomingToleranceCounts = 40;
static const uint8_t kJointZeroHomingStableCycles = 20;
static const int32_t kJointCommandMaxStepCounts = 200;
static const int32_t kMcpCommandMaxStepCounts = 80;
static const float kJointMaxTrackErrorDeg = 15.0f;
static const uint16_t kJointTargetSpeed = 600;
static const uint8_t kJointTargetAcc = 40;
static const int32_t kMcpJointModeMotorAbsGuardCounts = 1200;
static const uint32_t kJointControlDiagIntervalMs = 200;
static const bool kEnableReleaseGuard = false;

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// 缁熶竴瀹夎鏂瑰悜鍚庯紝缂栫爜鍣ㄦ暟鍊煎綊涓€鍖栵紝鏄犲皠鍒癧0, kEncoderModulo)
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

// 缂栫爜鍣ㄨ鏁扮鐜紝浣垮叾鎬诲湪鏈夋晥鍖洪棿
static int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

// 璁＄畻姝ｅ悜缂栫爜鍣ㄥ樊鍊?鍒扮洰鏍囬渶瑕佽浆鐨勪釜鏁?锛岀幆褰㈢┖闂翠笅
static int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    return wrapEncoderCount(to - from);
}

static int32_t signedEncoderDeltaFromZero(int32_t zero, int32_t current)
{
    int32_t delta = wrapEncoderCount(current - zero);
    if (delta > kEncoderHalfTurn) {
        delta -= kEncoderModulo;
    }
    return delta;
}

// 浼樺厛鑾峰彇鑷姩鏍囧畾鍚庣殑offset锛屽惁鍒欑敤鎵嬪姩琛ㄦ牸
static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success)
        return g_jointCalibResult[jointIndex].offset;
    return g_encoderOffsetManual[jointIndex];
}

// 缂栫爜鍣ㄨ鏁拌浆瑙掑害锛堝崟浣?搴︼級
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg)
{
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

// 鏍规嵁鏍囧畾淇℃伅鍜屽叧鑺傞厤缃檺骞呯洰鏍囪搴︼紙鍗曚綅:搴︼級
static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg)) targetDeg = 0.0f;
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0.0f;

    float maxDeg = 0.0f;
    if (g_jointCalibResult[jointIndex].success) {
        maxDeg = g_jointCalibResult[jointIndex].angleMax;
    } else {
        maxDeg = g_jointCalibConfig[jointIndex].angleScope
              - g_jointCalibConfig[jointIndex].bottomReserved
              - g_jointCalibConfig[jointIndex].topReserved;
    }

    if (!isfinite(maxDeg) || maxDeg <= 0.0f) return 0.0f;
    if (targetDeg < 0.0f) return 0.0f;
    if (targetDeg > maxDeg) return maxDeg;
    return targetDeg;
}

static int findMotorChannel(uint8_t bus, uint8_t id)
{
    for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id)
            return i;
    return -1;
}

// 姣忔帶鍒跺懆鏈熻鍙栦竴娆℃帶鍒?杩愬姩鍛戒护蹇収锛岄伩鍏嶆柊鏃ф暟鎹鐩?鑴忚
static bool readControlCommandSnapshot(TaskSharedData_t* sharedData, ControlCommandSnapshot_t* out)
{
    // 鑻ユ棤杈撳叆鎴栬緭鍑烘寚閽堟棤鏁堬紝鐩存帴杩斿洖澶辫触
    if (!sharedData || !out) return false;

    // 浼樺厛浣跨敤commandStateMutex, 鍚﹀垯鐢╰argetAnglesMutex
    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) return false;

    memcpy(out->targetAngles,         sharedData->targetAngles,         sizeof(out->targetAngles));
    memcpy(out->motorTargetRaw,       sharedData->motorTargetRaw,       sizeof(out->motorTargetRaw));
    memcpy(out->motorSweepTargetRaw,  sharedData->motorSweepTargetRaw,  sizeof(out->motorSweepTargetRaw));
    out->motorCommandToken         = sharedData->motor_command_token;
    out->motorSweepCommandToken    = sharedData->motor_sweep_command_token;
    out->jointCommandToken         = sharedData->joint_command_token;
    out->motorDirectCommandGeneration = sharedData->motor_direct_command_generation;
    out->motorDirectCommandSource  = sharedData->motor_direct_command_source;
    out->controlMode               = sharedData->control_mode;
    memcpy(out->tendonGuardEnabled, (const void*)sharedData->tendon_guard_enabled, sizeof(out->tendonGuardEnabled));
    memcpy(out->tendonGuardSign,    (const void*)sharedData->tendon_guard_sign,    sizeof(out->tendonGuardSign));
    memcpy(out->tendonGuardX1Abs,   (const void*)sharedData->tendon_guard_x1_abs,  sizeof(out->tendonGuardX1Abs));

    if (lock) xSemaphoreGive(lock);
    return true;
}

/*
 * 鎺у埗涓讳换鍔″惊鐜? * 涓昏娴佺▼锛? * 1. 鑾峰彇涓绘満鐩爣骞朵笂鎶ヤ紶鎰熷櫒鍙嶉
 * 2. 鏄犲皠鐢熸垚鍏宠妭瑙掓暟鎹拰纾佺紪鐮佽搴? * 3. 鎵ц鐩爣闄愬箙銆佸畨鍏?閲婃斁淇濇姢鍒ゅ畾
 * 4. 鎸夋帶鍒舵ā寮忓拰瀹夊叏鐘舵€佷笅鍙戠洰鏍? */
void controlTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    // 鎺у埗鍣ㄦ牳蹇冨垵濮嬪寲锛圥ID绛夐噸缃級
    g_controlSolver.begin();

    // 鏈湴缂撳瓨鍙橀噺澹版槑
    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};      // 鍏宠妭鐩爣瑙?搴?
    float magAngles[ENCODER_TOTAL_NUM]    = {0.0f};
    int32_t absolutePosition[ENCODER_TOTAL_NUM] = {0};
    int32_t motorZeroPosition[ENCODER_TOTAL_NUM] = {0};   // local motor zero in joint order
    int16_t outPulses[ENCODER_TOTAL_NUM]  = {0};         // 杈撳嚭鍒扮數璋冪殑鐩爣鑴夊啿璁℃暟

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    float filteredMappedCountStage1[ENCODER_TOTAL_NUM] = {0.0f};
    float filteredMappedCountStage2[ENCODER_TOTAL_NUM] = {0.0f};
    uint8_t mappedCountFilterValid[ENCODER_TOTAL_NUM] = {0};
    uint8_t joint16DiffFaultCounter = 0;     // joint16涓讳粠鍙嶉宸紓瓒呭樊璁℃暟
    const int joint16PrimaryCh   = findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);       // joint16涓荤數鏈篶h
    const int joint16SecondaryCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);           // joint16鍓數鏈篶h
    ReleaseGuardState releaseGuards[ENCODER_TOTAL_NUM];
    resetAllReleaseGuardStates(releaseGuards);

    uint32_t lastReleaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
    uint32_t lastServoInternalZeroAckToken = sharedData->servo_internal_zero_ack_token;
    uint8_t postServoZeroSettleCycles = 0;
    sharedData->reverse_release_fault_bitmap = 0;
    ControlCommandSnapshot_t commandSnapshot;
    memset(&commandSnapshot, 0, sizeof(commandSnapshot));
    // 璇诲彇蹇収锛屽疄闄呮槸閫氳繃鎸囬拡淇敼鏍堜笂鍙橀噺
    readControlCommandSnapshot(sharedData, &commandSnapshot);

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    TickType_t lastWakeTime = xTaskGetTickCount();    // 瀹氭椂瑙﹀彂tick
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);   // 鎺у埗鍛ㄦ湡闀垮害10ms
    bool prevMotorOnline[SERVO_TOTAL_NUM]  = {false};
    bool hotplugHoldMotor[SERVO_TOTAL_NUM] = {false};
    uint32_t lastMotorCommandToken      = commandSnapshot.motorCommandToken;
    uint32_t lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
    uint32_t lastJointCommandToken      = commandSnapshot.jointCommandToken;
    uint32_t lastMotorDirectGeneration  = commandSnapshot.motorDirectCommandGeneration;
    uint8_t  lastMotorDirectSource      = commandSnapshot.motorDirectCommandSource;
    bool jointZeroHomingActive = false;
    uint8_t jointZeroHomingStableCount = 0;
    bool prevJointControlActive = false;
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM]   = {0};
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM]  = {0};
    bool sweepAnchorValid[SERVO_TOTAL_NUM]        = {false};
    uint32_t lastJointControlDiagMs = 0;

    while (1)
    {
        if (!readControlCommandSnapshot(sharedData, &commandSnapshot)) {
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            continue;
        }

        // 妫€鏌oken鍙樺寲锛涘鍙樺寲娓呴櫎鐩稿叧hold璁板綍
        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        const bool motorCommandChanged = (motorCommandToken != lastMotorCommandToken);
        if (motorCommandChanged) {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
        }

        const uint32_t motorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
        const bool motorSweepCommandChanged = (motorSweepCommandToken != lastMotorSweepCommandToken);
        if (motorSweepCommandChanged) {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
        }

        const uint32_t motorDirectGeneration = commandSnapshot.motorDirectCommandGeneration;
        const bool motorDirectGenerationChanged =
            (motorDirectGeneration != lastMotorDirectGeneration);
        const bool directMotorCommandFresh =
            motorCommandChanged ||
            motorSweepCommandChanged ||
            motorDirectGenerationChanged;
        if (directMotorCommandFresh) {
            lastMotorCommandToken = motorCommandToken;
            lastMotorSweepCommandToken = motorSweepCommandToken;
            lastMotorDirectGeneration = motorDirectGeneration;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        if (jointCommandToken != lastJointCommandToken) {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastJointCommandToken = jointCommandToken;
        }

        const uint8_t motorDirectSource = commandSnapshot.motorDirectCommandSource;
        if (motorDirectSource != lastMotorDirectSource) {
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            lastMotorDirectSource = motorDirectSource;
        }

        // 姣忔閲嶇疆淇濇姢fault token鍙樺寲鏃讹紝娓呯悊鍏ㄩ儴閲婃斁淇濇姢
        const uint32_t releaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
        if (releaseFaultResetToken != lastReleaseFaultResetToken) {
            resetAllReleaseGuardStates(releaseGuards);
            sharedData->reverse_release_fault_bitmap = 0;
            lastReleaseFaultResetToken = releaseFaultResetToken;
        }

        const uint32_t servoInternalZeroAckToken = sharedData->servo_internal_zero_ack_token;
        if (servoInternalZeroAckToken != lastServoInternalZeroAckToken) {
            lastServoInternalZeroAckToken = servoInternalZeroAckToken;
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            jointZeroHomingActive = false;
            jointZeroHomingStableCount = 0;
            prevJointControlActive = false;
            lastMotorCommandToken = commandSnapshot.motorCommandToken;
            lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
            lastMotorDirectGeneration = commandSnapshot.motorDirectCommandGeneration;
            lastMotorDirectSource = commandSnapshot.motorDirectCommandSource;
            g_controlSolver.resetAll();
            postServoZeroSettleCycles = 10;
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            continue;
        }

        ServoTargetBatch_t targetBatch;
        memset(&targetBatch, 0, sizeof(targetBatch));
        targetBatch.timestamp = millis();
        targetBatch.source    = SERVO_TARGET_OWNER_CONTROL;
        int16_t jointCmdPos[ENCODER_TOTAL_NUM]    = {0}; // 鐢ㄤ簬璋冭瘯
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM]  = {0}; // 鐢ㄤ簬璋冭瘯

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        if (sharedData->servoFeedbackQueue)
            xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0);

        if (postServoZeroSettleCycles > 0) {
            postServoZeroSettleCycles--;
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            continue;
        }

        // 缁存姢鍦ㄧ嚎鐘舵€佽烦鍙? hotplug鍦烘櫙闇€hold
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            if (onlineNow && !prevMotorOnline[ch]) {
                hotplugHoldMotor[ch] = true;   // 鏂颁笂绾縣old
                sweepAnchorValid[ch]  = false;
            }
            if (!onlineNow) {
                sweepAnchorValid[ch]  = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }

        // 鍗忚瀵归綈锛屽吋瀹癸細娓?鎵€鏈塷verload fault
        sharedData->overload_fault_bitmap = 0;

        // 姝ラ3, 濉厖缁濆鑴夊啿缂撳啿鍖?ch鍒板叧鑺傛槧灏?
        memset(absolutePosition, 0, sizeof(absolutePosition));
        memset(motorZeroPosition, 0, sizeof(motorZeroPosition));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id  = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0) {
                motorZeroPosition[jointIndex] = 0;
            }
            if (ch >= 0 && servoData.onlineStatus[ch])
                absolutePosition[jointIndex] = servoData.servoAngles[ch];
        }

        bool joint16DualFault = false;
        {
            const uint8_t pBus = jointMap[kJoint16Index].busIndex;
            const uint8_t pId  = jointMap[kJoint16Index].servoID;
            const uint8_t sBus = joint16SecondaryMotor.busIndex;
            const uint8_t sId  = joint16SecondaryMotor.servoID;
            const int pCh = findMotorChannel(pBus, pId);
            const int sCh = findMotorChannel(sBus, sId);
            const bool pOnline = (pCh >= 0) && (servoData.onlineStatus[pCh] != 0);
            const bool sOnline = (sCh >= 0) && (servoData.onlineStatus[sCh] != 0);

            if (pOnline && sOnline)
            {
                const int32_t p = servoData.servoAngles[pCh];
                const int32_t s = servoData.servoAngles[sCh];
                const int32_t sProj = -s + kJoint16SecondaryOffset;
                const int32_t fused = (p + sProj) / 2;
                const int32_t pZero = 0;
                const int32_t sZero = 0;
                const int32_t sZeroProj = -sZero + kJoint16SecondaryOffset;
                motorZeroPosition[kJoint16Index] =
                    (pZero + sZeroProj) / 2;
                const int32_t diff  = (p >= sProj) ? (p - sProj) : (sProj - p);

                absolutePosition[kJoint16Index] = fused;

                // 宸紓澶э紝绱姞璁℃暟鍣紝鎸佺画寮傚父鍒や负涓诲壇鍙嶉鏁呴殰
                if (diff > kJoint16DiffThresholdCounts) {
                    if (joint16DiffFaultCounter < 255)
                        joint16DiffFaultCounter++;
                } else {
                    joint16DiffFaultCounter = 0;
                }
                joint16DualFault = (joint16DiffFaultCounter >= kJoint16DiffFaultCycles);
            }
            else
            {
                joint16DiffFaultCounter = 0;
                joint16DualFault = true;
            }
        }
        sharedData->joint16_dual_feedback_fault = joint16DualFault ? 1 : 0;

#if SOLVER_DIAG_LOG_ENABLE
        // 鎵撳嵃joint16鍙嶉璋冭瘯鏃ュ織
        const uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            const int pCh = findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
            const int sCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
            const long pAbs = (pCh >= 0) ? (long)servoData.servoAngles[pCh] : 0;
            const long sAbs = (sCh >= 0) ? (long)servoData.servoAngles[sCh] : 0;
            Serial.printf("[Solver] j16 p=%ld s=%ld fault=%d\r\n", pAbs, sAbs, joint16DualFault ? 1 : 0);
        }
#endif

        // can鎬荤嚎鍦ㄧ嚎鐘舵€佸垽鏂紙鍒ゅ畾涓婃姤鏁版嵁鍖呮椂鏁堟€э級
        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        // 缁勮鍏宠妭瑙掑害鏄犲皠鏁版嵁锛堢缂栫爜鍣級
        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
        mappedData.isValid   = canBusOnline;

        if (canBusOnline)
        {
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    continue;
                }

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                const int32_t offset = getEncoderOffset((uint8_t)i);
                int32_t mappedCount = signedEncoderDeltaFromZero(offset, orientedRaw);

                if (g_jointCalibResult[i].success) {
                    const JointCalibrationResult& calib = g_jointCalibResult[i];
                    int32_t hardMaxFromOffset = forwardEncoderDelta(offset, calib.encoderMax);
                    if (hardMaxFromOffset <= 0) {
                        hardMaxFromOffset = degToEncoderCount(calib.angleMax);
                    }
                    if (mappedCount > hardMaxFromOffset + kEncoderMarginCounts) {
                        mappedCount -= kEncoderModulo;
                    }
                }

                if (mappedCountFilterValid[i] == 0) {
                    filteredMappedCountStage1[i] = (float)mappedCount;
                    filteredMappedCountStage2[i] = (float)mappedCount;
                    mappedCountFilterValid[i] = 1;
                } else {
                    filteredMappedCountStage1[i] += kMagCountLpfAlpha * ((float)mappedCount - filteredMappedCountStage1[i]);
                    filteredMappedCountStage2[i] += kMagCountLpfAlpha * (filteredMappedCountStage1[i] - filteredMappedCountStage2[i]);
                }
                const int32_t filteredMappedCount =
                    (int32_t)(filteredMappedCountStage2[i] + (filteredMappedCountStage2[i] >= 0.0f ? 0.5f : -0.5f));

                // 鍐欏叆缂栫爜鍣ㄨ鏁板埌鏄犲皠鏁版嵁鍙婂叾鏈夋晥鏍囪锛屽悓鏃惰绠楄搴︼紙鍗曚綅锛氬害锛?                // mappedData[i] = clampMappedCountForProtocol(mappedCount); // 鍗忚浼犺緭鏈娇鐢紝淇濈暀娉ㄩ噴
                mappedData.angleValues[i] = filteredMappedCount;
                mappedData.validFlags[i]  = 1;
                magAngles[i]              = convertEncoderCountToDeg(filteredMappedCount);
            }
        }

        // 涓婃姤褰撳墠鍏宠妭瑙掓暟鎹埌鍏变韩闃熷垪
        if (sharedData->mappedAngleQueue)
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);

        // 鍏宠妭瑙掑懡浠ゆ嫹璐濆埌鏈湴缂撳瓨锛堢敤浜庡悗缁弻鐜級
        memcpy(localTargets, commandSnapshot.targetAngles, sizeof(localTargets));

        const bool hostControlEnabled  = (sharedData->control_enabled != 0);
        const uint8_t systemState      = sharedData->system_state;
        const uint8_t targetOwner      = sharedData->servo_target_owner;
        const bool controlOwnerActive  = (targetOwner == SERVO_TARGET_OWNER_CONTROL);
        const bool runningState        = (systemState == SYSTEM_STATE_RUNNING);
        const bool faultHoldState      = (systemState == SYSTEM_STATE_FAULT_HOLD);
        const bool controlEnabled      = hostControlEnabled && controlOwnerActive && (runningState || faultHoldState);
        const bool normalOutputAllowed = controlEnabled && runningState;
        const bool jointModeActive =
            (commandSnapshot.controlMode == CONTROL_MODE_JOINT);
        const bool jointControlActive =
            normalOutputAllowed &&
            jointModeActive &&
            (commandSnapshot.jointCommandToken != 0);
        const bool directMotorCommandActive =
            normalOutputAllowed &&
            (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR) &&
            directMotorCommandFresh;
        if (jointControlActive && !prevJointControlActive) {
            jointZeroHomingActive = false;
            jointZeroHomingStableCount = 0;
            g_controlSolver.resetAll();
        }
        if (!jointControlActive) {
            jointZeroHomingActive = false;
            jointZeroHomingStableCount = 0;
        }
        prevJointControlActive = jointControlActive;

        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT)
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);

        // Joint-angle mode uses tendon feedforward where a tendon model exists,
        // while channels without a tendon model keep the dual-loop PID fallback.
        bool jointSolverOk = false;
        if (jointControlActive) {
            jointSolverOk = g_controlSolver.computeTendonFeedforward(
                localTargets,
                magAngles,
                absolutePosition,
                motorZeroPosition,
                outPulses
            );
        }
        if (!jointSolverOk) {
            memset(outPulses, 0, sizeof(outPulses));
        }

        if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
        {
            // 鐩存帶妯″紡锛氫粎涓嬪彂涓绘満涓嬪彂鐨勭數鏈簉aw/sweep/abs鐩爣
            if (directMotorCommandActive) {
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id  = motorMap[ch].servoID;
                    if (bus >= NUM_BUSES) continue;

                    // joint16涓诲壇鐗规畩鏁呴殰鍏煎
                    const bool isJoint16Motor = ((int)ch == joint16PrimaryCh) || ((int)ch == joint16SecondaryCh);
                    if (!normalOutputAllowed || (joint16DualFault && isJoint16Motor)) {
                        if (servoData.onlineStatus[ch] != 0) {
                            const int16_t holdPos = clampServoPos(servoData.servoAngles[ch]);
                            appendServoTarget(&targetBatch, bus, id, holdPos,
                                              SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                        }
                        continue;
                    }

                    int16_t targetPos = clampServoPos(servoData.servoAngles[ch]);
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            // sweep妯″紡澶勭悊娉曪細鏈夋柊杈撳叆灏辩疮璁″睍寮€
                            const int16_t rawTarget = clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] = anchorSingleTurnSweepToCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch]  = rawTarget;
                                sweepAnchorValid[ch]    = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = clampServoPos(sweepExpandedTarget[ch]);
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPos((int32_t)commandSnapshot.motorTargetRaw[ch]);
                        } else {
                            sweepAnchorValid[ch] = false;
                            targetPos = expandSingleTurnTargetNearCurrent(
                                servoData.servoAngles[ch],
                                clampServoSingleTurnRaw(commandSnapshot.motorTargetRaw[ch])
                            );
                        }
                    }
                    else {
                        sweepAnchorValid[ch] = false;
                    }
                    appendServoTarget(&targetBatch, bus, id, targetPos,
                        SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
        }
        else
        {
            // 鍏宠妭妯″紡--閫愰€氶亾/鍏宠妭鍒ゆ柇鎬ュ仠/閲婃斁/瀹堟姢绛夋潯浠跺苟涓嬪彂鐩爣
            if (!jointControlActive) {
                vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
                continue;
            }
            bool zeroHomingAllReached = true;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id  = jointMap[jointIndex].servoID;
                const int jointMotorCh = findMotorChannel(bus, id);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                const bool mappedValid = canBusOnline && (mappedData.validFlags[jointIndex] != 0);
                const bool guardInputsValid = servoOnline && mappedValid;
                bool releaseFaultActive = false;
                bool holdByTendonGuard  = false;

                // 闈瀓oint16闇€閲婃斁淇濇姢鍜宼endon guard鍒ゅ畾
                if (kEnableReleaseGuard && jointIndex != kJoint16Index)
                {
                    const int32_t servoPos = (jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex];
                    releaseFaultActive = updateReleaseGuardState(&releaseGuards[jointIndex],
                                                                 localTargets[jointIndex],
                                                                 magAngles[jointIndex],
                                                                 servoPos,
                                                                 guardInputsValid);
                    holdByTendonGuard = shouldBlockReleaseByTendonGuard(
                        &commandSnapshot,
                        jointIndex,
                        localTargets[jointIndex],
                        magAngles[jointIndex],
                        absolutePosition[jointIndex],
                        guardInputsValid
                    );
                }

                // 鏄惁绱ф€ュ仠锛氬寮傚父鐘舵€?涓诲壇鏁呴殰/閲婃斁淇濇姢/瀹堟姢闇€hold
                bool emergency = (!normalOutputAllowed) ||
                    shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex);
                if (mappedValid) {
                    const float trackError = localTargets[jointIndex] - magAngles[jointIndex];
                    if (trackError > kJointMaxTrackErrorDeg || trackError < -kJointMaxTrackErrorDeg) {
                        emergency = true;
                    }
                }

                if (jointIndex <= 1) {
                    const int32_t motorAbs = (jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex];
                    if (motorAbs > kMcpJointModeMotorAbsGuardCounts ||
                        motorAbs < -kMcpJointModeMotorAbsGuardCounts) {
                        emergency = true;
                    }
                }

                if (jointIndex == kJoint16Index)
                    emergency = emergency || joint16DualFault;
                else
                    emergency = emergency || releaseFaultActive || holdByTendonGuard;

                // 鐑彃鎷攈old鍒ゅ畾
                const bool hotplugHoldActive =
                    (jointMotorCh >= 0) &&
                    (jointMotorCh < SERVO_TOTAL_NUM) &&
                    hotplugHoldMotor[jointMotorCh];

                if (bus >= NUM_BUSES) continue;
                if (!controlEnabled) continue;
                if (jointIndex == 1) {
                    continue;
                }
                if (jointIndex == 0)
                {
                    bool mcpEmergency = false;
                    bool mcpHotplugHold = false;
                    bool mcpAllZeroReached = true;
                    bool mcpAnyOnline = false;

                    for (uint8_t tendonIndex = 0; tendonIndex <= 1; tendonIndex++) {
                        const uint8_t mBus = jointMap[tendonIndex].busIndex;
                        const uint8_t mId = jointMap[tendonIndex].servoID;
                        const int mCh = findMotorChannel(mBus, mId);
                        const bool mOnline = (mCh >= 0) && (servoData.onlineStatus[mCh] != 0);
                        const bool mMappedValid = canBusOnline && (mappedData.validFlags[tendonIndex] != 0);
                        mcpAnyOnline = mcpAnyOnline || mOnline;

                        bool mEmergency = !normalOutputAllowed;

                        const int32_t motorAbs = (mCh >= 0) ? servoData.servoAngles[mCh] : absolutePosition[tendonIndex];
                        if (motorAbs > kMcpJointModeMotorAbsGuardCounts ||
                            motorAbs < -kMcpJointModeMotorAbsGuardCounts) {
                            mEmergency = true;
                        }

                        const bool mHotplug =
                            (mCh >= 0) &&
                            (mCh < SERVO_TOTAL_NUM) &&
                            hotplugHoldMotor[mCh];
                        mcpEmergency = mcpEmergency || mEmergency;
                        mcpHotplugHold = mcpHotplugHold || mHotplug;
                    }

                    if (mcpEmergency)
                    {
                        for (uint8_t tendonIndex = 0; tendonIndex <= 1; tendonIndex++) {
                            const uint8_t mBus = jointMap[tendonIndex].busIndex;
                            const uint8_t mId = jointMap[tendonIndex].servoID;
                            const int mCh = findMotorChannel(mBus, mId);
                            const bool mOnline = (mCh >= 0) && (servoData.onlineStatus[mCh] != 0);
                            if (mOnline) {
                                jointCmdPos[tendonIndex] = clampServoPos(absolutePosition[tendonIndex]);
                                jointCmdValid[tendonIndex] = 1;
                            }
                        }
                        continue;
                    }

                    if (jointZeroHomingActive)
                    {
                        for (uint8_t tendonIndex = 0; tendonIndex <= 1; tendonIndex++) {
                            const uint8_t mBus = jointMap[tendonIndex].busIndex;
                            const uint8_t mId = jointMap[tendonIndex].servoID;
                            const int mCh = findMotorChannel(mBus, mId);
                            const bool mOnline = (mCh >= 0) && (servoData.onlineStatus[mCh] != 0);
                            if (mOnline) {
                                appendServoTarget(&targetBatch, mBus, mId, 0,
                                    kJointTargetSpeed, kJointTargetAcc);
                                jointCmdPos[tendonIndex] = 0;
                                jointCmdValid[tendonIndex] = 1;
                                const int32_t zeroError = (absolutePosition[tendonIndex] >= 0)
                                    ? absolutePosition[tendonIndex]
                                    : -absolutePosition[tendonIndex];
                                if (zeroError > kJointZeroHomingToleranceCounts) {
                                    mcpAllZeroReached = false;
                                }
                            }
                        }
                        if (!mcpAllZeroReached) {
                            zeroHomingAllReached = false;
                        }
                        continue;
                    }

                    if (mcpHotplugHold)
                    {
                        for (uint8_t tendonIndex = 0; tendonIndex <= 1; tendonIndex++) {
                            const uint8_t mBus = jointMap[tendonIndex].busIndex;
                            const uint8_t mId = jointMap[tendonIndex].servoID;
                            const int mCh = findMotorChannel(mBus, mId);
                            const bool mOnline = (mCh >= 0) && (servoData.onlineStatus[mCh] != 0);
                            if (mOnline) {
                                const int16_t holdPos = clampServoPos(absolutePosition[tendonIndex]);
                                appendServoTarget(&targetBatch, mBus, mId, holdPos,
                                    kJointTargetSpeed, kJointTargetAcc);
                                jointCmdPos[tendonIndex] = holdPos;
                                jointCmdValid[tendonIndex] = 1;
                            }
                        }
                        continue;
                    }

                    if (mcpAnyOnline) {
                        for (uint8_t tendonIndex = 0; tendonIndex <= 1; tendonIndex++) {
                            const uint8_t mBus = jointMap[tendonIndex].busIndex;
                            const uint8_t mId = jointMap[tendonIndex].servoID;
                            const int mCh = findMotorChannel(mBus, mId);
                            const bool mOnline = (mCh >= 0) && (servoData.onlineStatus[mCh] != 0);
                            if (!mOnline) {
                                continue;
                            }
                            int32_t limitedTarget = outPulses[tendonIndex];
                            const int32_t currentPos = absolutePosition[tendonIndex];
                            const int32_t delta = limitedTarget - currentPos;
                            if (delta > kMcpCommandMaxStepCounts) {
                                limitedTarget = currentPos + kMcpCommandMaxStepCounts;
                            } else if (delta < -kMcpCommandMaxStepCounts) {
                                limitedTarget = currentPos - kMcpCommandMaxStepCounts;
                            }
                            if (limitedTarget > kMcpJointModeMotorAbsGuardCounts) {
                                limitedTarget = kMcpJointModeMotorAbsGuardCounts;
                            } else if (limitedTarget < -kMcpJointModeMotorAbsGuardCounts) {
                                limitedTarget = -kMcpJointModeMotorAbsGuardCounts;
                            }
                            const int16_t targetPos = clampServoPos(limitedTarget);
                            appendServoTarget(&targetBatch, mBus, mId, targetPos,
                                kJointTargetSpeed, kJointTargetAcc);
                            jointCmdPos[tendonIndex] = targetPos;
                            jointCmdValid[tendonIndex] = 1;
                        }
                    }
                    continue;
                }

                if (emergency)
                {
                    const int16_t holdPos = (jointIndex <= 1) ? 0 : clampServoPos(absolutePosition[jointIndex]);
                    if (servoOnline) {
                        appendServoTarget(&targetBatch, bus, id, holdPos,
                            kJointTargetSpeed, kJointTargetAcc);
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }
                    if (jointIndex <= 1) {
                        const uint32_t nowMs = millis();
                        if (nowMs - lastJointControlDiagMs >= kJointControlDiagIntervalMs) {
                            lastJointControlDiagMs = nowMs;
                            Serial.printf("[JOINT CTRL] J%02u HOLD target=%.2f actual=%.2f motorAbs=%ld solverTarget=%ld cmd=%d mappedValid=%u normal=%u\r\n",
                                          (unsigned)jointIndex,
                                          (double)localTargets[jointIndex],
                                          (double)magAngles[jointIndex],
                                          (long)((jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex]),
                                          (long)outPulses[jointIndex],
                                          (int)holdPos,
                                          (unsigned)mappedValid,
                                          (unsigned)normalOutputAllowed);
                        }
                    }
                    // joint16鍓酱鍚屾hold
                    if (jointIndex == kJoint16Index) {
                        const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                        if (secCh >= 0 && servoData.onlineStatus[secCh] != 0) {
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            appendServoTarget(&targetBatch,
                              joint16SecondaryMotor.busIndex,
                              joint16SecondaryMotor.servoID,
                              holdSec,
                              kJointTargetSpeed,
                              kJointTargetAcc);
                        }
                    }
                    continue;
                }

                if (jointZeroHomingActive)
                {
                    if (servoOnline) {
                        const int16_t zeroTarget = clampServoPos(motorZeroPosition[jointIndex]);
                        appendServoTarget(&targetBatch, bus, id, zeroTarget,
                            kJointTargetSpeed, kJointTargetAcc);
                        jointCmdPos[jointIndex] = zeroTarget;
                        jointCmdValid[jointIndex] = 1;
                        const int32_t zeroError = (absolutePosition[jointIndex] >= motorZeroPosition[jointIndex])
                            ? (absolutePosition[jointIndex] - motorZeroPosition[jointIndex])
                            : (motorZeroPosition[jointIndex] - absolutePosition[jointIndex]);
                        if (zeroError > kJointZeroHomingToleranceCounts) {
                            zeroHomingAllReached = false;
                        }
                    }
                    if (jointIndex == kJoint16Index) {
                        const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                        if (secCh >= 0 && servoData.onlineStatus[secCh] != 0) {
                            appendServoTarget(&targetBatch,
                                joint16SecondaryMotor.busIndex,
                                joint16SecondaryMotor.servoID,
                                0,
                                kJointTargetSpeed,
                                kJointTargetAcc);
                        }
                    }
                    continue;
                }

                // 鐑彃鎷攈old
                if (hotplugHoldActive)
                {
                    if (servoOnline) {
                        const int16_t holdPos = clampServoPos(absolutePosition[jointIndex]);
                        appendServoTarget(&targetBatch, bus, id, holdPos,
                            kJointTargetSpeed, kJointTargetAcc);
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }
                    if (jointIndex == kJoint16Index) {
                        const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                        if (secCh >= 0 && servoData.onlineStatus[secCh] != 0) {
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            appendServoTarget(&targetBatch,
                                joint16SecondaryMotor.busIndex,
                                joint16SecondaryMotor.servoID,
                                holdSec,
                                kJointTargetSpeed,
                                kJointTargetAcc);
                        }
                    }
                    continue;
                }

                // 闈炴€ュ仠姝ｅ父鐩爣杈撳嚭
                int32_t limitedTarget = outPulses[jointIndex];
                if (servoOnline && jointIndex > 1) {
                    const int32_t currentPos = absolutePosition[jointIndex];
                    const int32_t delta = limitedTarget - currentPos;
                    if (delta > kJointCommandMaxStepCounts) {
                        limitedTarget = currentPos + kJointCommandMaxStepCounts;
                    } else if (delta < -kJointCommandMaxStepCounts) {
                        limitedTarget = currentPos - kJointCommandMaxStepCounts;
                    }
                }
                const int16_t targetPos = clampServoPos(limitedTarget);
                appendServoTarget(&targetBatch, bus, id, targetPos, kJointTargetSpeed, kJointTargetAcc);
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;
                if (jointIndex <= 1) {
                    const uint32_t nowMs = millis();
                    if (nowMs - lastJointControlDiagMs >= kJointControlDiagIntervalMs) {
                        lastJointControlDiagMs = nowMs;
                        Serial.printf("[JOINT CTRL] J%02u RUN target=%.2f actual=%.2f motorAbs=%ld solverTarget=%ld limited=%ld cmd=%d mappedMotorTarget=%.1f\r\n",
                                      (unsigned)jointIndex,
                                      (double)localTargets[jointIndex],
                                      (double)magAngles[jointIndex],
                                      (long)((jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex]),
                                      (long)outPulses[jointIndex],
                                      (long)limitedTarget,
                                      (int)targetPos,
                                      (double)g_controlSolver.getMappedMotorTarget(jointIndex));
                    }
                }

                // joint16鍓酱鐩爣杈撳嚭(浣嶇疆闇€鍔犲亸缃?鎷夊姏琛ュ伩, 涓斿弽鍚戣緭鍑?
                if (jointIndex == kJoint16Index) {
                    if (joint16SecondaryMotor.busIndex < NUM_BUSES) {
                        const int32_t secTarget = -((int32_t)targetPos) + kJoint16SecondaryOffset + kJoint16TensionBias;
                        appendServoTarget(&targetBatch,
                            joint16SecondaryMotor.busIndex,
                            joint16SecondaryMotor.servoID,
                            clampServoPos(secTarget),
                            kJointTargetSpeed,
                            kJointTargetAcc);
                    }
                }
            }
            if (jointZeroHomingActive) {
                if (zeroHomingAllReached) {
                    if (jointZeroHomingStableCount < 255) {
                        jointZeroHomingStableCount++;
                    }
                    if (jointZeroHomingStableCount >= kJointZeroHomingStableCycles) {
                        jointZeroHomingActive = false;
                        jointZeroHomingStableCount = 0;
                        g_controlSolver.resetAll();
                    }
                } else {
                    jointZeroHomingStableCount = 0;
                }
            }
        }

        if (kEnableReleaseGuard) {
            sharedData->reverse_release_fault_bitmap = buildReleaseFaultBitmap(releaseGuards);
        } else {
            sharedData->reverse_release_fault_bitmap = 0;
        }

        // 璋冭瘯鍏宠妭鏁版嵁杈撳嚭
        if (sharedData->jointDebugQueue)
        {
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex   = jointIndex;
                debugData.timestamp    = millis();
                debugData.valid        = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg    = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output  = g_controlSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual  = (float)absolutePosition[jointIndex];
                debugData.loop2Output  = g_controlSolver.getPidOutput(jointIndex, 1);
                debugData.targetLength = g_controlSolver.getTargetTendonLength(jointIndex);
                debugData.actualLength = g_controlSolver.getActualTendonLength(jointIndex);
                debugData.mappedMotorTarget = g_controlSolver.getMappedMotorTarget(jointIndex);
                debugData.motorZeroAbs = motorZeroPosition[jointIndex];
                debugData.solverOutputPos = jointZeroHomingActive ? jointCmdPos[jointIndex] : outPulses[jointIndex];
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid     = jointCmdValid[jointIndex];
                debugData.zeroHoming   = jointZeroHomingActive ? 1 : 0;

                // 闃熷垪婊¤嚜鍔ㄤ涪寮冧竴甯ф棫鏁版嵁
                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        if (sharedData->servoTargetQueue && targetBatch.count > 0)
            xQueueOverwrite(sharedData->servoTargetQueue, &targetBatch);

        // 淇濊瘉绛夊懆鏈熻皟搴?        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
    }
}
