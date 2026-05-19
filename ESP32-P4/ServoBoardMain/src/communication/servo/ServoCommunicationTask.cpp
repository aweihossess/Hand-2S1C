#include "ServoCommunicationTask.h"

#include <string.h>

#include "../libraries/FTServo_Arduino/src/SMS_STS.h"

#ifndef FTSERVO_ARDUINO_EXTERNAL_BUILD
#include "../libraries/FTServo_Arduino/src/SCS.cpp"
#include "../libraries/FTServo_Arduino/src/SCSerial.cpp"
#include "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
#endif

extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

#ifndef SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
#define SERVO_CONFIGURE_MULTI_TURN_ON_BOOT 1
#endif

#define SERVO_MULTI_TURN_CONFIG_DELAY_MS 5

struct ServoFeedbackInternal {
    int16_t rawPosition;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    int32_t hardwareAbsolutePosition;
    int16_t turnCount;
    int16_t lastRawPosition;
    bool initialized;
    bool online;
    uint32_t lastUpdate;
};

class ServoBusDriver {
public:
    ServoBusDriver() : _serial(NULL), _writeCount(0), _syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT)
    {
        memset(_feedback, 0, sizeof(_feedback));
        memset(_torqueEnabled, 0, sizeof(_torqueEnabled));
        memset(_positionModeEnabled, 0, sizeof(_positionModeEnabled));
        memset(_softwareZeroOffset, 0, sizeof(_softwareZeroOffset));
    }

    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud)
    {
        HardwareSerial* serial = NULL;
        switch (busIndex) {
            case 0: serial = &Serial1; break;
            case 1: serial = &Serial2; break;
            case 2: serial = &Serial3; break;
            case 3: serial = &Serial4; break;
            default: return;
        }

        _serial = serial;
        _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
        _sms.pSerial = _serial;
    }

    bool configureMultiTurnMode(uint8_t id)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
            return false;
        }

        uint8_t zeroLimits[4] = {0, 0, 0, 0};
        bool ok = true;
        ok = (_sms.unLockEprom(id) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        ok = (_sms.genWrite(id, SMS_STS_MIN_ANGLE_LIMIT_L, zeroLimits, sizeof(zeroLimits)) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        ok = (_sms.LockEprom(id) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        return ok;
    }

    void ensurePositionMode(uint8_t id)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
            return;
        }
        if (_positionModeEnabled[id]) {
            return;
        }
        _sms.EnableTorque(id, 0);
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        _sms.unLockEprom(id);
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        _sms.WheelMode(id);
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        _sms.LockEprom(id);
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        _positionModeEnabled[id] = true;
    }

    void enablePositionControl(uint8_t id)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
            return;
        }
        ensurePositionMode(id);
        if (_torqueEnabled[id]) {
            return;
        }
        _sms.EnableTorque(id, 1);
        _torqueEnabled[id] = true;
    }

    void disableTorque(uint8_t id)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
            return;
        }
        _sms.EnableTorque(id, 0);
        _torqueEnabled[id] = false;
        _positionModeEnabled[id] = false;
    }

    void calibrateCurrentPositionAsZero(uint8_t id)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
            return;
        }
        _softwareZeroOffset[id] = _feedback[id].hardwareAbsolutePosition;
    }

    int32_t getHardwareAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _feedback[id].hardwareAbsolutePosition;
    }

    void setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        if (id == 0 || id > MAX_SERVO_ID) {
            return;
        }
        if (_writeCount >= MAX_SERVOS_PER_BUS) {
            return;
        }
        const int32_t currentMotorPosition =
            _feedback[id].hardwareAbsolutePosition - _softwareZeroOffset[id];
        const int32_t error = (int32_t)position - currentMotorPosition;
        int32_t speedCommand = error;
        if (speedCommand > (int32_t)speed) speedCommand = speed;
        if (speedCommand < -(int32_t)speed) speedCommand = -(int32_t)speed;
        if (speedCommand > -2 && speedCommand < 2) speedCommand = 0;

        _writeIDs[_writeCount] = id;
        _writePos[_writeCount] = (int16_t)speedCommand;
        _writeSpd[_writeCount] = speed;
        _writeAcc[_writeCount] = acc;
        _writeCount++;
    }

    void syncWriteAll()
    {
        if (_writeCount == 0 || !_serial) {
            return;
        }
        _sms.SyncWriteSpe(_writeIDs, _writeCount, _writePos, _writeAcc);
        _writeCount = 0;
    }

    int syncReadPositions(const uint8_t* ids, uint8_t count)
    {
        if (!_serial || !ids || count == 0) {
            return 0;
        }

        const uint8_t addrPresentPosition = 56;
        const uint8_t feedbackLen = 8;
        const uint16_t timeoutMs = (_syncReadTimeoutMs == 0) ? 1 : _syncReadTimeoutMs;
        int successCount = 0;

        _sms.syncReadBegin(count, feedbackLen, timeoutMs);
        const int txResult = _sms.syncReadPacketTx((uint8_t*)ids, count, addrPresentPosition, feedbackLen);
        if (txResult <= 0) {
            for (uint8_t i = 0; i < count; i++) {
                if (ids[i] <= MAX_SERVO_ID) {
                    _feedback[ids[i]].online = false;
                }
            }
            _sms.syncReadEnd();
            return 0;
        }

        for (uint8_t i = 0; i < count; i++) {
            const uint8_t id = ids[i];
            uint8_t rxBuf[feedbackLen];
            const int rxLen = _sms.syncReadPacketRx(id, rxBuf);
            if (id > MAX_SERVO_ID) {
                continue;
            }
            if (rxLen == feedbackLen) {
                const int16_t presentPosition = decodeSigned(rxBuf[0], rxBuf[1]);
                _updatePresentPosition(id, presentPosition);
                _feedback[id].speed = decodeSigned(rxBuf[2], rxBuf[3]);
                _feedback[id].load = decodeSignedWithSignBit(rxBuf[4], rxBuf[5], 10);
                _feedback[id].voltage = rxBuf[6];
                _feedback[id].temperature = rxBuf[7];
                _feedback[id].online = true;
                _feedback[id].lastUpdate = millis();
                successCount++;
            } else {
                _feedback[id].online = false;
            }
        }

        _sms.syncReadEnd();
        return successCount;
    }

    int16_t getRawPosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return -1;
        return _feedback[id].rawPosition;
    }

    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _feedback[id].hardwareAbsolutePosition - _softwareZeroOffset[id];
    }

    int32_t getSoftwareZeroOffset(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _softwareZeroOffset[id];
    }

    const ServoFeedbackInternal& getFeedback(uint8_t id) const
    {
        static ServoFeedbackInternal dummy;
        if (id > MAX_SERVO_ID) return dummy;
        return _feedback[id];
    }

    bool isOnline(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].online;
    }

    bool hasFeedback(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].initialized;
    }

private:
    static int16_t decodeSigned(uint8_t lo, uint8_t hi)
    {
        return decodeSignedWithSignBit(lo, hi, 15);
    }

    static int16_t decodeSignedWithSignBit(uint8_t lo, uint8_t hi, uint8_t signBit)
    {
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        const uint16_t signMask = (uint16_t)(1U << signBit);
        return (raw & signMask) ? -(int16_t)(raw & ~signMask) : (int16_t)raw;
    }

    void _setHardwarePosition(ServoFeedbackInternal& fb, int32_t hardwarePosition)
    {
        fb.hardwareAbsolutePosition = hardwarePosition;
        if (fb.hardwareAbsolutePosition > 30719) {
            fb.hardwareAbsolutePosition = 30719;
        } else if (fb.hardwareAbsolutePosition < -30719) {
            fb.hardwareAbsolutePosition = -30719;
        }

        int16_t rawSingleTurn = (int16_t)(fb.hardwareAbsolutePosition % 4096);
        if (rawSingleTurn < 0) {
            rawSingleTurn += 4096;
        }
        fb.rawPosition = rawSingleTurn;
        fb.lastRawPosition = rawSingleTurn;
        fb.turnCount = (int16_t)((fb.hardwareAbsolutePosition - rawSingleTurn) / 4096);
        fb.initialized = true;
    }

    void _updatePresentPosition(uint8_t id, int16_t presentPosition)
    {
        ServoFeedbackInternal& fb = _feedback[id];

        int16_t rawPosition = (int16_t)(presentPosition % 4096);
        if (rawPosition < 0) {
            rawPosition += 4096;
        }

        if (!fb.initialized) {
            _setHardwarePosition(fb, rawPosition);
            return;
        }

        const int32_t previousHardware = fb.hardwareAbsolutePosition;
        int32_t bestHardware = (int32_t)fb.turnCount * 4096 + rawPosition;
        int32_t bestDistance = abs32(bestHardware - previousHardware);

        for (int16_t turn = fb.turnCount - 2; turn <= fb.turnCount + 2; turn++) {
            const int32_t candidate = (int32_t)turn * 4096 + rawPosition;
            const int32_t distance = abs32(candidate - previousHardware);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestHardware = candidate;
            }
        }

        _setHardwarePosition(fb, bestHardware);
    }

    static int32_t abs32(int32_t value)
    {
        return value < 0 ? -value : value;
    }

    SMS_STS _sms;
    HardwareSerial* _serial;
    uint8_t _writeIDs[MAX_SERVOS_PER_BUS];
    int16_t _writePos[MAX_SERVOS_PER_BUS];
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t _writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t _writeCount;
    ServoFeedbackInternal _feedback[MAX_SERVO_ID + 1];
    bool _torqueEnabled[MAX_SERVO_ID + 1];
    bool _positionModeEnabled[MAX_SERVO_ID + 1];
    int32_t _softwareZeroOffset[MAX_SERVO_ID + 1];
    uint16_t _syncReadTimeoutMs;
};

static ServoBusDriver g_servoBuses[NUM_BUSES];

static void publishServoFeedback(TaskSharedData_t* sharedData);
static uint32_t g_lastServoTargetDiagMs = 0;

static ServoBusDriver* getBusByIndex(uint8_t busIndex)
{
    if (busIndex >= NUM_BUSES) {
        return NULL;
    }
    return &g_servoBuses[busIndex];
}

static bool addReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                          uint8_t busCounts[NUM_BUSES],
                          uint8_t bus,
                          uint8_t id)
{
    if (bus >= NUM_BUSES) {
        return false;
    }
    for (uint8_t i = 0; i < busCounts[bus]; i++) {
        if (busIds[bus][i] == id) {
            return true;
        }
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) {
        return false;
    }
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

static void beginServoBuses()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    g_servoBuses[0].begin(0, 21, 20, 1000000);
    g_servoBuses[1].begin(1, 23, 22, 1000000);
    g_servoBuses[2].begin(2, 27, 26, 1000000);
    g_servoBuses[3].begin(3, 33, 32, 1000000);
    initialized = true;
}

static void configureMappedServosForMultiTurn()
{
#if SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
    static bool configuredOnce = false;
    if (configuredOnce) {
        return;
    }

    uint8_t configuredIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t configuredCounts[NUM_BUSES] = {0};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyConfigured = false;
        for (uint8_t i = 0; i < configuredCounts[busIndex]; i++) {
            if (configuredIds[busIndex][i] == id) {
                alreadyConfigured = true;
                break;
            }
        }
        if (alreadyConfigured || configuredCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->configureMultiTurnMode(id)) {
            bus->ensurePositionMode(id);
            configuredIds[busIndex][configuredCounts[busIndex]++] = id;
        }
    }
    configuredOnce = true;
#endif
}

static void applyServoTargetBatch(const ServoTargetBatch_t& batch)
{
    bool busWritePending[NUM_BUSES] = {false};
    struct ServoTargetDiag {
        bool valid;
        uint8_t busIndex;
        uint8_t servoId;
        int16_t motorTarget;
        int32_t swZero;
        int32_t hardwareTarget;
        int32_t motorAbsNow;
        int32_t hardwareAbsNow;
    };
    ServoTargetDiag mcpDiag[2] = {};

    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        ServoBusDriver* bus = getBusByIndex(cmd.busIndex);
        if (!bus) {
            continue;
        }
        bus->enablePositionControl(cmd.servoId);
        if ((cmd.busIndex == motorMap[0].busIndex && cmd.servoId == motorMap[0].servoID) ||
            (cmd.busIndex == motorMap[1].busIndex && cmd.servoId == motorMap[1].servoID)) {
            const uint8_t motorIndex =
                (cmd.busIndex == motorMap[0].busIndex && cmd.servoId == motorMap[0].servoID) ? 0 : 1;
            ServoTargetDiag& diag = mcpDiag[motorIndex];
            diag.valid = true;
            diag.busIndex = cmd.busIndex;
            diag.servoId = cmd.servoId;
            diag.motorTarget = cmd.position;
            diag.swZero = bus->getSoftwareZeroOffset(cmd.servoId);
            diag.hardwareTarget = (int32_t)cmd.position + diag.swZero;
            if (diag.hardwareTarget < -30719) diag.hardwareTarget = -30719;
            if (diag.hardwareTarget > 30719) diag.hardwareTarget = 30719;
            diag.motorAbsNow = bus->getAbsolutePosition(cmd.servoId);
            diag.hardwareAbsNow = bus->getHardwareAbsolutePosition(cmd.servoId);
        }
        bus->setTarget(cmd.servoId, cmd.position, cmd.speed, cmd.acc);
        busWritePending[cmd.busIndex] = true;
    }

    const uint32_t nowMs = millis();
    if (false &&
        (mcpDiag[0].valid || mcpDiag[1].valid) &&
        nowMs - g_lastServoTargetDiagMs >= 1000) {
        g_lastServoTargetDiagMs = nowMs;
        Serial.print("[SERVO TARGET]");
        for (uint8_t motorIndex = 0; motorIndex < 2; motorIndex++) {
            if (!mcpDiag[motorIndex].valid) {
                continue;
            }
            const ServoTargetDiag& diag = mcpDiag[motorIndex];
            Serial.printf(" M%02u bus=%u id=%u motorTarget=%d swZero=%ld hardwareTarget=%ld motorAbsNow=%ld hardwareAbsNow=%ld",
                          (unsigned)motorIndex,
                          (unsigned)diag.busIndex,
                          (unsigned)diag.servoId,
                          (int)diag.motorTarget,
                          (long)diag.swZero,
                          (long)diag.hardwareTarget,
                          (long)diag.motorAbsNow,
                          (long)diag.hardwareAbsNow);
        }
        Serial.print("\r\n");
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }

}

static bool isServoTargetBatchAllowed(TaskSharedData_t* sharedData, const ServoTargetBatch_t& batch)
{
    if (!sharedData) {
        return false;
    }

    const uint8_t owner = sharedData->servo_target_owner;
    if (owner == SERVO_TARGET_OWNER_NONE || batch.source != owner) {
        return false;
    }

    if (batch.source == SERVO_TARGET_OWNER_CONTROL) {
        return sharedData->control_enabled != 0 &&
               sharedData->system_state == SYSTEM_STATE_RUNNING;
    }

    if (batch.source == SERVO_TARGET_OWNER_CALIBRATION) {
        return sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING;
    }

    return false;
}

static void emergencyStopMappedServos(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }

    uint8_t stoppedIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t stoppedCounts[NUM_BUSES] = {0};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyStopped = false;
        for (uint8_t i = 0; i < stoppedCounts[busIndex]; i++) {
            if (stoppedIds[busIndex][i] == id) {
                alreadyStopped = true;
                break;
            }
        }
        if (alreadyStopped || stoppedCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus) {
            continue;
        }

        bus->disableTorque(id);
        stoppedIds[busIndex][stoppedCounts[busIndex]++] = id;
    }
}

static uint8_t calibrateMappedServosCurrentPositionAsZero(TaskSharedData_t* sharedData)
{
    emergencyStopMappedServos(sharedData);

    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
    if (sharedData) {
        SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
        if (!lock || xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(sharedData->targetAngles, 0, sizeof(sharedData->targetAngles));
            memset(sharedData->motorTargetRaw, 0, sizeof(sharedData->motorTargetRaw));
            memset(sharedData->motorSweepTargetRaw, 0, sizeof(sharedData->motorSweepTargetRaw));
            sharedData->joint_command_token = 0;
            sharedData->motor_command_token = 0;
            sharedData->motor_sweep_command_token = 0;
            sharedData->motor_direct_command_generation++;
            sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
            sharedData->control_mode = CONTROL_MODE_JOINT;
            if (lock) {
                xSemaphoreGive(lock);
            }
        }
        sharedData->control_enabled = 0;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
        sharedData->system_state = SYSTEM_STATE_STOPPED;
    }

    uint8_t calibratedIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t calibratedCounts[NUM_BUSES] = {0};
    uint8_t calibratedTotal = 0;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyCalibrated = false;
        for (uint8_t i = 0; i < calibratedCounts[busIndex]; i++) {
            if (calibratedIds[busIndex][i] == id) {
                alreadyCalibrated = true;
                break;
            }
        }
        if (alreadyCalibrated || calibratedCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->hasFeedback(id)) {
            continue;
        }

        bus->calibrateCurrentPositionAsZero(id);
        Serial.printf("[SERVO ZERO] ch=%u bus=%u id=%u hardware_abs=%ld software_zero_offset=%ld software_abs=%ld\n",
                      (unsigned)ch,
                      (unsigned)busIndex,
                      (unsigned)id,
                      (long)bus->getHardwareAbsolutePosition(id),
                      (long)bus->getSoftwareZeroOffset(id),
                      (long)bus->getAbsolutePosition(id));
        calibratedIds[busIndex][calibratedCounts[busIndex]++] = id;
        calibratedTotal++;
    }

    publishServoFeedback(sharedData);
    return calibratedTotal;
}

static void publishServoFeedback(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    ServoAngleData_t servoData;
    ServoAngleData_t servoRawData;
    ServoTelemetryData_t telemetryData;
    memset(&servoData, 0, sizeof(servoData));
    memset(&servoRawData, 0, sizeof(servoRawData));
    memset(&telemetryData, 0, sizeof(telemetryData));

    servoData.timestamp = millis();
    servoRawData.timestamp = servoData.timestamp;
    telemetryData.timestamp = servoData.timestamp;

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            const uint8_t busIndex = motorMap[ch].busIndex;
            const uint8_t id = motorMap[ch].servoID;
            ServoBusDriver* bus = getBusByIndex(busIndex);
            if (bus && bus->isOnline(id)) {
                const ServoFeedbackInternal& fb = bus->getFeedback(id);
                servoData.servoAngles[ch] = bus->getAbsolutePosition(id);
                servoData.softwareZeroOffsets[ch] = (int32_t)bus->getHardwareAbsolutePosition(id) - servoData.servoAngles[ch];
                servoRawData.servoRawPositions[ch] = bus->getRawPosition(id);
                servoData.onlineStatus[ch] = 1;
                servoRawData.onlineStatus[ch] = 1;
                telemetryData.speed[ch] = fb.speed;
            telemetryData.load[ch] = fb.load;
            telemetryData.voltage[ch] = fb.voltage;
            telemetryData.temperature[ch] = fb.temperature;
            telemetryData.onlineStatus[ch] = 1;
        }
    }

    if (sharedData->servoFeedbackQueue) {
        xQueueOverwrite(sharedData->servoFeedbackQueue, &servoData);
    }
    if (sharedData->servoAngleQueue) {
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
    }
    if (sharedData->servoRawQueue) {
        xQueueOverwrite(sharedData->servoRawQueue, &servoRawData);
    }
    if (sharedData->servoTelemetryQueue) {
        xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
    }
    if (sharedData->servoTelemetrySnapshotQueue) {
        xQueueOverwrite(sharedData->servoTelemetrySnapshotQueue, &telemetryData);
    }
}

void servoCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    beginServoBuses();
    configureMappedServosForMultiTurn();

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriodTicks = pdMS_TO_TICKS(10);
    uint8_t readBusPhase = 0;
    uint32_t lastEmergencyStopToken = sharedData->servo_emergency_stop_token;
    uint32_t lastInternalZeroToken = sharedData->servo_internal_zero_token;
    uint32_t ignoreControlTargetsUntilMs = 0;

    for (;;)
    {
        const uint32_t emergencyStopToken = sharedData->servo_emergency_stop_token;
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyStopMappedServos(sharedData);
        }

        const uint32_t internalZeroToken = sharedData->servo_internal_zero_token;
        if (internalZeroToken != lastInternalZeroToken) {
            lastInternalZeroToken = internalZeroToken;
            const uint8_t calibratedTotal = calibrateMappedServosCurrentPositionAsZero(sharedData);
            if (sharedData->servoTargetQueue) {
                xQueueReset(sharedData->servoTargetQueue);
            }
            if (calibratedTotal > 0) {
                sharedData->servo_internal_zero_ack_token++;
                sharedData->servo_internal_zero_last_ack_ms = millis();
            }
            ignoreControlTargetsUntilMs = millis() + 1000;
            Serial.printf("[SERVO ZERO] token=%lu ack=%lu calibrated=%u\n",
                          (unsigned long)internalZeroToken,
                          (unsigned long)sharedData->servo_internal_zero_ack_token,
                          (unsigned)calibratedTotal);
        }

        ServoTargetBatch_t batch;
        while (sharedData->servoTargetQueue &&
               xQueueReceive(sharedData->servoTargetQueue, &batch, 0) == pdTRUE) {
            if ((int32_t)(millis() - ignoreControlTargetsUntilMs) < 0) {
                continue;
            }
            if (isServoTargetBatchAllowed(sharedData, batch)) {
                applyServoTargetBatch(batch);
            }
        }

        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        for (uint8_t offset = 0; offset < 2; offset++) {
            const uint8_t busIndex = (uint8_t)(readStartBus + offset);
            if (busIndex >= NUM_BUSES || readCounts[busIndex] == 0) {
                continue;
            }
            g_servoBuses[busIndex].syncReadPositions(readIds[busIndex], readCounts[busIndex]);
        }
        readBusPhase ^= 1;

        publishServoFeedback(sharedData);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
    }
}
