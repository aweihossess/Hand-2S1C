#include "UpperCommTask.h"
#include "../can/CanCommTask.h"
#include "../../shared/TaskSharedData.h"
#include "../../calibration/CalibrationTask.h"
#include "../../system/StateMachineTask.h"
#include "UpperCommCommandRouter.h"
#include "UpperCommProtocol.h"

#include <math.h>
#include <string.h>

extern volatile uint8_t g_calibrationUIStatus;

// UpperCommTask responsibilities:
// 1) parse host commands and update shared command state;
// 2) publish sensor, servo, telemetry, debug, and fault packets;
// 3) keep protocol ACK and fault heartbeat reporting synchronized.
static uint8_t g_sensorStreamMode = SENSOR_STREAM_MODE_SIGNED_I16;

// Joint-mode motion gate diagnostics bits (device -> host).
static const uint32_t kJointGateNoTarget = (1UL << 0);
static const uint32_t kJointGateControlDisabled = (1UL << 1);
static const uint32_t kJointGateOwnerNotControl = (1UL << 2);
static const uint32_t kJointGateModeNotJoint = (1UL << 3);
static const uint32_t kJointGateStateNotRunning = (1UL << 4);
static const uint32_t kJointGateFaultHold = (1UL << 5);
static const uint32_t kJointGateSystemFault = (1UL << 6);

// Encode raw CAN encoder counts for wire transport.
static uint16_t encodeRawCountForWire(uint16_t rawValue, bool channelValid)
{
    if (!channelValid) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }
    if (rawValue == (uint16_t)PROTOCOL_DISCONNECT_SENTINEL) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }
    return rawValue;
}

// Append a float using big-endian wire order.
static void appendFloatBigEndian(uint8_t* buffer, size_t* idx, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    buffer[(*idx)++] = (uint8_t)((bits >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(bits & 0xFF);
}

// Encode mapped encoder counts for the sensor packet.
// The payload uses signed int16 in stream mode 1, with 0x7FFF reserved as disconnect sentinel.
static uint16_t encodeMappedCountForWire(int16_t mappedCount, bool channelValid)
{
    if (!channelValid) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    if (mappedCount == (int16_t)PROTOCOL_DISCONNECT_SENTINEL) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    return (uint16_t)mappedCount;
}

// Send a protocol ACK packet.
static void sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PROTO_ACK;
    buffer[idx++] = cmd;
    buffer[idx++] = appliedMode;
    buffer[idx++] = status;
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

// Send the servo overload fault bitmap.
static void sendFaultStatusPacket(uint32_t overloadFaultBitmap)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_FAULT_STATUS;
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(overloadFaultBitmap & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

// Send the joint reverse-release fault bitmap.
static void sendReleaseFaultPacket(uint32_t releaseFaultBitmap)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_RELEASE_FAULT;
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(releaseFaultBitmap & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

// Send a compact "why joint motion is blocked" status packet for host-side debugging.
static void sendControlStatusPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    const uint8_t mode = sharedData->control_mode;
    const uint8_t controlEnabled = (sharedData->control_enabled != 0) ? 1 : 0;
    const uint8_t owner = sharedData->servo_target_owner;
    const uint8_t state = sharedData->system_state;
    const uint32_t faultBitmap = sharedData->system_fault_bitmap;
    const uint32_t jointToken = sharedData->joint_command_token;

    uint32_t reason = 0;
    if (jointToken == 0) reason |= kJointGateNoTarget;
    if (controlEnabled == 0) reason |= kJointGateControlDisabled;
    if (owner != SERVO_TARGET_OWNER_CONTROL) reason |= kJointGateOwnerNotControl;
    if (mode != CONTROL_MODE_JOINT) reason |= kJointGateModeNotJoint;
    if (state != SYSTEM_STATE_RUNNING) reason |= kJointGateStateNotRunning;
    if (state == SYSTEM_STATE_FAULT_HOLD) reason |= kJointGateFaultHold;
    if (faultBitmap != 0) reason |= kJointGateSystemFault;

    const uint8_t ready = (reason == 0) ? 1 : 0;

    uint8_t buffer[32];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_CONTROL_STATUS;
    buffer[idx++] = mode;
    buffer[idx++] = controlEnabled;
    buffer[idx++] = owner;
    buffer[idx++] = state;
    buffer[idx++] = ready;
    buffer[idx++] = (uint8_t)((faultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(faultBitmap & 0xFF);
    buffer[idx++] = (uint8_t)((reason >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((reason >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((reason >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(reason & 0xFF);
    buffer[idx++] = (uint8_t)((jointToken >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((jointToken >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((jointToken >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(jointToken & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

// Serialize one upstream data packet.
static void sendDataPacket(ServoStatus_t* pServo,
                           MappedAngleData_t* pSensorMapped,
                           RemoteSensorData_t* pSensorRaw,
                           ServoAngleData_t* pServoAngle,
                           ServoAngleData_t* pServoRaw,
                           ServoTelemetryData_t* pTelemetry,
                           RemoteTactileData_t* pTactile,
                           JointDebugData_t* pJointDebug)
{
    (void)pServo;

    uint8_t buffer[256];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // LEN placeholder.
    if (g_calibrationUIStatus != 0)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus;
    }
    else if (pSensorMapped)
    {
        buffer[idx++] = PACKET_TYPE_SENSOR;
        // 先发送原始计数（raw），再发送已零位映射后的计数（mapped）。
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const bool rawValid =
                pSensorRaw &&
                pSensorRaw->isValid &&
                (pSensorRaw->errorFlags[i] == 0);
            const uint16_t rawVal = rawValid ? pSensorRaw->encoderValues[i] : 0;
            const uint16_t wireRaw = encodeRawCountForWire(rawVal, rawValid);
            buffer[idx++] = (uint8_t)((wireRaw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(wireRaw & 0xFF);
        }

        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const bool channelValid =
                pSensorMapped->isValid &&
                (pSensorMapped->validFlags[i] != 0);
            const uint16_t val = encodeMappedCountForWire(pSensorMapped->angleValues[i], channelValid);
            buffer[idx++] = (uint8_t)((val >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(val & 0xFF);
        }
    }
    else if (pServoAngle)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (uint8_t)((angle >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(angle & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t offset = pServoAngle->softwareZeroOffsets[i];
            buffer[idx++] = (uint8_t)((offset >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((offset >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((offset >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(offset & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else if (pServoRaw)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_RAW;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const uint16_t raw = (uint16_t)pServoRaw->servoRawPositions[i];
            buffer[idx++] = (uint8_t)((raw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(raw & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoRaw->onlineStatus[i];
        }
    }
    else if (pTelemetry)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int16_t speed = pTelemetry->speed[i];
            const int16_t load = pTelemetry->load[i];
            buffer[idx++] = (uint8_t)(((uint16_t)speed >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)speed & 0xFF);
            buffer[idx++] = (uint8_t)(((uint16_t)load >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)load & 0xFF);
            buffer[idx++] = pTelemetry->voltage[i];
            buffer[idx++] = pTelemetry->temperature[i];
            buffer[idx++] = pTelemetry->onlineStatus[i];
        }
    }
    else if (pTactile)
    {
        buffer[idx++] = PACKET_TYPE_TACTILE;
        buffer[idx++] = pTactile->seq;
        for (int i = 0; i < TACTILE_SUMMARY_BYTES; i++)
        {
            buffer[idx++] = pTactile->values[i];
        }
    }
    else if (pJointDebug)
    {
        buffer[idx++] = PACKET_TYPE_JOINT1_DEBUG;
        buffer[idx++] = pJointDebug->jointIndex;
        buffer[idx++] = pJointDebug->valid;
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->magActualDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop1Output);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Actual);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Output);
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetLength);
        appendFloatBigEndian(buffer, &idx, pJointDebug->actualLength);
        appendFloatBigEndian(buffer, &idx, pJointDebug->mappedMotorTarget);
        const uint32_t motorZeroRaw = (uint32_t)pJointDebug->motorZeroAbs;
        buffer[idx++] = (uint8_t)((motorZeroRaw >> 24) & 0xFF);
        buffer[idx++] = (uint8_t)((motorZeroRaw >> 16) & 0xFF);
        buffer[idx++] = (uint8_t)((motorZeroRaw >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(motorZeroRaw & 0xFF);
        const uint16_t solverOutputRaw = (uint16_t)pJointDebug->solverOutputPos;
        buffer[idx++] = (uint8_t)((solverOutputRaw >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(solverOutputRaw & 0xFF);
        buffer[idx++] = pJointDebug->zeroHoming;
        buffer[idx++] = pJointDebug->cmdValid;
        const uint16_t cmdPosRaw = (uint16_t)pJointDebug->cmdTargetPos;
        buffer[idx++] = (uint8_t)((cmdPosRaw >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(cmdPosRaw & 0xFF);
    }
    else
    {
        return;
    }

    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);

    if (g_calibrationUIStatus != 0) {
        g_calibrationUIStatus = 0;
    }
}

// Decode a little-endian float from command payload.
static float decodeFloatLittleEndian(const uint8_t* data)
{
    uint32_t bits = 0;
    bits |= (uint32_t)data[0];
    bits |= ((uint32_t)data[1] << 8);
    bits |= ((uint32_t)data[2] << 16);
    bits |= ((uint32_t)data[3] << 24);
    float out = 0.0f;
    memcpy(&out, &bits, sizeof(out));
    return out;
}

// Parse a little-endian float array from command payload.
static bool parseFloatArrayLittleEndian(const uint8_t* payload, size_t payloadLen, float* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(float);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        outValues[i] = decodeFloatLittleEndian(payload + (size_t)i * sizeof(float));
    }
    return true;
}

// Parse a big-endian int16 array from command payload.
static bool parseInt16ArrayBigEndian(const uint8_t* payload, size_t payloadLen, int32_t* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(uint16_t);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        const uint16_t raw = ((uint16_t)payload[(size_t)i * 2] << 8) | payload[(size_t)i * 2 + 1];
        outValues[i] = (int16_t)raw;
    }
    return true;
}

static bool parseTendonGuardPayload(const uint8_t* payload,
                                    size_t payloadLen,
                                    uint8_t* enabled,
                                    int8_t* sign,
                                    int16_t* x1Abs)
{
    if (!payload || !enabled || !sign || !x1Abs) {
        return false;
    }
    if (payloadLen < kTendonGuardPayloadBytes) {
        return false;
    }
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        const size_t base = (size_t)i * 4;
        enabled[i] = (payload[base] != 0) ? 1 : 0;
        sign[i] = (((int8_t)payload[base + 1]) < 0) ? (int8_t)-1 : (int8_t)1;
        const uint16_t raw = ((uint16_t)payload[base + 2] << 8) | payload[base + 3];
        x1Abs[i] = clampServoAbsCommand((int16_t)raw);
    }
    return true;
}

// Return command payload length, excluding the command byte.
static size_t getCommandPayloadLength(uint8_t cmd)
{
    switch (cmd)
    {
        case CMD_CALIBRATE:
        case CMD_START:
        case CMD_STOP:
        case CMD_RESET:
        case CMD_SERVO_INTERNAL_ZERO:
            return 0;
        case CMD_ANGLE_CTRL:
            return kFloatPayloadBytes;
        case CMD_CALIB_DATA:
            return kFloatPayloadBytes;
        case CMD_MOTOR_POS:
            return kMotorPosPayloadBytes;
        case CMD_MOTOR_POS_SWEEP:
            return kMotorPosPayloadBytes;
        case CMD_MOTOR_POS_ABS:
            return kMotorPosPayloadBytes;
        case CMD_TENDON_GUARD:
            return kTendonGuardPayloadBytes;
        case CMD_SENSOR_STREAM_MODE:
            return 1;
        default:
            return (size_t)-1;
    }
}

// Handle legacy single-byte commands.
static void handleLegacySingleByteCommand(TaskSharedData_t* sharedData, uint8_t cmd)
{
    if (cmd == (uint8_t)'c') {
        postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        return;
    }
    if (cmd == (uint8_t)'b') {
        upperClearTargetAngles(sharedData);
    }
}

static void requestServoInternalZero(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    sharedData->control_enabled = 0;
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->system_state = SYSTEM_STATE_STOPPED;
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
    if (sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
    sharedData->servo_internal_zero_token++;
    sendProtoAckPacket(CMD_SERVO_INTERNAL_ZERO, 0, PROTO_ACK_STATUS_OK);
}

static bool textEquals(const char* value, const char* expected)
{
    if (!value || !expected) {
        return false;
    }
    while (*value && *expected) {
        if (*value != *expected) {
            return false;
        }
        value++;
        expected++;
    }
    return *value == '\0' && *expected == '\0';
}

static bool handleTextCommandLine(TaskSharedData_t* sharedData, const uint8_t* line, size_t len)
{
    if (!sharedData || !line || len == 0) {
        return false;
    }

    while (len > 0 && (line[0] == ' ' || line[0] == '\t' || line[0] == '\r' || line[0] == '\n')) {
        line++;
        len--;
    }
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r' || line[len - 1] == '\n')) {
        len--;
    }
    if (len == 0 || len >= 32) {
        return false;
    }

    char cmd[32];
    for (size_t i = 0; i < len; i++) {
        char c = (char)line[i];
        if (c >= 'A' && c <= 'Z') {
            c = (char)(c - 'A' + 'a');
        }
        cmd[i] = c;
    }
    cmd[len] = '\0';

    if (textEquals(cmd, "start") || textEquals(cmd, "enable") || textEquals(cmd, "run")) {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        Serial.println("<<<CMD:START>>>");
        return true;
    }
    if (textEquals(cmd, "stop") || textEquals(cmd, "disable") || textEquals(cmd, "halt")) {
        postSystemEvent(sharedData, SYSTEM_EVENT_STOP);
        Serial.println("<<<CMD:STOP>>>");
        return true;
    }
    if (textEquals(cmd, "reset")) {
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        Serial.println("<<<CMD:RESET>>>");
        return true;
    }
    if (textEquals(cmd, "zero") || textEquals(cmd, "setzero") || textEquals(cmd, "servozero")) {
        requestServoInternalZero(sharedData);
        Serial.println("<<<CMD:ZERO>>>");
        return true;
    }
    if (textEquals(cmd, "status")) {
        Serial.printf("<<<STATUS mode=%u enabled=%u owner=%u state=%u fault=0x%08lX joint_token=%lu motor_token=%lu>>>\r\n",
                      (unsigned)sharedData->control_mode,
                      (unsigned)sharedData->control_enabled,
                      (unsigned)sharedData->servo_target_owner,
                      (unsigned)sharedData->system_state,
                      (unsigned long)sharedData->overload_fault_bitmap,
                      (unsigned long)sharedData->joint_command_token,
                      (unsigned long)sharedData->motor_command_token);
        return true;
    }
    Serial.printf("<<<CMD:UNKNOWN %s>>>\r\n", cmd);
    return true;
}

// Apply a parsed downstream command frame.
static void handleParsedCommand(TaskSharedData_t* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];

    if (cmd == CMD_CALIBRATE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        return;
    }

    if (cmd == CMD_START)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        return;
    }

    if (cmd == CMD_STOP)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_STOP);
        return;
    }

    if (cmd == CMD_RESET)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        return;
    }

    // 如果收到 ANGLE_CTRL 指令，解析后下发关节角目标数组，切换为关节 PID 控制模式
    if (cmd == CMD_ANGLE_CTRL)
    {
        // 解析关节角目标数组
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            // 应用关节角目标数组
            upperApplyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_CALIB_DATA)
    {
        float zeroRaw[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, zeroRaw, ENCODER_TOTAL_NUM)) {
            upperCacheCalibZeroRaw(sharedData, zeroRaw, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_TARGET);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_SWEEP)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorSweepTargets(sharedData, motorTargets, SERVO_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }

    if (cmd == CMD_TENDON_GUARD)
    {
        uint8_t enabled[ENCODER_TOTAL_NUM] = {0};
        int8_t sign[ENCODER_TOTAL_NUM] = {0};
        int16_t x1Abs[ENCODER_TOTAL_NUM] = {0};
        if (parseTendonGuardPayload(frame + 1, frameLen - 1, enabled, sign, x1Abs)) {
            upperApplyTendonGuardConfig(sharedData, enabled, sign, x1Abs, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_SERVO_INTERNAL_ZERO)
    {
        requestServoInternalZero(sharedData);
        return;
    }

    if (cmd == CMD_SENSOR_STREAM_MODE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 2) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t requestedMode = frame[1];
            if (requestedMode == SENSOR_STREAM_MODE_LEGACY_U16_WRAP ||
                requestedMode == SENSOR_STREAM_MODE_SIGNED_I16) {
                g_sensorStreamMode = requestedMode;
            } else {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            }
        }
        sendProtoAckPacket(CMD_SENSOR_STREAM_MODE, g_sensorStreamMode, status);
    }
}

// Serial receive/transmit task.
void upperCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    MappedAngleData_t sensorMappedData;
    uint8_t rxBuffer[kSerialRxBufferSize];
    size_t rxLen = 0;
    uint32_t lastSensorMappedTimestampSent = 0;

    uint32_t lastFaultStatusSent = 0;
    uint32_t lastFaultStatusSentMs = 0;
    bool faultStatusSentInitialized = false;
    uint32_t lastReleaseFaultSent = 0;
    uint32_t lastReleaseFaultSentMs = 0;
    bool releaseFaultSentInitialized = false;
    uint32_t lastControlStatusSig = 0;
    uint32_t lastControlStatusSentMs = 0;

    Serial.println("<<<SYS_READY>>>");

    for (;;)
    {
        while (Serial.available())
        {
            const int byteVal = Serial.read();
            if (byteVal < 0) {
                break;
            }

            if (rxLen < sizeof(rxBuffer)) {
                rxBuffer[rxLen++] = (uint8_t)byteVal;
            } else {
                memmove(rxBuffer, rxBuffer + 1, sizeof(rxBuffer) - 1);
                rxBuffer[sizeof(rxBuffer) - 1] = (uint8_t)byteVal;
                rxLen = sizeof(rxBuffer);
            }
        }

        size_t parseOffset = 0;
        while (parseOffset < rxLen)
        {
            const uint8_t cur = rxBuffer[parseOffset];

            if (cur == (uint8_t)'c' || cur == (uint8_t)'b')
            {
                handleLegacySingleByteCommand(sharedData, cur);
                parseOffset += 1;
                continue;
            }

            if ((cur >= (uint8_t)'A' && cur <= (uint8_t)'Z') ||
                (cur >= (uint8_t)'a' && cur <= (uint8_t)'z') ||
                cur == (uint8_t)' ' || cur == (uint8_t)'\t')
            {
                size_t lineEnd = parseOffset;
                while (lineEnd < rxLen &&
                       rxBuffer[lineEnd] != (uint8_t)'\n' &&
                       rxBuffer[lineEnd] != (uint8_t)'\r') {
                    lineEnd++;
                }

                if (lineEnd >= rxLen) {
                    break;
                }

                handleTextCommandLine(sharedData, rxBuffer + parseOffset, lineEnd - parseOffset);
                parseOffset = lineEnd;
                while (parseOffset < rxLen &&
                       (rxBuffer[parseOffset] == (uint8_t)'\n' ||
                        rxBuffer[parseOffset] == (uint8_t)'\r')) {
                    parseOffset++;
                }
                continue;
            }

            if (cur != PROTOCOL_HEADER)
            {
                parseOffset += 1;
                continue;
            }

            if ((parseOffset + 4) > rxLen)
            {
                break;
            }

            const uint8_t wireLen = rxBuffer[parseOffset + 1];
            if (wireLen < 2)
            {
                parseOffset += 1;
                continue;
            }

            const size_t frameLen = (size_t)wireLen + 2;
            if ((parseOffset + frameLen) > rxLen)
            {
                break;
            }

            if (rxBuffer[parseOffset + frameLen - 1] != PROTOCOL_TAIL)
            {
                parseOffset += 1;
                continue;
            }

            const uint8_t cmd = rxBuffer[parseOffset + 2];
            const size_t payloadLen = (size_t)wireLen - 2; // CMD + payload + tail
            const size_t expectedPayloadLen = getCommandPayloadLength(cmd);
            if (expectedPayloadLen == (size_t)-1 || expectedPayloadLen != payloadLen)
            {
                parseOffset += frameLen;
                continue;
            }

            handleParsedCommand(sharedData, rxBuffer + parseOffset + 2, payloadLen + 1);
            parseOffset += frameLen;
        }

        if (parseOffset > 0)
        {
            const size_t remain = rxLen - parseOffset;
            if (remain > 0) {
                memmove(rxBuffer, rxBuffer + parseOffset, remain);
            }
            rxLen = remain;
        }

        bool sentPacket = false;
        if (sharedData->mappedAngleQueue &&
            xQueuePeek(sharedData->mappedAngleQueue, &sensorMappedData, 0) == pdTRUE &&
            sensorMappedData.timestamp != lastSensorMappedTimestampSent)
        {
            RemoteSensorData_t sensorRawData;
            RemoteSensorData_t* pSensorRaw = NULL;
            if (sharedData->canRxQueue &&
                xQueuePeek(sharedData->canRxQueue, &sensorRawData, 0) == pdTRUE) {
                pSensorRaw = &sensorRawData;
            }
            sendDataPacket(NULL, &sensorMappedData, pSensorRaw, NULL, NULL, NULL, NULL, NULL);
            lastSensorMappedTimestampSent = sensorMappedData.timestamp;
            sentPacket = true;
        }

        RemoteTactileData_t tactileData;
        if (sharedData->tactileQueue &&
            xQueueReceive(sharedData->tactileQueue, &tactileData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, &tactileData, NULL);
            sentPacket = true;
        }

        JointDebugData_t jointDebugData;
        uint8_t jointDebugSent = 0;
        while (jointDebugSent < 2 &&
               xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, &jointDebugData);
            jointDebugSent++;
            sentPacket = true;
        }

        ServoAngleData_t servoRawData;
        if (xQueueReceive(sharedData->servoRawQueue, &servoRawData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, &servoRawData, NULL, NULL, NULL);
            sentPacket = true;
        }

        ServoTelemetryData_t telemetryData;
        if (xQueueReceive(sharedData->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, &telemetryData, NULL, NULL);
            sentPacket = true;
        }

        bool sentServoAngle = false;
        ServoAngleData_t servoAngleData;
        if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, &servoAngleData, NULL, NULL, NULL, NULL);
            sentServoAngle = true;
            sentPacket = true;
        }

        if (!sentPacket && !sentServoAngle && g_calibrationUIStatus != 0)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        }

        const uint32_t nowMs = millis();
        const uint32_t faultBitmap = sharedData->overload_fault_bitmap;
        const bool bitmapChanged =
            (!faultStatusSentInitialized) || (faultBitmap != lastFaultStatusSent);
        const bool heartbeatDue =
            (!faultStatusSentInitialized) || ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (bitmapChanged || heartbeatDue) {
            sendFaultStatusPacket(faultBitmap);
            lastFaultStatusSent = faultBitmap;
            lastFaultStatusSentMs = nowMs;
            faultStatusSentInitialized = true;
        }

        const uint32_t releaseFaultBitmap = sharedData->reverse_release_fault_bitmap;
        const bool releaseBitmapChanged =
            (!releaseFaultSentInitialized) || (releaseFaultBitmap != lastReleaseFaultSent);
        const bool releaseHeartbeatDue =
            (!releaseFaultSentInitialized) || ((nowMs - lastReleaseFaultSentMs) >= kFaultStatusHeartbeatMs);
        if (releaseBitmapChanged || releaseHeartbeatDue) {
            sendReleaseFaultPacket(releaseFaultBitmap);
            lastReleaseFaultSent = releaseFaultBitmap;
            lastReleaseFaultSentMs = nowMs;
            releaseFaultSentInitialized = true;
        }

        const uint32_t controlSig =
            ((uint32_t)(sharedData->control_mode & 0xFF) << 24) |
            ((uint32_t)(sharedData->control_enabled & 0x01) << 23) |
            ((uint32_t)(sharedData->servo_target_owner & 0x0F) << 19) |
            ((uint32_t)(sharedData->system_state & 0x0F) << 15) |
            (sharedData->system_fault_bitmap & 0x7FFF);
        const bool controlChanged = (controlSig != lastControlStatusSig);
        const bool controlHeartbeatDue = ((nowMs - lastControlStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (controlChanged || controlHeartbeatDue) {
            sendControlStatusPacket(sharedData);
            lastControlStatusSig = controlSig;
            lastControlStatusSentMs = nowMs;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
