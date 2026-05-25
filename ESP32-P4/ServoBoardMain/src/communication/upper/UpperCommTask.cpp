#include "UpperCommTask.h"
#include "../can/CanCommTask.h"
#include "../../shared/TaskSharedData.h"
#include "../../calibration/CalibrationTask.h"
#include "../../system/StateMachineTask.h"
#include "UpperCommCommandRouter.h"
#include "UpperCommProtocol.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t g_calibrationUIStatus;

// UpperCommTask responsibilities:
// 1) parse host commands and update shared command state;
// 2) publish sensor, servo, telemetry, debug, and fault packets;
// 3) keep protocol ACK and fault heartbeat reporting synchronized.
static uint8_t g_sensorStreamMode = SENSOR_STREAM_MODE_SIGNED_I16;
static bool g_textMonitorMode = false;

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
        sharedData->control_mode = CONTROL_MODE_NONE;
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

static const char* skipTextSpaces(const char* p)
{
    while (p && (*p == ' ' || *p == '\t' || *p == '=' || *p == ':')) {
        p++;
    }
    return p;
}

static bool parseTextInt32(const char* p, int32_t* out, const char** endOut)
{
    if (!p || !out) {
        return false;
    }
    p = skipTextSpaces(p);
    if (!p || *p == '\0') {
        return false;
    }
    char* endPtr = NULL;
    const long value = strtol(p, &endPtr, 10);
    if (endPtr == p) {
        return false;
    }
    *out = (int32_t)value;
    if (endOut) {
        *endOut = endPtr;
    }
    return true;
}

static bool parseTextFloat(const char* p, float* out, const char** endOut)
{
    if (!p || !out) {
        return false;
    }
    p = skipTextSpaces(p);
    if (!p || *p == '\0') {
        return false;
    }
    char* endPtr = NULL;
    const float value = strtof(p, &endPtr);
    if (endPtr == p) {
        return false;
    }
    *out = value;
    if (endOut) {
        *endOut = endPtr;
    }
    return true;
}

static bool parseTextMotorTargetCommand(const char* cmd, uint8_t* channelOut, int32_t* targetOut)
{
    if (!cmd || !channelOut || !targetOut) {
        return false;
    }

    const char* p = cmd;
    if (p[0] == 'm' && p[1] >= '0' && p[1] <= '9') {
        p++;
    } else if (strncmp(p, "motor", 5) == 0) {
        p += 5;
    } else {
        return false;
    }

    int32_t channel = 0;
    if (!parseTextInt32(p, &channel, &p)) {
        return false;
    }
    int32_t target = 0;
    if (!parseTextInt32(p, &target, NULL)) {
        return false;
    }
    if (channel < 0 || channel >= SERVO_TOTAL_NUM) {
        Serial.printf("<<<MOTOR_TARGET bad_channel=%ld>>>\r\n", (long)channel);
        return false;
    }

    *channelOut = (uint8_t)channel;
    *targetOut = target;
    return true;
}

static bool parseTextJointTargetCommand(const char* cmd, uint8_t* jointOut, float* targetOut)
{
    if (!cmd || !jointOut || !targetOut) {
        return false;
    }

    const char* p = cmd;
    if (p[0] == 'j' && p[1] >= '0' && p[1] <= '9') {
        p++;
    } else if (strncmp(p, "joint", 5) == 0) {
        p += 5;
    } else if (strncmp(p, "degree", 6) == 0) {
        p += 6;
    } else if (strncmp(p, "deg", 3) == 0) {
        p += 3;
    } else {
        return false;
    }

    int32_t joint = 0;
    if (!parseTextInt32(p, &joint, &p)) {
        return false;
    }
    float target = 0.0f;
    if (!parseTextFloat(p, &target, NULL)) {
        return false;
    }
    if (joint < 0 || joint >= ENCODER_TOTAL_NUM) {
        Serial.printf("<<<JOINT_TARGET bad_joint=%ld>>>\r\n", (long)joint);
        return false;
    }

    *jointOut = (uint8_t)joint;
    *targetOut = target;
    return true;
}

static bool parseTextMcpAnglePairCommand(const char* cmd, float* j00Out, float* j01Out)
{
    if (!cmd || !j00Out || !j01Out) {
        return false;
    }

    const char* p = skipTextSpaces(cmd);
    if (!p) {
        return false;
    }

    if ((p[0] == 'j' || p[0] == 'm') && (p[1] == '\0' || p[1] == ' ' || p[1] == '\t' || p[1] == '=' || p[1] == ':')) {
        p++;
        return parseTextFloat(p, j00Out, &p) && parseTextFloat(p, j01Out, NULL);
    }

    uint8_t seenMask = 0;
    float values[2] = {0.0f, 0.0f};
    while (p && *p) {
        p = skipTextSpaces(p);
        if (!p || *p == '\0') {
            break;
        }
        if ((*p != 'j' && *p != 'm') || (p[1] != '0' && p[1] != '1')) {
            return false;
        }
        const uint8_t index = (uint8_t)(p[1] - '0');
        p += 2;
        float value = 0.0f;
        if (!parseTextFloat(p, &value, &p)) {
            return false;
        }
        values[index] = value;
        seenMask |= (uint8_t)(1U << index);
    }

    if (seenMask != 0x03) {
        return false;
    }
    *j00Out = values[0];
    *j01Out = values[1];
    return true;
}

static float clampTextJointTargetDeg(float targetDeg, uint8_t joint, bool* clamped)
{
    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    bool hasLimit = true;
    if (joint == 0) {
        minDeg = -20.0f;
        maxDeg = 30.0f;
    } else if (joint == 1) {
        minDeg = 0.0f;
        maxDeg = 90.0f;
    } else {
        hasLimit = false;
    }

    if (clamped) {
        *clamped = false;
    }
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
        if (clamped) {
            *clamped = true;
        }
    }
    if (!hasLimit) {
        return targetDeg;
    }
    if (targetDeg < minDeg) {
        if (clamped) {
            *clamped = true;
        }
        return minDeg;
    }
    if (targetDeg > maxDeg) {
        if (clamped) {
            *clamped = true;
        }
        return maxDeg;
    }
    return targetDeg;
}

static void applyTextMotorTarget(TaskSharedData_t* sharedData, uint8_t channel, int32_t target)
{
    if (!sharedData) {
        return;
    }

    if (sharedData->control_mode != CONTROL_MODE_DIRECT_MOTOR) {
        Serial.printf("<<<MOTOR_TARGET rejected: mode=%u send DIRECT first>>>\r\n",
                      (unsigned)sharedData->control_mode);
        return;
    }

    int32_t targets[SERVO_TOTAL_NUM] = {0};
    ServoAngleData_t servoData;
    bool hasServoData = false;
    if (sharedData->servoAngleQueue &&
        xQueuePeek(sharedData->servoAngleQueue, &servoData, 0) == pdTRUE) {
        hasServoData = true;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("<<<MOTOR_TARGET lock_timeout>>>");
        return;
    }

    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (hasServoData && servoData.onlineStatus[i] != 0) {
            targets[i] = servoData.servoAngles[i];
        } else {
            targets[i] = sharedData->motorTargetRaw[i];
        }
    }
    if (lock) {
        xSemaphoreGive(lock);
    }

    targets[channel] = clampServoAbsCommand(target);
    upperApplyMotorTargets(sharedData, targets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_ABSOLUTE);

    int32_t motorAbsNow = 0;
    int32_t swZeroOfs = 0;
    uint8_t online = 0;
    if (hasServoData) {
        motorAbsNow = servoData.servoAngles[channel];
        swZeroOfs = servoData.softwareZeroOffsets[channel];
        online = servoData.onlineStatus[channel];
    }
    int32_t hardwareTarget = targets[channel] + swZeroOfs;
    if (hardwareTarget < -30719) hardwareTarget = -30719;
    if (hardwareTarget > 30719) hardwareTarget = 30719;
    const int32_t hardwareAbsNow = motorAbsNow + swZeroOfs;
    const uint8_t willApply =
        (sharedData->control_enabled != 0 &&
         sharedData->servo_target_owner == SERVO_TARGET_OWNER_CONTROL &&
         sharedData->system_state == SYSTEM_STATE_RUNNING &&
         sharedData->control_mode == CONTROL_MODE_DIRECT_MOTOR) ? 1 : 0;

    Serial.printf("<<<MOTOR_TARGET M%02u target=%ld sent=%d swZero=%ld hardwareTarget=%ld motorAbsNow=%ld hardwareAbsNow=%ld online=%u apply_now=%u enabled=%u owner=%u state=%u mode=%u>>>\r\n",
                  (unsigned)channel,
                  (long)target,
                  (int)targets[channel],
                  (long)swZeroOfs,
                  (long)hardwareTarget,
                  (long)motorAbsNow,
                  (long)hardwareAbsNow,
                  (unsigned)online,
                  (unsigned)willApply,
                  (unsigned)sharedData->control_enabled,
                  (unsigned)sharedData->servo_target_owner,
                  (unsigned)sharedData->system_state,
                  (unsigned)sharedData->control_mode);
    if (!willApply) {
        Serial.println("<<<MOTOR_TARGET pending: send START then DIRECT to apply>>>");
    }
}

static void applyTextJointTarget(TaskSharedData_t* sharedData, uint8_t joint, float targetDeg)
{
    if (!sharedData) {
        return;
    }

    if (sharedData->control_mode != CONTROL_MODE_JOINT) {
        Serial.printf("<<<JOINT_TARGET rejected: mode=%u send DEGREE first>>>\r\n",
                      (unsigned)sharedData->control_mode);
        return;
    }

    float targets[ENCODER_TOTAL_NUM] = {0.0f};

    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("<<<JOINT_TARGET lock_timeout>>>");
        return;
    }

    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        targets[i] = sharedData->targetAngles[i];
    }
    if (lock) {
        xSemaphoreGive(lock);
    }

    bool targetClamped = false;
    const float appliedTargetDeg = clampTextJointTargetDeg(targetDeg, joint, &targetClamped);
    targets[joint] = appliedTargetDeg;
    upperApplyTargetAngles(sharedData, targets, ENCODER_TOTAL_NUM);
    Serial.printf("<<<JOINT_TARGET J%02u target_deg=%.2f applied_deg=%.2f clamped=%u mode=degree>>>\r\n",
                  (unsigned)joint,
                  (double)targetDeg,
                  (double)appliedTargetDeg,
                  (unsigned)(targetClamped ? 1 : 0));
}

static void applyTextMcpAnglePairTarget(TaskSharedData_t* sharedData, float j00Deg, float j01Deg)
{
    if (!sharedData) {
        return;
    }

    if (sharedData->control_mode != CONTROL_MODE_JOINT) {
        Serial.printf("<<<JOINT_TARGET rejected: mode=%u send DEGREE first>>>\r\n",
                      (unsigned)sharedData->control_mode);
        return;
    }

    float targets[ENCODER_TOTAL_NUM] = {0.0f};
    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("<<<JOINT_TARGET lock_timeout>>>");
        return;
    }

    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        targets[i] = sharedData->targetAngles[i];
    }
    if (lock) {
        xSemaphoreGive(lock);
    }

    bool j00Clamped = false;
    bool j01Clamped = false;
    const float appliedJ00 = clampTextJointTargetDeg(j00Deg, 0, &j00Clamped);
    const float appliedJ01 = clampTextJointTargetDeg(j01Deg, 1, &j01Clamped);
    targets[0] = appliedJ00;
    targets[1] = appliedJ01;
    upperApplyTargetAngles(sharedData, targets, ENCODER_TOTAL_NUM);
    Serial.printf("<<<JOINT_TARGET_PAIR J00 target_deg=%.2f applied_deg=%.2f clamped=%u J01 target_deg=%.2f applied_deg=%.2f clamped=%u mode=degree>>>\r\n",
                  (double)j00Deg,
                  (double)appliedJ00,
                  (unsigned)(j00Clamped ? 1 : 0),
                  (double)j01Deg,
                  (double)appliedJ01,
                  (unsigned)(j01Clamped ? 1 : 0));
}

static void printServoTextSnapshot(TaskSharedData_t* sharedData)
{
    if (!sharedData || !sharedData->servoAngleQueue) {
        Serial.println("<<<SERVO no_queue>>>");
        return;
    }

    ServoAngleData_t servoData;
    if (xQueuePeek(sharedData->servoAngleQueue, &servoData, 0) != pdTRUE) {
        Serial.println("<<<SERVO no_data>>>");
        return;
    }

    Serial.printf("<<<SERVO timestamp=%lu>>>\r\n", (unsigned long)servoData.timestamp);
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (servoData.onlineStatus[i] == 0) {
            continue;
        }
        const int32_t motorAbs = servoData.servoAngles[i];
        const int32_t swZeroOfs = servoData.softwareZeroOffsets[i];
        const int32_t hardwareAbs = motorAbs + swZeroOfs;
        Serial.printf("<<<SERVO M%02u motor_abs=%ld hardware_abs=%ld sw_zero_ofs=%ld online=%u>>>\r\n",
                      (unsigned)i,
                      (long)motorAbs,
                      (long)hardwareAbs,
                      (long)swZeroOfs,
                      (unsigned)servoData.onlineStatus[i]);
    }
}

static void printLoadTextSnapshot(TaskSharedData_t* sharedData, int channelFilter)
{
    if (!sharedData) {
        Serial.println("<<<LOAD no_shared_data>>>");
        return;
    }

    ServoTelemetryData_t telemetry;
    QueueHandle_t queue =
        sharedData->servoTelemetrySnapshotQueue ? sharedData->servoTelemetrySnapshotQueue : sharedData->servoTelemetryQueue;
    if (!queue || xQueuePeek(queue, &telemetry, 0) != pdTRUE) {
        Serial.println("<<<LOAD no_data>>>");
        return;
    }

    if (channelFilter >= SERVO_TOTAL_NUM) {
        Serial.printf("<<<LOAD bad_channel=%d>>>\r\n", channelFilter);
        return;
    }

    Serial.printf("<<<LOAD timestamp=%lu>>>\r\n", (unsigned long)telemetry.timestamp);
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (channelFilter >= 0 && i != (uint8_t)channelFilter) {
            continue;
        }
        if (telemetry.onlineStatus[i] == 0) {
            if (channelFilter >= 0) {
                Serial.printf("<<<LOAD M%02u online=0>>>\r\n", (unsigned)i);
            }
            continue;
        }
        Serial.printf("<<<LOAD M%02u load=%d current=%d speed=%d voltage=%u temperature=%u online=%u>>>\r\n",
                      (unsigned)i,
                      (int)telemetry.load[i],
                      (int)telemetry.current[i],
                      (int)telemetry.speed[i],
                      (unsigned)telemetry.voltage[i],
                      (unsigned)telemetry.temperature[i],
                      (unsigned)telemetry.onlineStatus[i]);
    }
}

static int parseOptionalLoadChannel(const char* cmd)
{
    if (!cmd) {
        return -1;
    }

    const char* p = cmd + 4;
    while (*p == ' ' || *p == '\t' || *p == ':' || *p == '=') {
        p++;
    }
    if (*p == '\0') {
        return -1;
    }
    if (*p == 'm') {
        p++;
    }

    int32_t channel = -1;
    if (!parseTextInt32(p, &channel, NULL)) {
        return -2;
    }
    if (channel < 0 || channel >= SERVO_TOTAL_NUM) {
        return -2;
    }
    return (int)channel;
}

static void printEncoderTextSnapshot(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        Serial.println("<<<ENCODER no_shared_data>>>");
        return;
    }

    MappedAngleData_t mappedData;
    RemoteSensorData_t rawData;
    const bool hasMapped =
        sharedData->mappedAngleQueue &&
        xQueuePeek(sharedData->mappedAngleQueue, &mappedData, 0) == pdTRUE;
    const bool hasRaw =
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &rawData, 0) == pdTRUE;

    if (!hasMapped && !hasRaw) {
        Serial.println("<<<ENCODER no_data>>>");
        return;
    }

    const uint32_t mappedTimestamp = hasMapped ? mappedData.timestamp : 0;
    const uint32_t rawTimestamp = hasRaw ? rawData.timestamp : 0;
    Serial.printf("<<<ENCODER mapped_ts=%lu raw_ts=%lu>>>\r\n",
                  (unsigned long)mappedTimestamp,
                  (unsigned long)rawTimestamp);

    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        const uint16_t raw = hasRaw ? rawData.encoderValues[i] : 0;
        const uint8_t rawValid =
            (hasRaw && rawData.isValid && rawData.errorFlags[i] == 0) ? 1 : 0;
        const int16_t mapped = hasMapped ? mappedData.angleValues[i] : 0;
        const uint8_t mappedValid =
            (hasMapped && mappedData.isValid && mappedData.validFlags[i] != 0) ? 1 : 0;
        const float deg = ((float)mapped * 360.0f) / 16384.0f;
        Serial.printf("<<<ENC J%02u raw=%u raw_valid=%u mapped=%d mapped_valid=%u deg=%.2f>>>\r\n",
                      (unsigned)i,
                      (unsigned)raw,
                      (unsigned)rawValid,
                      (int)mapped,
                      (unsigned)mappedValid,
                      deg);
    }
}

static void selectTextControlMode(TaskSharedData_t* sharedData, uint8_t mode)
{
    if (!sharedData) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("<<<MODE lock_timeout>>>");
        return;
    }

    sharedData->control_mode = mode;
    if (mode == CONTROL_MODE_JOINT) {
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->motor_command_token = 0;
        sharedData->motor_sweep_command_token = 0;
        Serial.println("<<<MODE:DEGREE waiting_for_angle_target>>>");
    } else if (mode == CONTROL_MODE_DIRECT_MOTOR) {
        sharedData->joint_command_token = 0;
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        Serial.println("<<<MODE:DIRECT waiting_for_motor_target>>>");
    } else {
        sharedData->joint_command_token = 0;
        sharedData->motor_command_token = 0;
        sharedData->motor_sweep_command_token = 0;
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        Serial.println("<<<MODE:NONE>>>");
    }

    if (lock) {
        xSemaphoreGive(lock);
    }
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
    if (textEquals(cmd, "servo") || textEquals(cmd, "motor")) {
        printServoTextSnapshot(sharedData);
        return true;
    }
    if (strncmp(cmd, "load", 4) == 0) {
        const int channelFilter = parseOptionalLoadChannel(cmd);
        if (channelFilter == -2) {
            Serial.println("<<<LOAD bad_channel>>>");
        } else {
            printLoadTextSnapshot(sharedData, channelFilter);
        }
        return true;
    }
    if (textEquals(cmd, "encoder") || textEquals(cmd, "enc")) {
        printEncoderTextSnapshot(sharedData);
        return true;
    }
    if (textEquals(cmd, "degree") || textEquals(cmd, "deg") || textEquals(cmd, "joint")) {
        selectTextControlMode(sharedData, CONTROL_MODE_JOINT);
        return true;
    }
    if (textEquals(cmd, "direct") || textEquals(cmd, "derict") || textEquals(cmd, "motor_mode")) {
        selectTextControlMode(sharedData, CONTROL_MODE_DIRECT_MOTOR);
        return true;
    }
    if (textEquals(cmd, "none") || textEquals(cmd, "idle")) {
        selectTextControlMode(sharedData, CONTROL_MODE_NONE);
        return true;
    }
    float pairJ00 = 0.0f;
    float pairJ01 = 0.0f;
    if (parseTextMcpAnglePairCommand(cmd, &pairJ00, &pairJ01)) {
        applyTextMcpAnglePairTarget(sharedData, pairJ00, pairJ01);
        return true;
    }
    uint8_t motorChannel = 0;
    int32_t motorTarget = 0;
    if (parseTextMotorTargetCommand(cmd, &motorChannel, &motorTarget)) {
        applyTextMotorTarget(sharedData, motorChannel, motorTarget);
        return true;
    }
    uint8_t jointIndex = 0;
    float jointTarget = 0.0f;
    if (parseTextJointTargetCommand(cmd, &jointIndex, &jointTarget)) {
        applyTextJointTarget(sharedData, jointIndex, jointTarget);
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
    if (textEquals(cmd, "text") || textEquals(cmd, "monitor") || textEquals(cmd, "quiet")) {
        g_textMonitorMode = true;
        Serial.println("<<<MONITOR:TEXT telemetry=off>>>");
        return true;
    }
    if (textEquals(cmd, "binary") || textEquals(cmd, "telemetry") || textEquals(cmd, "ui")) {
        g_textMonitorMode = false;
        Serial.println("<<<MONITOR:BINARY telemetry=on>>>");
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
        if (!g_textMonitorMode &&
            sharedData->mappedAngleQueue &&
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
        if (!g_textMonitorMode &&
            sharedData->tactileQueue &&
            xQueueReceive(sharedData->tactileQueue, &tactileData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, &tactileData, NULL);
            sentPacket = true;
        }

        JointDebugData_t jointDebugData;
        uint8_t jointDebugSent = 0;
        while (!g_textMonitorMode &&
               jointDebugSent < 2 &&
               xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, &jointDebugData);
            jointDebugSent++;
            sentPacket = true;
        }

        ServoAngleData_t servoRawData;
        if (!g_textMonitorMode &&
            xQueueReceive(sharedData->servoRawQueue, &servoRawData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, &servoRawData, NULL, NULL, NULL);
            sentPacket = true;
        }

        ServoTelemetryData_t telemetryData;
        if (!g_textMonitorMode &&
            xQueueReceive(sharedData->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, &telemetryData, NULL, NULL);
            sentPacket = true;
        }

        bool sentServoAngle = false;
        ServoAngleData_t servoAngleData;
        if (!g_textMonitorMode &&
            xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, &servoAngleData, NULL, NULL, NULL, NULL);
            sentServoAngle = true;
            sentPacket = true;
        }

        if (!g_textMonitorMode && !sentPacket && !sentServoAngle && g_calibrationUIStatus != 0)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        }

        const uint32_t nowMs = millis();
        const uint32_t faultBitmap = sharedData->overload_fault_bitmap;
        const bool bitmapChanged =
            (!faultStatusSentInitialized) || (faultBitmap != lastFaultStatusSent);
        const bool heartbeatDue =
            (!faultStatusSentInitialized) || ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (!g_textMonitorMode && (bitmapChanged || heartbeatDue)) {
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
        if (!g_textMonitorMode && (releaseBitmapChanged || releaseHeartbeatDue)) {
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
        if (!g_textMonitorMode && (controlChanged || controlHeartbeatDue)) {
            sendControlStatusPacket(sharedData);
            lastControlStatusSig = controlSig;
            lastControlStatusSentMs = nowMs;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
