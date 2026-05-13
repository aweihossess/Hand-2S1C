#include "UpperCommCommandRouter.h"

#include <math.h>
#include <string.h>

static uint32_t g_lastAngleCommandDiagMs = 0;

// Clamp a host absolute motor command to the firmware multi-turn range.
int16_t clampServoAbsCommand(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

// 是的，这里是“关节控制模式”下向所有关节应用新的目标角度的部分
void upperApplyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    // 检查指针有效性
    if (!sharedData || !angles) {
        return;
    }
    // 防止数量超出最大编码器数
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;

    // 获取用于保护目标角度的互斥锁
    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    
    // 在获得互斥锁后，写入完整的关节目标集合
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        // 增加命令令牌，切换控制模式到关节模式
        sharedData->joint_command_token++;
        sharedData->control_mode = CONTROL_MODE_JOINT;
        const uint32_t nowMs = millis();
        if (nowMs - g_lastAngleCommandDiagMs >= 200) {
            g_lastAngleCommandDiagMs = nowMs;
            Serial.printf("[ANGLE CMD] token=%lu J0=%.2f J1=%.2f J2=%.2f J3=%.2f\r\n",
                          (unsigned long)sharedData->joint_command_token,
                          (double)sharedData->targetAngles[0],
                          (double)sharedData->targetAngles[1],
                          (double)sharedData->targetAngles[2],
                          (double)sharedData->targetAngles[3]);
        }
        xSemaphoreGive(lock);
    }
}

// 清零所有关节目标角。保留为历史兼容且可用于紧急零位恢复。
// 注意：此函数会将目标角度数组（长度为 ENCODER_TOTAL_NUM）全部设为 0，并调用 upperApplyTargetAngles 应用这些零值。
void upperClearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    upperApplyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

// 应用目标电机指令（直接电机原始命令，直通模式）
void upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source)
{
    // 检查指针有效性
    if (!sharedData || !targets) {
        return;
    }
    // 防止数量超过实际电机数
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    // 获取指令状态互斥锁
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // 将直驱目标写入 motorMap 通道顺序
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorTargetRaw[i] = targets[i];
    }
    // 增加指令令牌，更新指令代数与来源，切换为直驱控制模式
    sharedData->motor_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = source;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Store sweep targets as single-turn raw positions in motorMap order.
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorSweepTargetRaw[i] = targets[i];
    }
    sharedData->motor_sweep_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_SWEEP;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperApplyTendonGuardConfig(TaskSharedData_t* sharedData,
                                 const uint8_t* enabled,
                                 const int8_t* sign,
                                 const int16_t* x1Abs,
                                 uint8_t count)
{
    if (!sharedData || !enabled || !sign || !x1Abs) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Store tendon guard config in joint order.
    for (uint8_t i = 0; i < count; i++) {
        sharedData->tendon_guard_enabled[i] = enabled[i] ? 1 : 0;
        sharedData->tendon_guard_sign[i] = (sign[i] < 0) ? -1 : 1;
        sharedData->tendon_guard_x1_abs[i] = clampServoAbsCommand(x1Abs[i]);
    }
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperCacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count)
{
    if (!sharedData || !values) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Cache the host-provided raw calibration values.
    for (uint8_t i = 0; i < count; i++) {
        float v = values[i];
        if (!isfinite(v)) {
            v = 0.0f;
        }
        sharedData->calib_zero_raw_cache[i] = (int32_t)lroundf(v);
    }
    for (uint8_t i = count; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->calib_zero_raw_cache[i] = 0;
    }
    sharedData->calib_zero_raw_valid = 1;
    if (lock) {
        xSemaphoreGive(lock);
    }
}
