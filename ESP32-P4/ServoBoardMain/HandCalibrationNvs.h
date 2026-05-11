#ifndef HAND_CALIBRATION_NVS_H
#define HAND_CALIBRATION_NVS_H

#include "TaskSharedData.h"

// 从 NVS 恢复磁编零点与机构零位对应的多圈电机位置；无有效数据则保持调用前的状态。
void handCalibrationNvsLoad(TaskSharedData_t* sd);

// 将当前 sharedData 中已标定的磁编零点 / 电机零点写入 NVS（按 valid 标志分项保存）。
void handCalibrationNvsSave(TaskSharedData_t* sd);

#endif
