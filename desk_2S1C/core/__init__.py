"""核心控制模块"""

from .protocol import (
    ControlMode,
    ENCODER_COUNT,
    MOTOR_COUNT,
    BAUDRATE,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    RAW_TO_DEG,
    DEG_TO_RAW,
    PACKET_TYPE_SENSOR,
    PACKET_TYPE_SERVO_ANGLE,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SERVO_TELEM,
    build_start_cmd,
    build_stop_cmd,
    build_reset_cmd,
    build_angle_cmd,
    build_motor_pos_cmd,
    build_motor_pos_abs_cmd,
)
from .data_models import (
    HandModel,
    JointEncoder,
    MotorSlot,
    MotorState,
)
from .comm_layer import (
    LowerComputerComm,
    list_ports,
    is_valid_port,
)
from .core_logic import (
    HandController,
    HandPose,
    PRESET_POSES,
)

__all__ = [
    # 协议
    'ControlMode',
    'ENCODER_COUNT',
    'MOTOR_COUNT',
    'BAUDRATE',
    'MOTOR_ABS_CMD_MIN',
    'MOTOR_ABS_CMD_MAX',
    'RAW_TO_DEG',
    'DEG_TO_RAW',
    'PACKET_TYPE_SENSOR',
    'PACKET_TYPE_SERVO_ANGLE',
    'PACKET_TYPE_SERVO_RAW',
    'PACKET_TYPE_SERVO_TELEM',
    'build_start_cmd',
    'build_stop_cmd',
    'build_reset_cmd',
    'build_angle_cmd',
    'build_motor_pos_cmd',
    'build_motor_pos_abs_cmd',
    # 数据模型
    'HandModel',
    'JointEncoder',
    'MotorSlot',
    'MotorState',
    # 通信层
    'LowerComputerComm',
    'list_ports',
    'is_valid_port',
    # 核心逻辑
    'HandController',
    'HandPose',
    'PRESET_POSES',
]
