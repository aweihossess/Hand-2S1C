"""
灵巧手数据模型
定义 HandModel 和相关的数据类
"""

from dataclasses import dataclass, field
from typing import List
from enum import Enum

from .protocol import ENCODER_COUNT, MOTOR_COUNT, RAW_TO_DEG


class MotorState(Enum):
    """电机状态"""
    IDLE = 0
    RUNNING = 1
    ERROR = 2
    CALIBRATING = 3


@dataclass
class JointEncoder:
    """
    关节编码器
    每个关节一个传感器
    """
    encoder_id: int = 0
    raw: int = 0              # 原始读数 (14位，0~16383)
    zero_raw: int = 0         # 零点校准值
    target_angle_deg: float = 0.0  # 目标角度 (度)
    error: bool = False       # 传感器错误标志
    
    @property
    def current_angle_deg(self) -> float:
        """
        当前角度 (度)
        正确处理编码器值跨越0点的情况
        """
        # 14位编码器范围是 0-16383
        MAX_ENCODER = 16384
        HALF_ENCODER = 8192
        
        # 计算差值，考虑环形编码器的特性
        diff = self.raw - self.zero_raw
        
        # 如果差值超过半圈，调整以得到最小角度差
        if diff > HALF_ENCODER:
            diff -= MAX_ENCODER
        elif diff < -HALF_ENCODER:
            diff += MAX_ENCODER
        
        return diff * RAW_TO_DEG


def _default_encoders() -> List[JointEncoder]:
    """创建默认编码器列表"""
    return [JointEncoder(encoder_id=i) for i in range(ENCODER_COUNT)]


@dataclass
class MotorSlot:
    """
    电机槽
    仅电机状态，角度信息由同索引的 JointEncoder 表示
    """
    motor_id: int
    error: bool = False
    state: MotorState = MotorState.IDLE


def _default_motors() -> List[MotorSlot]:
    """创建默认电机列表"""
    return [MotorSlot(motor_id=i) for i in range(MOTOR_COUNT)]


@dataclass
class HandModel:
    """
    灵巧手统一数据模型
    包含状态与控制合一的所有数据
    """
    # 21路关节编码器
    encoders: List[JointEncoder] = field(default_factory=_default_encoders)
    
    # 22路电机槽
    motors: List[MotorSlot] = field(default_factory=_default_motors)
    
    # 校准状态: NO_CALIB, IDLE, PENDING, SUCCESS, FAILED
    calib_status: str = "IDLE"
    
    # 时间戳
    timestamp: float = 0.0
    
    # 数据有效性标志
    has_sensor_data: bool = False
    has_servo_angle_data: bool = False
    has_servo_raw_data: bool = False
    
    # 舵机多圈绝对角度和在线状态 (来自 PACKET_TYPE_SERVO_ANGLE)
    servo_angles: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    
    # 舵机单圈原始位置和在线状态 (来自 PACKET_TYPE_SERVO_RAW)
    servo_raw_positions: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_raw_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    
    # 舵机遥测数据 (来自 PACKET_TYPE_SERVO_TELEM)
    servo_speed: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_load: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_voltage: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_temperature: List[int] = field(default_factory=lambda: [0] * MOTOR_COUNT)
    servo_telem_online: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    
    # 过载故障状态
    servo_overload_fault: List[bool] = field(default_factory=lambda: [False] * MOTOR_COUNT)
    
    # 关节反绕释放保护故障状态
    joint_reverse_release_fault: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    
    # 关节调试信息 (来自 PACKET_TYPE_JOINT_DEBUG)
    joint_debug_valid: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    joint_debug_target_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_actual_deg: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop1_output: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop2_actual: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_loop2_output: List[float] = field(default_factory=lambda: [0.0] * ENCODER_COUNT)
    joint_debug_cmd_valid: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)
    joint_debug_cmd_target_pos: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    
    @property
    def angles(self) -> List[float]:
        """21路关节当前角度 (度)"""
        return [e.current_angle_deg for e in self.encoders]
    
    @property
    def target_angles(self) -> List[float]:
        """21路关节目标角度 (度)"""
        return [e.target_angle_deg for e in self.encoders]
    
    def set_target_angles(self, angles: List[float]) -> None:
        """设置全部关节目标角度"""
        for i, a in enumerate(angles):
            if i < len(self.encoders):
                self.encoders[i].target_angle_deg = float(a)
    
    def set_encoder_zeros(self, zero_raw_list: List[int]) -> None:
        """
        设置全部编码器零点值
        注意: zero_raw 应该是编码器的原始读数，范围在 0-16383 (14位)
        如果传入的是有符号值，需要正确转换
        """
        for i, z in enumerate(zero_raw_list):
            if i < len(self.encoders):
                # 确保零点值在 0-16383 范围内 (14位编码器)
                zero_val = int(z)
                # 如果是负数，加上 16384 转换为正值
                if zero_val < 0:
                    zero_val += 16384
                # 确保在有效范围内
                zero_val = zero_val % 16384
                self.encoders[i].zero_raw = zero_val
    
    def copy_state_from(self, other: "HandModel") -> None:
        """复制状态数据 (不覆盖目标角度和零点)"""
        # 复制编码器状态
        for i, e in enumerate(self.encoders):
            if i < len(other.encoders):
                e.raw = other.encoders[i].raw
                e.error = other.encoders[i].error
        
        # 复制电机状态
        for i, m in enumerate(self.motors):
            if i < len(other.motors):
                m.error = other.motors[i].error
                m.state = other.motors[i].state
        
        # 复制舵机数据
        self.servo_angles = list(other.servo_angles)
        self.servo_online = list(other.servo_online)
        self.servo_raw_positions = list(other.servo_raw_positions)
        self.servo_raw_online = list(other.servo_raw_online)
        self.servo_speed = list(other.servo_speed)
        self.servo_load = list(other.servo_load)
        self.servo_voltage = list(other.servo_voltage)
        self.servo_temperature = list(other.servo_temperature)
        self.servo_telem_online = list(other.servo_telem_online)
        self.servo_overload_fault = list(other.servo_overload_fault)
        self.joint_reverse_release_fault = list(other.joint_reverse_release_fault)
        
        # 复制调试信息
        self.joint_debug_valid = list(other.joint_debug_valid)
        self.joint_debug_target_deg = list(other.joint_debug_target_deg)
        self.joint_debug_actual_deg = list(other.joint_debug_actual_deg)
        self.joint_debug_loop1_output = list(other.joint_debug_loop1_output)
        self.joint_debug_loop2_actual = list(other.joint_debug_loop2_actual)
        self.joint_debug_loop2_output = list(other.joint_debug_loop2_output)
        self.joint_debug_cmd_valid = list(other.joint_debug_cmd_valid)
        self.joint_debug_cmd_target_pos = list(other.joint_debug_cmd_target_pos)
        
        # 复制数据有效性标志
        self.has_sensor_data = bool(other.has_sensor_data)
        self.has_servo_angle_data = bool(other.has_servo_angle_data)
        self.has_servo_raw_data = bool(other.has_servo_raw_data)
