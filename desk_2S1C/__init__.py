"""
机械手上位机软件 - 2S1C

用于控制基于ESP32-P4的机械手系统
实现21路关节角度控制和22路舵机直控
兼容陈亮上位机的通信协议

版本: 2.0.0
"""

__version__ = "2.0.0"
__author__ = "AI Assistant"

from .core import (
    HandController,
    HandPose,
    HandModel,
    PRESET_POSES,
    LowerComputerComm,
    list_ports,
    ControlMode,
    ENCODER_COUNT,
    MOTOR_COUNT,
)

__all__ = [
    'HandController',
    'HandPose',
    'HandModel',
    'PRESET_POSES',
    'LowerComputerComm',
    'list_ports',
    'ControlMode',
    'ENCODER_COUNT',
    'MOTOR_COUNT',
]
