"""
灵巧手控制器核心逻辑
提供高级控制接口
"""

import math
import queue
import threading
import time
from typing import List, Optional, Callable
from enum import IntEnum

from .protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    DEG_TO_RAW,
    RAW_TO_DEG,
)
from .data_models import HandModel, MotorState
from .comm_layer import LowerComputerComm, list_ports


class HandPose(IntEnum):
    """预设手势枚举"""
    OPEN = 0           # 张开
    FIST = 1           # 握拳
    GRIP = 2           # 握持
    POINT = 3          # 指向
    PEACE = 4          # 剪刀手
    OK = 5             # OK手势
    THUMB_UP = 6       # 点赞
    PINCH = 7          # 捏取
    RELAX = 8          # 放松


# 预设手势配置 (21路关节角度，单位：度)
PRESET_POSES: dict = {
    HandPose.OPEN: [
        0, 0, 0, 0,      # thumb
        0, 0, 0,         # index
        0, 0, 0,         # middle
        0, 0, 0,         # ring
        0, 0, 0,         # pinky
        0, 0, 0, 0, 0    # wrist + others
    ],
    HandPose.FIST: [
        30, 60, 30, 0,   # thumb
        90, 90, 90,      # index
        90, 90, 90,      # middle
        90, 90, 90,      # ring
        90, 90, 90,      # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.GRIP: [
        45, 45, 30, 0,   # thumb
        45, 60, 45,      # index
        45, 60, 45,      # middle
        45, 60, 45,      # ring
        45, 60, 45,      # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.POINT: [
        30, 30, 15, 0,   # thumb
        0, 0, 0,         # index (pointing)
        90, 90, 90,      # middle
        90, 90, 90,      # ring
        90, 90, 90,      # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.PEACE: [
        30, 30, 15, 0,   # thumb
        0, 0, 0,         # index
        0, 0, 0,         # middle
        90, 90, 90,      # ring
        90, 90, 90,      # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.OK: [
        60, 60, 45, 0,   # thumb bent
        60, 60, 45,      # index touching thumb
        0, 0, 0,         # middle
        0, 0, 0,         # ring
        0, 0, 0,         # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.THUMB_UP: [
        0, 0, 0, 0,      # thumb up
        90, 90, 90,      # index
        90, 90, 90,      # middle
        90, 90, 90,      # ring
        90, 90, 90,      # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.PINCH: [
        45, 45, 30, 0,   # thumb
        45, 45, 45,      # index
        0, 0, 0,         # middle
        0, 0, 0,         # ring
        0, 0, 0,         # pinky
        0, 0, 0, 0, 0
    ],
    HandPose.RELAX: [
        10, 10, 5, 0,
        15, 15, 10,
        15, 15, 10,
        15, 15, 10,
        15, 15, 10,
        0, 0, 0, 0, 0
    ],
}


class HandController:
    """
    灵巧手控制器
    提供高级控制接口
    """
    
    def __init__(self):
        self.comm: Optional[LowerComputerComm] = None
        self.current_state = HandModel()
        
        self.state_lock = threading.Lock()
        self.running = False
        self.update_callbacks: List[Callable[[HandModel], None]] = []
        
        # 控制参数
        self._pid_enabled = False
        self._paused = False
        self._control_mode = ControlMode.JOINT_ANGLE
        self._started = False
        
        # 更新线程
        self._update_thread: Optional[threading.Thread] = None
    
    def set_control_mode(self, mode: ControlMode):
        """设置控制模式"""
        self._control_mode = mode
    
    def get_control_mode(self) -> ControlMode:
        """获取当前控制模式"""
        return self._control_mode
    
    def set_pid_control(self, enabled: bool):
        """设置PID控制使能"""
        self._pid_enabled = enabled
    
    def is_pid_enabled(self) -> bool:
        """检查PID是否使能"""
        return self._pid_enabled
    
    def pause(self):
        """暂停发送控制指令"""
        self._paused = True
    
    def resume(self):
        """恢复发送控制指令"""
        self._paused = False
    
    def is_paused(self) -> bool:
        """检查是否暂停"""
        return self._paused
    
    def toggle_pause(self) -> bool:
        """切换暂停状态"""
        self._paused = not self._paused
        return self._paused
    
    def initialize(self, comm_port: str) -> bool:
        """
        初始化控制器并连接串口
        返回是否成功
        """
        try:
            self.comm = LowerComputerComm(comm_port)
        except (ValueError, RuntimeError) as exc:
            print(f"连接失败: {exc}")
            return False
        
        if not self.comm.connect():
            return False
        
        self.running = True
        self._started = False
        
        # 启动状态更新线程
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        
        return True
    
    def shutdown(self):
        """安全关闭控制器"""
        # 停止下位机
        if self.comm:
            try:
                self.comm.send_command("stop")
                time.sleep(0.15)
            except Exception as e:
                print(f"警告: 发送停止命令失败: {e}")
        
        # 标记停止
        self.running = False
        self._started = False
        
        # 断开连接
        if self.comm:
            try:
                self.comm.disconnect()
            except Exception as e:
                print(f"警告: 断开连接失败: {e}")
            time.sleep(0.1)
        
        # 等待线程退出
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
        self._update_thread = None
        
        # 清理
        with self.state_lock:
            self.update_callbacks.clear()
        self._paused = False
    
    def _update_loop(self):
        """状态更新线程"""
        while self.running:
            try:
                if not self.comm:
                    break
                
                # 从接收队列获取数据
                new_model = self.comm.rx_queue.get(timeout=0.1)
                
                with self.state_lock:
                    self.current_state.copy_state_from(new_model)
                    self.current_state.calib_status = new_model.calib_status
                    self.current_state.timestamp = new_model.timestamp
                
                # 调用回调
                for cb in self.update_callbacks:
                    try:
                        cb(self.current_state)
                    except Exception:
                        pass
                        
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    print(f"更新循环异常: {e}")
                break
    
    def register_update_callback(self, callback: Callable[[HandModel], None]):
        """注册状态更新回调函数"""
        self.update_callbacks.append(callback)
    
    def unregister_update_callback(self, callback: Callable[[HandModel], None]):
        """注销状态更新回调函数"""
        if callback in self.update_callbacks:
            self.update_callbacks.remove(callback)
    
    # ===== 控制命令 =====
    
    def start(self):
        """启动控制"""
        if self._paused:
            return
        if self.comm:
            self.comm.send_command("start")
            self._started = True
    
    def stop(self):
        """停止控制"""
        if self.comm:
            self.comm.send_command("stop")
        self._started = False
    
    def reset(self):
        """复位系统"""
        if self._paused:
            return
        if self.comm:
            self.comm.send_command("reset")
        self._started = False
        
        # 重置目标角度
        with self.state_lock:
            self.current_state.set_target_angles([0.0] * ENCODER_COUNT)
    
    def is_started(self) -> bool:
        """检查是否已启动"""
        return self._started
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.comm is not None and self.comm.is_connected()
    
    # ===== 关节角度控制 =====
    
    def set_target_angles(self, angles: List[float]):
        """
        设置21路关节目标角度 (度)
        JOINT_ANGLE模式下发送到下位机
        """
        if self._paused:
            return
        
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")
        
        angles = self._validate_angles(angles)
        
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            self.comm.send_command(self.current_state)
    
    def set_target_angles_live(self, angles: List[float]):
        """
        实时设置关节目标角度 (用于滑条跟随)
        使用高速通道，避免队列积压
        """
        if self._paused:
            return
        
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")
        
        angles = self._validate_angles(angles)
        
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            self.comm.send_command(("angle_live", list(angles)))
    
    def invalidate_joint_commands(self) -> int:
        """清除待发的关节控制命令"""
        if not self.comm:
            return 0
        try:
            return int(self.comm.invalidate_pending_joint_commands())
        except Exception:
            return 0
    
    # ===== 电机直控 =====
    
    def set_motor_positions_raw(self, motor_pos_raw: List[int]):
        """
        设置22路电机单圈原始位置
        DIRECT_MOTOR模式下发送
        """
        if self._paused:
            return
        
        if len(motor_pos_raw) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个位置值")
        
        raw = [int(v) & 0xFFFF for v in motor_pos_raw]
        
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos", raw))
    
    def set_motor_positions_raw_sweep(self, motor_pos_raw: List[int]):
        """
        设置电机滑条扫动目标
        """
        if self._paused:
            return
        
        if len(motor_pos_raw) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个位置值")
        
        raw = [int(v) & 0xFFFF for v in motor_pos_raw]
        
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_sweep", raw))
    
    def set_motor_positions_absolute(self, motor_abs: List[int]):
        """
        设置22路电机多圈绝对位置
        范围: -30719 ~ 30719
        """
        if self._paused:
            return
        
        if len(motor_abs) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个绝对位置值")
        
        raw = [max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(v))) for v in motor_abs]
        
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs", raw))
    
    def send_motor_positions_absolute_force(self, motor_abs: List[int]) -> None:
        """
        发送 22 路多圈绝对位置（不检查当前 UI 控制模式）。
        下位机收到 CMD_MOTOR_POS_ABS 会切换到直控模式。用于 MCP Fe/AA 等。
        """
        if self._paused or not self.comm:
            return
        if len(motor_abs) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个绝对位置值")
        raw = [max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(v))) for v in motor_abs]
        self.comm.send_command(("motor_pos_abs", raw))
    
    def reset_motor_abs_counter(self, motor_index: int):
        """重置指定电机的绝对位置计数器"""
        if not self.comm:
            return
        
        idx = int(motor_index)
        if idx < 0 or idx >= MOTOR_COUNT:
            raise ValueError(f"motor_index 必须在 0..{MOTOR_COUNT - 1} 范围内")
        
        self.comm.send_command(("reset_motor_abs", idx))
    
    # ===== 高级功能 =====
    
    def set_tendon_guard_config(self, configs: List[tuple]):
        """
        设置腱绳保护配置
        每关节: (enabled, sign, x1_abs)
        """
        if not self.comm:
            return
        
        if len(configs) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 组腱绳保护配置")
        
        self.comm.send_command(("tendon_guard", list(configs)))
    
    def set_pid_params(self, outer_params: List[float], inner_params: List[float]):
        """
        设置双环PID参数
        参数顺序: kp, ki, kd, deadband, integral_limit, output_limit
        """
        if not self.comm:
            return
        
        if len(outer_params) != 6 or len(inner_params) != 6:
            raise ValueError("外环和内环PID参数都需要6个值")
        
        self.comm.send_command(("pid_params", (
            [float(v) for v in outer_params],
            [float(v) for v in inner_params]
        )))
    
    def apply_pose(self, pose: HandPose) -> bool:
        """
        应用预设手势
        返回是否成功发送
        """
        if not self.is_connected():
            return False
        
        target_angles = PRESET_POSES.get(pose, PRESET_POSES[HandPose.OPEN])
        self.set_target_angles(target_angles)
        return True
    
    def set_encoder_zeros(self, zero_raw_list: List[int]):
        """
        设置磁编码器零点
        将零点数据保存并下发到下位机
        
        Args:
            zero_raw_list: 21路编码器的零点原始值列表
        """
        if not self.comm:
            raise RuntimeError("未连接到设备")
        
        if len(zero_raw_list) != ENCODER_COUNT:
            raise ValueError(f"需要提供 {ENCODER_COUNT} 个零点值")
        
        # 保存到当前状态
        with self.state_lock:
            self.current_state.set_encoder_zeros(zero_raw_list)
        
        # 下发到下位机
        self.comm.send_command(("calib_data", list(zero_raw_list)))
        
        print(f"[标定] 已设置 {ENCODER_COUNT} 路编码器零点")
    
    def send_raw_command(self, cmd: str):
        """
        发送原始字符串命令到下位机
        用于MCP双电机控制等特殊命令
        
        Args:
            cmd: 命令字符串（如 "TORQUE:0\n", "ZERO_ALL\n"）
        """
        if not self.comm:
            raise RuntimeError("未连接到设备")
        
        # 直接通过串口发送原始命令
        if self.comm.serial and self.comm.serial.is_open:
            self.comm.serial.write(cmd.encode('utf-8'))
            print(f"[发送] {cmd.strip()}")
        else:
            raise RuntimeError("串口未打开")
    
    def get_current_angles(self) -> List[float]:
        """获取当前关节角度"""
        with self.state_lock:
            return self.current_state.angles
    
    def get_target_angles(self) -> List[float]:
        """获取目标关节角度"""
        with self.state_lock:
            return self.current_state.target_angles
    
    def get_status(self) -> dict:
        """获取控制器状态字典"""
        with self.state_lock:
            # 准备编码器数据
            encoder_raw = []
            encoder_angles = []
            encoder_zero = []
            encoder_target = []
            encoder_errors = []
            
            for enc in self.current_state.encoders:
                encoder_raw.append(enc.raw)
                encoder_angles.append(enc.current_angle_deg)
                encoder_zero.append(enc.zero_raw)
                encoder_target.append(enc.target_angle_deg)
                encoder_errors.append(enc.error)
            
            return {
                'connected': self.is_connected(),
                'started': self._started,
                'paused': self._paused,
                'control_mode': self._control_mode.name,
                'pid_enabled': self._pid_enabled,
                'calib_status': self.current_state.calib_status,
                # 编码器数据 (来自S3 CAN总线)
                'encoder_raw': encoder_raw,
                'encoder_angles': encoder_angles,
                'encoder_zero': encoder_zero,
                'encoder_target': encoder_target,
                'encoder_errors': encoder_errors,
                # 舵机多圈角度和在线状态
                'servo_angles': list(self.current_state.servo_angles),
                'servo_online': list(self.current_state.servo_online),
                # 舵机单圈原始位置
                'servo_raw_positions': list(self.current_state.servo_raw_positions),
                # 舵机遥测数据 (速度、负载/电流、电压、温度)
                'servo_speed': list(self.current_state.servo_speed),
                'servo_load': list(self.current_state.servo_load),
                'servo_voltage': list(self.current_state.servo_voltage),
                'servo_temperature': list(self.current_state.servo_temperature),
                # 故障状态
                'servo_overload_fault': list(self.current_state.servo_overload_fault),
                'joint_reverse_release_fault': list(self.current_state.joint_reverse_release_fault),
            }
    
    def _validate_angles(self, angles: List[float]) -> List[float]:
        """验证并清理角度值"""
        out: List[float] = []
        for a in angles:
            try:
                v = float(a)
            except (TypeError, ValueError):
                v = 0.0
            if not math.isfinite(v):
                v = 0.0
            out.append(v)
        return out
