"""
通信层 - 与下位机的串口通信
使用独立的RX和TX线程
"""

import queue
import threading
import time
from typing import Callable, List, Optional, Union

import serial
import serial.tools.list_ports

from .protocol import (
    BAUDRATE,
    ENCODER_COUNT,
    MOTOR_COUNT,
    RX_POLL_SLEEP,
    RX_QUEUE_MAXSIZE,
    SERIAL_TIMEOUT,
    TX_QUEUE_GET_TIMEOUT,
    TX_QUEUE_MAXSIZE,
    SENSOR_STREAM_MODE_SIGNED_I16,
    CMD_SENSOR_STREAM_MODE,
    PROTO_ACK_STATUS_OK,
    PACKET_TYPE_MCP_ROPE_PD,
    parse_calib_ack,
    parse_fault_status_packet,
    parse_frame,
    parse_joint_debug_packet,
    parse_proto_ack,
    parse_release_fault_packet,
    parse_sensor_packet,
    parse_servo_angle_packet,
    parse_servo_raw_packet,
    parse_servo_telem_packet,
    build_start_cmd,
    build_stop_cmd,
    build_reset_cmd,
    build_calibrate_cmd,
    build_angle_cmd,
    build_calib_data_cmd,
    build_motor_pos_cmd,
    build_motor_pos_sweep_cmd,
    build_motor_pos_abs_cmd,
    build_reset_motor_abs_cmd,
    build_stream_mode_cmd,
    build_tendon_guard_cmd,
)
from .data_models import HandModel, MotorState


# 命令类型: 字符串命令 / HandModel / 元组命令
SendCmd = Union[str, HandModel, tuple]


# 与下位机 PACKET_TYPE_MCP_ROPE_PD 及「简易触发」共用同一提示文案
MCP_ROPE_PD_NOTIFY_LINE = "[绳长PD] 已经激活PD模式（下位机 M00/M01 绳空间闭环）"


def list_ports() -> List[tuple]:
    """列出可用串口"""
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def is_valid_port(port: str) -> bool:
    """检查串口是否有效"""
    if not port or not port.strip():
        return False
    available = [p[0] for p in list_ports()]
    return port.strip() in available


class LowerComputerComm:
    """
    下位机通信类
    管理串口连接，使用独立的RX和TX线程
    """
    
    def __init__(
        self,
        port: str,
        baudrate: int = BAUDRATE,
        serial_notify: Optional[Callable[[str], None]] = None,
        rope_pd_easy_notify: Optional[Callable[[], bool]] = None,
    ):
        self.port = (port or "").strip()
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self._serial_notify: Optional[Callable[[str], None]] = serial_notify
        # 为 true 时：关节角度模式 + START 后，首次 E0/E1 有效 0x01 也弹出与 0x0B 相同提示（无需等固件去抖包）
        self._rope_pd_easy_notify: Optional[Callable[[], bool]] = rope_pd_easy_notify
        self._mcp_rope_pd_message_sent: bool = False
        
        # 接收和发送队列
        self.rx_queue: queue.Queue = queue.Queue(maxsize=RX_QUEUE_MAXSIZE)
        self.tx_queue: queue.Queue = queue.Queue(maxsize=TX_QUEUE_MAXSIZE)
        
        # 线程控制
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        
        # 接收缓冲区
        self._rx_buffer = bytearray()
        
        # 上一次的数据模型 (用于状态保持)
        self._last_model: Optional[HandModel] = None
        
        # 流模式设置
        self._stream_mode_requested = SENSOR_STREAM_MODE_SIGNED_I16
        self._stream_mode_applied: Optional[int] = None
        self._stream_mode_ack_pending = False
        self._stream_mode_ack_deadline = 0.0
        self._stream_mode_ack_warned = False
        
        # 验证串口
        if self.port:
            if not is_valid_port(self.port):
                raise ValueError(f"端口 '{self.port}' 不在当前可用串口列表中")
    
    def connect(self) -> bool:
        """连接串口"""
        if self.serial and getattr(self.serial, "is_open", False):
            return True
        
        if not self.port:
            return False
        
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=SERIAL_TIMEOUT
            )
            self.running = True
            self._rx_buffer = bytearray()
            
            # 启动RX和TX线程
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
            
            # 请求传感器流模式
            self._stream_mode_applied = None
            self._stream_mode_ack_pending = True
            self._stream_mode_ack_warned = False
            self._stream_mode_ack_deadline = time.time() + 2.0
            self.send_command(("stream_mode", self._stream_mode_requested))
            
            return True
        except Exception as exc:
            print(f"连接失败: {exc}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.serial is not None and self.serial.is_open and self.running
    
    def reset_mcp_rope_pd_message_gate(self) -> None:
        """允许再次弹出绳长 PD 提示（固件 0x0B 与关节模式简易触发共用同一门闩）。"""
        self._mcp_rope_pd_message_sent = False
    
    def _emit_mcp_rope_pd_notify(self) -> None:
        if self._mcp_rope_pd_message_sent:
            return
        self._mcp_rope_pd_message_sent = True
        print(MCP_ROPE_PD_NOTIFY_LINE)
        if self._serial_notify:
            try:
                self._serial_notify(MCP_ROPE_PD_NOTIFY_LINE)
            except Exception:
                pass
    
    def _try_mcp_rope_pd_easy_hint(self, errors: List[bool]) -> None:
        """关节角度模式 + START 后：首次 E0/E1 无断连的 0x01 即提示（与 0x0B 同文案）。"""
        if self._mcp_rope_pd_message_sent or not self._rope_pd_easy_notify:
            return
        try:
            if not self._rope_pd_easy_notify():
                return
        except Exception:
            return
        if len(errors) < 2:
            return
        if errors[0] or errors[1]:
            return
        self._emit_mcp_rope_pd_notify()
    
    def _rx_loop(self):
        """接收线程主循环"""
        while self.running and self.serial and self.serial.is_open:
            try:
                # 读取可用数据
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    self._rx_buffer.extend(chunk)
                    
                    # 解析帧
                    packets, self._rx_buffer = parse_frame(bytes(self._rx_buffer))
                    
                    # 处理数据包
                    model = self._process_packets(packets)
                    if model:
                        try:
                            self.rx_queue.put_nowait(model)
                        except queue.Full:
                            pass  # 队列满，丢弃旧数据
                else:
                    time.sleep(RX_POLL_SLEEP)
                
                # 检查流模式ACK超时
                self._check_stream_mode_ack_timeout()
                
            except Exception as exc:
                if self.running:
                    print(f"RX错误: {exc}")
    
    def _check_stream_mode_ack_timeout(self):
        """检查流模式ACK是否超时"""
        if self._stream_mode_ack_pending and not self._stream_mode_ack_warned:
            if time.time() >= self._stream_mode_ack_deadline:
                print("协议警告: 未收到固件的流模式ACK，继续使用当前解码方式")
                self._stream_mode_ack_warned = True
                self._stream_mode_ack_pending = False
    
    def _process_packets(self, packets: List[tuple]) -> Optional[HandModel]:
        """处理解析出的数据包，返回HandModel"""
        model = HandModel()
        
        # 复制上一次的状态
        if self._last_model:
            model.copy_state_from(self._last_model)
            model.calib_status = self._last_model.calib_status
            model.has_sensor_data = bool(self._last_model.has_sensor_data)
            model.has_servo_angle_data = bool(self._last_model.has_servo_angle_data)
            model.has_servo_raw_data = bool(self._last_model.has_servo_raw_data)
        
        model.timestamp = time.time()
        emitted = False
        
        for pkt_type, payload in packets:
            # PACKET_TYPE_SENSOR (0x01) - 传感器数据 (来自S3 CAN总线的磁编码器)
            if pkt_type == 0x01:
                angles, errors = parse_sensor_packet(payload)
                if angles:
                    for i in range(min(ENCODER_COUNT, len(angles), len(model.encoders))):
                        model.encoders[i].encoder_id = i
                        model.encoders[i].raw = angles[i]
                        model.encoders[i].error = errors[i] if i < len(errors) else False
                    
                    for i in range(min(ENCODER_COUNT, len(model.motors))):
                        model.motors[i].motor_id = i
                        model.motors[i].error = errors[i] if i < len(errors) else False
                        model.motors[i].state = (
                            MotorState.ERROR if model.motors[i].error else MotorState.RUNNING
                        )
                    
                    model.has_sensor_data = True
                    emitted = True
                    if errors:
                        self._try_mcp_rope_pd_easy_hint(errors)
            
            # PACKET_TYPE_CALIB_ACK (0x02) - 标定确认
            elif pkt_type == 0x02:
                code = parse_calib_ack(payload)
                if code == 1:
                    model.calib_status = "PENDING"
                elif code == 2:
                    model.calib_status = "SUCCESS"
                elif code == 3:
                    model.calib_status = "FAILED"
                emitted = True
            
            # PACKET_TYPE_SERVO_ANGLE (0x03) - 舵机多圈绝对角度
            elif pkt_type == 0x03:
                parsed = parse_servo_angle_packet(payload)
                if parsed is not None:
                    angles, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(angles))):
                        model.servo_angles[i] = int(angles[i])
                        model.servo_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_angle_data = True
                    emitted = True
            
            # PACKET_TYPE_SERVO_RAW (0x09) - 舵机单圈原始位置
            elif pkt_type == 0x09:
                parsed = parse_servo_raw_packet(payload)
                if parsed is not None:
                    raw_positions, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(raw_positions))):
                        model.servo_raw_positions[i] = int(raw_positions[i])
                        model.servo_raw_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_raw_data = True
                    emitted = True
            
            # PACKET_TYPE_SERVO_TELEM (0x05) - 舵机遥测数据
            elif pkt_type == 0x05:
                parsed = parse_servo_telem_packet(payload)
                if parsed is not None:
                    speeds, loads, volts, temps, online = parsed
                    for i in range(min(MOTOR_COUNT, len(speeds))):
                        model.servo_speed[i] = int(speeds[i])
                        model.servo_load[i] = int(loads[i])
                        model.servo_voltage[i] = int(volts[i])
                        model.servo_temperature[i] = int(temps[i])
                        model.servo_telem_online[i] = bool(online[i]) if i < len(online) else False
                    emitted = True
            
            # PACKET_TYPE_JOINT_DEBUG (0x04) - 关节调试信息
            elif pkt_type == 0x04:
                parsed = parse_joint_debug_packet(payload)
                if parsed is not None:
                    (
                        joint_index, valid, target_deg, actual_deg,
                        loop1_out, loop2_act, loop2_out, cmd_valid, cmd_target_pos
                    ) = parsed
                    if 0 <= joint_index < ENCODER_COUNT:
                        model.joint_debug_valid[joint_index] = valid
                        model.joint_debug_target_deg[joint_index] = float(target_deg)
                        model.joint_debug_actual_deg[joint_index] = float(actual_deg)
                        model.joint_debug_loop1_output[joint_index] = float(loop1_out)
                        model.joint_debug_loop2_actual[joint_index] = float(loop2_act)
                        model.joint_debug_loop2_output[joint_index] = float(loop2_out)
                        model.joint_debug_cmd_valid[joint_index] = bool(cmd_valid)
                        model.joint_debug_cmd_target_pos[joint_index] = int(cmd_target_pos)
                    emitted = True
            
            # PACKET_TYPE_FAULT_STATUS (0x07) - 过载故障
            elif pkt_type == 0x07:
                fault_bitmap = parse_fault_status_packet(payload)
                if fault_bitmap is not None:
                    for i in range(MOTOR_COUNT):
                        model.servo_overload_fault[i] = ((fault_bitmap >> i) & 0x01) != 0
                    emitted = True
            
            # PACKET_TYPE_RELEASE_FAULT (0x08) - 反绕释放保护故障
            elif pkt_type == 0x08:
                release_fault_bitmap = parse_release_fault_packet(payload)
                if release_fault_bitmap is not None:
                    for i in range(ENCODER_COUNT):
                        model.joint_reverse_release_fault[i] = ((release_fault_bitmap >> i) & 0x01) != 0
                    emitted = True
            
            # PACKET_TYPE_PROTO_ACK (0x06) - 协议ACK
            elif pkt_type == 0x06:
                parsed = parse_proto_ack(payload)
                if parsed is not None:
                    ack_cmd, applied_mode, status = parsed
                    if ack_cmd == CMD_SENSOR_STREAM_MODE:
                        self._stream_mode_applied = int(applied_mode)
                        self._stream_mode_ack_pending = False
                        if status != PROTO_ACK_STATUS_OK and not self._stream_mode_ack_warned:
                            print(f"协议警告: 流模式被拒绝 (status={status}, applied={applied_mode})")
                            self._stream_mode_ack_warned = True

            elif pkt_type == PACKET_TYPE_MCP_ROPE_PD:
                if len(payload) >= 1 and payload[0] == 0x01:
                    self._emit_mcp_rope_pd_notify()
        
        self._last_model = model
        return model if emitted else None
    
    def send_command(self, cmd: SendCmd):
        """发送命令到发送队列"""
        # 高速率命令使用特殊队列管理
        if self._is_high_rate_command(cmd):
            self._enqueue_latest_high_rate(cmd)
            return
        
        try:
            self.tx_queue.put_nowait(cmd)
        except queue.Full:
            pass  # 队列满，丢弃
    
    @staticmethod
    def _is_high_rate_command(cmd: SendCmd) -> bool:
        """检查是否为高速率命令"""
        return isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] in (
            "motor_pos_sweep",
            "motor_pos_abs",
            "angle_live",
        )
    
    @staticmethod
    def _is_joint_command(cmd: SendCmd) -> bool:
        """检查是否为关节控制命令"""
        if isinstance(cmd, HandModel):
            return True
        return isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live"
    
    def invalidate_pending_joint_commands(self) -> int:
        """清除发送队列中待发的关节控制命令"""
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if not self._is_joint_command(item)]
            removed = len(pending) - len(kept)
            if removed <= 0:
                return 0
            pending.clear()
            pending.extend(kept)
            self.tx_queue.not_full.notify_all()
            return removed
    
    def _enqueue_latest_high_rate(self, cmd: SendCmd):
        """高速率命令只保留最新的"""
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if not self._is_high_rate_command(item)]
            pending.clear()
            pending.extend(kept)
            
            maxsize = self.tx_queue.maxsize
            if maxsize > 0 and len(pending) >= maxsize:
                return
            
            pending.append(cmd)
            self.tx_queue.unfinished_tasks += 1
            self.tx_queue.not_empty.notify()
    
    def _tx_loop(self):
        """发送线程主循环"""
        while self.running and self.serial and self.serial.is_open:
            try:
                cmd = self.tx_queue.get(timeout=TX_QUEUE_GET_TIMEOUT)
                data = self._encode_command(cmd)
                if data:
                    with self._tx_lock:
                        self.serial.write(data)
            except queue.Empty:
                pass
            except Exception as exc:
                if self.running:
                    print(f"TX错误: {exc}")
    
    def _encode_command(self, cmd: SendCmd) -> Optional[bytes]:
        """编码命令为字节数据"""
        # 字符串命令
        if cmd == "start":
            return build_start_cmd()
        if cmd == "stop":
            return build_stop_cmd()
        if cmd == "reset":
            return build_reset_cmd()
        if cmd == "calibrate":
            return build_calibrate_cmd()
        
        # 元组命令
        if isinstance(cmd, tuple) and len(cmd) == 3:
            cmd_type, enc_zeros, motor_abs = cmd
            if cmd_type == "calib_data":
                return build_calib_data_cmd(enc_zeros, mechanism_motor_abs=list(motor_abs))

        if isinstance(cmd, tuple) and len(cmd) == 2:
            cmd_type, cmd_data = cmd
            
            if cmd_type == "calib_data":
                return build_calib_data_cmd(cmd_data)
            if cmd_type == "motor_pos":
                return build_motor_pos_cmd(cmd_data)
            if cmd_type == "motor_pos_sweep":
                return build_motor_pos_sweep_cmd(cmd_data)
            if cmd_type == "motor_pos_abs":
                return build_motor_pos_abs_cmd(cmd_data)
            if cmd_type == "reset_motor_abs":
                return build_reset_motor_abs_cmd(cmd_data)
            if cmd_type == "angle_live":
                return build_angle_cmd(cmd_data)
            if cmd_type == "stream_mode":
                return build_stream_mode_cmd(cmd_data)
            if cmd_type == "tendon_guard":
                return build_tendon_guard_cmd(cmd_data)
        
        # HandModel 命令 (发送目标角度)
        if isinstance(cmd, HandModel):
            angles = cmd.target_angles
            if len(angles) != ENCODER_COUNT:
                return None
            return build_angle_cmd(list(angles))
        
        return None
