"""
串口通信协议模块 - 与ESP32-P4下位机兼容
帧格式: 0xFE LEN TYPE_OR_CMD PAYLOAD 0xFF
注意: 所有多字节数据使用大端序 (Big Endian)
"""

import struct
from enum import IntEnum
from typing import List, Optional, Tuple


class ControlMode(IntEnum):
    """控制模式"""
    JOINT_ANGLE = 0
    DIRECT_MOTOR = 1


# ===== 协议常量 =====
PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF

BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
RX_QUEUE_MAXSIZE = 200
TX_QUEUE_MAXSIZE = 100
RX_POLL_SLEEP = 0.005
TX_QUEUE_GET_TIMEOUT = 0.1

# 数量定义
ENCODER_COUNT = 21
MOTOR_COUNT = 22

# 传感器流模式
SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

# 协议ACK状态

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1

# 错误标志
ERROR_VAL_FLAG = 0xFFFF
DISCONNECT_SENTINEL = 0x7FFF

# 电机绝对位置限制 (与下位机 clampServoPos 一致)
MOTOR_ABS_CMD_MIN = -30719
MOTOR_ABS_CMD_MAX = 30719

# 角度转换因子 (14位编码器 16384 = 360°)
RAW_TO_DEG = 360.0 / 16384.0
DEG_TO_RAW = 16384.0 / 360.0

# ===== 上行包类型 (下位机 -> 上位机) =====
PACKET_TYPE_SENSOR = 0x01           # 21路关节传感器值
PACKET_TYPE_CALIB_ACK = 0x02        # 标定UI状态
PACKET_TYPE_SERVO_ANGLE = 0x03      # 22路舵机多圈绝对位置
PACKET_TYPE_JOINT_DEBUG = 0x04      # 关节调试信息
PACKET_TYPE_SERVO_TELEM = 0x05      # 舵机遥测数据
PACKET_TYPE_PROTO_ACK = 0x06        # 协议命令ACK
PACKET_TYPE_FAULT_STATUS = 0x07     # 过载故障位图
PACKET_TYPE_RELEASE_FAULT = 0x08    # 反绕释放保护故障位图
PACKET_TYPE_SERVO_RAW = 0x09        # 22路舵机单圈raw位置


# ===== 下行命令 (上位机 -> 下位机) =====
CMD_CALIBRATE = 0xCA
CMD_ANGLE_CTRL = 0xCB
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE
CMD_CALIB_DATA = 0xCF
CMD_MOTOR_POS = 0xD0
CMD_SENSOR_STREAM_MODE = 0xD1
CMD_MOTOR_POS_SWEEP = 0xD2
CMD_MOTOR_POS_ABS = 0xD3
CMD_TENDON_GUARD = 0xD4
CMD_PID_PARAMS = 0xD5
CMD_RESET_MOTOR_ABS = 0xD6


def _build_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    """构建命令帧"""
    payload = bytes(payload)
    # LEN = CMD byte + payload + tail byte
    frame_len = 1 + len(payload) + 1
    return bytes([PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF]) + payload + bytes([PROTOCOL_TAIL])


def parse_frame(data: bytes) -> Tuple[List[Tuple[int, bytes]], bytearray]:
    """
    解析数据帧
    返回: (解析出的数据包列表, 剩余缓冲区)
    """
    result: List[Tuple[int, bytes]] = []
    buffer = bytearray(data)
    
    while len(buffer) >= 4:
        try:
            head_idx = buffer.index(PROTOCOL_HEADER)
        except ValueError:
            break
        
        buffer = buffer[head_idx:]
        if len(buffer) < 4:
            break
        
        payload_len = buffer[1]
        frame_len = 2 + payload_len
        
        if len(buffer) < frame_len:
            break
        
        frame = buffer[:frame_len]
        if frame[-1] != PROTOCOL_TAIL:
            buffer = buffer[1:]
            continue
        
        pkt_type = frame[2]
        payload = bytes(frame[3:-1])
        result.append((pkt_type, payload))
        buffer = buffer[frame_len:]
    
    return result, buffer


# ===== 数据包解析函数 =====

def parse_sensor_packet(payload: bytes) -> Tuple[List[int], List[bool]]:
    """
    解析传感器数据包 (PACKET_TYPE_SENSOR)
    格式: 21路编码器值，每路2字节 (大端有符号int16)
    返回: (角度值列表, 错误标志列表)
    """
    angles: List[int] = []
    errors: List[bool] = []
    
    expected_len = ENCODER_COUNT * 2  # 21 * 2 = 42 bytes
    if len(payload) < expected_len:
        print(f"[解析错误] 传感器数据包长度不足: {len(payload)} < {expected_len}")
        return angles, errors
    
    try:
        for i in range(ENCODER_COUNT):
            byte0 = payload[i * 2]
            byte1 = payload[i * 2 + 1]
            raw_u16 = (byte0 << 8) | byte1
            is_error = (raw_u16 == DISCONNECT_SENTINEL)
            errors.append(is_error)
            if is_error:
                angles.append(0)
            else:
                # 大端有符号int16
                angle = struct.unpack(">h", bytes([byte0, byte1]))[0]
                angles.append(angle)
    except Exception as e:
        print(f"[解析错误] 传感器数据解析异常: {e}")
        return [], []
    
    return angles, errors


def parse_servo_angle_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    """
    解析舵机角度包 (PACKET_TYPE_SERVO_ANGLE)
    格式: [position(4B大端int32) * N] + [online_flag(1B) * N]
    返回: (位置列表, 在线标志列表)
    """
    if not payload or (len(payload) % 5) != 0:
        return None
    
    channel_count = len(payload) // 5
    if channel_count <= 0:
        return None
    
    angles: List[int] = []
    for i in range(channel_count):
        start = i * 4
        angles.append(struct.unpack(">i", payload[start:start + 4])[0])
    
    online_flags: List[bool] = []
    base = channel_count * 4
    for i in range(channel_count):
        online_flags.append(payload[base + i] == 1)
    
    return angles, online_flags


def parse_servo_raw_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    """
    解析舵机单圈原始位置包 (PACKET_TYPE_SERVO_RAW)
    格式: [raw_position(2B大端int16) * N] + [online_flag(1B) * N]
    """
    if not payload or (len(payload) % 3) != 0:
        return None
    
    channel_count = len(payload) // 3
    if channel_count <= 0:
        return None
    
    raw_positions: List[int] = []
    for i in range(channel_count):
        start = i * 2
        raw_positions.append(struct.unpack(">h", payload[start:start + 2])[0])
    
    online_flags: List[bool] = []
    base = channel_count * 2
    for i in range(channel_count):
        online_flags.append(payload[base + i] == 1)
    
    return raw_positions, online_flags


def parse_servo_telem_packet(payload: bytes) -> Optional[Tuple[List[int], List[int], List[int], List[int], List[bool]]]:
    """
    解析舵机遥测数据包 (PACKET_TYPE_SERVO_TELEM)
    格式: [speed(2B) + load(2B) + voltage(1B) + temp(1B) + online(1B)] * N
    返回: (速度列表, 负载列表, 电压列表, 温度列表, 在线标志列表)
    """
    if not payload or (len(payload) % 7) != 0:
        return None
    
    channel_count = len(payload) // 7
    if channel_count <= 0:
        return None
    
    speeds: List[int] = []
    loads: List[int] = []
    volts: List[int] = []
    temps: List[int] = []
    online: List[bool] = []
    
    for i in range(channel_count):
        base = i * 7
        speeds.append(struct.unpack(">h", payload[base:base + 2])[0])
        loads.append(struct.unpack(">h", payload[base + 2:base + 4])[0])
        volts.append(payload[base + 4])
        temps.append(payload[base + 5])
        online.append(payload[base + 6] == 1)
    
    return speeds, loads, volts, temps, online


def parse_joint_debug_packet(payload: bytes) -> Optional[Tuple]:
    """
    解析关节调试数据包 (PACKET_TYPE_JOINT_DEBUG)
    格式: joint_index(1B) + valid(1B) + target_deg(4B) + actual_deg(4B) + 
          loop1_out(4B) + loop2_act(4B) + loop2_out(4B) + [cmd_valid(1B) + cmd_target_pos(2B)]
    """
    legacy_len = 2 + 5 * 4  # 2 + 20 = 22 bytes
    extended_len = legacy_len + 1 + 2  # 25 bytes
    
    if len(payload) not in (legacy_len, extended_len):
        return None
    
    joint_index, valid, target_deg, actual_deg, loop1_out, loop2_act, loop2_out = struct.unpack(
        ">BBfffff", payload[:legacy_len]
    )
    
    if len(payload) == extended_len:
        cmd_valid = payload[legacy_len] == 1
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1:legacy_len + 3])[0]
    else:
        cmd_valid = False
        cmd_target_pos = 0
    
    return (
        joint_index, valid == 1, target_deg, actual_deg,
        loop1_out, loop2_act, loop2_out, cmd_valid, cmd_target_pos
    )


def parse_fault_status_packet(payload: bytes) -> Optional[int]:
    """解析故障状态包 (PACKET_TYPE_FAULT_STATUS)"""
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


def parse_release_fault_packet(payload: bytes) -> Optional[int]:
    """解析释放故障包 (PACKET_TYPE_RELEASE_FAULT)"""
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


def parse_calib_ack(payload: bytes) -> Optional[int]:
    """解析标定确认包"""
    if len(payload) < 1:
        return None
    return payload[0]


def parse_proto_ack(payload: bytes) -> Optional[Tuple[int, int, int]]:
    """解析协议ACK包"""
    if len(payload) < 3:
        return None
    return payload[0], payload[1], payload[2]


# ===== 命令构建函数 =====

def build_start_cmd() -> bytes:
    """构建启动控制命令"""
    return _build_command_frame(CMD_START)


def build_stop_cmd() -> bytes:
    """构建停止控制命令"""
    return _build_command_frame(CMD_STOP)


def build_reset_cmd() -> bytes:
    """构建复位命令"""
    return _build_command_frame(CMD_RESET)


def build_calibrate_cmd() -> bytes:
    """构建标定命令"""
    return _build_command_frame(CMD_CALIBRATE)


def build_angle_cmd(angles: List[float]) -> bytes:
    """
    构建关节角度控制命令
    格式: 21路float32 (小端) - 注意：与参考实现保持一致
    """
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")
    
    payload = struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return _build_command_frame(CMD_ANGLE_CTRL, payload)


def build_calib_data_cmd(zero_encoder_raw: List[int]) -> bytes:
    """
    构建标定数据命令
    格式: 21路float32 (小端)
    
    zero_encoder_raw 应该是 0-16383 范围内的值
    """
    if len(zero_encoder_raw) != ENCODER_COUNT:
        raise ValueError(f"需要 {ENCODER_COUNT} 个编码器零值")
    
    # 确保所有值都在有效范围内
    normalized_vals = []
    for v in zero_encoder_raw:
        val = int(v)
        # 确保在 0-16383 范围内
        if val < 0:
            val += 16384
        val = val % 16384
        normalized_vals.append(float(val))
    
    # 调试输出前5个值
    print(f"[build_calib_data_cmd] 发送零点数据: {normalized_vals[:5]}...")
    
    payload = struct.pack(f"<{ENCODER_COUNT}f", *normalized_vals)
    return _build_command_frame(CMD_CALIB_DATA, payload)


def build_motor_pos_cmd(motor_pos_raw: List[int]) -> bytes:
    """
    构建电机位置控制命令 (单圈raw)
    格式: 每路2字节 (大端uint16)
    """
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个电机位置值")
    
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    
    return _build_command_frame(CMD_MOTOR_POS, bytes(payload))


def build_motor_pos_sweep_cmd(motor_pos_raw: List[int]) -> bytes:
    """构建电机滑条扫动命令"""
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个电机位置值")
    
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    
    return _build_command_frame(CMD_MOTOR_POS_SWEEP, bytes(payload))


def build_motor_pos_abs_cmd(motor_abs: List[int]) -> bytes:
    """
    构建电机绝对位置控制命令
    格式: 每路2字节 (大端int16)，范围 -30719~30719
    """
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个电机绝对位置值")
    
    payload = bytearray()
    for value in motor_abs:
        v = int(value)
        if v < MOTOR_ABS_CMD_MIN:
            v = MOTOR_ABS_CMD_MIN
        elif v > MOTOR_ABS_CMD_MAX:
            v = MOTOR_ABS_CMD_MAX
        payload.extend(struct.pack(">h", v))
    
    return _build_command_frame(CMD_MOTOR_POS_ABS, bytes(payload))


def build_reset_motor_abs_cmd(motor_index: int) -> bytes:
    """构建重置电机绝对位置计数器命令"""
    idx = int(motor_index)
    if idx < 0 or idx >= MOTOR_COUNT:
        raise ValueError(f"motor_index 必须在 0..{MOTOR_COUNT - 1} 范围内")
    return _build_command_frame(CMD_RESET_MOTOR_ABS, bytes([idx & 0xFF]))


def build_stream_mode_cmd(mode: int) -> bytes:
    """构建传感器流模式切换命令"""
    mode_int = int(mode)
    if mode_int not in (SENSOR_STREAM_MODE_LEGACY_U16_WRAP, SENSOR_STREAM_MODE_SIGNED_I16):
        raise ValueError("不支持的传感器流模式")
    return _build_command_frame(CMD_SENSOR_STREAM_MODE, bytes([mode_int & 0xFF]))


def build_tendon_guard_cmd(configs: List[Tuple[bool, int, int]]) -> bytes:
    """
    构建腱绳保护配置命令
    每关节4字节: enabled(uint8) + sign(int8) + x1_abs(int16大端)
    """
    if len(configs) != ENCODER_COUNT:
        raise ValueError(f"需要 {ENCODER_COUNT} 组腱绳保护配置")
    
    payload = bytearray()
    for enabled, sign_raw, x1_raw in configs:
        enabled_u8 = 1 if bool(enabled) else 0
        sign_i8 = 1 if int(sign_raw) >= 0 else -1
        x1 = int(x1_raw)
        if x1 < MOTOR_ABS_CMD_MIN:
            x1 = MOTOR_ABS_CMD_MIN
        elif x1 > MOTOR_ABS_CMD_MAX:
            x1 = MOTOR_ABS_CMD_MAX
        
        payload.append(enabled_u8 & 0xFF)
        payload.extend(struct.pack("b", sign_i8))
        payload.extend(struct.pack(">h", x1))
    
    return _build_command_frame(CMD_TENDON_GUARD, bytes(payload))


def build_pid_params_cmd(outer_params: List[float], inner_params: List[float]) -> bytes:
    """
    构建PID参数命令
    外环6个float32 + 内环6个float32 (小端)
    参数顺序: kp, ki, kd, deadband, integral_limit, output_limit
    """
    if len(outer_params) != 6:
        raise ValueError("外环PID参数需要6个值")
    if len(inner_params) != 6:
        raise ValueError("内环PID参数需要6个值")
    
    outer = [float(v) for v in outer_params]
    inner = [float(v) for v in inner_params]
    payload = struct.pack("<12f", *(outer + inner))
    return _build_command_frame(CMD_PID_PARAMS, payload)
