import struct
from enum import Enum
from typing import List, Optional, Tuple


class ControlMode(Enum):
    JOINT_ANGLE = 0
    DIRECT_MOTOR = 1


PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF

# Upstream packet types (device -> desktop)
PACKET_TYPE_SENSOR = 0x01
PACKET_TYPE_CALIB_ACK = 0x02
PACKET_TYPE_SERVO_ANGLE = 0x03
PACKET_TYPE_JOINT_DEBUG = 0x04
PACKET_TYPE_SERVO_TELEM = 0x05
PACKET_TYPE_PROTO_ACK = 0x06
PACKET_TYPE_FAULT_STATUS = 0x07
PACKET_TYPE_RELEASE_FAULT = 0x08
PACKET_TYPE_SERVO_RAW = 0x09
PACKET_TYPE_CONTROL_STATUS = 0x0B

# Downstream commands (desktop -> device)
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
CMD_LOCAL_TEST_PARAMS = 0xD4
CMD_LOCAL_TEST_START = 0xD5
CMD_LOCAL_TEST_STOP = 0xD6
CMD_SERVO_INTERNAL_ZERO = 0xD7

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1

# JOINT_GATE_NO_TARGET 的意思是“未收到 targetDegs 指令”，用于 joint gate 层阻止电机运动的原因位标志的第 0 位
JOINT_GATE_NO_TARGET = 1 << 0
JOINT_GATE_CONTROL_DISABLED = 1 << 1
JOINT_GATE_OWNER_NOT_CONTROL = 1 << 2
JOINT_GATE_MODE_NOT_JOINT = 1 << 3
JOINT_GATE_STATE_NOT_RUNNING = 1 << 4
JOINT_GATE_FAULT_HOLD = 1 << 5
JOINT_GATE_SYSTEM_FAULT = 1 << 6

# Decoupled dimensions
ENCODER_COUNT = 21
MOTOR_COUNT = 22

CALIB_LOAD_THRESHOLD = 0.3
ERROR_VAL_FLAG = 0xFFFF
DISCONNECT_SENTINEL = 0x7FFF

# Serial settings
BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
RX_QUEUE_MAXSIZE = 200
TX_QUEUE_MAXSIZE = 100
RX_POLL_SLEEP = 0.005
TX_QUEUE_GET_TIMEOUT = 0.1


def _build_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    # Symmetric downstream frame: [0xFE][LEN][CMD][PAYLOAD][0xFF]
    # LEN = CMD byte + payload + tail byte.
    frame_len = 1 + len(payload) + 1
    return bytes([PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF]) + payload + bytes([PROTOCOL_TAIL])


def parse_sensor_packet(payload: bytes) -> Tuple[List[int], List[int], List[bool]]:
    """Parse sensor packet into raw counts, mapped counts, and error flags."""
    raw_counts: List[int] = []
    mapped_counts: List[int] = []
    errors: List[bool] = []
    legacy_bytes = ENCODER_COUNT * 2
    dual_bytes = ENCODER_COUNT * 4
    if len(payload) < legacy_bytes:
        return raw_counts, mapped_counts, errors

    # New format: [raw(21 x int16/u16)] + [mapped(21 x int16)].
    if len(payload) >= dual_bytes:
        raw_base = 0
        mapped_base = legacy_bytes
        for i in range(ENCODER_COUNT):
            raw_u16 = (payload[raw_base + i * 2] << 8) | payload[raw_base + i * 2 + 1]
            mapped_u16 = (payload[mapped_base + i * 2] << 8) | payload[mapped_base + i * 2 + 1]
            is_error = (raw_u16 == DISCONNECT_SENTINEL) or (mapped_u16 == DISCONNECT_SENTINEL)
            errors.append(is_error)
            if is_error:
                raw_counts.append(0)
                mapped_counts.append(0)
            else:
                raw_counts.append(int(raw_u16))
                mapped_counts.append(struct.unpack(">h", payload[mapped_base + i * 2 : mapped_base + i * 2 + 2])[0])
        return raw_counts, mapped_counts, errors

    # Legacy format fallback: payload only carries mapped counts.
    for i in range(ENCODER_COUNT):
        mapped_u16 = (payload[i * 2] << 8) | payload[i * 2 + 1]
        is_error = (mapped_u16 == DISCONNECT_SENTINEL)
        errors.append(is_error)
        if is_error:
            raw_counts.append(0)
            mapped_counts.append(0)
        else:
            mapped_val = struct.unpack(">h", payload[i * 2 : i * 2 + 2])[0]
            raw_counts.append(int(mapped_val))
            mapped_counts.append(mapped_val)
    return raw_counts, mapped_counts, errors


def parse_proto_ack(payload: bytes) -> Optional[Tuple[int, int, int]]:
    if len(payload) < 3:
        return None
    return payload[0], payload[1], payload[2]


def parse_calib_ack(payload: bytes) -> Optional[int]:
    if len(payload) < 1:
        return None
    return payload[0]


def parse_servo_angle_packet(payload: bytes) -> Optional[Tuple[List[int], List[int], List[bool], bool]]:
    # Legacy: N * angle_i32 + N * online_u8.
    # New:    N * angle_i32 + N * software_zero_offset_i32 + N * online_u8.
    if not payload:
        return None

    has_offsets = False
    if (len(payload) % 9) == 0:
        channel_count = len(payload) // 9
        has_offsets = True
    elif (len(payload) % 5) == 0:
        channel_count = len(payload) // 5
    else:
        return None
    if channel_count <= 0:
        return None

    angles: List[int] = []
    for i in range(channel_count):
        start = i * 4
        angles.append(struct.unpack(">i", payload[start:start + 4])[0])

    base = channel_count * 4
    offsets: List[int] = [0] * channel_count
    if has_offsets:
        for i in range(channel_count):
            start = base + i * 4
            offsets[i] = struct.unpack(">i", payload[start:start + 4])[0]
        base += channel_count * 4

    online_flags: List[bool] = []
    for i in range(channel_count):
        online_flags.append(payload[base + i] == 1)
    return angles, offsets, online_flags, has_offsets


def parse_servo_telem_packet(
    payload: bytes,
) -> Optional[Tuple[List[int], List[int], List[int], List[int], List[bool]]]:
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


def parse_servo_raw_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
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


def parse_fault_status_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


def parse_release_fault_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


def parse_control_status_packet(payload: bytes) -> Optional[Tuple[int, int, int, int, bool, int, int, int]]:
    if len(payload) != 17:
        return None
    mode = payload[0]
    control_enabled = payload[1]
    owner = payload[2]
    state = payload[3]
    ready = payload[4] == 1
    fault_bitmap = struct.unpack(">I", payload[5:9])[0]
    reason_bitmap = struct.unpack(">I", payload[9:13])[0]
    joint_command_token = struct.unpack(">I", payload[13:17])[0]
    return mode, control_enabled, owner, state, ready, fault_bitmap, reason_bitmap, joint_command_token


def parse_joint_debug_packet(payload: bytes):
    legacy_len = 2 + 5 * 4
    cmd_len = legacy_len + 1 + 2
    cmd_ts_len = cmd_len + 4
    extended_len = 2 + 8 * 4 + 4 + 2 + 1 + 1 + 2
    if len(payload) not in (legacy_len, cmd_len, cmd_ts_len, extended_len):
        return None

    joint_index, valid, target_deg, actual_deg, loop1_out, loop2_act, loop2_out = struct.unpack(
        ">BBfffff", payload[:legacy_len]
    )
    target_length = 0.0
    actual_length = 0.0
    mapped_motor_target = 0.0
    motor_zero_abs = 0
    solver_output_pos = 0
    zero_homing = False

    if len(payload) == extended_len:
        (
            joint_index,
            valid,
            target_deg,
            actual_deg,
            loop1_out,
            loop2_act,
            loop2_out,
            target_length,
            actual_length,
            mapped_motor_target,
        ) = struct.unpack(">BBffffffff", payload[:34])
        motor_zero_abs = struct.unpack(">i", payload[34:38])[0]
        solver_output_pos = struct.unpack(">h", payload[38:40])[0]
        zero_homing = payload[40] == 1
        cmd_valid = payload[41] == 1
        cmd_target_pos = struct.unpack(">h", payload[42:44])[0]
        device_timestamp_ms = 0
        return (
            joint_index,
            valid == 1,
            target_deg,
            actual_deg,
            loop1_out,
            loop2_act,
            loop2_out,
            target_length,
            actual_length,
            mapped_motor_target,
            motor_zero_abs,
            solver_output_pos,
            zero_homing,
            cmd_valid,
            cmd_target_pos,
            device_timestamp_ms,
        )

    if len(payload) >= cmd_len:
        cmd_valid = payload[legacy_len] == 1
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1 : legacy_len + 3])[0]
    else:
        cmd_valid = False
        cmd_target_pos = 0

    if len(payload) == cmd_ts_len:
        device_timestamp_ms = struct.unpack(">I", payload[cmd_len : cmd_len + 4])[0]
    else:
        device_timestamp_ms = 0

    return (
        joint_index,
        valid == 1,
        target_deg,
        actual_deg,
        loop1_out,
        loop2_act,
        loop2_out,
        target_length,
        actual_length,
        mapped_motor_target,
        motor_zero_abs,
        solver_output_pos,
        zero_homing,
        cmd_valid,
        cmd_target_pos,
        device_timestamp_ms,
    )


def parse_frame(data: bytes) -> Tuple[List[Tuple[int, bytes]], bytearray]:
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


def build_calibrate_cmd() -> bytes:
    return _build_command_frame(CMD_CALIBRATE)


def build_start_cmd() -> bytes:
    return _build_command_frame(CMD_START)


def build_stop_cmd() -> bytes:
    return _build_command_frame(CMD_STOP)


def build_reset_cmd() -> bytes:
    return _build_command_frame(CMD_RESET)


def build_calib_data_cmd(zero_encoder_raw: List[int]) -> bytes:
    if len(zero_encoder_raw) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} encoder zero values")
    vals = [float(v) for v in zero_encoder_raw]
    payload = struct.pack(f"<{ENCODER_COUNT}f", *vals)
    return _build_command_frame(CMD_CALIB_DATA, payload)


def build_angle_cmd(angles: List[float]) -> bytes:
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    payload = struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return _build_command_frame(CMD_ANGLE_CTRL, payload)


def build_motor_pos_cmd(motor_pos_raw: List[int]) -> bytes:
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor raw values")
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    return _build_command_frame(CMD_MOTOR_POS, bytes(payload))


def build_motor_pos_sweep_cmd(motor_pos_raw: List[int]) -> bytes:
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor raw values")
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    return _build_command_frame(CMD_MOTOR_POS_SWEEP, bytes(payload))


# 与下位机 clampServoPos 一致的多圈绝对位置（int16 大端每路）
MOTOR_ABS_CMD_MIN = -30719
MOTOR_ABS_CMD_MAX = 30719


def build_motor_pos_abs_cmd(motor_abs: List[int]) -> bytes:
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    payload = bytearray()
    for value in motor_abs:
        v = int(value)
        if v < MOTOR_ABS_CMD_MIN:
            v = MOTOR_ABS_CMD_MIN
        elif v > MOTOR_ABS_CMD_MAX:
            v = MOTOR_ABS_CMD_MAX
        payload.extend(struct.pack(">h", v))
    return _build_command_frame(CMD_MOTOR_POS_ABS, bytes(payload))


def build_stream_mode_cmd(mode: int) -> bytes:
    mode_int = int(mode)
    if mode_int not in (SENSOR_STREAM_MODE_LEGACY_U16_WRAP, SENSOR_STREAM_MODE_SIGNED_I16):
        raise ValueError("unsupported sensor stream mode")
    return _build_command_frame(CMD_SENSOR_STREAM_MODE, bytes([mode_int & 0xFF]))


def build_local_test_params_cmd(freq_hz: float, amplitude_deg: float) -> bytes:
    payload = struct.pack("<ff", freq_hz, amplitude_deg)
    return _build_command_frame(CMD_LOCAL_TEST_PARAMS, payload)


def build_local_test_start_cmd() -> bytes:
    return _build_command_frame(CMD_LOCAL_TEST_START, b"")


def build_local_test_stop_cmd() -> bytes:
    return _build_command_frame(CMD_LOCAL_TEST_STOP, b"")


def build_servo_internal_zero_cmd() -> bytes:
    return _build_command_frame(CMD_SERVO_INTERNAL_ZERO, b"")
