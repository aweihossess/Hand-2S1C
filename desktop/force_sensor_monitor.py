"""Force sensor Modbus-RTU monitor.

Based on the supplied manual:
- Default serial settings: 9600 baud, 8 data bits, no parity, 1 stop bit.
- Default slave address: 1.
- Live weight/force value: holding registers 0x0000 and 0x0001.
- The 32-bit signed value uses the lower 16-bit word first.

The UI uses Tkinter so it can run without Qt. On Windows it can talk to COM
ports through pyserial when installed, or through the native Win32 serial API.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import os
import queue
import re
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


def configure_tcl_tk_env() -> None:
    python_dir = Path(sys.executable).resolve().parent
    tcl_root = python_dir / "tcl"
    tcl_dir = tcl_root / "tcl8.6"
    tk_dir = tcl_root / "tk8.6"
    if (tcl_dir / "init.tcl").exists():
        os.environ.setdefault("TCL_LIBRARY", tcl_dir.as_posix())
    if (tk_dir / "tk.tcl").exists():
        os.environ.setdefault("TK_LIBRARY", tk_dir.as_posix())
    if tcl_root.exists():
        os.environ.setdefault("TCLLIBPATH", tcl_root.as_posix())


configure_tcl_tk_env()

from tkinter import filedialog, messagebox, ttk
import tkinter as tk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # The Win32 backend below is enough on this Windows setup.
    serial = None
    list_ports = None

if sys.platform == "win32":
    import winreg
    from ctypes import wintypes


BAUD_RATES = [2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200, 256000, 500000]
DEFAULT_BAUDRATE = 115200
DEFAULT_CHANNEL = 5
REFERENCE_CHANNEL = 1
DISPLAY_CHANNELS = [1, 2, 3, 4, 5]
CHANNEL_COLORS = {
    1: "#dc2626",
    2: "#f97316",
    3: "#84cc16",
    4: "#06b6d4",
    5: "#2563eb",
}
MAX_SAMPLES = 5000
MAX_TABLE_ROWS = 300
RELATIVE_UNITS_PER_NEWTON = 10.0
FORCE_ZERO_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "force_sensor_zero.json"
ENCODER_ZERO_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "encoder_zero.json"
ENCODER_COUNT = 21
SERVO_COUNT = 22
ENCODER_RAW_TO_DEG = 360.0 / 16384.0
ENCODER_SERVO_DEFAULT_BAUDRATE = 921600
ENCODER_SERVO_MAX_HISTORY = 1200
UPPER_PROTOCOL_HEADER = 0xFE
UPPER_PROTOCOL_TAIL = 0xFF
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
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE
CMD_SENSOR_STREAM_MODE = 0xD1
SENSOR_STREAM_MODE_SIGNED_I16 = 1
DISCONNECT_SENTINEL = 0x7FFF
ENCODER_SERVO_COLORS = [
    "#dc2626",
    "#ea580c",
    "#ca8a04",
    "#65a30d",
    "#16a34a",
    "#059669",
    "#0891b2",
    "#0284c7",
    "#2563eb",
    "#4f46e5",
    "#7c3aed",
    "#9333ea",
    "#c026d3",
    "#db2777",
    "#e11d48",
    "#475569",
]


@dataclass(frozen=True)
class ForceSample:
    timestamp: float
    channel: int
    value: int
    low_word: int
    high_word: int
    request_hex: str
    response_hex: str


@dataclass(frozen=True)
class EncoderServoSnapshot:
    timestamp: float
    encoder_raw: list[int]
    encoder_mapped: list[int]
    encoder_deg: list[float]
    encoder_valid: list[bool]
    servo_abs: list[int]
    servo_hardware_abs: list[int]
    servo_zero_offset: list[int]
    servo_online: list[bool]
    servo_raw: list[int]
    servo_raw_online: list[bool]
    servo_speed: list[int]
    servo_load: list[int]
    servo_voltage: list[int]
    servo_temperature: list[int]
    last_line: str


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def format_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def format_force_n(relative_value: int) -> str:
    force_n = relative_value / RELATIVE_UNITS_PER_NEWTON
    if force_n == 0:
        return "0 N"
    return f"{force_n:.1f} N"


def load_force_zero_baselines() -> dict[int, int]:
    try:
        with open(FORCE_ZERO_CONFIG_PATH, "r", encoding="utf-8") as fp:
            payload = json.load(fp)
    except FileNotFoundError:
        return {}
    except Exception:
        return {}

    baselines: dict[int, int] = {}
    raw = payload.get("baselines", payload) if isinstance(payload, dict) else {}
    if not isinstance(raw, dict):
        return {}
    for key, value in raw.items():
        try:
            channel = int(key)
            if channel in DISPLAY_CHANNELS:
                baselines[channel] = int(value)
        except (TypeError, ValueError):
            continue
    return baselines


def save_force_zero_baselines(baselines: dict[int, int]) -> None:
    FORCE_ZERO_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "Force sensor no-tension zero baseline. Relative force N = (raw - baseline) / 10.",
        "channels": DISPLAY_CHANNELS,
        "baselines": {str(channel): int(baselines[channel]) for channel in DISPLAY_CHANNELS},
    }
    with open(FORCE_ZERO_CONFIG_PATH, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
        fp.write("\n")


def delete_force_zero_baselines() -> None:
    try:
        FORCE_ZERO_CONFIG_PATH.unlink()
    except FileNotFoundError:
        pass


def load_encoder_zero_offsets() -> dict[int, float]:
    try:
        with open(ENCODER_ZERO_CONFIG_PATH, "r", encoding="utf-8") as fp:
            payload = json.load(fp)
    except FileNotFoundError:
        return {}
    except Exception:
        return {}

    offsets: dict[int, float] = {}
    raw = payload.get("zero_deg", payload) if isinstance(payload, dict) else {}
    if not isinstance(raw, dict):
        return {}
    for key, value in raw.items():
        try:
            joint = int(key)
            if 0 <= joint < ENCODER_COUNT:
                offsets[joint] = float(value)
        except (TypeError, ValueError):
            continue
    return offsets


def save_encoder_zero_offsets(offsets: dict[int, float]) -> None:
    ENCODER_ZERO_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "Encoder display zero. UI relative deg = device deg - zero_deg; degree targets add zero_deg before sending.",
        "zero_deg": {str(joint): float(offsets[joint]) for joint in sorted(offsets)},
    }
    with open(ENCODER_ZERO_CONFIG_PATH, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
        fp.write("\n")


def delete_encoder_zero_offsets() -> None:
    try:
        ENCODER_ZERO_CONFIG_PATH.unlink()
    except FileNotFoundError:
        pass


def build_upper_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    frame_len = 1 + len(payload) + 1
    return (
        bytes((UPPER_PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF))
        + payload
        + bytes((UPPER_PROTOCOL_TAIL,))
    )


def build_upper_stream_mode_cmd(mode: int = SENSOR_STREAM_MODE_SIGNED_I16) -> bytes:
    return build_upper_command_frame(CMD_SENSOR_STREAM_MODE, bytes((mode & 0xFF,)))


def parse_upper_frame_bytes(buffer: bytearray) -> tuple[list[tuple[int, bytes]], list[str]]:
    frames: list[tuple[int, bytes]] = []
    lines: list[str] = []

    while buffer:
        if buffer[0] == UPPER_PROTOCOL_HEADER:
            if len(buffer) < 4:
                break
            wire_len = buffer[1]
            frame_len = wire_len + 2
            if wire_len < 2:
                del buffer[0]
                continue
            if len(buffer) < frame_len:
                break
            frame = bytes(buffer[:frame_len])
            del buffer[:frame_len]
            if frame[-1] != UPPER_PROTOCOL_TAIL:
                continue
            frames.append((frame[2], frame[3:-1]))
            continue

        next_header = buffer.find(bytes((UPPER_PROTOCOL_HEADER,)))
        next_newline_candidates = [idx for idx in (buffer.find(b"\n"), buffer.find(b"\r")) if idx >= 0]
        next_newline = min(next_newline_candidates) if next_newline_candidates else -1

        if next_newline >= 0 and (next_header < 0 or next_newline < next_header):
            raw_line = bytes(buffer[:next_newline])
            del buffer[: next_newline + 1]
            while buffer[:1] in (b"\n", b"\r"):
                del buffer[0]
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
            continue

        if next_header > 0:
            raw_line = bytes(buffer[:next_header]).strip()
            del buffer[:next_header]
            if raw_line:
                lines.append(raw_line.decode("utf-8", errors="replace"))
            continue

        if len(buffer) > 512:
            raw_line = bytes(buffer).strip()
            buffer.clear()
            if raw_line:
                lines.append(raw_line.decode("utf-8", errors="replace"))
        break

    return frames, lines


def parse_encoder_sensor_payload(payload: bytes) -> tuple[list[int], list[int], list[bool]]:
    raw_counts = [0] * ENCODER_COUNT
    mapped_counts = [0] * ENCODER_COUNT
    valid = [False] * ENCODER_COUNT
    legacy_len = ENCODER_COUNT * 2
    dual_len = ENCODER_COUNT * 4
    if len(payload) >= dual_len:
        for index in range(ENCODER_COUNT):
            raw_offset = index * 2
            mapped_offset = legacy_len + index * 2
            raw_u16 = (payload[raw_offset] << 8) | payload[raw_offset + 1]
            mapped_u16 = (payload[mapped_offset] << 8) | payload[mapped_offset + 1]
            is_valid = raw_u16 != DISCONNECT_SENTINEL and mapped_u16 != DISCONNECT_SENTINEL
            raw_counts[index] = int(raw_u16 if is_valid else 0)
            mapped_counts[index] = struct.unpack(">h", payload[mapped_offset : mapped_offset + 2])[0] if is_valid else 0
            valid[index] = is_valid
    elif len(payload) >= legacy_len:
        for index in range(ENCODER_COUNT):
            offset = index * 2
            mapped_u16 = (payload[offset] << 8) | payload[offset + 1]
            is_valid = mapped_u16 != DISCONNECT_SENTINEL
            mapped = struct.unpack(">h", payload[offset : offset + 2])[0] if is_valid else 0
            raw_counts[index] = mapped
            mapped_counts[index] = mapped
            valid[index] = is_valid
    return raw_counts, mapped_counts, valid


def parse_servo_angle_payload(payload: bytes) -> Optional[tuple[list[int], list[int], list[bool]]]:
    if not payload:
        return None
    has_offsets = (len(payload) % 9) == 0
    if has_offsets:
        count = len(payload) // 9
    elif (len(payload) % 5) == 0:
        count = len(payload) // 5
    else:
        return None
    if count <= 0:
        return None

    count = min(count, SERVO_COUNT)
    angles = [0] * SERVO_COUNT
    offsets = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        start = index * 4
        angles[index] = struct.unpack(">i", payload[start : start + 4])[0]
    base = count * 4
    if has_offsets:
        for index in range(count):
            start = base + index * 4
            offsets[index] = struct.unpack(">i", payload[start : start + 4])[0]
        base += count * 4
    for index in range(count):
        online[index] = payload[base + index] == 1
    return angles, offsets, online


def parse_servo_raw_payload(payload: bytes) -> Optional[tuple[list[int], list[bool]]]:
    if not payload or (len(payload) % 3) != 0:
        return None
    count = min(len(payload) // 3, SERVO_COUNT)
    raw_positions = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        start = index * 2
        raw_positions[index] = struct.unpack(">h", payload[start : start + 2])[0]
    base = count * 2
    for index in range(count):
        online[index] = payload[base + index] == 1
    return raw_positions, online


def parse_servo_telem_payload(payload: bytes) -> Optional[tuple[list[int], list[int], list[int], list[int], list[bool]]]:
    if not payload or (len(payload) % 7) != 0:
        return None
    count = min(len(payload) // 7, SERVO_COUNT)
    speeds = [0] * SERVO_COUNT
    loads = [0] * SERVO_COUNT
    volts = [0] * SERVO_COUNT
    temps = [0] * SERVO_COUNT
    online = [False] * SERVO_COUNT
    for index in range(count):
        base = index * 7
        speeds[index] = struct.unpack(">h", payload[base : base + 2])[0]
        loads[index] = struct.unpack(">h", payload[base + 2 : base + 4])[0]
        volts[index] = payload[base + 4]
        temps[index] = payload[base + 5]
        online[index] = payload[base + 6] == 1
    return speeds, loads, volts, temps, online


def parse_joint_debug_payload(payload: bytes) -> Optional[tuple[int, bool, float, float, int]]:
    legacy_len = 2 + 5 * 4
    cmd_len = legacy_len + 1 + 2
    cmd_ts_len = cmd_len + 4
    extended_len = 2 + 8 * 4 + 4 + 2 + 1 + 1 + 2
    if len(payload) not in (legacy_len, cmd_len, cmd_ts_len, extended_len):
        return None
    if len(payload) == extended_len:
        joint_index, valid, target_deg, actual_deg = struct.unpack(">BBff", payload[:10])
        cmd_target_pos = struct.unpack(">h", payload[42:44])[0]
        return joint_index, valid == 1, float(target_deg), float(actual_deg), int(cmd_target_pos)
    joint_index, valid, target_deg, actual_deg = struct.unpack(">BBff", payload[:10])
    cmd_target_pos = 0
    if len(payload) >= cmd_len:
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1 : legacy_len + 3])[0]
    return joint_index, valid == 1, float(target_deg), float(actual_deg), int(cmd_target_pos)


def text_key_values(line: str) -> dict[str, str]:
    return dict(re.findall(r"([A-Za-z_]+)=([^\s>]+)", line))


def build_read_holding_registers(slave_id: int, start_register: int, count: int) -> bytes:
    payload = bytes(
        (
            slave_id & 0xFF,
            0x03,
            (start_register >> 8) & 0xFF,
            start_register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        )
    )
    return append_crc(payload)


def weight_register_for_channel(channel: int) -> int:
    if channel < 1:
        raise ValueError("channel must be >= 1")
    return (channel - 1) * 2


def build_write_single_register(slave_id: int, register: int, value: int) -> bytes:
    payload = bytes(
        (
            slave_id & 0xFF,
            0x06,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        )
    )
    return append_crc(payload)


def validate_crc(frame: bytes) -> None:
    if len(frame) < 4:
        raise ValueError("response too short")
    expected = frame[-2] | (frame[-1] << 8)
    actual = crc16_modbus(frame[:-2])
    if expected != actual:
        raise ValueError(f"CRC mismatch: got 0x{expected:04X}, expected 0x{actual:04X}")


def decode_live_weight_response(frame: bytes, slave_id: int) -> tuple[int, int, int]:
    validate_crc(frame)
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x03:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    if frame[2] < 4 or len(frame) < 9:
        raise ValueError("response does not contain two registers")

    low_word = (frame[3] << 8) | frame[4]
    high_word = (frame[5] << 8) | frame[6]
    raw = (high_word << 16) | low_word
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw, low_word, high_word


def decode_holding_registers_response(frame: bytes, slave_id: int) -> list[int]:
    validate_crc(frame)
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x03:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    byte_count = frame[2]
    if byte_count % 2:
        raise ValueError("register byte count should be even")
    if len(frame) != byte_count + 5:
        raise ValueError(f"response length mismatch: got {len(frame)}, expected {byte_count + 5}")
    registers = []
    for index in range(byte_count // 2):
        offset = 3 + index * 2
        registers.append((frame[offset] << 8) | frame[offset + 1])
    return registers


def words_to_u32_low_first(low_word: int, high_word: int) -> int:
    return ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)


def words_to_i32_low_first(low_word: int, high_word: int) -> int:
    raw = words_to_u32_low_first(low_word, high_word)
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw


def decode_write_single_register_response(frame: bytes, request: bytes, slave_id: int) -> None:
    validate_crc(frame)
    if len(frame) != 8:
        raise ValueError("write response should be 8 bytes")
    if frame[0] != slave_id:
        raise ValueError(f"unexpected slave id {frame[0]}, expected {slave_id}")
    if frame[1] & 0x80:
        code = frame[2] if len(frame) > 2 else 0
        raise ValueError(f"Modbus exception 0x{code:02X}")
    if frame[1] != 0x06:
        raise ValueError(f"unexpected function 0x{frame[1]:02X}")
    if frame[:6] != request[:6]:
        raise ValueError("write response does not echo the request")


class Win32Serial:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    FILE_ATTRIBUTE_NORMAL = 0x80
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
    PURGE_RXCLEAR = 0x0008
    PURGE_TXCLEAR = 0x0004

    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("fFlags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    def __init__(self, port: str, baudrate: int, timeout_s: float = 0.25) -> None:
        if sys.platform != "win32":
            raise RuntimeError("native serial backend is only available on Windows")
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._bind_kernel32_functions()
        name = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        self.handle = self.kernel32.CreateFileW(
            name,
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            self.FILE_ATTRIBUTE_NORMAL,
            None,
        )
        if self.handle == self.INVALID_HANDLE_VALUE:
            self._raise_last_error(f"open {port}")

        try:
            self._configure(baudrate, timeout_s)
        except Exception:
            self.close()
            raise

    def _bind_kernel32_functions(self) -> None:
        self.kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.LPVOID,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.HANDLE,
        ]
        self.kernel32.CreateFileW.restype = wintypes.HANDLE
        self.kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        self.kernel32.CloseHandle.restype = wintypes.BOOL
        self.kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self.DCB)]
        self.kernel32.GetCommState.restype = wintypes.BOOL
        self.kernel32.BuildCommDCBW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(self.DCB)]
        self.kernel32.BuildCommDCBW.restype = wintypes.BOOL
        self.kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self.DCB)]
        self.kernel32.SetCommState.restype = wintypes.BOOL
        self.kernel32.SetCommTimeouts.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(self.COMMTIMEOUTS),
        ]
        self.kernel32.SetCommTimeouts.restype = wintypes.BOOL
        self.kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self.kernel32.PurgeComm.restype = wintypes.BOOL
        self.kernel32.WriteFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPCVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.WriteFile.restype = wintypes.BOOL
        self.kernel32.ReadFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.ReadFile.restype = wintypes.BOOL
        self.kernel32.FlushFileBuffers.argtypes = [wintypes.HANDLE]
        self.kernel32.FlushFileBuffers.restype = wintypes.BOOL

    def _raise_last_error(self, action: str) -> None:
        code = ctypes.get_last_error()
        raise OSError(code, f"Win32 serial {action} failed: {ctypes.FormatError(code)}")

    def _configure(self, baudrate: int, timeout_s: float) -> None:
        dcb = self.DCB()
        dcb.DCBlength = ctypes.sizeof(self.DCB)
        if not self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)):
            self._raise_last_error("GetCommState")

        config = f"baud={baudrate} parity=N data=8 stop=1"
        if not self.kernel32.BuildCommDCBW(config, ctypes.byref(dcb)):
            self._raise_last_error("BuildCommDCB")
        dcb.BaudRate = baudrate
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.fFlags |= 0x00000001  # fBinary
        dcb.fFlags &= ~0x00000002  # fParity
        if not self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
            self._raise_last_error("SetCommState")

        timeout_ms = max(1, int(timeout_s * 1000))
        timeouts = self.COMMTIMEOUTS()
        timeouts.ReadIntervalTimeout = 20
        timeouts.ReadTotalTimeoutMultiplier = 0
        timeouts.ReadTotalTimeoutConstant = timeout_ms
        timeouts.WriteTotalTimeoutMultiplier = 0
        timeouts.WriteTotalTimeoutConstant = timeout_ms
        if not self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)):
            self._raise_last_error("SetCommTimeouts")

    def reset_input_buffer(self) -> None:
        self.kernel32.PurgeComm(self.handle, self.PURGE_RXCLEAR | self.PURGE_TXCLEAR)

    def write(self, data: bytes) -> int:
        written = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(data)
        if not self.kernel32.WriteFile(
            self.handle, buffer, len(data), ctypes.byref(written), None
        ):
            self._raise_last_error("WriteFile")
        return int(written.value)

    def read(self, size: int) -> bytes:
        read_count = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(size)
        if not self.kernel32.ReadFile(
            self.handle, buffer, size, ctypes.byref(read_count), None
        ):
            self._raise_last_error("ReadFile")
        return buffer.raw[: read_count.value]

    def flush(self) -> None:
        self.kernel32.FlushFileBuffers(self.handle)

    def close(self) -> None:
        if getattr(self, "handle", None):
            self.kernel32.CloseHandle(self.handle)
            self.handle = None


def list_serial_ports() -> list[str]:
    ports: list[str] = []
    if list_ports is not None:
        ports.extend(port.device for port in list_ports.comports())
    elif sys.platform == "win32":
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            index = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, index)
                    ports.append(str(value))
                    index += 1
                except OSError:
                    break
        except OSError:
            pass
    if "COM7" not in ports:
        ports.insert(0, "COM7")
    return sorted(dict.fromkeys(ports))


def open_serial_port(port: str, baudrate: int, timeout_s: float = 0.25):
    if serial is not None:
        return serial.Serial(
            port,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )
    return Win32Serial(port, baudrate, timeout_s)


def read_exact(port, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < size and time.monotonic() < deadline:
        chunk = port.read(size - len(data))
        if chunk:
            data.extend(chunk)
        else:
            time.sleep(0.005)
    return bytes(data)


class SensorPoller(threading.Thread):
    def __init__(
        self,
        outbox: "queue.Queue[tuple[str, object]]",
        command_queue: "queue.Queue[tuple[str, bytes, int]]",
        stop_event: threading.Event,
        port_name: str,
        baudrate: int,
        slave_id: int,
        channel: int,
        interval_ms: int,
    ) -> None:
        super().__init__(daemon=True)
        self.outbox = outbox
        self.command_queue = command_queue
        self.stop_event = stop_event
        self.port_name = port_name
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.channel = channel
        self.channels = list(DISPLAY_CHANNELS)
        self.interval_ms = interval_ms

    def run(self) -> None:
        requests = {
            channel: build_read_holding_registers(
                self.slave_id, weight_register_for_channel(channel), 2
            )
            for channel in self.channels
        }
        port = None
        last_error: Optional[str] = None
        try:
            self.outbox.put(("status", f"opening {self.port_name} @ {self.baudrate} 8N1"))
            port = open_serial_port(self.port_name, self.baudrate, timeout_s=0.25)
            self.outbox.put(("connected", True))
            self.outbox.put(("status", "connected"))
            while not self.stop_event.is_set():
                started = time.monotonic()
                try:
                    self._process_commands(port)
                    for channel, request in requests.items():
                        if hasattr(port, "reset_input_buffer"):
                            port.reset_input_buffer()
                        port.write(request)
                        if hasattr(port, "flush"):
                            port.flush()
                        response = read_exact(port, 9, timeout_s=0.3)
                        value, low_word, high_word = decode_live_weight_response(
                            response, self.slave_id
                        )
                        self.outbox.put(
                            (
                                "sample",
                                ForceSample(
                                    timestamp=time.time(),
                                    channel=channel,
                                    value=value,
                                    low_word=low_word,
                                    high_word=high_word,
                                    request_hex=format_hex(request),
                                    response_hex=format_hex(response),
                                ),
                            )
                        )
                except Exception as exc:
                    last_error = str(exc)
                    self.outbox.put(("error", last_error))

                elapsed = time.monotonic() - started
                wait_s = max(0.01, self.interval_ms / 1000.0 - elapsed)
                self.stop_event.wait(wait_s)
        except Exception as exc:
            last_error = str(exc)
            self.outbox.put(("error", last_error))
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
            self.outbox.put(("connected", False))
            if self.stop_event.is_set():
                self.outbox.put(("status", "disconnected"))
            elif last_error:
                self.outbox.put(("status", f"disconnected: {last_error}"))
            else:
                self.outbox.put(("status", "disconnected"))

    def _process_commands(self, port) -> None:
        while not self.stop_event.is_set():
            try:
                label, request, response_len = self.command_queue.get_nowait()
            except queue.Empty:
                return
            if hasattr(port, "reset_input_buffer"):
                port.reset_input_buffer()
            port.write(request)
            if hasattr(port, "flush"):
                port.flush()
            response = read_exact(port, response_len, timeout_s=0.3)
            if response_len == 8:
                decode_write_single_register_response(response, request, self.slave_id)
            else:
                registers = decode_holding_registers_response(response, self.slave_id)
                if label.startswith("诊断读取") and len(registers) >= 2:
                    weight = words_to_i32_low_first(registers[0], registers[1])
                    self.outbox.put(
                        (
                            "diagnostic",
                            {
                                "label": label,
                                "weight": weight,
                                "low": registers[0],
                                "high": registers[1],
                            },
                        )
                    )
            self.outbox.put(("frame", (label, format_hex(request), format_hex(response))))
            self.outbox.put(("status", f"{label} ok"))


class EncoderServoPoller(threading.Thread):
    def __init__(
        self,
        outbox: "queue.Queue[tuple[str, object]]",
        command_queue: "queue.Queue[tuple[str, object]]",
        stop_event: threading.Event,
        port_name: str,
        baudrate: int,
    ) -> None:
        super().__init__(daemon=True)
        self.outbox = outbox
        self.command_queue = command_queue
        self.stop_event = stop_event
        self.port_name = port_name
        self.baudrate = baudrate
        self.buffer = bytearray()
        self.encoder_raw = [0] * ENCODER_COUNT
        self.encoder_mapped = [0] * ENCODER_COUNT
        self.encoder_deg = [0.0] * ENCODER_COUNT
        self.encoder_valid = [False] * ENCODER_COUNT
        self.servo_abs = [0] * SERVO_COUNT
        self.servo_hardware_abs = [0] * SERVO_COUNT
        self.servo_zero_offset = [0] * SERVO_COUNT
        self.servo_online = [False] * SERVO_COUNT
        self.servo_raw = [0] * SERVO_COUNT
        self.servo_raw_online = [False] * SERVO_COUNT
        self.servo_speed = [0] * SERVO_COUNT
        self.servo_load = [0] * SERVO_COUNT
        self.servo_voltage = [0] * SERVO_COUNT
        self.servo_temperature = [0] * SERVO_COUNT
        self.last_line = "--"
        self.last_emit = 0.0

    def run(self) -> None:
        port = None
        last_error: Optional[str] = None
        try:
            self.outbox.put(("status", f"opening {self.port_name} @ {self.baudrate} 8N1"))
            port = open_serial_port(self.port_name, self.baudrate, timeout_s=0.05)
            self.outbox.put(("connected", True))
            self.outbox.put(("status", "connected"))
            self._write_bytes(port, build_upper_stream_mode_cmd())
            self._write_text(port, "binary")
            self._write_text(port, "encoder")
            self._write_text(port, "servo")
            self._write_text(port, "load")
            next_snapshot = time.monotonic() + 0.75
            while not self.stop_event.is_set():
                self._process_commands(port)
                now = time.monotonic()
                if now >= next_snapshot:
                    self._write_text(port, "encoder")
                    self._write_text(port, "servo")
                    self._write_text(port, "load")
                    next_snapshot = now + 0.75
                chunk = port.read(512)
                if chunk:
                    self.buffer.extend(chunk)
                    frames, lines = parse_upper_frame_bytes(self.buffer)
                    updated = False
                    for pkt_type, payload in frames:
                        updated = self._handle_frame(pkt_type, payload) or updated
                    for line in lines:
                        updated = self._handle_text_line(line) or updated
                    if updated:
                        self._emit_snapshot()
                else:
                    self.stop_event.wait(0.01)
        except Exception as exc:
            last_error = str(exc)
            self.outbox.put(("error", last_error))
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
            self.outbox.put(("connected", False))
            if self.stop_event.is_set():
                self.outbox.put(("status", "disconnected"))
            elif last_error:
                self.outbox.put(("status", f"disconnected: {last_error}"))
            else:
                self.outbox.put(("status", "disconnected"))

    def _write_bytes(self, port, data: bytes) -> None:
        port.write(data)
        if hasattr(port, "flush"):
            port.flush()

    def _write_text(self, port, text: str) -> None:
        payload = text.strip().encode("ascii", errors="ignore") + b"\n"
        self._write_bytes(port, payload)

    def _process_commands(self, port) -> None:
        while not self.stop_event.is_set():
            try:
                kind, payload = self.command_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "text":
                text = str(payload).strip()
                if text:
                    self._write_text(port, text)
                    self.outbox.put(("tx", text))
            elif kind == "bytes":
                data = bytes(payload)
                if data:
                    self._write_bytes(port, data)
                    self.outbox.put(("tx", format_hex(data)))

    def _handle_frame(self, pkt_type: int, payload: bytes) -> bool:
        if pkt_type == PACKET_TYPE_SENSOR:
            raw, mapped, valid = parse_encoder_sensor_payload(payload)
            self.encoder_raw = raw
            self.encoder_mapped = mapped
            self.encoder_valid = valid
            self.encoder_deg = [mapped[index] * ENCODER_RAW_TO_DEG for index in range(ENCODER_COUNT)]
            return True
        if pkt_type == PACKET_TYPE_SERVO_ANGLE:
            parsed = parse_servo_angle_payload(payload)
            if parsed is None:
                return False
            angles, offsets, online = parsed
            self.servo_abs = angles
            self.servo_zero_offset = offsets
            self.servo_online = online
            self.servo_hardware_abs = [
                self.servo_abs[index] + self.servo_zero_offset[index] for index in range(SERVO_COUNT)
            ]
            return True
        if pkt_type == PACKET_TYPE_SERVO_RAW:
            parsed = parse_servo_raw_payload(payload)
            if parsed is None:
                return False
            self.servo_raw, self.servo_raw_online = parsed
            return True
        if pkt_type == PACKET_TYPE_SERVO_TELEM:
            parsed = parse_servo_telem_payload(payload)
            if parsed is None:
                return False
            speed, load, voltage, temperature, online = parsed
            self.servo_speed = speed
            self.servo_load = load
            self.servo_voltage = voltage
            self.servo_temperature = temperature
            self.servo_online = [
                self.servo_online[index] or online[index] for index in range(SERVO_COUNT)
            ]
            return True
        if pkt_type == PACKET_TYPE_JOINT_DEBUG:
            parsed = parse_joint_debug_payload(payload)
            if parsed is None:
                return False
            joint_index, valid, _target_deg, actual_deg, _cmd_target_pos = parsed
            if 0 <= joint_index < ENCODER_COUNT:
                self.encoder_deg[joint_index] = actual_deg
                self.encoder_valid[joint_index] = valid
            return True
        if pkt_type in (PACKET_TYPE_PROTO_ACK, PACKET_TYPE_CALIB_ACK, PACKET_TYPE_FAULT_STATUS, PACKET_TYPE_RELEASE_FAULT, PACKET_TYPE_CONTROL_STATUS):
            self.last_line = f"RX packet 0x{pkt_type:02X}: {format_hex(payload)}"
            self.outbox.put(("line", self.last_line))
            return False
        return False

    def _handle_text_line(self, line: str) -> bool:
        self.last_line = line
        self.outbox.put(("line", line))
        values = text_key_values(line)
        updated = False

        match = re.search(r"\bENC\s+J(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < ENCODER_COUNT:
                if "raw" in values:
                    self.encoder_raw[index] = int(float(values["raw"]))
                if "mapped" in values:
                    self.encoder_mapped[index] = int(float(values["mapped"]))
                if "deg" in values:
                    self.encoder_deg[index] = float(values["deg"])
                elif "mapped" in values:
                    self.encoder_deg[index] = self.encoder_mapped[index] * ENCODER_RAW_TO_DEG
                valid = values.get("mapped_valid", values.get("raw_valid"))
                if valid is not None:
                    self.encoder_valid[index] = valid not in ("0", "false", "False")
                updated = True

        match = re.search(r"\bSERVO\s+M(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < SERVO_COUNT:
                if "motor_abs" in values:
                    self.servo_abs[index] = int(float(values["motor_abs"]))
                if "hardware_abs" in values:
                    self.servo_hardware_abs[index] = int(float(values["hardware_abs"]))
                if "sw_zero_ofs" in values:
                    self.servo_zero_offset[index] = int(float(values["sw_zero_ofs"]))
                if "online" in values:
                    self.servo_online[index] = values["online"] not in ("0", "false", "False")
                updated = True

        match = re.search(r"\bLOAD\s+M(\d+)", line)
        if match:
            index = int(match.group(1))
            if 0 <= index < SERVO_COUNT:
                if "load" in values:
                    self.servo_load[index] = int(float(values["load"]))
                if "speed" in values:
                    self.servo_speed[index] = int(float(values["speed"]))
                if "voltage" in values:
                    self.servo_voltage[index] = int(float(values["voltage"]))
                if "temperature" in values:
                    self.servo_temperature[index] = int(float(values["temperature"]))
                if "online" in values:
                    self.servo_online[index] = values["online"] not in ("0", "false", "False")
                updated = True

        return updated

    def _emit_snapshot(self) -> None:
        now = time.time()
        if now - self.last_emit < 0.03:
            return
        self.last_emit = now
        self.outbox.put(
            (
                "snapshot",
                EncoderServoSnapshot(
                    timestamp=now,
                    encoder_raw=list(self.encoder_raw),
                    encoder_mapped=list(self.encoder_mapped),
                    encoder_deg=list(self.encoder_deg),
                    encoder_valid=list(self.encoder_valid),
                    servo_abs=list(self.servo_abs),
                    servo_hardware_abs=list(self.servo_hardware_abs),
                    servo_zero_offset=list(self.servo_zero_offset),
                    servo_online=list(self.servo_online),
                    servo_raw=list(self.servo_raw),
                    servo_raw_online=list(self.servo_raw_online),
                    servo_speed=list(self.servo_speed),
                    servo_load=list(self.servo_load),
                    servo_voltage=list(self.servo_voltage),
                    servo_temperature=list(self.servo_temperature),
                    last_line=self.last_line,
                ),
            )
        )


class EncoderServoWindow(tk.Toplevel):
    def __init__(self, parent: tk.Misc, default_port: str, default_baudrate: int) -> None:
        super().__init__(parent)
        self.title("Encoder / Servo 监视与目标输出")
        self.geometry("1220x760")
        self.minsize(980, 640)

        self.outbox: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.command_queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.poller: Optional[EncoderServoPoller] = None
        self.connected = False
        self.history: list[EncoderServoSnapshot] = []
        self.latest_snapshot: Optional[EncoderServoSnapshot] = None
        self.encoder_zero_deg: dict[int, float] = load_encoder_zero_offsets()

        self.port_var = tk.StringVar(value=default_port)
        self.baud_var = tk.StringVar(value=str(default_baudrate))
        self.status_var = tk.StringVar(value="未连接")
        self.last_line_var = tk.StringVar(value="RX: --")
        self.encoder_zero_status_var = tk.StringVar(value=self._encoder_zero_status_text())
        self.mode_var = tk.StringVar(value="degree")
        self.target_index_var = tk.IntVar(value=0)
        self.target_value_var = tk.StringVar(value="0")

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_outbox)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=2)
        self.rowconfigure(3, weight=3)

        connection = ttk.LabelFrame(self, text="Encoder / Servo 串口")
        connection.grid(row=0, column=0, padx=10, pady=(10, 6), sticky="ew")
        connection.columnconfigure(12, weight=1)

        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=12)
        self.baud_combo = ttk.Combobox(
            connection,
            textvariable=self.baud_var,
            values=[str(v) for v in BAUD_RATES + [ENCODER_SERVO_DEFAULT_BAUDRATE]],
            width=10,
        )
        self.refresh_button = ttk.Button(connection, text="刷新串口", command=self._refresh_ports)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(8, 4), pady=8)
        self.port_combo.grid(row=0, column=1, padx=(0, 10), pady=8)
        ttk.Label(connection, text="波特率").grid(row=0, column=2, padx=(8, 4), pady=8)
        self.baud_combo.grid(row=0, column=3, padx=(0, 10), pady=8)
        self.refresh_button.grid(row=0, column=4, padx=4, pady=8)
        self.connect_button.grid(row=0, column=5, padx=4, pady=8)
        ttk.Label(connection, text="状态").grid(row=0, column=6, padx=(20, 4), pady=8)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=7, padx=(0, 10), pady=8, sticky="w")

        commands = ttk.LabelFrame(self, text="目标输出与快照命令")
        commands.grid(row=1, column=0, padx=10, pady=6, sticky="ew")
        for col in range(16):
            commands.columnconfigure(col, weight=0)
        commands.columnconfigure(15, weight=1)

        quick = [
            ("START", "start"),
            ("STOP", "stop"),
            ("ZERO", "zero"),
            ("ENCODER", "encoder"),
            ("SERVO", "servo"),
            ("STATUS", "status"),
            ("BINARY", "binary"),
        ]
        for col, (label, command) in enumerate(quick):
            ttk.Button(commands, text=label, command=lambda c=command: self._send_text(c)).grid(
                row=0, column=col, padx=3, pady=8
            )

        target_col = len(quick) + 1
        ttk.Label(commands, text="模式").grid(row=0, column=target_col, padx=(20, 4), pady=8)
        self.mode_combo = ttk.Combobox(
            commands,
            textvariable=self.mode_var,
            values=["degree", "direct"],
            width=8,
            state="readonly",
        )
        self.mode_combo.grid(row=0, column=target_col + 1, padx=(0, 8), pady=8)
        ttk.Label(commands, text="索引").grid(row=0, column=target_col + 2, padx=(4, 4), pady=8)
        self.target_index_spin = tk.Spinbox(commands, from_=0, to=SERVO_COUNT - 1, textvariable=self.target_index_var, width=5)
        self.target_index_spin.grid(row=0, column=target_col + 3, padx=(0, 8), pady=8)
        ttk.Label(commands, text="目标").grid(row=0, column=target_col + 4, padx=(4, 4), pady=8)
        self.target_entry = ttk.Entry(commands, textvariable=self.target_value_var, width=10)
        self.target_entry.grid(row=0, column=target_col + 5, padx=(0, 8), pady=8)
        ttk.Button(commands, text="发送目标", command=self._send_target).grid(
            row=0, column=target_col + 6, padx=4, pady=8
        )
        ttk.Button(commands, text="记录角度零点", command=self._set_encoder_zero).grid(
            row=1, column=0, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Button(commands, text="清除角度零点", command=self._clear_encoder_zero).grid(
            row=1, column=2, columnspan=2, padx=3, pady=(0, 8), sticky="w"
        )
        ttk.Label(commands, text="零点状态").grid(row=1, column=4, padx=(16, 4), pady=(0, 8), sticky="w")
        ttk.Label(commands, textvariable=self.encoder_zero_status_var).grid(
            row=1, column=5, columnspan=12, padx=(0, 8), pady=(0, 8), sticky="w"
        )

        self.encoder_plot = tk.Canvas(
            self,
            height=230,
            bg="white",
            highlightthickness=1,
            highlightbackground="#d0d7de",
        )
        self.encoder_plot.grid(row=2, column=0, padx=10, pady=6, sticky="nsew")
        self.encoder_plot.bind("<Configure>", lambda _event: self._draw_encoder_plot())

        data = ttk.Frame(self)
        data.grid(row=3, column=0, padx=10, pady=6, sticky="nsew")
        data.columnconfigure(0, weight=1)
        data.columnconfigure(1, weight=2)
        data.rowconfigure(0, weight=1)

        encoder_frame = ttk.LabelFrame(data, text="Encoder 角度")
        encoder_frame.grid(row=0, column=0, padx=(0, 6), sticky="nsew")
        encoder_frame.columnconfigure(0, weight=1)
        encoder_frame.rowconfigure(0, weight=1)
        self.encoder_table = ttk.Treeview(
            encoder_frame,
            columns=("joint", "raw", "mapped", "deg", "valid"),
            show="headings",
            height=12,
        )
        for key, text, width, anchor in (
            ("joint", "Joint", 70, "center"),
            ("raw", "Raw", 86, "e"),
            ("mapped", "Mapped", 86, "e"),
            ("deg", "Deg", 86, "e"),
            ("valid", "Valid", 64, "center"),
        ):
            self.encoder_table.heading(key, text=text)
            self.encoder_table.column(key, width=width, anchor=anchor, stretch=(key == "deg"))
        enc_scroll = ttk.Scrollbar(encoder_frame, orient="vertical", command=self.encoder_table.yview)
        self.encoder_table.configure(yscrollcommand=enc_scroll.set)
        self.encoder_table.grid(row=0, column=0, sticky="nsew")
        enc_scroll.grid(row=0, column=1, sticky="ns")
        for index in range(ENCODER_COUNT):
            self.encoder_table.insert("", "end", iid=f"J{index}", values=(f"J{index:02d}", "--", "--", "--", "--"))

        servo_frame = ttk.LabelFrame(data, text="Servo 数字读数")
        servo_frame.grid(row=0, column=1, padx=(6, 0), sticky="nsew")
        servo_frame.columnconfigure(0, weight=1)
        servo_frame.rowconfigure(0, weight=1)
        self.servo_table = ttk.Treeview(
            servo_frame,
            columns=("motor", "abs", "hardware", "zero", "raw", "speed", "load", "volt", "temp", "online"),
            show="headings",
            height=12,
        )
        servo_columns = (
            ("motor", "Motor", 64, "center"),
            ("abs", "Abs", 86, "e"),
            ("hardware", "HwAbs", 86, "e"),
            ("zero", "ZeroOfs", 86, "e"),
            ("raw", "Raw", 72, "e"),
            ("speed", "Speed", 72, "e"),
            ("load", "Load", 72, "e"),
            ("volt", "V", 52, "e"),
            ("temp", "Temp", 58, "e"),
            ("online", "Online", 64, "center"),
        )
        for key, text, width, anchor in servo_columns:
            self.servo_table.heading(key, text=text)
            self.servo_table.column(key, width=width, anchor=anchor, stretch=(key in ("abs", "hardware")))
        servo_scroll = ttk.Scrollbar(servo_frame, orient="vertical", command=self.servo_table.yview)
        self.servo_table.configure(yscrollcommand=servo_scroll.set)
        self.servo_table.grid(row=0, column=0, sticky="nsew")
        servo_scroll.grid(row=0, column=1, sticky="ns")
        for index in range(SERVO_COUNT):
            self.servo_table.insert(
                "",
                "end",
                iid=f"M{index}",
                values=(f"M{index:02d}", "--", "--", "--", "--", "--", "--", "--", "--", "--"),
            )

        line_frame = ttk.LabelFrame(self, text="通讯状态")
        line_frame.grid(row=4, column=0, padx=10, pady=(6, 10), sticky="ew")
        line_frame.columnconfigure(0, weight=1)
        ttk.Label(line_frame, textvariable=self.last_line_var).grid(row=0, column=0, padx=8, pady=6, sticky="w")

    def _refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if not self.port_var.get() and ports:
            candidates = [port for port in ports if port.upper() != "COM7"]
            self.port_var.set(candidates[0] if candidates else ports[0])

    def _toggle_connection(self) -> None:
        if self.poller is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        try:
            port = self.port_var.get().strip()
            baudrate = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "请检查 Encoder / Servo 波特率。", parent=self)
            return
        if not port:
            messagebox.showwarning("缺少串口", "请选择 Encoder / Servo 串口。", parent=self)
            return

        self.stop_event.clear()
        self.poller = EncoderServoPoller(self.outbox, self.command_queue, self.stop_event, port, baudrate)
        self.poller.start()
        self._set_controls_connected(True)

    def _disconnect(self) -> None:
        self.stop_event.set()
        poller = self.poller
        self.poller = None
        if poller is not None:
            poller.join(timeout=1.5)
        self._set_controls_connected(False)

    def _set_controls_connected(self, connected: bool) -> None:
        self.connected = connected
        self.connect_button.configure(text="断开" if connected else "连接")
        state = "disabled" if connected else "normal"
        self.port_combo.configure(state=state)
        self.baud_combo.configure(state=state)
        self.refresh_button.configure(state=state)

    def _process_outbox(self) -> None:
        try:
            while True:
                kind, payload = self.outbox.get_nowait()
                if kind == "snapshot":
                    self._handle_snapshot(payload)
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self.status_var.set(str(payload))
                elif kind == "connected":
                    self._set_controls_connected(bool(payload))
                    if not payload:
                        self.poller = None
                elif kind == "line":
                    self.last_line_var.set(f"RX: {payload}")
                elif kind == "tx":
                    self.last_line_var.set(f"TX: {payload}")
        except queue.Empty:
            pass
        if self.winfo_exists():
            self.after(50, self._process_outbox)

    def _encoder_display_deg(self, snapshot: EncoderServoSnapshot, joint: int) -> float:
        return snapshot.encoder_deg[joint] - self.encoder_zero_deg.get(joint, 0.0)

    def _encoder_zero_status_text(self) -> str:
        if not self.encoder_zero_deg:
            return "未保存角度零点"
        joints = sorted(self.encoder_zero_deg)
        preview = ", ".join(f"J{joint:02d}={self.encoder_zero_deg[joint]:.2f}" for joint in joints[:8])
        if len(joints) > 8:
            preview += f", ... 共{len(joints)}路"
        return f"已保存角度零点: {preview}"

    def _refresh_encoder_table_and_plot(self) -> None:
        snapshot = self.latest_snapshot
        if snapshot is None:
            self._draw_encoder_plot()
            return
        for index in range(ENCODER_COUNT):
            self.encoder_table.item(
                f"J{index}",
                values=(
                    f"J{index:02d}",
                    snapshot.encoder_raw[index],
                    snapshot.encoder_mapped[index],
                    f"{self._encoder_display_deg(snapshot, index):.2f}",
                    "OK" if snapshot.encoder_valid[index] else "--",
                ),
            )
        self._draw_encoder_plot()

    def _set_encoder_zero(self) -> None:
        snapshot = self.latest_snapshot
        if snapshot is None:
            messagebox.showinfo("等待数据", "请先连接 Encoder / Servo，并等待 Encoder 有读数。", parent=self)
            return
        valid_joints = [index for index, valid in enumerate(snapshot.encoder_valid) if valid]
        if not valid_joints:
            messagebox.showinfo("等待数据", "当前没有有效 Encoder 读数，无法记录角度零点。", parent=self)
            return
        self.encoder_zero_deg = {index: snapshot.encoder_deg[index] for index in valid_joints}
        try:
            save_encoder_zero_offsets(self.encoder_zero_deg)
        except Exception as exc:
            messagebox.showerror("保存失败", f"角度零点保存失败: {exc}", parent=self)
            return
        self.encoder_zero_status_var.set(self._encoder_zero_status_text())
        self.last_line_var.set(f"角度零点已保存: {ENCODER_ZERO_CONFIG_PATH}")
        self._refresh_encoder_table_and_plot()

    def _clear_encoder_zero(self) -> None:
        self.encoder_zero_deg.clear()
        try:
            delete_encoder_zero_offsets()
        except Exception as exc:
            messagebox.showerror("清除失败", f"角度零点配置删除失败: {exc}", parent=self)
            return
        self.encoder_zero_status_var.set(self._encoder_zero_status_text())
        self.last_line_var.set("角度零点已清除")
        self._refresh_encoder_table_and_plot()

    def _handle_snapshot(self, snapshot: EncoderServoSnapshot) -> None:
        self.latest_snapshot = snapshot
        self.history.append(snapshot)
        if len(self.history) > ENCODER_SERVO_MAX_HISTORY:
            del self.history[:300]
        for index in range(ENCODER_COUNT):
            self.encoder_table.item(
                f"J{index}",
                values=(
                    f"J{index:02d}",
                    snapshot.encoder_raw[index],
                    snapshot.encoder_mapped[index],
                    f"{self._encoder_display_deg(snapshot, index):.2f}",
                    "OK" if snapshot.encoder_valid[index] else "--",
                ),
            )
        for index in range(SERVO_COUNT):
            online = snapshot.servo_online[index] or snapshot.servo_raw_online[index]
            self.servo_table.item(
                f"M{index}",
                values=(
                    f"M{index:02d}",
                    snapshot.servo_abs[index],
                    snapshot.servo_hardware_abs[index],
                    snapshot.servo_zero_offset[index],
                    snapshot.servo_raw[index],
                    snapshot.servo_speed[index],
                    snapshot.servo_load[index],
                    snapshot.servo_voltage[index],
                    snapshot.servo_temperature[index],
                    "OK" if online else "--",
                ),
            )
        self.last_line_var.set(f"RX: {snapshot.last_line}")
        self._draw_encoder_plot()

    def _draw_encoder_plot(self) -> None:
        canvas = self.encoder_plot
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 56, 18, 18, 34
        x0, y0 = pad_l, height - pad_b
        x1, y1 = width - pad_r, pad_t
        canvas.create_line(x0, y0, x1, y0, fill="#889096")
        canvas.create_line(x0, y0, x0, y1, fill="#889096")
        canvas.create_text(x0 + 8, y1 + 8, text="Encoder relative angles (deg)", anchor="w", fill="#334155")

        visible = self.history[-300:]
        if not visible:
            canvas.create_text(width / 2, height / 2, text="等待 Encoder / Servo 数据...", fill="#667085")
            return

        values: list[float] = []
        for snapshot in visible:
            for index, valid in enumerate(snapshot.encoder_valid):
                if valid:
                    values.append(self._encoder_display_deg(snapshot, index))
        if not values:
            values = [
                self._encoder_display_deg(snapshot, index)
                for snapshot in visible
                for index in range(ENCODER_COUNT)
            ]
        v_min, v_max = min(values), max(values)
        if v_min == v_max:
            v_min -= 1.0
            v_max += 1.0
        t0 = visible[0].timestamp
        t1 = visible[-1].timestamp
        if t0 == t1:
            t1 = t0 + 1.0

        for i in range(5):
            y = y0 - (y0 - y1) * i / 4
            value = v_min + (v_max - v_min) * i / 4
            canvas.create_line(x0, y, x1, y, fill="#edf2f7")
            canvas.create_text(6, y, text=f"{value:.1f}", anchor="w", fill="#667085")

        for joint in range(ENCODER_COUNT):
            points: list[float] = []
            color = ENCODER_SERVO_COLORS[joint % len(ENCODER_SERVO_COLORS)]
            for snapshot in visible:
                if not snapshot.encoder_valid[joint]:
                    continue
                value = self._encoder_display_deg(snapshot, joint)
                x = x0 + (x1 - x0) * (snapshot.timestamp - t0) / (t1 - t0)
                y = y0 - (y0 - y1) * (value - v_min) / (v_max - v_min)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=1)

        legend_x = x0 + 8
        legend_y = height - 14
        for joint in range(min(ENCODER_COUNT, 12)):
            color = ENCODER_SERVO_COLORS[joint % len(ENCODER_SERVO_COLORS)]
            canvas.create_text(legend_x, legend_y, text=f"J{joint:02d}", anchor="w", fill=color)
            legend_x += 48
        canvas.create_text((x0 + x1) / 2, height - 10, text="时间", fill="#667085")

    def _send_text(self, text: str) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        self.command_queue.put(("text", text))

    def _send_target(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接 Encoder / Servo 串口。", parent=self)
            return
        try:
            index = int(self.target_index_var.get())
            target_text = self.target_value_var.get().strip()
            target_value = float(target_text)
        except ValueError:
            messagebox.showwarning("参数错误", "目标索引和目标值需要是数字。", parent=self)
            return

        mode = self.mode_var.get().strip().lower()
        if mode == "degree":
            if not 0 <= index < ENCODER_COUNT:
                messagebox.showwarning("索引越界", f"degree 模式索引范围为 0-{ENCODER_COUNT - 1}。", parent=self)
                return
            device_target = target_value + self.encoder_zero_deg.get(index, 0.0)
            command = f"degree; j{index} {device_target:.3f}"
        else:
            if not 0 <= index < SERVO_COUNT:
                messagebox.showwarning("索引越界", f"direct 模式索引范围为 0-{SERVO_COUNT - 1}。", parent=self)
                return
            command = f"direct; m{index} {int(round(target_value))}"
        self.command_queue.put(("text", command))

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


class ForceSensorApp(tk.Tk):
    def __init__(self, default_port: str, default_servo_port: str, default_servo_baudrate: int) -> None:
        super().__init__()
        self.title("力传感器数据监视器")
        self.geometry("1120x760")
        self.minsize(900, 620)

        self.samples: list[ForceSample] = []
        self.baselines: dict[int, int] = load_force_zero_baselines()
        self.latest_by_channel: dict[int, ForceSample] = {}
        self.outbox: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.command_queue: "queue.Queue[tuple[str, bytes, int]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.poller: Optional[SensorPoller] = None
        self.connected = False
        self.encoder_servo_window: Optional[EncoderServoWindow] = None
        self.default_servo_port = default_servo_port
        self.default_servo_baudrate = default_servo_baudrate

        self.port_var = tk.StringVar(value=default_port)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.slave_var = tk.IntVar(value=1)
        self.channel_var = tk.IntVar(value=DEFAULT_CHANNEL)
        self.interval_var = tk.IntVar(value=100)
        self.value_var = tk.StringVar(value="--")
        self.relative_var = tk.StringVar(value="--")
        self.channel_value_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.channel_relative_vars = {channel: tk.StringVar(value="--") for channel in DISPLAY_CHANNELS}
        self.status_var = tk.StringVar(value="未连接")
        self.count_var = tk.StringVar(value="0")
        self.low_word_var = tk.StringVar(value="--")
        self.high_word_var = tk.StringVar(value="--")
        self.request_var = tk.StringVar(value="--")
        self.response_var = tk.StringVar(value="--")
        self.diagnostic_var = tk.StringVar(value="诊断: --")
        self.zero_status_var = tk.StringVar(value=self._zero_status_text())

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_outbox)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(4, weight=1)

        connection = ttk.LabelFrame(self, text="连接")
        connection.grid(row=0, column=0, padx=10, pady=(10, 6), sticky="ew")
        for col in range(12):
            connection.columnconfigure(col, weight=0)
        connection.columnconfigure(11, weight=1)

        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=12)
        self.baud_combo = ttk.Combobox(
            connection, textvariable=self.baud_var, values=[str(v) for v in BAUD_RATES], width=10
        )
        self.slave_spin = tk.Spinbox(connection, from_=1, to=255, textvariable=self.slave_var, width=6)
        self.channel_spin = tk.Spinbox(connection, from_=1, to=8, textvariable=self.channel_var, width=5)
        self.interval_spin = tk.Spinbox(
            connection, from_=20, to=5000, increment=20, textvariable=self.interval_var, width=8
        )
        self.refresh_button = ttk.Button(connection, text="刷新串口", command=self._refresh_ports)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        self.encoder_servo_button = ttk.Button(
            connection,
            text="Encoder / Servo",
            command=self._open_encoder_servo_window,
        )

        widgets = [
            ("串口", self.port_combo),
            ("波特率", self.baud_combo),
            ("从站地址", self.slave_spin),
            ("通道", self.channel_spin),
            ("轮询间隔(ms)", self.interval_spin),
        ]
        col = 0
        for label, widget in widgets:
            ttk.Label(connection, text=label).grid(row=0, column=col, padx=(8, 4), pady=8)
            widget.grid(row=0, column=col + 1, padx=(0, 10), pady=8)
            col += 2
        self.refresh_button.grid(row=0, column=col, padx=4, pady=8)
        self.connect_button.grid(row=0, column=col + 1, padx=4, pady=8)
        self.encoder_servo_button.grid(row=0, column=col + 2, padx=(12, 8), pady=8)

        readout = ttk.LabelFrame(self, text="实时数据")
        readout.grid(row=1, column=0, padx=10, pady=6, sticky="ew")
        readout.columnconfigure(1, weight=1)
        readout.columnconfigure(3, weight=1)

        ttk.Label(readout, text="主通道当前值").grid(row=0, column=0, padx=10, pady=8, sticky="w")
        tk.Label(readout, textvariable=self.value_var, font=("Segoe UI", 30, "bold")).grid(
            row=0, column=1, padx=6, pady=8, sticky="w"
        )
        ttk.Label(readout, text="主通道相对力(N)").grid(row=0, column=2, padx=10, pady=8, sticky="w")
        tk.Label(readout, textvariable=self.relative_var, font=("Segoe UI", 24, "bold")).grid(
            row=0, column=3, padx=6, pady=8, sticky="w"
        )

        for index, channel in enumerate(DISPLAY_CHANNELS, start=1):
            color = CHANNEL_COLORS[channel]
            ttk.Label(readout, text=f"CH{channel}").grid(
                row=index, column=0, padx=10, pady=3, sticky="w"
            )
            tk.Label(
                readout,
                textvariable=self.channel_value_vars[channel],
                font=("Segoe UI", 16, "bold"),
                fg=color,
            ).grid(row=index, column=1, padx=6, pady=3, sticky="w")
            ttk.Label(readout, text=f"CH{channel}相对力(N)").grid(
                row=index, column=2, padx=10, pady=3, sticky="w"
            )
            tk.Label(
                readout,
                textvariable=self.channel_relative_vars[channel],
                font=("Segoe UI", 16, "bold"),
                fg=color,
            ).grid(row=index, column=3, padx=6, pady=3, sticky="w")

        info_row = 1 + len(DISPLAY_CHANNELS)
        ttk.Label(readout, text="状态").grid(row=info_row, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.status_var).grid(row=info_row, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(readout, text="样本数").grid(row=info_row, column=2, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.count_var).grid(row=info_row, column=3, padx=6, pady=4, sticky="w")

        word_row = info_row + 1
        ttk.Label(readout, text="低字").grid(row=word_row, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.low_word_var).grid(row=word_row, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(readout, text="高字").grid(row=word_row, column=2, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.high_word_var).grid(row=word_row, column=3, padx=6, pady=4, sticky="w")

        ttk.Button(readout, text="记录零位", command=self._set_baseline).grid(
            row=word_row + 1, column=0, padx=10, pady=8, sticky="w"
        )
        ttk.Button(readout, text="清除零位", command=self._clear_baseline).grid(
            row=word_row + 1, column=1, padx=6, pady=8, sticky="w"
        )
        ttk.Button(readout, text="设备去皮", command=lambda: self._queue_tare_command(1)).grid(
            row=word_row + 1, column=2, padx=6, pady=8, sticky="w"
        )
        ttk.Button(readout, text="取消去皮", command=lambda: self._queue_tare_command(2)).grid(
            row=word_row + 1, column=3, padx=6, pady=8, sticky="w"
        )
        ttk.Label(readout, text="零位状态").grid(row=word_row + 2, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(readout, textvariable=self.zero_status_var).grid(
            row=word_row + 2, column=1, columnspan=3, padx=6, pady=4, sticky="w"
        )
        ttk.Button(readout, text="诊断读取", command=self._queue_diagnostic_read).grid(
            row=word_row + 3, column=0, padx=10, pady=4, sticky="w"
        )
        ttk.Label(readout, textvariable=self.diagnostic_var).grid(
            row=word_row + 3, column=1, columnspan=3, padx=6, pady=4, sticky="w"
        )

        self.plot_canvas = tk.Canvas(self, height=220, bg="white", highlightthickness=1, highlightbackground="#d0d7de")
        self.plot_canvas.grid(row=2, column=0, padx=10, pady=6, sticky="nsew")
        self.plot_canvas.bind("<Configure>", lambda _event: self._draw_plot())

        frames = ttk.LabelFrame(self, text="Modbus 帧")
        frames.grid(row=3, column=0, padx=10, pady=6, sticky="ew")
        frames.columnconfigure(1, weight=1)
        ttk.Label(frames, text="请求").grid(row=0, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(frames, textvariable=self.request_var).grid(row=0, column=1, padx=6, pady=4, sticky="w")
        ttk.Label(frames, text="响应").grid(row=1, column=0, padx=10, pady=4, sticky="w")
        ttk.Label(frames, textvariable=self.response_var).grid(row=1, column=1, padx=6, pady=4, sticky="w")

        log = ttk.LabelFrame(self, text="记录")
        log.grid(row=4, column=0, padx=10, pady=(6, 10), sticky="nsew")
        log.columnconfigure(0, weight=1)
        log.rowconfigure(1, weight=1)
        buttons = ttk.Frame(log)
        buttons.grid(row=0, column=0, sticky="ew")
        ttk.Button(buttons, text="清空", command=self._clear_samples).pack(side="right", padx=4, pady=6)
        ttk.Button(buttons, text="导出 CSV", command=self._export_csv).pack(side="right", padx=4, pady=6)

        self.table = ttk.Treeview(
            log, columns=("time", "channel", "value", "relative", "response"), show="headings"
        )
        self.table.heading("time", text="时间")
        self.table.heading("channel", text="通道")
        self.table.heading("value", text="当前值")
        self.table.heading("relative", text="相对力(N)")
        self.table.heading("response", text="响应帧")
        self.table.column("time", width=90, anchor="center")
        self.table.column("channel", width=70, anchor="center")
        self.table.column("value", width=120, anchor="e")
        self.table.column("relative", width=120, anchor="e")
        self.table.column("response", width=470, anchor="w")
        yscroll = ttk.Scrollbar(log, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=yscroll.set)
        self.table.grid(row=1, column=0, sticky="nsew")
        yscroll.grid(row=1, column=1, sticky="ns")

    def _refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _open_encoder_servo_window(self) -> None:
        if self.encoder_servo_window is not None and self.encoder_servo_window.winfo_exists():
            self.encoder_servo_window.lift()
            self.encoder_servo_window.focus_set()
            return
        self.encoder_servo_window = EncoderServoWindow(
            self,
            self.default_servo_port,
            self.default_servo_baudrate,
        )

    def _toggle_connection(self) -> None:
        if self.poller is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            slave_id = int(self.slave_var.get())
            channel = int(self.channel_var.get())
            interval_ms = int(self.interval_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "请检查波特率、从站地址和轮询间隔。")
            return
        if not port:
            messagebox.showwarning("缺少串口", "请输入串口号，例如 COM7。")
            return

        self.stop_event.clear()
        self.poller = SensorPoller(
            self.outbox,
            self.command_queue,
            self.stop_event,
            port,
            baud,
            slave_id,
            channel,
            interval_ms,
        )
        self.poller.start()
        self._set_controls_connected(True)

    def _disconnect(self) -> None:
        self.stop_event.set()
        poller = self.poller
        self.poller = None
        if poller is not None:
            poller.join(timeout=1.5)
        self._set_controls_connected(False)

    def _set_controls_connected(self, connected: bool) -> None:
        self.connected = connected
        self.connect_button.configure(text="断开" if connected else "连接")
        state = "disabled" if connected else "normal"
        for widget in (
            self.port_combo,
            self.baud_combo,
            self.slave_spin,
            self.channel_spin,
            self.interval_spin,
            self.refresh_button,
        ):
            widget.configure(state=state)

    def _process_outbox(self) -> None:
        try:
            while True:
                kind, payload = self.outbox.get_nowait()
                if kind == "sample":
                    self._handle_sample(payload)
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self.status_var.set(str(payload))
                elif kind == "frame":
                    label, request_hex, response_hex = payload
                    self.request_var.set(f"{label}: {request_hex}")
                    self.response_var.set(response_hex)
                elif kind == "diagnostic":
                    info = payload
                    self.diagnostic_var.set(
                        "诊断: {label}, 重量={weight}, 低字=0x{low:04X}, 高字=0x{high:04X}".format(
                            label=info["label"],
                            weight=info["weight"],
                            low=info["low"],
                            high=info["high"],
                        )
                    )
                elif kind == "connected":
                    self._set_controls_connected(bool(payload))
                    if not payload:
                        self.poller = None
        except queue.Empty:
            pass
        self.after(50, self._process_outbox)

    def _queue_tare_command(self, value: int) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接传感器。")
            return
        slave_id = int(self.slave_var.get())
        request = build_write_single_register(slave_id, 0x0015, value)
        label = "设备去皮" if value == 1 else "取消去皮"
        self.command_queue.put((label, request, 8))
        self.request_var.set(f"{label}: {format_hex(request)}")
        self.response_var.set("--")

    def _queue_diagnostic_read(self) -> None:
        if self.poller is None:
            messagebox.showinfo("未连接", "请先连接传感器。")
            return
        slave_id = int(self.slave_var.get())
        channel = int(self.channel_var.get())
        start_register = weight_register_for_channel(channel)
        request = build_read_holding_registers(slave_id, start_register, 2)
        label = f"诊断读取CH{channel}"
        self.command_queue.put((label, request, 9))
        self.request_var.set(f"{label}: {format_hex(request)}")
        self.response_var.set("--")

    def _handle_sample(self, sample: ForceSample) -> None:
        self.samples.append(sample)
        self.latest_by_channel[sample.channel] = sample
        if len(self.samples) > MAX_SAMPLES:
            del self.samples[:1000]

        relative = self._relative_value(sample.value, sample.channel)
        relative_force_n = format_force_n(relative)
        if sample.channel == int(self.channel_var.get()):
            self.value_var.set(str(sample.value))
            self.relative_var.set(relative_force_n)
            self.low_word_var.set(f"0x{sample.low_word:04X} ({sample.low_word})")
            self.high_word_var.set(f"0x{sample.high_word:04X} ({sample.high_word})")
        if sample.channel in self.channel_value_vars:
            self.channel_value_vars[sample.channel].set(str(sample.value))
            self.channel_relative_vars[sample.channel].set(relative_force_n)
        self.count_var.set(str(len(self.samples)))
        self.request_var.set(sample.request_hex)
        self.response_var.set(sample.response_hex)
        self._append_table_row(sample, relative)
        self._draw_plot()

    def _relative_value(self, value: int, channel: int) -> int:
        baseline = self.baselines.get(channel)
        return value if baseline is None else value - baseline

    def _zero_status_text(self) -> str:
        if all(channel in self.baselines for channel in DISPLAY_CHANNELS):
            values = ", ".join(
                f"CH{channel}={self.baselines[channel]}" for channel in DISPLAY_CHANNELS
            )
            return f"已保存零位: {values}"
        if self.baselines:
            values = ", ".join(
                f"CH{channel}={value}" for channel, value in sorted(self.baselines.items())
            )
            return f"部分零位: {values}"
        return "未保存零位"

    def _refresh_relative_displays(self) -> None:
        main = self.latest_by_channel.get(int(self.channel_var.get()))
        self.relative_var.set(
            format_force_n(self._relative_value(main.value, main.channel)) if main else "--"
        )
        for channel in DISPLAY_CHANNELS:
            sample = self.latest_by_channel.get(channel)
            self.channel_relative_vars[channel].set(
                format_force_n(self._relative_value(sample.value, channel)) if sample else "--"
            )

    def _append_table_row(self, sample: ForceSample, relative: int) -> None:
        rows = self.table.get_children()
        if len(rows) >= MAX_TABLE_ROWS:
            self.table.delete(rows[0])
        local_time = time.strftime("%H:%M:%S", time.localtime(sample.timestamp))
        item = self.table.insert(
            "",
            "end",
            values=(
                local_time,
                f"CH{sample.channel}",
                sample.value,
                format_force_n(relative),
                sample.response_hex,
            ),
        )
        self.table.see(item)

    def _draw_plot(self) -> None:
        canvas = self.plot_canvas
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 54, 16, 16, 32
        x0, y0 = pad_l, height - pad_b
        x1, y1 = width - pad_r, pad_t
        canvas.create_line(x0, y0, x1, y0, fill="#889096")
        canvas.create_line(x0, y0, x0, y1, fill="#889096")

        visible = self.samples[-800:]
        if not visible:
            canvas.create_text(width / 2, height / 2, text="等待数据...", fill="#667085")
            return
        values = [
            self._relative_value(sample.value, sample.channel) / RELATIVE_UNITS_PER_NEWTON
            for sample in visible
        ]
        v_min, v_max = min(values), max(values)
        if v_min == v_max:
            v_min -= 1
            v_max += 1
        t0 = visible[0].timestamp
        t1 = visible[-1].timestamp
        if t0 == t1:
            t1 = t0 + 1

        for i in range(5):
            y = y0 - (y0 - y1) * i / 4
            value = v_min + (v_max - v_min) * i / 4
            canvas.create_line(x0, y, x1, y, fill="#edf2f7")
            canvas.create_text(6, y, text=f"{value:.1f} N", anchor="w", fill="#667085")

        plot_channels = [
            (channel, CHANNEL_COLORS[channel], f"CH{channel}") for channel in DISPLAY_CHANNELS
        ]
        seen_channels = set()
        legend_x = x0 + 8
        for channel, color, label in plot_channels:
            if channel in seen_channels:
                continue
            seen_channels.add(channel)
            channel_points: list[float] = []
            for sample in visible:
                if sample.channel != channel:
                    continue
                value = self._relative_value(sample.value, sample.channel) / RELATIVE_UNITS_PER_NEWTON
                x = x0 + (x1 - x0) * (sample.timestamp - t0) / (t1 - t0)
                y = y0 - (y0 - y1) * (value - v_min) / (v_max - v_min)
                channel_points.extend((x, y))
            if len(channel_points) >= 4:
                canvas.create_line(*channel_points, fill=color, width=2)
            canvas.create_line(legend_x, y1 + 8, legend_x + 22, y1 + 8, fill=color, width=3)
            canvas.create_text(legend_x + 28, y1 + 8, text=label, anchor="w", fill=color)
            legend_x += 76
        canvas.create_text((x0 + x1) / 2, height - 10, text="时间", fill="#667085")

    def _set_baseline(self) -> None:
        missing_channels = [
            channel for channel in DISPLAY_CHANNELS if channel not in self.latest_by_channel
        ]
        if missing_channels:
            messagebox.showinfo(
                "等待数据",
                "请先连接并等待 CH1-CH5 都有读数，再记录零位。\n"
                f"当前缺少: {', '.join(f'CH{channel}' for channel in missing_channels)}",
            )
            return

        self.baselines = {
            channel: self.latest_by_channel[channel].value for channel in DISPLAY_CHANNELS
        }
        try:
            save_force_zero_baselines(self.baselines)
        except Exception as exc:
            messagebox.showerror("保存失败", f"零位保存失败: {exc}")
            return
        self.zero_status_var.set(self._zero_status_text())
        self.status_var.set(f"零位已保存: {FORCE_ZERO_CONFIG_PATH}")
        self._refresh_relative_displays()
        self._draw_plot()

    def _clear_baseline(self) -> None:
        self.baselines.clear()
        try:
            delete_force_zero_baselines()
        except Exception as exc:
            messagebox.showerror("清除失败", f"零位配置删除失败: {exc}")
            return
        self.zero_status_var.set(self._zero_status_text())
        self.status_var.set("零位已清除")
        self._refresh_relative_displays()
        self._draw_plot()

    def _clear_samples(self) -> None:
        self.samples.clear()
        self.latest_by_channel.clear()
        self.table.delete(*self.table.get_children())
        self.count_var.set("0")
        self.value_var.set("--")
        self.relative_var.set("--")
        for channel in DISPLAY_CHANNELS:
            self.channel_value_vars[channel].set("--")
            self.channel_relative_vars[channel].set("--")
        self.low_word_var.set("--")
        self.high_word_var.set("--")
        self.request_var.set("--")
        self.response_var.set("--")
        self._draw_plot()

    def _export_csv(self) -> None:
        if not self.samples:
            messagebox.showinfo("没有数据", "当前没有可导出的样本。")
            return
        default_dir = Path.cwd() / "run_data"
        default_dir.mkdir(exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="导出 CSV",
            initialdir=str(default_dir),
            initialfile=f"force_sensor_{time.strftime('%Y%m%d_%H%M%S')}.csv",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(
                [
                    "timestamp",
                    "local_time",
                    "channel",
                    "value",
                    "relative_value",
                    "relative_force_n",
                    "low_word",
                    "high_word",
                    "request_hex",
                    "response_hex",
                ]
            )
            for sample in self.samples:
                writer.writerow(
                    [
                        f"{sample.timestamp:.6f}",
                        time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(sample.timestamp)),
                        sample.channel,
                        sample.value,
                        relative_value := self._relative_value(sample.value, sample.channel),
                        f"{relative_value / RELATIVE_UNITS_PER_NEWTON:.1f}",
                        sample.low_word,
                        sample.high_word,
                        sample.request_hex,
                        sample.response_hex,
                    ]
                )
        self.status_var.set(f"已导出 {path}")

    def _on_close(self) -> None:
        if self.encoder_servo_window is not None and self.encoder_servo_window.winfo_exists():
            self.encoder_servo_window._disconnect()
            self.encoder_servo_window.destroy()
        self._disconnect()
        self.destroy()


def run_protocol_selftest() -> None:
    request = build_read_holding_registers(1, 0, 2)
    expected_request = bytes.fromhex("01 03 00 00 00 02 C4 0B")
    if request != expected_request:
        raise AssertionError(f"request mismatch: {format_hex(request)}")
    diagnostic_request = build_read_holding_registers(1, 0, 6)
    expected_diagnostic_request = bytes.fromhex("01 03 00 00 00 06 C5 C8")
    if diagnostic_request != expected_diagnostic_request:
        raise AssertionError(f"diagnostic request mismatch: {format_hex(diagnostic_request)}")
    channel5_request = build_read_holding_registers(1, weight_register_for_channel(5), 2)
    expected_channel5_request = bytes.fromhex("01 03 00 08 00 02 45 C9")
    if channel5_request != expected_channel5_request:
        raise AssertionError(f"CH5 request mismatch: {format_hex(channel5_request)}")
    tare_request = build_write_single_register(1, 0x0015, 1)
    expected_tare_request = bytes.fromhex("01 06 00 15 00 01 59 CE")
    if tare_request != expected_tare_request:
        raise AssertionError(f"tare request mismatch: {format_hex(tare_request)}")
    decode_write_single_register_response(tare_request, tare_request, 1)
    response = bytes.fromhex("01 03 04 04 D2 00 00 5B 3A")
    value, low_word, high_word = decode_live_weight_response(response, 1)
    if (value, low_word, high_word) != (1234, 0x04D2, 0x0000):
        raise AssertionError((value, low_word, high_word))
    stream_mode = build_upper_stream_mode_cmd()
    if stream_mode != bytes.fromhex("FE 03 D1 01 FF"):
        raise AssertionError(f"stream mode mismatch: {format_hex(stream_mode)}")
    upper_buffer = bytearray(bytes.fromhex("FE 04 01 00 01 FF") + b"<<<SYS_READY>>>\r\n")
    frames, lines = parse_upper_frame_bytes(upper_buffer)
    if frames != [(PACKET_TYPE_SENSOR, bytes.fromhex("00 01"))]:
        raise AssertionError(frames)
    if lines != ["<<<SYS_READY>>>"]:
        raise AssertionError(lines)
    print("protocol selftest ok")


def run_cli_diagnostic(port_name: str, baudrate: int, slave_id: int, channel: int) -> None:
    start_register = weight_register_for_channel(channel)
    request = build_read_holding_registers(slave_id, start_register, 2)
    print(f"Opening {port_name} @ {baudrate} 8N1, slave {slave_id}, CH{channel}")
    print(f"Register: 0x{start_register:04X}")
    print(f"Request:  {format_hex(request)}")
    port = open_serial_port(port_name, baudrate, timeout_s=0.5)
    try:
        if hasattr(port, "reset_input_buffer"):
            port.reset_input_buffer()
        port.write(request)
        if hasattr(port, "flush"):
            port.flush()
        response = read_exact(port, 9, timeout_s=0.8)
        print(f"Response: {format_hex(response)}")
        registers = decode_holding_registers_response(response, slave_id)
        if len(registers) < 2:
            raise ValueError(f"expected 2 registers, got {len(registers)}")
        weight = words_to_i32_low_first(registers[0], registers[1])
        print(f"CH{channel}重量: {weight}")
        print(f"低字:       0x{registers[0]:04X} ({registers[0]})")
        print(f"高字:       0x{registers[1]:04X} ({registers[1]})")
    finally:
        port.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Force sensor Modbus-RTU monitor")
    parser.add_argument("--port", default="COM7", help="serial port, default: COM7")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"serial baudrate, default: {DEFAULT_BAUDRATE}",
    )
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus slave id, default: 1")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL, help=f"weight channel, default: {DEFAULT_CHANNEL}")
    parser.add_argument("--servo-port", default="", help="Encoder / Servo serial port")
    parser.add_argument(
        "--servo-baudrate",
        type=int,
        default=ENCODER_SERVO_DEFAULT_BAUDRATE,
        help=f"Encoder / Servo baudrate, default: {ENCODER_SERVO_DEFAULT_BAUDRATE}",
    )
    parser.add_argument("--diagnose", action="store_true", help="read the selected channel once")
    parser.add_argument("--selftest", action="store_true", help="check CRC and frame decoding")
    args = parser.parse_args()

    if args.selftest:
        run_protocol_selftest()
        return 0
    if args.diagnose:
        run_cli_diagnostic(args.port, args.baudrate, args.slave_id, args.channel)
        return 0

    app = ForceSensorApp(
        default_port=args.port,
        default_servo_port=args.servo_port,
        default_servo_baudrate=args.servo_baudrate,
    )
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
