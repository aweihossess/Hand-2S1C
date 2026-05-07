"""
主窗口 GUI - 机械手上位机
使用 PyQt6 构建
"""

import sys
import os
import time
import json
from typing import Optional, List, Tuple

# 添加父目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grandparent_dir = os.path.dirname(parent_dir)
if grandparent_dir not in sys.path:
    sys.path.insert(0, grandparent_dir)

# 零点配置文件路径
ZERO_CONFIG_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "zero_config.json")

# MCP 滑块机械限位（与机构 MCP 屈伸/展收范围一致，度）
# FE：伸展约 22°–26°（取 -26）、屈曲约 84°–87°（取 +87）；AA：总行程约 25°–30°（±15）
MCP_FE_SLIDER_MIN = -26
MCP_FE_SLIDER_MAX = 87
MCP_AA_SLIDER_MIN = -15
MCP_AA_SLIDER_MAX = 15
# 单指/台架测试时常把两路磁编接到最小编号通道：E0→FE(θ2)、E1→AA(θ1)；语义仍按食指 MCP，与整手布局无关
MCP_ENCODER_FE_DEFAULT = 0
MCP_ENCODER_AA_DEFAULT = 1

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QComboBox, QSpinBox,
    QDoubleSpinBox, QGroupBox, QGridLayout, QTabWidget,
    QTextEdit, QProgressBar, QCheckBox, QMessageBox,
    QSplitter, QFrame, QStatusBar, QScrollArea, QFrame
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont

try:
    from desk_2S1C.core import (
        HandController, HandPose, PRESET_POSES,
        HandModel, MotorState,
        ENCODER_COUNT, MOTOR_COUNT,
        RAW_TO_DEG, DEG_TO_RAW,
        MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX,
        list_ports, ControlMode,
    )
    from desk_2S1C.core.mcp_kinematics import delta_rope_lengths_mm
except ImportError:
    # 如果包导入失败，使用相对导入
    sys.path.insert(0, parent_dir)
    from core import (
        HandController, HandPose, PRESET_POSES,
        HandModel, MotorState,
        ENCODER_COUNT, MOTOR_COUNT,
        RAW_TO_DEG, DEG_TO_RAW,
        MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX,
        list_ports, ControlMode,
    )
    from core.mcp_kinematics import delta_rope_lengths_mm


class DataUpdateThread(QThread):
    """后台数据更新线程"""
    data_updated = pyqtSignal(object)  # HandModel
    
    def __init__(self, controller: HandController):
        super().__init__()
        self.controller = controller
        self.running = False
        self.interval_ms = 50  # 20Hz
    
    def run(self):
        self.running = True
        counter = 0
        while self.running:
            if self.controller.is_connected():
                status = self.controller.get_status()
                
                # 调试输出（每50次打印一次）
                counter += 1
                if counter % 50 == 0:
                    encoder_raw = status.get('encoder_raw', [])
                    print(f"[DataUpdateThread] encoder_raw长度={len(encoder_raw)}, 连接状态={status.get('connected')}")
                
                self.data_updated.emit(status)
            time.sleep(self.interval_ms / 1000.0)
    
    def stop(self):
        self.running = False


class EncoderIndicator(QFrame):
    """磁编码器状态指示器 - 显示S3通过CAN发来的编码器数据"""
    
    def __init__(self, encoder_id: int, name: str, parent=None):
        super().__init__(parent)
        self.encoder_id = encoder_id
        self.name = name
        self.raw_value = 0
        self.angle_deg = 0.0
        self.zero_raw = 0
        self.target_angle = 0.0
        self.error = False
        self.setup_ui()
    
    def setup_ui(self):
        self.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.setMinimumSize(90, 140)
        self.setMaximumWidth(110)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(1)
        
        # ID标签
        self.id_label = QLabel(f"E{self.encoder_id:02d}")
        self.id_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.id_label.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        layout.addWidget(self.id_label)
        
        # 名称标签
        self.name_label = QLabel(self.name)
        self.name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.name_label.setFont(QFont("Microsoft YaHei", 7))
        self.name_label.setWordWrap(True)
        layout.addWidget(self.name_label)
        
        # 状态指示
        self.status_label = QLabel("●")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFont(QFont("Arial", 12))
        layout.addWidget(self.status_label)
        
        # 角度显示 (主要数据)
        self.angle_label = QLabel("0.0°")
        self.angle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.angle_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        layout.addWidget(self.angle_label)
        
        # 原始值显示
        self.raw_label = QLabel("Raw: 0")
        self.raw_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.raw_label.setFont(QFont("Arial", 7))
        layout.addWidget(self.raw_label)
        
        # 零点显示
        self.zero_label = QLabel("Zero: 0")
        self.zero_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.zero_label.setFont(QFont("Arial", 7))
        layout.addWidget(self.zero_label)
        
        # 目标角度
        self.target_label = QLabel("Tgt: 0°")
        self.target_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.target_label.setFont(QFont("Arial", 7))
        layout.addWidget(self.target_label)
        
        self.update_display()
    
    def update_status(self, raw: int, angle_deg: float, zero_raw: int = 0, 
                     target_angle: float = 0.0, error: bool = False):
        self.raw_value = raw
        self.angle_deg = angle_deg
        self.zero_raw = zero_raw
        self.target_angle = target_angle
        self.error = error
        self.update_display()
    
    def update_display(self):
        # 状态颜色：错误=红色，正常=绿色
        if self.error:
            self.status_label.setStyleSheet("color: #f44336;")  # 红色
        else:
            self.status_label.setStyleSheet("color: #4CAF50;")  # 绿色
        
        # 更新角度显示（主要数据，大字）
        self.angle_label.setText(f"{self.angle_deg:.1f}°")
        
        # 更新其他数值
        self.raw_label.setText(f"Raw: {self.raw_value}")
        self.zero_label.setText(f"Zero: {self.zero_raw}")
        self.target_label.setText(f"Tgt: {self.target_angle:.1f}°")


class ServoIndicator(QFrame):
    """舵机状态指示器 - 显示位置、速度、负载(电流)、电压、温度"""
    
    def __init__(self, servo_id: int, parent=None):
        super().__init__(parent)
        self.servo_id = servo_id
        self.online = False
        self.position = 0
        self.speed = 0
        self.load = 0
        self.voltage = 0.0
        self.temperature = 0
        self.setup_ui()
    
    def setup_ui(self):
        self.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.setMinimumSize(90, 140)
        self.setMaximumWidth(110)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(1)
        
        # ID标签
        self.id_label = QLabel(f"M{self.servo_id:02d}")
        self.id_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.id_label.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        layout.addWidget(self.id_label)
        
        # 在线状态指示
        self.status_label = QLabel("●")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFont(QFont("Arial", 12))
        layout.addWidget(self.status_label)
        
        # 位置显示
        self.pos_label = QLabel("Pos: 0")
        self.pos_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pos_label.setFont(QFont("Arial", 8))
        layout.addWidget(self.pos_label)
        
        # 速度显示
        self.speed_label = QLabel("Spd: 0")
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setFont(QFont("Arial", 8))
        layout.addWidget(self.speed_label)
        
        # 负载(电流)显示
        self.load_label = QLabel("Load: 0")
        self.load_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.load_label.setFont(QFont("Arial", 8))
        layout.addWidget(self.load_label)
        
        # 电压显示
        self.voltage_label = QLabel("V: 0.0V")
        self.voltage_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.voltage_label.setFont(QFont("Arial", 8))
        layout.addWidget(self.voltage_label)
        
        # 温度显示
        self.temp_label = QLabel("T: 0°C")
        self.temp_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.temp_label.setFont(QFont("Arial", 8))
        layout.addWidget(self.temp_label)
        
        self.update_display()
    
    def update_status(self, online: bool, position: int, speed: int = 0, 
                     load: int = 0, voltage: float = 0.0, temperature: int = 0):
        self.online = online
        self.position = position
        self.speed = speed
        self.load = load
        self.voltage = voltage
        self.temperature = temperature
        self.update_display()
    
    def update_display(self):
        # 在线状态颜色
        if self.online:
            self.status_label.setStyleSheet("color: #4CAF50;")  # 绿色
        else:
            self.status_label.setStyleSheet("color: #f44336;")  # 红色
        
        # 更新各数值显示
        self.pos_label.setText(f"Pos: {self.position}")
        self.speed_label.setText(f"Spd: {self.speed}")
        self.load_label.setText(f"Load: {self.load}")
        self.voltage_label.setText(f"V: {self.voltage:.1f}V")
        self.temp_label.setText(f"T: {self.temperature}°C")
        
        # 根据温度设置颜色警告
        if self.temperature > 60:
            self.temp_label.setStyleSheet("color: #f44336; font-weight: bold;")  # 红色警告
        elif self.temperature > 45:
            self.temp_label.setStyleSheet("color: #FF9800;")  # 橙色注意
        else:
            self.temp_label.setStyleSheet("color: black;")


class JointSlider(QWidget):
    """关节角度滑块控件"""
    value_changed = pyqtSignal(int, float)
    
    def __init__(self, joint_id: int, name: str, 
                 min_val: float = -90, max_val: float = 90, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.name = name
        self.min_val = min_val
        self.max_val = max_val
        self.setup_ui()
    
    def setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        
        # 名称标签
        self.name_label = QLabel(f"{self.name}")
        self.name_label.setMinimumWidth(100)
        self.name_label.setFont(QFont("Microsoft YaHei", 9))
        layout.addWidget(self.name_label)
        
        # 滑块
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(self.min_val * 10))
        self.slider.setMaximum(int(self.max_val * 10))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_changed)
        layout.addWidget(self.slider, 1)
        
        # 数值输入
        self.spin = QDoubleSpinBox()
        self.spin.setRange(self.min_val, self.max_val)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(1)
        self.spin.setValue(0)
        self.spin.setSuffix("°")
        self.spin.setMinimumWidth(70)
        self.spin.valueChanged.connect(self.on_spin_changed)
        layout.addWidget(self.spin)
    
    def on_slider_changed(self, value):
        angle = value / 10.0
        self.spin.blockSignals(True)
        self.spin.setValue(angle)
        self.spin.blockSignals(False)
        self.value_changed.emit(self.joint_id, angle)
    
    def on_spin_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(value * 10))
        self.slider.blockSignals(False)
        self.value_changed.emit(self.joint_id, value)
    
    def set_value(self, value: float):
        self.slider.blockSignals(True)
        self.spin.blockSignals(True)
        self.slider.setValue(int(value * 10))
        self.spin.setValue(value)
        self.slider.blockSignals(False)
        self.spin.blockSignals(False)
    
    def get_value(self) -> float:
        return self.spin.value()


class MotorSlider(QWidget):
    """电机位置滑块控件 - 带微调按钮"""
    value_changed = pyqtSignal(int, int)
    fine_adjust = pyqtSignal(int, int)  # 微调信号: motor_id, delta
    
    def __init__(self, motor_id: int, min_val: int = 0, max_val: int = 4095, parent=None):
        super().__init__(parent)
        self.motor_id = motor_id
        self.min_val = min_val
        self.max_val = max_val
        self.setup_ui()
    
    def setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 1, 2, 1)
        layout.setSpacing(4)
        
        # ID标签
        self.id_label = QLabel(f"M{self.motor_id:02d}")
        self.id_label.setMinimumWidth(40)
        self.id_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.id_label)
        
        # 逆时针微调按钮（放松）
        self.ccw_btn = QPushButton("←")
        self.ccw_btn.setToolTip("逆时针/放松 (-50)")
        self.ccw_btn.setFixedSize(24, 24)
        self.ccw_btn.setStyleSheet("QPushButton { font-size: 10px; padding: 0px; }")
        self.ccw_btn.clicked.connect(self.on_ccw_clicked)
        layout.addWidget(self.ccw_btn)
        
        # 滑块
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(self.min_val)
        self.slider.setMaximum(self.max_val)
        self.slider.setValue(2048)
        self.slider.valueChanged.connect(self.on_slider_changed)
        layout.addWidget(self.slider, 1)
        
        # 顺时针微调按钮（拉紧）
        self.cw_btn = QPushButton("→")
        self.cw_btn.setToolTip("顺时针/拉紧 (+50)")
        self.cw_btn.setFixedSize(24, 24)
        self.cw_btn.setStyleSheet("QPushButton { font-size: 10px; padding: 0px; }")
        self.cw_btn.clicked.connect(self.on_cw_clicked)
        layout.addWidget(self.cw_btn)
        
        # 数值输入
        self.spin = QSpinBox()
        self.spin.setRange(self.min_val, self.max_val)
        self.spin.setValue(2048)
        self.spin.setMinimumWidth(60)
        self.spin.setMaximumWidth(70)
        self.spin.valueChanged.connect(self.on_spin_changed)
        layout.addWidget(self.spin)
    
    def on_slider_changed(self, value):
        self.spin.blockSignals(True)
        self.spin.setValue(value)
        self.spin.blockSignals(False)
        self.value_changed.emit(self.motor_id, value)
    
    def on_spin_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(value)
        self.slider.blockSignals(False)
        self.value_changed.emit(self.motor_id, value)
    
    def on_ccw_clicked(self):
        """逆时针微调（放松）"""
        current = self.spin.value()
        delta = -50  # 逆时针50步
        new_val = max(self.min_val, current + delta)
        self.set_value(new_val)
        self.fine_adjust.emit(self.motor_id, delta)
    
    def on_cw_clicked(self):
        """顺时针微调（拉紧）"""
        current = self.spin.value()
        delta = 50  # 顺时针50步
        new_val = min(self.max_val, current + delta)
        self.set_value(new_val)
        self.fine_adjust.emit(self.motor_id, delta)
    
    def set_value(self, value: int):
        self.slider.blockSignals(True)
        self.spin.blockSignals(True)
        self.slider.setValue(value)
        self.spin.setValue(value)
        self.slider.blockSignals(False)
        self.spin.blockSignals(False)
    
    def get_value(self) -> int:
        return self.spin.value()
    
    def set_enabled(self, enabled: bool):
        """设置控件启用/禁用状态"""
        self.slider.setEnabled(enabled)
        self.spin.setEnabled(enabled)
        self.ccw_btn.setEnabled(enabled)
        self.cw_btn.setEnabled(enabled)


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械手上位机 v2.0 - 2S1C")
        self.setMinimumSize(1400, 900)
        
        # 创建控制器
        self.controller = HandController()
        self.controller.register_update_callback(self.on_hand_model_updated)
        
        # 数据更新线程
        self.update_thread: Optional[DataUpdateThread] = None
        
        self.setup_ui()
        self.setup_status_bar()
        
        # 自动加载零点配置（如果存在）
        self.auto_load_zero_config()

        # 自动加载MCP零点配置
        self._load_mcp_zero_config()

        # UI更新定时器
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(50)  # 20Hz

        # Fe/AA 滑块实时控制（节流，避免串口过载）
        self.mcp_live_timer = QTimer(self)
        self.mcp_live_timer.setSingleShot(True)
        self.mcp_live_timer.timeout.connect(self._send_mcp_fe_aa_live)
    
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setSpacing(10)
        
        # 创建分割器
        splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(splitter)
        
        # ===== 左侧面板 =====
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)
        
        # 连接控制组
        self.setup_connection_group(left_layout)
        
        # 系统控制组
        self.setup_control_group(left_layout)
        
        # 磁编码器标定组
        self.setup_calibration_group(left_layout)
        
        # 预设手势组
        self.setup_preset_group(left_layout)
        
        # 故障状态
        self.setup_fault_group(left_layout)
        
        # 日志输出
        left_layout.addWidget(QLabel("系统日志:"))
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumHeight(150)
        left_layout.addWidget(self.log_output)
        
        left_layout.addStretch()
        splitter.addWidget(left_panel)
        
        # ===== 中间面板 - 电机和编码器状态 =====
        self.setup_servo_encoder_panel(splitter)
        
        # ===== 右侧面板 - 电机直控 =====
        self.setup_motor_control_panel(splitter)
        
        # 设置分割器比例
        splitter.setSizes([350, 500, 550])
    
    def setup_connection_group(self, parent_layout):
        """连接控制组"""
        group = QGroupBox("串口连接")
        layout = QGridLayout(group)
        
        # 串口选择
        layout.addWidget(QLabel("串口:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        layout.addWidget(self.port_combo, 0, 1)
        
        # 刷新按钮
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self.refresh_btn, 0, 2)
        
        # 波特率
        layout.addWidget(QLabel("波特率:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["921600", "460800", "115200"])
        self.baud_combo.setCurrentText("921600")
        layout.addWidget(self.baud_combo, 1, 1)
        
        # 连接按钮
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; padding: 8px; }"
        )
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_btn, 1, 2)
        
        # 状态指示
        self.status_indicator = QLabel("● 未连接")
        self.status_indicator.setStyleSheet("color: #f44336; font-weight: bold; font-size: 12px;")
        layout.addWidget(self.status_indicator, 2, 0, 1, 2)
        
        # 初始刷新串口列表
        self.refresh_ports()
        
        parent_layout.addWidget(group)
    
    def setup_control_group(self, parent_layout):
        """系统控制组"""
        group = QGroupBox("系统控制")
        layout = QGridLayout(group)
        
        # 控制模式
        layout.addWidget(QLabel("控制模式:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("关节角度模式", ControlMode.JOINT_ANGLE)
        self.mode_combo.addItem("电机直控模式", ControlMode.DIRECT_MOTOR)
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        layout.addWidget(self.mode_combo, 0, 1, 1, 2)
        
        # 控制按钮
        self.start_btn = QPushButton("▶ 启动控制 (START)")
        self.start_btn.setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; padding: 10px; }"
        )
        self.start_btn.clicked.connect(self.on_start_clicked)
        self.start_btn.setEnabled(False)
        layout.addWidget(self.start_btn, 1, 0, 1, 3)
        
        self.stop_btn = QPushButton("⏹ 停止控制 (STOP)")
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; padding: 10px; }"
        )
        self.stop_btn.clicked.connect(self.on_stop_clicked)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn, 2, 0)
        
        self.reset_btn = QPushButton("⏏ 复位 (RESET)")
        self.reset_btn.clicked.connect(self.on_reset_clicked)
        self.reset_btn.setEnabled(False)
        layout.addWidget(self.reset_btn, 2, 1, 1, 2)
        
        # PID使能
        self.pid_checkbox = QCheckBox("启用PID控制")
        self.pid_checkbox.stateChanged.connect(self.on_pid_changed)
        layout.addWidget(self.pid_checkbox, 3, 0, 1, 3)
        
        parent_layout.addWidget(group)
    
    def setup_calibration_group(self, parent_layout):
        """磁编码器标定组 - 零位标定功能"""
        group = QGroupBox("磁编码器标定")
        layout = QGridLayout(group)
        
        # 标定状态显示
        self.calib_status_label = QLabel("状态: 未标定")
        self.calib_status_label.setStyleSheet("color: #FF9800; font-weight: bold;")
        layout.addWidget(self.calib_status_label, 0, 0, 1, 2)
        
        # 标零按钮
        self.zero_calib_btn = QPushButton("🔧 标定零点")
        self.zero_calib_btn.setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; padding: 8px; }"
        )
        self.zero_calib_btn.setToolTip("将当前磁编码器位置设为角度零点")
        self.zero_calib_btn.clicked.connect(self.on_zero_calibration_clicked)
        self.zero_calib_btn.setEnabled(False)
        layout.addWidget(self.zero_calib_btn, 1, 0)
        
        # 应用零点按钮
        self.apply_calib_btn = QPushButton("✓ 应用零点")
        self.apply_calib_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; padding: 8px; }"
        )
        self.apply_calib_btn.setToolTip("将零点数据下发到下位机")
        self.apply_calib_btn.clicked.connect(self.on_apply_calibration_clicked)
        self.apply_calib_btn.setEnabled(False)
        layout.addWidget(self.apply_calib_btn, 1, 1)
        
        # 第二行按钮
        # 保存配置按钮
        self.save_calib_btn = QPushButton("💾 保存配置")
        self.save_calib_btn.setToolTip("保存当前零点到文件")
        self.save_calib_btn.clicked.connect(self.on_save_calibration_clicked)
        self.save_calib_btn.setEnabled(False)
        layout.addWidget(self.save_calib_btn, 2, 0)
        
        # 加载配置按钮
        self.load_calib_btn = QPushButton("📂 加载配置")
        self.load_calib_btn.setToolTip("从文件加载零点配置")
        self.load_calib_btn.clicked.connect(self.on_load_calibration_clicked)
        layout.addWidget(self.load_calib_btn, 2, 1)
        
        # 零点数据预览
        self.zero_preview_label = QLabel("零点数据: 无")
        self.zero_preview_label.setStyleSheet("color: gray; font-size: 9px;")
        self.zero_preview_label.setWordWrap(True)
        layout.addWidget(self.zero_preview_label, 3, 0, 1, 2)
        
        parent_layout.addWidget(group)
    
    def setup_preset_group(self, parent_layout):
        """双电机MCP关节标定与控制"""
        group = QGroupBox("双电机MCP关节标定与控制")
        layout = QGridLayout(group)
        # 确认标定零点
        self.mcp_confirm_zero_btn = QPushButton("✓ 确认标定零点")
        self.mcp_confirm_zero_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; padding: 15px; font-weight: bold; font-size: 16px; }"
        )
        self.mcp_confirm_zero_btn.setToolTip("将当前M00/M01电机位置记录为MCP零点")
        self.mcp_confirm_zero_btn.clicked.connect(self.on_mcp_confirm_zero)
        self.mcp_confirm_zero_btn.setEnabled(False)
        layout.addWidget(self.mcp_confirm_zero_btn, 0, 0, 1, 3)

        # 当前零点位置显示
        self.mcp_zero_pos_label = QLabel("当前零点: M00=2048, M01=2048 (默认)")
        self.mcp_zero_pos_label.setStyleSheet("color: #666; font-size: 11px;")
        self.mcp_zero_pos_label.setWordWrap(True)
        layout.addWidget(self.mcp_zero_pos_label, 1, 0, 1, 3)

        # 标定提示
        self.mcp_result_label = QLabel("提示: 调整M00/M01到目标位置后，点击上方按钮记录为零点")
        self.mcp_result_label.setWordWrap(True)
        self.mcp_result_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(self.mcp_result_label, 2, 0, 1, 3)

        # Fe/AA控制（θ1=AA，θ2=FE；正解绳长 ΔL 映射到 M00/M01）
        feaa_label = QLabel("Fe/AA控制")
        feaa_label.setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;")
        layout.addWidget(feaa_label, 3, 0, 1, 3)

        mcp_geom_hint = QLabel(
            f"几何：θ1=AA∈[{MCP_AA_SLIDER_MIN}°, {MCP_AA_SLIDER_MAX}°]，θ2=FE∈[{MCP_FE_SLIDER_MIN}°, {MCP_FE_SLIDER_MAX}°]；"
            "连接后滑块从磁编码器同步（索引见配置）。"
        )
        mcp_geom_hint.setWordWrap(True)
        mcp_geom_hint.setStyleSheet("color: #666; font-size: 9px;")
        layout.addWidget(mcp_geom_hint, 4, 0, 1, 3)

        ctrl_layout = QHBoxLayout()
        
        self.mcp_fe_slider = QSlider(Qt.Orientation.Horizontal)
        self.mcp_fe_slider.setRange(MCP_FE_SLIDER_MIN, MCP_FE_SLIDER_MAX)
        self.mcp_fe_slider.setValue(0)
        self.mcp_fe_slider.setEnabled(False)
        self.mcp_fe_slider.setToolTip("θ2 = FE（屈曲），度")
        self.mcp_fe_slider.valueChanged.connect(self.on_mcp_fe_aa_slider_changed)
        ctrl_layout.addWidget(QLabel("Fe:"))
        ctrl_layout.addWidget(self.mcp_fe_slider)
        
        self.mcp_aa_slider = QSlider(Qt.Orientation.Horizontal)
        self.mcp_aa_slider.setRange(MCP_AA_SLIDER_MIN, MCP_AA_SLIDER_MAX)
        self.mcp_aa_slider.setValue(0)
        self.mcp_aa_slider.setEnabled(False)
        self.mcp_aa_slider.setToolTip("θ1 = AA（外展/内收），度")
        self.mcp_aa_slider.valueChanged.connect(self.on_mcp_fe_aa_slider_changed)
        ctrl_layout.addWidget(QLabel("AA:"))
        ctrl_layout.addWidget(self.mcp_aa_slider)
        
        layout.addLayout(ctrl_layout, 5, 0, 1, 3)
        
        self.mcp_apply_btn = QPushButton("应用 Fe/AA 位置（与滑块实时一致，用于立即确认）")
        self.mcp_apply_btn.setEnabled(False)
        self.mcp_apply_btn.setToolTip("按有效绳长模型计算 M00/M01 目标并发送 CMD_MOTOR_POS_ABS")
        self.mcp_apply_btn.clicked.connect(self.on_mcp_apply_clicked)
        layout.addWidget(self.mcp_apply_btn, 6, 0, 1, 3)
        
        parent_layout.addWidget(group)
        
        # MCP零点位置存储 (M00, M01)；绳长→脉冲比例 counts/mm（见 mcp_zero_config.json）
        self.mcp_zero_positions = [2048, 2048]
        self.mcp_rope_k_counts_per_mm = [80.0, 80.0]
        self.mcp_encoder_fe_index = MCP_ENCODER_FE_DEFAULT
        self.mcp_encoder_aa_index = MCP_ENCODER_AA_DEFAULT
        self.mcp_pending_encoder_sync = False
        # 已从 mcp_zero_config.json 加载到零点时，连接后可直接用 Fe/AA
        self.mcp_zero_ready = False
    
    def setup_fault_group(self, parent_layout):
        """故障状态组"""
        group = QGroupBox("故障状态")
        layout = QVBoxLayout(group)
        
        self.fault_text = QLabel("无故障")
        self.fault_text.setStyleSheet("color: #4CAF50;")
        layout.addWidget(self.fault_text)
        
        parent_layout.addWidget(group)
    
    def setup_servo_encoder_panel(self, splitter):
        """舵机和编码器状态面板 - 电机信息在上，编码器信息在下"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(8)
        
        # ===== 上半部分：舵机状态 (22路) =====
        motor_title = QLabel("▶ 电机状态 (22路) - 来自P4舵机总线")
        motor_title.setFont(QFont("Microsoft YaHei", 11, QFont.Weight.Bold))
        motor_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(motor_title)
        
        # 创建舵机指示器
        motor_scroll = QScrollArea()
        motor_scroll.setWidgetResizable(True)
        motor_scroll.setMaximumHeight(280)
        motor_content = QWidget()
        motor_layout = QVBoxLayout(motor_content)
        
        # 按总线分组
        servo_bus_configs = [
            ("Bus 0 (M0-M5)", 0, 6),
            ("Bus 1 (M6-M11)", 6, 6),
            ("Bus 2 (M12-M17)", 12, 6),
            ("Bus 3 (M18-M21)", 18, 4),
        ]
        
        self.servo_indicators: List[ServoIndicator] = []
        
        for bus_name, start, count in servo_bus_configs:
            bus_group = QGroupBox(bus_name)
            bus_layout = QHBoxLayout(bus_group)
            bus_layout.setSpacing(4)
            
            for i in range(start, min(start + count, MOTOR_COUNT)):
                indicator = ServoIndicator(i)
                self.servo_indicators.append(indicator)
                bus_layout.addWidget(indicator)
            
            motor_layout.addWidget(bus_group)
        
        motor_layout.addStretch()
        motor_scroll.setWidget(motor_content)
        layout.addWidget(motor_scroll)
        
        # 舵机状态栏
        self.servo_status_label = QLabel("电机状态: 等待数据...")
        self.servo_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.servo_status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.servo_status_label)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("background-color: #cccccc;")
        line.setMaximumHeight(2)
        layout.addWidget(line)
        
        # ===== 下半部分：磁编码器状态 (21路) =====
        encoder_title = QLabel("▶ 磁编码器状态 (21路) - 来自S3 CAN总线")
        encoder_title.setFont(QFont("Microsoft YaHei", 11, QFont.Weight.Bold))
        encoder_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(encoder_title)
        
        # 编码器名称映射
        # E0-E3 (J00-J03) 为通用指，E4-E6 为食指，E7-E9 为中指，E10-E12 为无名指，E13-E15 为小指，E16-E20 为手腕
        encoder_names = [
            "通用指根", "通用指中", "通用指尖", "通用指掌",
            "食指根", "食指中", "食指尖",
            "中指根", "中指中", "中指尖",
            "无名指根", "无名指中", "无名指尖",
            "小指根", "小指中", "小指尖",
            "腕1", "腕2", "腕3", "腕4", "腕5"
        ]
        
        # 创建编码器指示器
        encoder_scroll = QScrollArea()
        encoder_scroll.setWidgetResizable(True)
        encoder_content = QWidget()
        encoder_layout = QVBoxLayout(encoder_content)
        
        # 按手指分组显示编码器
        encoder_groups = [
            ("拇指 (E0-E3)", 0, 4),
            ("食指 (E4-E6)", 4, 3),
            ("中指 (E7-E9)", 7, 3),
            ("无名指 (E10-E12)", 10, 3),
            ("小指 (E13-E15)", 13, 3),
            ("手腕 (E16-E20)", 16, 5),
        ]
        
        self.encoder_indicators: List[EncoderIndicator] = []
        
        for group_name, start, count in encoder_groups:
            group = QGroupBox(group_name)
            group_layout = QHBoxLayout(group)
            group_layout.setSpacing(4)
            
            for i in range(start, min(start + count, ENCODER_COUNT)):
                name = encoder_names[i] if i < len(encoder_names) else f"编码器{i}"
                indicator = EncoderIndicator(i, name)
                self.encoder_indicators.append(indicator)
                group_layout.addWidget(indicator)
            
            encoder_layout.addWidget(group)
        
        encoder_layout.addStretch()
        encoder_scroll.setWidget(encoder_content)
        layout.addWidget(encoder_scroll)
        
        # 编码器状态栏
        self.encoder_status_label = QLabel("编码器状态: 等待CAN数据...")
        self.encoder_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.encoder_status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.encoder_status_label)
        
        splitter.addWidget(panel)
    
    def setup_motor_control_panel(self, splitter):
        """电机直控面板 - 支持实时控制模式"""
        panel = QWidget()
        motor_layout = QVBoxLayout(panel)
        motor_layout.setSpacing(5)
        
        # 标题
        title = QLabel("◆ 电机直控面板")
        title.setFont(QFont("Microsoft YaHei", 12, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(title)
        
        # 说明
        info = QLabel(
            "直接控制22路电机位置 (0-4095 单圈 / -30719~30719 多圈绝对)。"
            "发令时会自动切到「电机直控模式」。拖动滑块请勾选「启用实时」，否则点「发送全部电机位置」。"
        )
        info.setWordWrap(True)
        info.setStyleSheet("color: gray;")
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(info)
        
        # 控制模式选择
        mode_layout = QHBoxLayout()
        
        # 实时控制复选框
        self.motor_live_mode_checkbox = QCheckBox("启用实时控制")
        self.motor_live_mode_checkbox.setToolTip("启用后，滑动滑块时电机会立即响应")
        self.motor_live_mode_checkbox.stateChanged.connect(self.on_motor_live_mode_changed)
        mode_layout.addWidget(self.motor_live_mode_checkbox)
        
        # 实时模式状态指示
        self.motor_live_status = QLabel("(点击启用实时控制)")
        self.motor_live_status.setStyleSheet("color: gray; font-size: 10px;")
        mode_layout.addWidget(self.motor_live_status)
        
        mode_layout.addStretch()
        
        # 绝对位置复选框
        self.abs_pos_checkbox = QCheckBox("使用多圈绝对位置模式")
        self.abs_pos_checkbox.stateChanged.connect(self.on_abs_mode_changed)
        mode_layout.addWidget(self.abs_pos_checkbox)
        
        motor_layout.addLayout(mode_layout)
        
        # 电机滑块
        self.motor_sliders: List[MotorSlider] = []
        for i in range(MOTOR_COUNT):
            slider = MotorSlider(i, 0, 4095)
            slider.value_changed.connect(self.on_motor_value_changed)
            slider.fine_adjust.connect(self.on_motor_fine_adjust)
            self.motor_sliders.append(slider)
            motor_layout.addWidget(slider)
        
        motor_btn_layout = QHBoxLayout()
        
        self.center_all_btn = QPushButton("全部置中 (2048)")
        self.center_all_btn.clicked.connect(self.on_center_all_clicked)
        motor_btn_layout.addWidget(self.center_all_btn)
        
        self.send_motor_btn = QPushButton("发送全部电机位置")
        self.send_motor_btn.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; padding: 10px; }"
        )
        self.send_motor_btn.clicked.connect(self.on_send_motor_clicked)
        motor_btn_layout.addWidget(self.send_motor_btn)
        
        motor_layout.addLayout(motor_btn_layout)
        motor_layout.addStretch()
        
        # 实时模式标志
        self.motor_live_mode = False
        # 用于存储各电机当前值（实时模式用）
        self.motor_live_values = [2048] * MOTOR_COUNT
        # 发送定时器（用于批量发送，减少通信频率）
        self.motor_live_timer = QTimer()
        self.motor_live_timer.timeout.connect(self._send_motor_live_batch)
        
        # 初始状态：禁用所有电机滑块（连接后启用）
        for slider in self.motor_sliders:
            slider.set_enabled(False)
        
        splitter.addWidget(panel)
    
    def setup_status_bar(self):
        """状态栏"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪 - 请先连接串口")
        
        # 显示信息
        self.mode_status_label = QLabel("模式: 关节角度")
        self.status_bar.addPermanentWidget(self.mode_status_label)
        
        self.pid_status_label = QLabel("PID: 关闭")
        self.status_bar.addPermanentWidget(self.pid_status_label)
    
    def refresh_ports(self):
        """刷新串口列表"""
        current = self.port_combo.currentText()
        self.port_combo.clear()
        
        ports = list_ports()
        for device, desc in ports:
            self.port_combo.addItem(f"{device} - {desc}", device)
        
        if not ports:
            self.port_combo.addItem("未找到串口", "")
        
        # 恢复选择
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
    
    def log(self, message: str):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_output.append(f"[{timestamp}] {message}")
        # 滚动到底部
        scrollbar = self.log_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    # ===== 事件处理 =====
    
    def on_connect_clicked(self):
        """连接按钮点击"""
        if self.controller.is_connected():
            # 断开连接
            self.controller.shutdown()
            self.connect_btn.setText("连接")
            self.connect_btn.setStyleSheet(
                "QPushButton { background-color: #4CAF50; color: white; padding: 8px; }"
            )
            self.status_indicator.setText("● 未连接")
            self.status_indicator.setStyleSheet("color: #f44336; font-weight: bold; font-size: 12px;")
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.reset_btn.setEnabled(False)
            
            # 禁用标定按钮
            self.zero_calib_btn.setEnabled(False)
            self.apply_calib_btn.setEnabled(False)
            
            # 禁用MCP标定按钮
            self.mcp_confirm_zero_btn.setEnabled(False)
            self.mcp_apply_btn.setEnabled(False)
            self.mcp_fe_slider.setEnabled(False)
            self.mcp_aa_slider.setEnabled(False)
            if hasattr(self, "mcp_live_timer"):
                self.mcp_live_timer.stop()
            self.mcp_pending_encoder_sync = False
            
            # 禁用电机实时控制并关闭实时模式
            self.motor_live_mode_checkbox.setEnabled(False)
            self.motor_live_mode_checkbox.setChecked(False)
            self.motor_live_mode = False
            
            # 禁用所有电机滑块控件
            for slider in self.motor_sliders:
                slider.set_enabled(False)
            
            self.log("已断开连接")
            self.status_bar.showMessage("已断开")
            
            if self.update_thread:
                self.update_thread.stop()
                self.update_thread = None
        else:
            # 连接
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "警告", "请选择有效的串口")
                return
            
            self.log(f"正在连接 {port}...")
            
            if self.controller.initialize(port):
                self.connect_btn.setText("断开")
                self.connect_btn.setStyleSheet(
                    "QPushButton { background-color: #f44336; color: white; padding: 8px; }"
                )
                self.status_indicator.setText("● 已连接")
                self.status_indicator.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(True)
                self.reset_btn.setEnabled(True)
                
                # 启用标定按钮
                self.zero_calib_btn.setEnabled(True)
                
                # 启用 MCP「确认零点」；Fe/AA 是否可用由是否已有持久化零点决定
                self.mcp_confirm_zero_btn.setEnabled(True)
                self._refresh_mcp_fe_aa_controls()
                
                # 启用电机实时控制复选框
                self.motor_live_mode_checkbox.setEnabled(True)
                
                # 启用所有电机滑块控件（包括微调按钮）
                for slider in self.motor_sliders:
                    slider.set_enabled(True)
                
                # 如果已有零点数据（从文件加载的），自动下发到下位机
                if hasattr(self, 'current_zero_raw') and self.current_zero_raw:
                    try:
                        self.controller.set_encoder_zeros(self.current_zero_raw)
                        self.log(f"✓ 已自动下发零点到下位机（共{ENCODER_COUNT}路）")
                        self.calib_status_label.setText("状态: 已应用（自动）")
                        self.calib_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
                    except Exception as e:
                        self.log(f"⚠ 自动下发零点失败: {str(e)}")
                        self.apply_calib_btn.setEnabled(True)
                    
                    self.save_calib_btn.setEnabled(True)
                
                self.log(f"已连接到 {port}")
                self.status_bar.showMessage(f"已连接: {port}")
                
                # 启动数据更新线程
                self.update_thread = DataUpdateThread(self.controller)
                self.update_thread.data_updated.connect(self.on_status_updated)
                self.update_thread.start()
            else:
                QMessageBox.critical(self, "错误", f"无法连接到 {port}")
                self.log(f"连接失败: {port}")
    
    def on_start_clicked(self):
        """启动控制"""
        self.controller.start()
        self.log("控制已启动")
        self.status_bar.showMessage("控制运行中")
    
    def on_stop_clicked(self):
        """停止控制"""
        self.controller.stop()
        self.log("控制已停止")
        self.status_bar.showMessage("控制已停止")
    
    def on_reset_clicked(self):
        """复位"""
        self.controller.reset()
        self.log("系统已复位")
        
        # 重置滑块
        for slider in self.joint_sliders:
            slider.set_value(0)
    
    def on_mode_changed(self, index):
        """控制模式改变"""
        mode = self.mode_combo.currentData()
        self.controller.set_control_mode(mode)
        
        if mode == ControlMode.JOINT_ANGLE:
            self.mode_status_label.setText("模式: 关节角度")
        else:
            self.mode_status_label.setText("模式: 电机直控")
        
        self.log(f"切换到{self.mode_combo.currentText()}")
    
    def on_pid_changed(self, state):
        """PID使能改变"""
        enabled = state == Qt.CheckState.Checked.value
        self.controller.set_pid_control(enabled)
        self.pid_status_label.setText(f"PID: {'开启' if enabled else '关闭'}")
    
    def on_pose_clicked(self, pose: HandPose):
        """预设手势按钮点击"""
        pose_name = self.sender().text()
        self.log(f"执行手势: {pose_name}")
        
        if self.controller.apply_pose(pose):
            # 更新滑块显示
            angles = PRESET_POSES.get(pose, PRESET_POSES[HandPose.OPEN])
            for i, angle in enumerate(angles[:ENCODER_COUNT]):
                if i < len(self.joint_sliders):
                    self.joint_sliders[i].set_value(angle)
        else:
            self.log(f"手势发送失败 - 请检查连接")
    
    def on_joint_value_changed(self, joint_id: int, value: float):
        """关节滑块值改变 - 实时发送"""
        if self.controller.is_connected() and self.controller.is_started():
            angles = self.controller.get_target_angles()
            angles[joint_id] = value
            self.controller.set_target_angles_live(angles)
    
    def on_send_all_clicked(self):
        """发送全部关节角度"""
        angles = [slider.get_value() for slider in self.joint_sliders]
        self.controller.set_target_angles(angles)
        self.log(f"关节角度已发送")
    
    def on_zero_all_clicked(self):
        """全部归零"""
        for slider in self.joint_sliders:
            slider.set_value(0)
        self.on_send_all_clicked()
        self.log("所有关节已归零")
    
    def on_read_current_clicked(self):
        """读取当前角度到滑块"""
        angles = self.controller.get_current_angles()
        for i, angle in enumerate(angles[:ENCODER_COUNT]):
            if i < len(self.joint_sliders):
                self.joint_sliders[i].set_value(angle)
        self.log("已读取当前角度")
    
    def _ensure_direct_motor_mode_for_motor_panel(self) -> None:
        """电机直控面板的指令在下位机仅在 DIRECT_MOTOR 下生效；必要时同步 UI。"""
        if self.controller.get_control_mode() == ControlMode.DIRECT_MOTOR:
            return
        idx = self.mode_combo.findData(ControlMode.DIRECT_MOTOR)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)
    
    def on_motor_value_changed(self, motor_id: int, value: int):
        """电机滑块值改变"""
        self.motor_live_values[motor_id] = value
        # 实时模式下立即发令（需已连接且为电机直控模式，由发送路径保证）
        if self.motor_live_mode and self.controller.is_connected():
            if not self.motor_live_timer.isActive():
                self.motor_live_timer.start(50)  # 50ms后批量发送
    
    def on_motor_fine_adjust(self, motor_id: int, delta: int):
        """电机微调按钮点击 - 顺时针/逆时针微调"""
        if not self.controller.is_connected():
            return
        
        # 更新存储值
        new_value = self.motor_sliders[motor_id].get_value()
        self.motor_live_values[motor_id] = new_value
        
        # 实时模式下立即发送
        if self.motor_live_mode:
            if not self.motor_live_timer.isActive():
                self.motor_live_timer.start(30)  # 30ms后批量发送
        else:
            # 非实时模式，单独发送这个电机
            try:
                # 构建单个电机的位置列表（只改变这个电机）
                positions = [self.motor_sliders[i].get_value() for i in range(MOTOR_COUNT)]
                self._ensure_direct_motor_mode_for_motor_panel()
                if self.abs_pos_checkbox.isChecked():
                    self.controller.set_motor_positions_absolute(positions)
                else:
                    self.controller.set_motor_positions_raw(positions)
                
                direction = "顺时针/拉紧" if delta > 0 else "逆时针/放松"
                self.log(f"M{motor_id:02d} {direction} {abs(delta)}步")
                
            except Exception as e:
                self.log(f"⚠ 电机微调失败: {str(e)}")
    
    def on_motor_live_mode_changed(self, state):
        """电机实时控制模式切换"""
        self.motor_live_mode = (state == Qt.CheckState.Checked.value)
        
        if self.motor_live_mode:
            self.motor_live_status.setText("(实时控制已启用 - 滑动滑块立即生效)")
            self.motor_live_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
            self.log("🎮 电机实时控制模式已启用")
            
            # 如果已连接，立即发送当前值
            if self.controller.is_connected():
                self._send_motor_live_batch()
        else:
            self.motor_live_status.setText("(点击启用实时控制)")
            self.motor_live_status.setStyleSheet("color: gray; font-size: 10px;")
            self.log("⏹ 电机实时控制模式已关闭")
            
            # 停止定时器
            if self.motor_live_timer.isActive():
                self.motor_live_timer.stop()
    
    def _send_motor_live_batch(self):
        """批量发送电机实时控制值"""
        if not self.controller.is_connected():
            return
        
        # 停止定时器，防止重复触发
        self.motor_live_timer.stop()
        
        try:
            self._ensure_direct_motor_mode_for_motor_panel()
            # 获取所有电机当前值
            positions = self.motor_live_values.copy()
            
            # 根据模式发送
            if self.abs_pos_checkbox.isChecked():
                self.controller.set_motor_positions_absolute(positions)
            else:
                self.controller.set_motor_positions_raw(positions)
            
        except Exception as e:
            self.log(f"⚠ 实时发送失败: {str(e)}")
    
    def on_send_motor_clicked(self):
        """发送全部电机位置"""
        positions = [slider.get_value() for slider in self.motor_sliders]
        self.motor_live_values = list(positions)
        self._ensure_direct_motor_mode_for_motor_panel()
        
        if self.abs_pos_checkbox.isChecked():
            self.controller.set_motor_positions_absolute(positions)
            self.log("电机绝对位置已发送")
        else:
            self.controller.set_motor_positions_raw(positions)
            self.log("电机原始位置已发送")
    
    def on_center_all_clicked(self):
        """全部置中 - M00/M01使用记录的零点，其他电机使用2048"""
        for i, slider in enumerate(self.motor_sliders):
            if i == 0:  # M00
                zero_pos = self.mcp_zero_positions[0]
                slider.set_value(zero_pos)
            elif i == 1:  # M01
                zero_pos = self.mcp_zero_positions[1]
                slider.set_value(zero_pos)
            else:  # 其他电机使用2048
                slider.set_value(2048)
        self.on_send_motor_clicked()
        self.log(f"所有电机已置中 (M00={self.mcp_zero_positions[0]}, M01={self.mcp_zero_positions[1]}, 其他=2048)")
    
    def on_abs_mode_changed(self, state):
        """绝对位置模式改变"""
        is_abs = state == Qt.CheckState.Checked.value
        
        for slider in self.motor_sliders:
            if is_abs:
                slider.slider.setRange(-30719, 30719)
                slider.spin.setRange(-30719, 30719)
                slider.set_value(0)
            else:
                slider.slider.setRange(0, 4095)
                slider.spin.setRange(0, 4095)
                slider.set_value(2048)
    
    def on_hand_model_updated(self, model: HandModel):
        """手部数据模型更新回调"""
        pass  # 使用状态轮询
    
    def on_status_updated(self, status: dict):
        """状态更新回调"""
        # ===== 更新舵机状态 =====
        servo_online = status.get('servo_online', [])
        servo_angles = status.get('servo_angles', [])
        servo_speed = status.get('servo_speed', [])
        servo_load = status.get('servo_load', [])
        servo_voltage = status.get('servo_voltage', [])
        servo_temperature = status.get('servo_temperature', [])
        
        # 统计在线舵机
        online_count = sum(1 for x in servo_online if x)
        
        # 计算平均电压和最高温度
        avg_voltage = sum(servo_voltage) / len(servo_voltage) / 10.0 if servo_voltage else 0
        max_temp = max(servo_temperature) if servo_temperature else 0
        
        self.servo_status_label.setText(
            f"电机在线: {online_count}/{MOTOR_COUNT} | "
            f"平均电压: {avg_voltage:.1f}V | "
            f"最高温度: {max_temp}°C"
        )
        
        # 更新舵机指示器（包含遥测数据）
        for i in range(min(MOTOR_COUNT, len(self.servo_indicators))):
            online = servo_online[i] if i < len(servo_online) else False
            angle = servo_angles[i] if i < len(servo_angles) else 0
            speed = servo_speed[i] if i < len(servo_speed) else 0
            load = servo_load[i] if i < len(servo_load) else 0
            voltage = (servo_voltage[i] / 10.0) if i < len(servo_voltage) else 0.0
            temp = servo_temperature[i] if i < len(servo_temperature) else 0
            
            self.servo_indicators[i].update_status(
                online, angle, speed, load, voltage, temp
            )
        
        # ===== 更新编码器状态 (来自S3 CAN总线) =====
        encoder_raw = status.get('encoder_raw', [])
        encoder_angles = status.get('encoder_angles', [])
        encoder_zero = status.get('encoder_zero', [])
        encoder_target = status.get('encoder_target', [])
        encoder_errors = status.get('encoder_errors', [])
        
        # 统计正常编码器
        valid_count = sum(1 for e in encoder_errors if not e) if encoder_errors else 0
        
        # 计算平均角度
        avg_angle = sum(encoder_angles) / len(encoder_angles) if encoder_angles else 0
        
        self.encoder_status_label.setText(
            f"编码器正常: {valid_count}/{ENCODER_COUNT} | "
            f"平均角度: {avg_angle:.1f}° | "
            f"来源: S3-CAN"
        )
        
        # 更新编码器指示器
        update_count = 0
        for i in range(min(ENCODER_COUNT, len(self.encoder_indicators))):
            if i < len(encoder_raw):
                raw = encoder_raw[i]
                angle = encoder_angles[i] if i < len(encoder_angles) else 0.0
                zero = encoder_zero[i] if i < len(encoder_zero) else 0
                target = encoder_target[i] if i < len(encoder_target) else 0.0
                error = encoder_errors[i] if i < len(encoder_errors) else True
                
                self.encoder_indicators[i].update_status(raw, angle, zero, target, error)
                update_count += 1

        if (
            getattr(self, "mcp_pending_encoder_sync", False)
            and self.mcp_fe_slider.isEnabled()
            and encoder_angles
            and len(encoder_angles) >= ENCODER_COUNT
            and len(encoder_errors) >= ENCODER_COUNT
        ):
            if self._try_sync_mcp_sliders_from_encoders(encoder_angles, encoder_errors):
                self.mcp_pending_encoder_sync = False
        
        # 更新MCP电机角度显示（使用舵机角度，假设M1=舵机0, M2=舵机1）
        if servo_angles and len(servo_angles) >= 2:
            m1_angle = servo_angles[0] / 100.0  # 转换为角度（假设舵机值是角度*100）
            m2_angle = servo_angles[1] / 100.0
            self.update_mcp_angle_display(m1_angle, m2_angle)
        
        # 更新故障显示
        overload = status.get('servo_overload_fault', [])
        release = status.get('joint_reverse_release_fault', [])
        
        has_fault = any(overload) or any(release)
        if has_fault:
            fault_msg = []
            if any(overload):
                fault_indices = [i for i, f in enumerate(overload) if f]
                fault_msg.append(f"过载: {fault_indices}")
            if any(release):
                fault_indices = [i for i, f in enumerate(release) if f]
                fault_msg.append(f"反绕: {fault_indices}")
            self.fault_text.setText(" | ".join(fault_msg))
            self.fault_text.setStyleSheet("color: #f44336; font-weight: bold;")
        else:
            self.fault_text.setText("无故障")
            self.fault_text.setStyleSheet("color: #4CAF50;")
    
    # ===== 标定相关方法 =====
    
    def on_zero_calibration_clicked(self):
        """点击标定零点按钮 - 记录当前编码器原始值作为零点"""
        if not self.controller.is_connected():
            QMessageBox.warning(self, "警告", "请先连接设备")
            return
        
        # 获取当前编码器原始值
        status = self.controller.get_status()
        encoder_raw = status.get('encoder_raw', [])
        
        if len(encoder_raw) < ENCODER_COUNT:
            QMessageBox.warning(self, "警告", "尚未收到编码器数据，请等待连接稳定后重试")
            return
        
        # 记录零点 - 将负数转换为0-16383范围
        self.current_zero_raw = []
        for i, raw in enumerate(encoder_raw[:ENCODER_COUNT]):
            val = int(raw)
            # 如果是负数（有符号int16），转换为14位无符号
            if val < 0:
                val += 16384
            self.current_zero_raw.append(val)
        
        # 更新UI显示
        self.calib_status_label.setText("状态: 已记录零点")
        self.calib_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        
        # 显示零点数据预览（转换后的值）
        preview = ", ".join([f"E{i}={v}" for i, v in enumerate(self.current_zero_raw[:5])])
        self.zero_preview_label.setText(f"零点数据: {preview}... (共{ENCODER_COUNT}个)")
        
        # 启用应用和保存按钮
        self.apply_calib_btn.setEnabled(True)
        self.save_calib_btn.setEnabled(True)
        
        # 自动保存配置
        self.auto_save_zero_config()
        
        self.log(f"✓ 零点标定完成 - 记录了 {ENCODER_COUNT} 路编码器零点（已归一化到0-16383）")
        self.log(f"  配置已自动保存，下次启动时将自动加载")
        
        QMessageBox.information(self, "标定完成", 
            f"已成功记录 {ENCODER_COUNT} 路编码器零点\n"
            f"转换后的零点值已归一化到0-16383范围\n"
            f"点击【应用零点】下发到下位机\n"
            f"配置已自动保存到文件，下次启动自动加载")
    
    def on_apply_calibration_clicked(self):
        """应用零点 - 将零点数据下发到下位机"""
        if not hasattr(self, 'current_zero_raw') or not self.current_zero_raw:
            QMessageBox.warning(self, "警告", "请先进行零点标定")
            return
        
        if not self.controller.is_connected():
            QMessageBox.warning(self, "警告", "设备未连接")
            return
        
        # 通过控制器设置零点（同时更新上位机状态和下位机）
        self.controller.set_encoder_zeros(self.current_zero_raw)
        
        self.log(f"零点数据已应用 - 下发到下位机并更新本地状态")
        self.calib_status_label.setText("状态: 已应用")
        
        # 立即更新显示
        QMessageBox.information(self, "应用成功", 
            "零点数据已下发到下位机\n"
            "当前位置已被设为零点（所有角度应显示为0或接近0）\n"
            "如果角度不为0，请检查编码器数据是否正常接收")
    
    def on_save_calibration_clicked(self):
        """保存零点配置到文件"""
        if not hasattr(self, 'current_zero_raw') or not self.current_zero_raw:
            QMessageBox.warning(self, "警告", "没有可保存的零点数据")
            return
        
        from PyQt6.QtWidgets import QFileDialog
        import json
        
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "保存零点配置",
            "encoder_zero_config.json",
            "JSON文件 (*.json);;所有文件 (*.*)"
        )
        
        if filename:
            try:
                config = {
                    'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                    'encoder_count': ENCODER_COUNT,
                    'zero_raw_values': self.current_zero_raw,
                    'description': '磁编码器零点配置'
                }
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                
                self.log(f"零点配置已保存到: {filename}")
                QMessageBox.information(self, "保存成功", f"配置已保存到:\n{filename}")
            except Exception as e:
                QMessageBox.critical(self, "保存失败", f"错误: {str(e)}")
    
    def on_load_calibration_clicked(self):
        """从文件加载零点配置"""
        from PyQt6.QtWidgets import QFileDialog
        import json
        
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "加载零点配置",
            "",
            "JSON文件 (*.json);;所有文件 (*.*)"
        )
        
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                
                loaded_zeros = config.get('zero_raw_values', [])
                
                if len(loaded_zeros) != ENCODER_COUNT:
                    QMessageBox.warning(self, "配置错误", 
                        f"配置文件中编码器数量不匹配\n文件: {len(loaded_zeros)}, 需要: {ENCODER_COUNT}")
                    return
                
                self.current_zero_raw = loaded_zeros
                
                # 更新UI
                self.calib_status_label.setText(f"状态: 已加载 ({timestamp})")
                self.calib_status_label.setStyleSheet("color: #2196F3; font-weight: bold;")
                
                preview = ", ".join([f"E{i}={v}" for i, v in enumerate(self.current_zero_raw[:5])])
                self.zero_preview_label.setText(f"零点数据(加载): {preview}...")
                
                # 启用按钮
                self.apply_calib_btn.setEnabled(True)
                self.save_calib_btn.setEnabled(True)
                
                self.log(f"✓ 零点配置已加载: {filename}")
                
                # 如果已连接，自动应用；否则询问
                if self.controller.is_connected():
                    try:
                        self.controller.set_encoder_zeros(self.current_zero_raw)
                        self.log(f"✓ 零点配置已自动应用到下位机")
                        self.calib_status_label.setText("状态: 已应用（加载）")
                        QMessageBox.information(self, "应用成功", "零点配置已加载并应用到下位机")
                    except Exception as e:
                        QMessageBox.warning(self, "应用失败", f"零点已加载但应用失败: {str(e)}")
                else:
                    # 询问是否立即应用
                    reply = QMessageBox.question(
                        self, 
                        "配置加载成功",
                        "零点配置已加载，是否立即应用到下位机？\n（需要设备已连接）",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                    )
                    
                    if reply == QMessageBox.StandardButton.Yes:
                        self.on_apply_calibration_clicked()
                
            except Exception as e:
                QMessageBox.critical(self, "加载失败", f"错误: {str(e)}")
    
    def update_ui(self):
        """UI更新定时器"""
        pass  # 主要由信号驱动
    
    # ===== 双电机MCP关节控制 =====
    
    def on_mcp_enter_calibration(self):
        """进入MCP标定模式"""
        if not self.controller.is_connected():
            QMessageBox.warning(self, "警告", "请先连接设备")
            return
        
        reply = QMessageBox.question(
            self,
            "进入标定模式",
            "即将进入MCP双电机标定模式:\n\n"
            "1. 电机会进入无力矩状态（自由旋转）\n"
            "2. 使用【+10°/-10°】按钮微调电机位置\n"
            "3. 观察编码器角度，调整至接近0°\n"
            "4. 两个电机都调整到拉紧状态后，点击【确认标定零点】\n\n"
            "是否确认进入标定模式？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            try:
                # 发送进入标定模式命令
                self.controller.send_raw_command("TORQUE:0\n")
                self.mcp_in_calibration = True
                
                # 更新UI
                self._enable_mcp_calib_controls()
                
                self.mcp_result_label.setText("提示: 使用+10°/-10°按钮微调电机，观察角度显示，调整至接近0°")
                
                self.log("🎯 已进入MCP标定模式")
                self.log("  ✓ 力矩已关闭，电机可手动控制")
                self.log("  → 使用微调按钮调整电机位置")
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"进入标定模式失败: {str(e)}")
    
    def on_mcp_exit_calibration(self):
        """退出MCP标定模式"""
        try:
            self.controller.send_raw_command("TORQUE:1\n")
            self.mcp_in_calibration = False
            
            # 更新UI
            self._disable_mcp_calib_controls()
            
            self.mcp_result_label.setText("提示: 已退出标定模式，如需重新标定请点击【进入标定模式】")
            
            self.log("✗ 已退出MCP标定模式")
            self.log("  ✓ 力矩已恢复")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"退出标定模式失败: {str(e)}")
    
    def on_mcp_motor_move(self, motor_id: int, delta: int):
        """
        MCP电机微调控制
        
        Args:
            motor_id: 1=电机1, 2=电机2
            delta: 移动量（正数=拉紧/顺时针，负数=放松/逆时针）
        """
        if not self.mcp_in_calibration:
            QMessageBox.warning(self, "警告", "请先进入标定模式")
            return
        
        try:
            # 发送相对移动命令
            cmd = f"MOVE{motor_id}:{delta}\n"
            self.controller.send_raw_command(cmd)
            
            # 更新显示
            if motor_id == 1:
                current_text = self.mcp_m1_pos_label.text()
                try:
                    current = int(current_text.replace("°", ""))
                except:
                    current = 0
                new_pos = current + delta
                self.mcp_m1_pos_label.setText(f"{new_pos}°")
            else:
                current_text = self.mcp_m2_pos_label.text()
                try:
                    current = int(current_text.replace("°", ""))
                except:
                    current = 0
                new_pos = current + delta
                self.mcp_m2_pos_label.setText(f"{new_pos}°")
            
            self.log(f"  电机{motor_id} {'拉紧' if delta > 0 else '放松'} {abs(delta)}°")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"电机控制失败: {str(e)}")
    
    def on_mcp_confirm_zero(self):
        """确认标定零点 - 记录当前M00/M01位置为零点"""
        # 获取当前M00和M01的位置
        m00_pos = self.motor_sliders[0].get_value()
        m01_pos = self.motor_sliders[1].get_value()

        # 确认对话框
        reply = QMessageBox.question(
            self,
            "确认标定零点",
            f"请确认将当前位置记录为MCP零点:\n\n"
            f"M00 (电机0): {m00_pos}\n"
            f"M01 (电机1): {m01_pos}\n\n"
            f"建议此时 Fe/AA 滑块为 0（θ1=θ2=0），与绳长正解零位一致。\n\n"
            f"点击【Yes】确认当前位置为零点\n"
            f"点击【No】取消并继续调整",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            try:
                # 保存零点位置
                self.mcp_zero_positions = [m00_pos, m01_pos]

                # 发送标定命令到下位机
                self.controller.send_raw_command("ZERO_ALL\n")
                self.log("⚡ 双电机零点标定命令已发送")

                # 保存到EEPROM
                self.controller.send_raw_command("SAVE\n")
                self.log("✓ 零点已保存到EEPROM（掉电不丢失）")

                # 更新UI显示
                self.mcp_zero_pos_label.setText(f"当前零点: M00={m00_pos}, M01={m01_pos} (已保存)")
                self.mcp_zero_pos_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 11px;")
                self.mcp_result_label.setText(
                    "✓ 标定完成！下一次连接将按磁编码器同步 Fe/AA 滑块（E{}/E{}）".format(
                        self.mcp_encoder_fe_index, self.mcp_encoder_aa_index
                    )
                )

                # 自动保存配置到文件
                self._save_mcp_zero_config()

                self.mcp_zero_ready = True
                self._refresh_mcp_fe_aa_controls()
                # 标定完成当帧若已有编码器数据，立即尝试同步滑块
                if self.controller.is_connected():
                    ang: List[float] = []
                    err: List[bool] = []
                    with self.controller.state_lock:
                        st = self.controller.current_state
                        if st.has_sensor_data:
                            ang = [e.current_angle_deg for e in st.encoders]
                            err = [e.error for e in st.encoders]
                    if len(ang) >= ENCODER_COUNT and self._try_sync_mcp_sliders_from_encoders(ang, err):
                        self.mcp_pending_encoder_sync = False

                QMessageBox.information(
                    self,
                    "标定成功",
                    "双电机MCP关节零点标定完成！\n\n"
                    "✓ 当前位置已记录为零点\n"
                    "✓ 零点数据已保存到设备EEPROM\n"
                    "✓ 配置已保存到文件\n"
                    "✓ 下次启动自动加载\n\n"
                    f"零点位置: M00={m00_pos}, M01={m01_pos}\n\n"
                    f"现在可拖动 Fe/AA 实时控制；请先「启动控制 (START)」。\n"
                    f"滑块位置将与磁编码器 E{self.mcp_encoder_fe_index}(FE)/"
                    f"E{self.mcp_encoder_aa_index}(AA) 对齐。\n"
                    "仍可用下方按钮再发一次确认。",
                )

            except Exception as e:
                QMessageBox.critical(self, "错误", f"标定失败: {str(e)}")

    
    def _save_mcp_zero_config(self):
        """保存MCP零点配置"""
        try:
            config = {
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'type': 'MCP_DUAL_MOTOR',
                'motor_count': 2,
                'mcp_zero_positions': self.mcp_zero_positions,  # 保存实际的零点位置
                'mcp_rope_k_counts_per_mm': [
                    float(self.mcp_rope_k_counts_per_mm[0]),
                    float(self.mcp_rope_k_counts_per_mm[1]),
                ],
                'mcp_encoder_fe_index': int(self.mcp_encoder_fe_index),
                'mcp_encoder_aa_index': int(self.mcp_encoder_aa_index),
                'description': 'MCP双电机零点与绳长比例'
            }

            filename = os.path.join(os.path.dirname(ZERO_CONFIG_FILE), "mcp_zero_config.json")
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)

            self.log(f"  ✓ MCP零点配置已保存到文件 (M00={self.mcp_zero_positions[0]}, M01={self.mcp_zero_positions[1]})")
        except Exception as e:
            self.log(f"  ⚠ 保存配置文件失败: {str(e)}")

    def _load_mcp_zero_config(self):
        """加载MCP零点配置"""
        try:
            filename = os.path.join(os.path.dirname(ZERO_CONFIG_FILE), "mcp_zero_config.json")
            if os.path.exists(filename):
                with open(filename, 'r', encoding='utf-8') as f:
                    config = json.load(f)

                loaded_positions = config.get('mcp_zero_positions', [2048, 2048])
                timestamp = config.get('timestamp', '未知')

                self.mcp_zero_positions = loaded_positions

                km = config.get('mcp_rope_k_counts_per_mm')
                if isinstance(km, list) and len(km) >= 2:
                    self.mcp_rope_k_counts_per_mm = [float(km[0]), float(km[1])]

                i_fe = config.get('mcp_encoder_fe_index')
                if isinstance(i_fe, int) and 0 <= i_fe < ENCODER_COUNT:
                    self.mcp_encoder_fe_index = i_fe
                i_aa = config.get('mcp_encoder_aa_index')
                if isinstance(i_aa, int) and 0 <= i_aa < ENCODER_COUNT:
                    self.mcp_encoder_aa_index = i_aa

                self.mcp_zero_ready = True

                # 更新UI显示
                self.mcp_zero_pos_label.setText(f"当前零点: M00={loaded_positions[0]}, M01={loaded_positions[1]} (已加载 {timestamp})")
                self.mcp_zero_pos_label.setStyleSheet("color: #4CAF50; font-size: 11px;")
                self.log(f"✓ MCP零点配置已加载 (M00={loaded_positions[0]}, M01={loaded_positions[1]}, {timestamp})")
            else:
                self.mcp_zero_ready = False
                self.log(
                    "ℹ 未找到MCP零点配置文件，使用默认 M00/M01=2048；"
                    "首次请「确认标定零点」后再用 Fe/AA。"
                )
        except Exception as e:
            self.mcp_zero_ready = False
            self.log(f"⚠ 加载MCP零点配置失败: {str(e)}")

    def _apply_mcp_slider_ranges(self) -> None:
        """应用 MCP 机械限位（与资料一致）。"""
        self.mcp_fe_slider.setRange(MCP_FE_SLIDER_MIN, MCP_FE_SLIDER_MAX)
        self.mcp_aa_slider.setRange(MCP_AA_SLIDER_MIN, MCP_AA_SLIDER_MAX)

    def _try_sync_mcp_sliders_from_encoders(
        self,
        encoder_angles: List[float],
        encoder_errors: List[bool],
    ) -> bool:
        """
        用磁编码器当前角（已过零校准的关节角）填充 Fe/AA 滑块。
        返回是否已成功同步（两路均无 error 且在合法序号内）。
        """
        ife = self.mcp_encoder_fe_index
        iaa = self.mcp_encoder_aa_index
        if (
            ife < 0 or ife >= ENCODER_COUNT
            or iaa < 0 or iaa >= ENCODER_COUNT
            or ife >= len(encoder_angles)
            or iaa >= len(encoder_angles)
        ):
            return False
        err_fe = encoder_errors[ife] if ife < len(encoder_errors) else True
        err_aa = encoder_errors[iaa] if iaa < len(encoder_errors) else True
        if err_fe or err_aa:
            return False
        fe_deg = float(encoder_angles[ife])
        aa_deg = float(encoder_angles[iaa])
        fe_i = int(round(max(MCP_FE_SLIDER_MIN, min(MCP_FE_SLIDER_MAX, fe_deg))))
        aa_i = int(round(max(MCP_AA_SLIDER_MIN, min(MCP_AA_SLIDER_MAX, aa_deg))))
        self.mcp_fe_slider.blockSignals(True)
        self.mcp_aa_slider.blockSignals(True)
        self.mcp_fe_slider.setValue(fe_i)
        self.mcp_aa_slider.setValue(aa_i)
        self.mcp_fe_slider.blockSignals(False)
        self.mcp_aa_slider.blockSignals(False)
        return True

    def _refresh_mcp_fe_aa_controls(self) -> None:
        """
        串口已连接时：若已有持久化 MCP 零点，则启用 Fe/AA / 应用；
        Fe/AA 初值在收到磁编码器数据后同步（不强行置 0）。
        """
        if not self.controller.is_connected():
            return
        self.mcp_confirm_zero_btn.setEnabled(True)
        if self.mcp_zero_ready:
            self._apply_mcp_slider_ranges()
            self.mcp_pending_encoder_sync = True
            self.mcp_fe_slider.setEnabled(True)
            self.mcp_aa_slider.setEnabled(True)
            self.mcp_apply_btn.setEnabled(True)
        else:
            self.mcp_fe_slider.setEnabled(False)
            self.mcp_aa_slider.setEnabled(False)
            self.mcp_apply_btn.setEnabled(False)

    def on_mcp_fe_aa_slider_changed(self, _value: int = 0) -> None:
        """Fe/AA 滑块变化时节流触发实时电机指令。"""
        if not self.mcp_fe_slider.isEnabled():
            return
        if not self.controller.is_connected() or not self.controller.is_started():
            return
        self.mcp_live_timer.start(40)

    def _send_mcp_fe_aa_live(self) -> None:
        """节流到期后发送一次 CMD_MOTOR_POS_ABS（无弹窗）。"""
        if not self.controller.is_connected() or not self.controller.is_started():
            return
        if not self.mcp_fe_slider.isEnabled():
            return
        res = self._compute_mcp_fe_aa_positions()
        if res is None:
            return
        positions, _, _, _, _ = res
        try:
            self.controller.send_motor_positions_absolute_force(positions)
        except Exception:
            pass

    def _compute_mcp_fe_aa_positions(
        self,
    ) -> Optional[Tuple[List[int], float, float, float, float]]:
        """
        根据当前 Fe/AA 滑块计算 22 路绝对目标：M00/M01 由绳长模型，其余保持当前反馈。
        返回 (positions, fe_deg, aa_deg, d_l1_mm, d_l2_mm)；未就绪则 None。
        """
        if not self.controller.is_connected():
            return None
        with self.controller.state_lock:
            if not self.controller.current_state.has_servo_angle_data:
                return None
            positions = list(self.controller.current_state.servo_angles)
        while len(positions) < MOTOR_COUNT:
            positions.append(0)
        positions = positions[:MOTOR_COUNT]
        fe_value = float(self.mcp_fe_slider.value())
        aa_value = float(self.mcp_aa_slider.value())
        d_l1, d_l2 = delta_rope_lengths_mm(aa_value, fe_value)
        k0 = float(self.mcp_rope_k_counts_per_mm[0])
        k1 = float(self.mcp_rope_k_counts_per_mm[1])
        dm0 = int(round(k0 * d_l1))
        dm1 = int(round(k1 * d_l2))
        m0_cmd = int(self.mcp_zero_positions[0]) + dm0
        m1_cmd = int(self.mcp_zero_positions[1]) + dm1
        positions[0] = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, m0_cmd))
        positions[1] = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, m1_cmd))
        return positions, fe_value, aa_value, d_l1, d_l2

    def update_mcp_angle_display(self, m1_angle: float, m2_angle: float):
        """更新MCP电机角度显示（在状态更新时调用）"""
        if hasattr(self, 'mcp_m1_angle_label'):
            self.mcp_m1_angle_label.setText(f"M1角度: {m1_angle:.1f}°")
            # 接近0度时变绿色
            if abs(m1_angle) < 5:
                self.mcp_m1_angle_label.setStyleSheet("font-size: 14px; color: #4CAF50; font-weight: bold;")
            else:
                self.mcp_m1_angle_label.setStyleSheet("font-size: 14px; color: #2196F3;")
        
        if hasattr(self, 'mcp_m2_angle_label'):
            self.mcp_m2_angle_label.setText(f"M2角度: {m2_angle:.1f}°")
            if abs(m2_angle) < 5:
                self.mcp_m2_angle_label.setStyleSheet("font-size: 14px; color: #4CAF50; font-weight: bold;")
            else:
                self.mcp_m2_angle_label.setStyleSheet("font-size: 14px; color: #2196F3;")
    
    def on_mcp_apply_clicked(self):
        """手动再发一次当前 Fe/AA（与滑块实时逻辑相同，带提示框）。"""
        if not self.controller.is_connected():
            QMessageBox.warning(self, "警告", "请先连接设备")
            return
        if not self.controller.is_started():
            QMessageBox.warning(self, "警告", "请先点击「启动控制 (START)」后再应用 Fe/AA")
            return

        res = self._compute_mcp_fe_aa_positions()
        if res is None:
            QMessageBox.warning(self, "警告", "正在等待舵机角度数据，请稍后再试")
            return

        positions, fe_value, aa_value, d_l1, d_l2 = res
        k0 = float(self.mcp_rope_k_counts_per_mm[0])
        k1 = float(self.mcp_rope_k_counts_per_mm[1])
        dm0 = positions[0] - int(self.mcp_zero_positions[0])
        dm1 = positions[1] - int(self.mcp_zero_positions[1])

        self.log(
            f"MCP Fe/AA: AA={aa_value:.1f}°(θ1), FE={fe_value:.1f}°(θ2) | "
            f"ΔL1={d_l1:.3f} ΔL2={d_l2:.3f} mm | "
            f"Δ脉宽={dm0},{dm1} → M00={positions[0]}, M01={positions[1]}"
        )
        if hasattr(self, "mcp_motor_pos_label"):
            self.mcp_motor_pos_label.setText(
                f"电机: M00={positions[0]} M01={positions[1]} (k=[{k0:.2f},{k1:.2f}] ct/mm)"
            )

        try:
            self.controller.send_motor_positions_absolute_force(positions)
            self.log("  ✓ CMD_MOTOR_POS_ABS 已发送（下位机会切入直控模式）")
        except Exception as e:
            self.log(f"  ✗ 发送失败: {str(e)}")
            QMessageBox.critical(self, "错误", f"发送失败: {str(e)}")
            return

        QMessageBox.information(
            self,
            "位置已应用",
            f"θ1=AA: {aa_value:.1f}°\nθ2=FE: {fe_value:.1f}°\n\n"
            f"ΔL1={d_l1:.3f} mm, ΔL2={d_l2:.3f} mm\n"
            f"M00={positions[0]}, M01={positions[1]}\n\n"
            f"已发送多圈绝对位置（其余电机保持当前反馈值）。",
        )
    
    # ===== 自动加载/保存零点配置 =====
    
    def auto_load_zero_config(self):
        """启动时自动加载零点配置文件"""
        try:
            if os.path.exists(ZERO_CONFIG_FILE):
                with open(ZERO_CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                
                loaded_zeros = config.get('zero_raw_values', [])
                timestamp = config.get('timestamp', '未知')
                
                if len(loaded_zeros) == ENCODER_COUNT:
                    self.current_zero_raw = loaded_zeros
                    
                    # 更新UI显示
                    self.calib_status_label.setText(f"状态: 已加载 ({timestamp})")
                    self.calib_status_label.setStyleSheet("color: #2196F3; font-weight: bold;")
                    
                    preview = ", ".join([f"E{i}={v}" for i, v in enumerate(self.current_zero_raw[:3])])
                    self.zero_preview_label.setText(f"已加载零点: {preview}... (共{ENCODER_COUNT}个)")
                    
                    self.apply_calib_btn.setEnabled(False)  # 等待连接后才能应用
                    self.save_calib_btn.setEnabled(True)
                    
                    self.log(f"✓ 自动加载零点配置成功 - 时间: {timestamp}")
                    self.log(f"  连接设备后将自动下发零点到下位机")
                else:
                    self.log(f"⚠ 零点配置文件编码器数量不匹配，忽略")
            else:
                self.log(f"ℹ 未找到零点配置文件，首次使用请进行标定")
        except Exception as e:
            self.log(f"⚠ 加载零点配置失败: {str(e)}")
    
    def auto_save_zero_config(self):
        """自动保存零点配置到默认文件"""
        if not hasattr(self, 'current_zero_raw') or not self.current_zero_raw:
            return
        
        try:
            config = {
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'encoder_count': ENCODER_COUNT,
                'zero_raw_values': self.current_zero_raw,
                'description': '磁编码器零点配置（自动保存）'
            }
            
            with open(ZERO_CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
            
            self.log(f"✓ 零点配置已自动保存")
        except Exception as e:
            self.log(f"⚠ 自动保存零点配置失败: {str(e)}")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.update_thread:
            self.update_thread.stop()
            self.update_thread.wait(1000)
        
        self.controller.shutdown()
        event.accept()
