#!/usr/bin/env python3
"""
机械手上位机主程序 - 2S1C
兼容ESP32-P4下位机

启动命令: python main.py
"""

import sys
import os

# 添加父目录到路径，使得可以导入 desk_2S1C 包
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# 如果父目录没有 desk_2S1C，则将当前目录作为包根
if not os.path.exists(os.path.join(parent_dir, 'desk_2S1C')):
    sys.path.insert(0, current_dir)

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

try:
    from desk_2S1C.gui.main_window import MainWindow
except ImportError:
    # 如果上述导入失败，尝试直接导入
    from gui.main_window import MainWindow


def main():
    """主函数"""
    # 高DPI支持
    if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
        QApplication.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    
    app = QApplication(sys.argv)
    app.setApplicationName("机械手上位机")
    app.setApplicationVersion("2.0")
    
    # 设置全局字体
    font = QFont("Microsoft YaHei", 10)
    app.setFont(font)
    
    # 设置应用程序样式
    app.setStyle('Fusion')
    
    # 创建并显示主窗口
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
