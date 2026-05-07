#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
修复on_mcp_confirm_zero方法
"""

with open('main_window.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 新的方法实现
new_method = '''    def on_mcp_confirm_zero(self):
        """确认标定零点 - 记录当前M00/M01位置为零点"""
        # 获取当前M00和M01的位置
        m00_pos = self.motor_sliders[0].get_value()
        m01_pos = self.motor_sliders[1].get_value()

        # 确认对话框
        reply = QMessageBox.question(
            self,
            "确认标定零点",
            f"请确认将当前位置记录为MCP零点:\\n\\n"
            f"M00 (电机0): {m00_pos}\\n"
            f"M01 (电机1): {m01_pos}\\n\\n"
            f"点击【Yes】确认当前位置为零点\\n"
            f"点击【No】取消并继续调整",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            try:
                # 保存零点位置
                self.mcp_zero_positions = [m00_pos, m01_pos]

                # 发送标定命令到下位机
                self.controller.send_raw_command("ZERO_ALL\\n")
                self.log("⚡ 双电机零点标定命令已发送")

                # 保存到EEPROM
                self.controller.send_raw_command("SAVE\\n")
                self.log("✓ 零点已保存到EEPROM（掉电不丢失）")

                # 更新UI显示
                self.mcp_zero_pos_label.setText(f"当前零点: M00={m00_pos}, M01={m01_pos} (已保存)")
                self.mcp_zero_pos_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 11px;")
                self.mcp_result_label.setText("✓ 标定完成！当前位置已设为零点，可正常使用Fe/AA控制")

                # 自动保存配置到文件
                self._save_mcp_zero_config()

                # 启用Fe/AA控制
                self.mcp_fe_slider.setEnabled(True)
                self.mcp_aa_slider.setEnabled(True)
                self.mcp_apply_btn.setEnabled(True)

                QMessageBox.information(self, "标定成功",
                    "双电机MCP关节零点标定完成！\\n\\n"
                    "✓ 当前位置已记录为零点\\n"
                    "✓ 零点数据已保存到设备EEPROM\\n"
                    "✓ 配置已保存到文件\\n"
                    "✓ 下次启动自动加载\\n\\n"
                    f"零点位置: M00={m00_pos}, M01={m01_pos}\\n\\n"
                    "现在可以使用Fe/AA滑块控制关节运动")

            except Exception as e:
                QMessageBox.critical(self, "错误", f"标定失败: {str(e)}")

    # 删除旧的方法
    # _enable_mcp_calib_controls 和 _disable_mcp_calib_controls 已删除
'''

# 查找旧方法的开始和结束
old_start = '    def on_mcp_confirm_zero(self):'
old_end = '    def _save_mcp_zero_config(self):'

start_idx = content.find(old_start)
end_idx = content.find(old_end)

if start_idx != -1 and end_idx != -1:
    print(f"找到旧方法: 从 {start_idx} 到 {end_idx}")
    # 替换
    new_content = content[:start_idx] + new_method + content[end_idx:]
    
    with open('main_window.py', 'w', encoding='utf-8') as f:
        f.write(new_content)
    print("✓ 文件已更新")
else:
    print(f"✗ 未找到标记: start={start_idx}, end={end_idx}")
