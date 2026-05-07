import re

with open('main_window.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Find and replace setup_preset_group function
old_start = '    def setup_preset_group(self, parent_layout):\n        """双电机MCP关节标定控制组 - 完整标定向导"""'
new_start = '    def setup_preset_group(self, parent_layout):\n        """双电机MCP关节标定控制组 - 零点标定"""'

if old_start in content:
    content = content.replace(old_start, new_start)
    print('Replaced docstring')

# Remove step1 section - find from step1_label to mcp_calib_status
step1_pattern = r'        # ========== 步骤1: 进入标定模式 ==========.*?(?=        # ========== 步骤2)'
content = re.sub(step1_pattern, '', content, flags=re.DOTALL)
print('Removed step1')

# Replace step2 text
content = content.replace('步骤2: 确认标定零点', '确认标定零点')
content = content.replace('步骤3: Fe/AA控制（标定后使用）', 'Fe/AA控制')

# Remove the old tooltip text and result label text
old_tip = '确认当前位置为零点并保存到EEPROM'
new_tip = '将当前M00/M01电机位置记录为MCP零点'
content = content.replace(old_tip, new_tip)

old_result = '请先进入标定模式，在右侧电机面板使用M00/M01的微调按钮调整至0°，然后确认标定'
new_result = '调整M00/M01到目标位置后，点击上方按钮记录为零点'
content = content.replace(old_result, new_result)

# Change button style
old_style = 'padding: 12px; font-weight: bold; font-size: 14px;'
new_style = 'padding: 15px; font-weight: bold; font-size: 16px;'
content = content.replace(old_style, new_style)

# Add mcp_zero_positions initialization
old_init = '        # 标定模式标志\n        self.mcp_in_calibration = False'
new_init = '        # MCP零点位置存储 (M00, M01)\n        self.mcp_zero_positions = [2048, 2048]'
content = content.replace(old_init, new_init)

# Add mcp_zero_pos_label after mcp_confirm_zero_btn
old_btn = '        layout.addWidget(self.mcp_confirm_zero_btn, 4, 0, 1, 3)'
new_btn = '''        layout.addWidget(self.mcp_confirm_zero_btn, 4, 0, 1, 3)
        
        # 当前零点位置显示
        self.mcp_zero_pos_label = QLabel("当前零点: M00=2048, M01=2048 (默认)")
        self.mcp_zero_pos_label.setStyleSheet("color: #666; font-size: 11px;")
        self.mcp_zero_pos_label.setWordWrap(True)
        layout.addWidget(self.mcp_zero_pos_label, 5, 0, 1, 3)'''
content = content.replace(old_btn, new_btn)

# Fix layout row numbers
content = content.replace('layout.addWidget(self.mcp_result_label, 5, 0, 1, 3)', 
                          'layout.addWidget(self.mcp_result_label, 6, 0, 1, 3)')
content = content.replace('layout.addWidget(step3_label, 6, 0, 1, 3)', 
                          'layout.addWidget(feaa_label, 7, 0, 1, 3)')
content = content.replace('layout.addLayout(ctrl_layout, 7, 0, 1, 3)', 
                          'layout.addLayout(ctrl_layout, 8, 0, 1, 3)')
content = content.replace('layout.addWidget(self.mcp_apply_btn, 8, 0, 1, 3)', 
                          'layout.addWidget(self.mcp_apply_btn, 9, 0, 1, 3)')

with open('main_window.py', 'w', encoding='utf-8') as f:
    f.write(content)

print('Fixes applied')
