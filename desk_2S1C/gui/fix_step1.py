import re

with open('main_window.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Remove step1 section completely - from "# ========== 步骤1:" to the start of step2
# Pattern to match step1 and step2 header
old_section = '''        # ========== 步骤1: 进入标定模式 ==========
        step1_label = QLabel("步骤1: 进入标定模式")
        step1_label.setStyleSheet("font-weight: bold; color: #333;")
        layout.addWidget(step1_label, 0, 0, 1, 3)
        
        self.mcp_enter_calib_btn = QPushButton("🎯 进入标定模式")
        self.mcp_enter_calib_btn.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; padding: 10px; font-weight: bold; }"
        )
        self.mcp_enter_calib_btn.setToolTip("关闭力矩，进入手动微调模式")
        self.mcp_enter_calib_btn.clicked.connect(self.on_mcp_enter_calibration)
        self.mcp_enter_calib_btn.setEnabled(False)
        layout.addWidget(self.mcp_enter_calib_btn, 1, 0, 1, 2)
        
        self.mcp_exit_calib_btn = QPushButton("✗ 退出标定")
        self.mcp_exit_calib_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; }"
        )
        self.mcp_exit_calib_btn.setToolTip("退出标定模式，恢复力矩")
        self.mcp_exit_calib_btn.clicked.connect(self.on_mcp_exit_calibration)
        self.mcp_exit_calib_btn.setEnabled(False)
        layout.addWidget(self.mcp_exit_calib_btn, 1, 2)
        
        self.mcp_calib_status = QLabel("状态: 未进入标定模式")
        self.mcp_calib_status.setStyleSheet("color: gray;")
        layout.addWidget(self.mcp_calib_status, 2, 0, 1, 3)
        
        # ========== 步骤2: 确认标定零点 =========='''

new_section = '''        # 确认标定零点'''

if old_section in content:
    content = content.replace(old_section, new_section)
    print('Step 1 removed successfully')
else:
    print('Could not find exact match for step1 section')

# Now remove all references to old buttons throughout the file
# Remove lines containing these patterns
patterns_to_remove = [
    r'self\.mcp_enter_calib_btn\.setEnabled\([^)]*\)',
    r'self\.mcp_exit_calib_btn\.setEnabled\([^)]*\)',
    r'self\._enable_mcp_calib_controls\(\)',
    r'self\._disable_mcp_calib_controls\(\)',
]

for pattern in patterns_to_remove:
    matches = re.findall(pattern, content)
    if matches:
        print(f'Found {len(matches)} matches for: {pattern[:50]}...')
        # Remove the lines containing these patterns
        lines = content.split('\n')
        new_lines = []
        for line in lines:
            if not re.search(pattern, line):
                new_lines.append(line)
            else:
                print(f'  Removed: {line.strip()[:60]}')
        content = '\n'.join(new_lines)

with open('main_window.py', 'w', encoding='utf-8') as f:
    f.write(content)

print('Done!')
