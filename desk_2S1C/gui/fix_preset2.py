with open('main_window.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find and delete lines from step1 to step2 (lines 608-633 based on the offset)
# We need to identify the exact range
output_lines = []
skip_until_step2 = False
for i, line in enumerate(lines):
    if '# ========== 步骤1: 进入标定模式 ==========' in line:
        skip_until_step2 = True
        continue
    if skip_until_step2 and '# ========== 步骤2: 确认标定零点 ==========' in line:
        skip_until_step2 = False
    if skip_until_step2:
        continue
    output_lines.append(line)

with open('main_window.py', 'w', encoding='utf-8') as f:
    f.writelines(output_lines)

print(f'Removed step1 section, {len(lines) - len(output_lines)} lines deleted')
