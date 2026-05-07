with open('main_window.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find lines to delete (step1 section: lines 608-633 approximately)
# We need to find the exact boundaries
start_marker = None
end_marker = None

for i, line in enumerate(lines):
    if '# ========== 步骤1: 进入标定模式 ==========' in line:
        start_marker = i
    if start_marker and '# ========== 步骤2: 确认标定零点 ==========' in line:
        end_marker = i
        break

if start_marker is not None and end_marker is not None:
    print(f'Found step1 section: lines {start_marker+1} to {end_marker}')
    # Keep lines before start_marker and from end_marker onwards
    # But modify end_marker line to remove "步骤2: "
    lines[end_marker] = lines[end_marker].replace('步骤2: 确认标定零点', '确认标定零点')
    lines[end_marker] = lines[end_marker].replace('# ========== 步骤2: 确认标定零点 ==========', '# 确认标定零点')
    new_lines = lines[:start_marker] + lines[end_marker:]
    print(f'Deleted {end_marker - start_marker} lines')
else:
    print(f'Could not find boundaries: start={start_marker}, end={end_marker}')
    new_lines = lines

with open('main_window.py', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print('Done')
