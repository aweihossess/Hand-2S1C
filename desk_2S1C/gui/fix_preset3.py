with open('main_window.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Lines 608-633 contain step1 (0-indexed: 607-632)
# We want to delete lines 608-633 (inclusive)
# and also remove the step2 label text but keep the button

# First, let's just delete lines 608-633
new_lines = lines[:607]  # Up to line 607 (0-indexed, so line 608 in 1-indexed)

# Keep from line 634 onwards, but we need to adjust
# Line 634 in 1-indexed is line 633 in 0-indexed
# But we also want to modify line 634 (step2 label)
remaining = lines[633:]  # From line 634 onwards

# Modify the first line of remaining (which is step2 label)
# Change "步骤2: 确认标定零点" to "确认标定零点"
if remaining:
    remaining[0] = remaining[0].replace('步骤2: 确认标定零点', '确认标定零点')

# Also need to change row numbers in layout.addWidget calls
# Original: row 3 becomes row 0, row 4 becomes row 1, etc.
adjusted = []
for line in remaining:
    # Adjust row numbers by subtracting 3
    # row 3 -> 0, row 4 -> 1, row 5 -> 2, row 6 -> 3, row 7 -> 4, row 8 -> 5
    new_line = line
    new_line = new_line.replace(', 3, 0, 1, 3)', ', 0, 0, 1, 3)')
    new_line = new_line.replace(', 4, 0, 1, 3)', ', 1, 0, 1, 3)')
    new_line = new_line.replace(', 5, 0, 1, 3)', ', 2, 0, 1, 3)')
    new_line = new_line.replace(', 6, 0, 1, 3)', ', 3, 0, 1, 3)')
    new_line = new_line.replace(', 7, 0, 1, 3)', ', 4, 0, 1, 3)')
    new_line = new_line.replace(', 8, 0, 1, 3)', ', 5, 0, 1, 3)')
    adjusted.append(new_line)

# Also update step3 label
for i, line in enumerate(adjusted):
    if '步骤3: Fe/AA控制（标定后使用）' in line:
        adjusted[i] = line.replace('步骤3: Fe/AA控制（标定后使用）', 'Fe/AA控制')
        break

# Combine
final_lines = new_lines + adjusted

with open('main_window.py', 'w', encoding='utf-8') as f:
    f.writelines(final_lines)

print(f'Deleted lines 608-633, {len(lines) - len(final_lines)} lines removed')
