# 力传感器数据监视器

## 协议依据

- 官方上位机当前使用 `COM7 / 115200 / 8N1 / 从站地址 1`。
- 重量数据使用 Modbus-RTU `03` 功能码读取保持寄存器。
- 每个通道占 2 个寄存器，32 位有符号整数，低 16 位寄存器在前。
- CH1 起始寄存器为 `0x0000`，CH5 起始寄存器为 `0x0008`。
- CH5 读取命令为 `01 03 00 08 00 02 45 C9`。

## 启动

```powershell
& 'C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' .\desktop\force_sensor_monitor.py --port COM7
```

其中 `COM7` 为力传感器串口。Encoder / Servo 控制板串口在第二个窗口中选择并连接，副界面默认串口为 `COM6`。

## 使用说明

1. 打开前先关闭官方上位机，避免 COM7 被占用。
2. 软件默认每 100 ms 读取 CH1 到 CH5。
3. 实时数据区显示 CH1-CH5 的当前值和相对力，换算关系为 `N = 相对值 / 10`。
4. 力传感器通道与电机张力对应关系为：CH2=M00，CH1=M01，CH3=M02，CH4=M03，CH5=M04。
5. 图表同时显示 CH1-CH5 的相对力曲线，单位为 N，图例会标出对应电机。
6. 松开电机，让力传感器放在桌面且无拉力后，点击“记录零位”，当前 CH1-CH5 原始值会保存为无拉力零位。
7. 保存的零位写入 `desktop/config/force_sensor_zero.json`，下次打开上位机会自动加载。
8. “清除零位”会删除保存的零位；“清空”只清除记录，不会清除零位。
9. “设备去皮”写寄存器 `0x0015=1`；“取消去皮”写 `0x0015=2`。
10. CSV 会导出 `channel`、`motor`、`relative_value` 和 `relative_force_n` 字段，方便按电机区分每根绳的张力。

## Encoder / Servo 界面

1. 主界面点击 `Encoder / Servo` 打开第二个界面。
2. 连接另一个串口后，界面默认请求二进制遥测，角度曲线只显示 J00-J03；完整 21 路 Encoder 数字读数仍保留在表格中。
3. Servo 数据只在数字表格中显示，包括 `Abs`、`HwAbs`、`ZeroOfs`、`Raw`、`Speed`、`Load`、电压、温度和在线状态。
4. Encoder 表格和曲线的 `Deg` 直接显示控制板/固件报出的机械零位角度。
5. `degree` 模式输入的是机械零位坐标下的 joint 目标角度，上位机会原样发送，不再加主机端角度偏置；普通“发送目标”按钮只发当前索引的单关节目标，“发送J00-J03目标”按钮会用 `CMD_ANGLE_CTRL` 一帧下发 21 路角度数组，其中 J00-J03 同时设为当前目标值。默认勾选主界面训练区的 `degree 力反馈` 后，手动发送 `degree` 目标会先清空 `tensionbias`，但在角度移动阶段只允许 M04 回程绳按 CH5 维持约 15 N 张力，M00-M03 不做张紧补偿；当 J00-J03 中已下发目标的关节都进入 `1 deg` 内并保持约 `1 s` 后，上位机才开始按完整张力窗口更新 M00-M04 的 `tensionbias`。如果张紧过程中角度又偏离目标窗口，上位机会清除 bias，并回到先等角度到位的阶段。
6. “固件机械零位”按钮只用于删除旧的主机 `encoder_zero.json`；上位机不再保存角度零点。
7. `direct` 模式发送 `direct; m索引 目标位置`，用于 servo abs position 目标值。
8. 主界面的 `Force Target Control` 只走 direct 电机目标分段步进闭环；默认 100 ms 更新一次，`Far step=20 counts`。`abs(target-current) <= 5 N` 时每 200 ms 走 1 count，并且所有启用通道都连续保持在 1 N 内 2 s 后自动退出；超过 5 N 时每次走 `Far step`。启动时会先清除并关闭 `tensionbias`，再发送 `start/direct`。
9. `START`、`STOP`、`ZERO`、`ENCODER`、`SERVO`、`STATUS` 按钮沿用 Arduino Serial Monitor 的文本命令流程。

## 训练数据采集

1. 先连接力传感器，再打开并连接 `Encoder / Servo` 窗口。
2. `degree` 目标均以固件里已经标定好的机械零位为坐标；上位机不再做相对角度零点。
3. 主窗口“训练数据采集”区域默认按 200 ms 写一行 CSV，并每 5 s 发送一个覆盖式 degree 目标。
4. 默认关节安全范围为 `J0:-60:60,J1:0:60,J2:0:50,J3:-10:100`，表示机械零位坐标下的 degree，测试前可以直接修改。
5. 自动目标会按关节轮流发送，并优先覆盖低位、高位、偏低、偏高、中位，避免 J2/J3 长时间停在零点附近。
6. 勾选“采集前自动预紧”后，开始采集会先根据 CH1-CH5 的相对力独立驱动 M00-M04；M00-M03 默认力窗口为 `-100 N <= force <= -5 N`。
7. “预紧步进”格式为 `M00:100,M01:100,M02:100,M03:100,M04:100`，数值是每次调整的张紧 abs 偏置增量；如果某根绳方向相反，把对应步进改成负数。
8. M04 当前保持为回程/预紧绳，不参与关节前馈和角度 PID；CH5 力传感器会在张紧阶段给 M04 叠加 `tensionbias`，默认维持 `-20 N <= force <= -15 N`，即约 15 N 张力。
9. 采集过程中如果勾选自动预紧，上位机会显式打开训练用 `tensionbias` 并按力窗口维护张力：太松时增加对应电机的偏置，太紧时减小该偏置；手动 `degree 力反馈` 在角度移动阶段只维护 M04 回程张力，到位后才维护完整 M00-M04 张力窗口。M00-M04 bias 限幅均为 `±7200 counts`，这仍然不同于主界面 `Force Target Control` 的 direct 电机目标闭环。
10. `Load保护(abs)` 默认为空，表示不使用电机 load 触发停止；当前主要依靠力传感器张力窗口保护。需要备用保护时可手动填入阈值。
11. “安全放松”按钮用于可行域实验前松绳：上位机会清除 `tensionbias`、发送 `start`，等待约 300 ms 让固件状态机完成 START 清命令流程后，再切到 `direct` 并通过一帧 22 路 `CMD_MOTOR_POS_ABS` 整组目标命令同时发送 M00-M04 的目标位置 `-6000 counts`；M05-M21 保持当前读数。该按钮不再按力传感器阈值做循环闭环，也不再逐条发送单电机文本目标。
12. `Encoder / Servo` 窗口中的“五路 ABS”会先预写整组目标并发送 `START`，等待 300 ms 后再次下发整组目标；界面根据 `CMD_MOTOR_POS_ABS` ACK 和 M00-M04 实时 ABS 回读显示“等待、运动中、已到位或超时”，避免把“命令已入队”误认为电机已经执行。
13. “记录当前点”按钮用于手动线性打点：每点击一次，会把当前 M00-M04 绳张力 `tension_m00_n` 到 `tension_m04_n`、J00-J03 关节角 `joint_j00_deg` 到 `joint_j03_deg`、以及 M00-M04 的 servo abs/load/current 追加到 `run_data/linearity_records/linearity_record_YYYYmmdd_HHMMSS.csv`。
14. “开始连续记录”按钮用于旁路记录当前实验过程：点击后新建 `run_data/continuous_records/continuous_record_YYYYmmdd_HHMMSS.csv`，按界面间隔持续写入与“记录当前点”相同的数据，并额外写入 `elapsed_s`；再次点击停止。该记录按钮只写 CSV，不发送 degree/direct/tension 命令。
15. “线性系统判别实验”的初始输入已改为 M00-M04 五个电机的 ABS 位置，格式为 `3500;4500;4000;3000;2000`。程序会执行 `direct 五电机到初始 ABS -> 五电机到位并停留 1 秒 -> 记录实际 ABS 基准 S_i_0 -> direct 单电机扰动/五电机保持`，不再发送初始关节 `degree` 目标或启动力反馈张紧。到位只检查五路 ABS 误差是否均在 `10 counts` 内；角度波动、力波动和电机速度只写入 CSV，不参与判断。从每次移动开始，程序每 `100 ms` 写入过程数据；到位连续保持完成点标记为 `perturb`。记录该最终点后，程序会重新下发同一个五路目标，继续保持目标不变 `1.0 s`，过程标记为 `perturb_post_hold_trace`，结束点标记为 `perturb_post_hold`，随后才执行下一个扰动。默认扫描 `0,+50,+100,+50,0,-50,-100,-50,0`；被测电机取 `baseline_abs + delta_counts`，其它电机保持各自 `baseline_abs`；数据写到 `run_data/linearity_YYYYmmdd_HHMMSS.csv`；说明见 `docs/linearity_system_identification_experiment.md`。
16. 采集文件自动写到 `run_data/training_collect_YYYYmmdd_HHMMSS.csv`。
17. CSV 包含每个时刻的完整目标向量 `target_j00_relative_deg` 到 `target_j20_relative_deg`，以及力传感器读数、按电机映射后的张力、Encoder 角度、Servo 位置、速度、load、电压、温度、在线状态、张力窗口维护动作和 `tension_bias_m00_counts` 到 `tension_bias_m04_counts`。其中 `relative_deg`/`deg_rel` 是历史字段名，当前数值就是固件机械零位坐标下的 degree。
18. 关节控制模型训练时，优先使用 `target_j00..j03_relative_deg`、`joint_j00..j03_deg_rel`、`servo_m00..m04_abs/speed/load` 和最近 0.6-1.0 s 的历史窗口；这些角度不再叠加主机端零点。

### 采集模式

训练区的 `mode` 下拉决定自动 degree 目标怎么发：

| mode | 用途 |
| --- | --- |
| `coverage` | 原来的覆盖式采集：每个目标周期只改变一个关节，其余关节保持上一次目标。适合快速铺开角度范围。 |
| `single` | 单关节解耦采集：一个关节运动，其余关节回到 0 附近。适合看单根/单组绳对关节的主要影响。 |
| `combo` | 组合姿态采集：一次发送 J00-J03 的预设组合目标。适合采耦合关系。 |
| `roundtrip` | 往返动态采集：单个关节按低位-高位-低位-中位扫动，其余关节回 0。适合覆盖摩擦、回差和方向相关性。 |
| `step` | 阶跃目标：一次发送 J00-J03 机械零位角 `[10, 20, 20, 35]`，后续保持该目标。适合观察到固定姿态的响应过程。 |

CSV 会记录 `target_mode`、`target_phase`、`target_joint`，并同时记录 `target_j00_relative_deg` 到 `target_j20_relative_deg` 的完整目标向量。`relative_deg` 是历史字段名，现在等同于机械零位坐标下的 degree。建议一次采集只固定一个 mode，后续分析时按 `target_mode` 分组。

## 双串口检测

```powershell
& 'C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' .\desktop\test_dual_serial_detection.py --force-port COM7 --seconds 6
```

该脚本会自动扫描除 COM7 外的串口，并同时检测力传感器、Encoder 和 Servo 数据。
