# 力传感器数据监视器

## 协议依据

- 官方上位机当前使用 `COM7 / 115200 / 8N1 / 从站地址 1`。
- 重量数据使用 Modbus-RTU `03` 功能码读取保持寄存器。
- 每个通道占 2 个寄存器，32 位有符号整数，低 16 位寄存器在前。
- CH1 起始寄存器为 `0x0000`，CH5 起始寄存器为 `0x0008`。
- CH5 读取命令为 `01 03 00 08 00 02 45 C9`。

## 启动

```powershell
& 'C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' .\desktop\force_sensor_monitor.py --port COM7 --servo-port COMx
```

其中 `COM7` 为力传感器串口，`COMx` 替换为 Encoder / Servo 控制板实际串口。

## 使用说明

1. 打开前先关闭官方上位机，避免 COM7 被占用。
2. 软件默认读取 CH1 到 CH5。
3. 实时数据区显示 CH1-CH5 的当前值和相对力，换算关系为 `N = 相对值 / 10`。
4. 图表同时显示 CH1-CH5 的相对力曲线，单位为 N。
5. 松开电机，让力传感器放在桌面且无拉力后，点击“记录零位”，当前 CH1-CH5 原始值会保存为无拉力零位。
6. 保存的零位写入 `desktop/config/force_sensor_zero.json`，下次打开上位机会自动加载。
7. “清除零位”会删除保存的零位；“清空”只清除记录，不会清除零位。
8. “设备去皮”写寄存器 `0x0015=1`；“取消去皮”写 `0x0015=2`。
9. CSV 会导出 `channel`、`relative_value` 和 `relative_force_n` 字段，方便区分通道并保留原始相对值。

## Encoder / Servo 界面

1. 主界面点击 `Encoder / Servo` 打开第二个界面。
2. 连接另一个串口后，界面默认请求二进制遥测，实时显示 21 路 Encoder 角度曲线。
3. Servo 数据只在数字表格中显示，包括 `Abs`、`HwAbs`、`ZeroOfs`、`Raw`、`Speed`、`Load`、电压、温度和在线状态。
4. 点击“记录角度零点”会把当前有效 Encoder 角度保存为零点，写入 `desktop/config/encoder_zero.json`。
5. 保存角度零点后，Encoder 表格和曲线的 `Deg` 显示为相对零点角度。
6. `degree` 模式输入的是相对零点的 joint 目标角度，上位机会自动加回零点偏置后发送给下位机。
7. `direct` 模式发送 `direct; m索引 目标位置`，用于 servo abs position 目标值。
8. `START`、`STOP`、`ZERO`、`ENCODER`、`SERVO`、`STATUS` 按钮沿用 Arduino Serial Monitor 的文本命令流程。

## 双串口检测

```powershell
& 'C:\Users\A\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' .\desktop\test_dual_serial_detection.py --force-port COM7 --seconds 6
```

该脚本会自动扫描除 COM7 外的串口，并同时检测力传感器、Encoder 和 Servo 数据。
