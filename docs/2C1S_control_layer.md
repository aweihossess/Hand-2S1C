# 2C1S 版本控制层说明

## 1. 当前 GitHub 版本

当前最新实验分支：

```text
codex/tendon-ff-angle-pid-integral
```

当前提交：

```text
c2210fc Tune angle PID integral control
```

该分支是在 `codex/tendon-feedforward-angle-pid` 的基础上继续调参得到的版本。核心特征是：

```text
绳长模型做前馈
2x2 角度 PID 做反馈
J01 积分项增强
```

历史相关分支：

| 分支 | 用途 |
| --- | --- |
| `codex/direct-joint-cleanup` | 直驱/关节控制串口流程清理版本 |
| `codex/tendon-length-pd` | 绳长模型 + 绳长 PD 反馈版本 |
| `codex/tendon-feedforward-angle-pid` | 绳长前馈 + 角度 PID 反馈初版 |
| `codex/tendon-ff-angle-pid-integral` | 当前版本，增强 J01 积分项 |

## 2. 2C1S 控制对象

2C1S 在当前上下文中指：

```text
2C = 两个控制自由度 / 两个关节角度
1S = 一组双舵机腱绳驱动系统
```

当前主要控制对象为 MCP 关节组：

```text
J00 = MCP-AA
J01 = MCP-FE
M00 = model R tendon
M01 = model L tendon
```

其中：

- `J00/J01` 是磁编码器反馈得到的关节角度。
- `M00/M01` 是两个舵机对应的电机相对位置。
- 两个舵机和两个角度不是一一独立关系，而是强耦合关系。

即：

```text
M00 会影响 J00，也会影响 J01
M01 会影响 J00，也会影响 J01
```

因此当前控制器采用 2x2 角度反馈矩阵，而不是两个互相独立的一维 PID。

## 3. 控制层输入输出

### 3.1 输入

控制层主要输入包括：

| 输入 | 来源 | 含义 |
| --- | --- | --- |
| `targetAngles[]` | 串口命令 / 上位机 | 目标关节角度 |
| `magAngles[]` | CAN 磁编码器 | 当前关节角度反馈 |
| `servoAngles[]` | 舵机反馈 | 当前 motor_abs |
| `softwareZeroOffsets[]` | 舵机零位表 | 舵机软件零位 |
| 系统状态 | StateMachineTask | START/STOP/RESET、owner、fault |

### 3.2 输出

控制层输出为舵机目标：

```text
motorTarget
```

该目标是 motor_abs 坐标下的位置命令。

舵机硬件目标为：

```text
hardwareTarget = motorTarget + sw_zero_ofs
```

其中：

```text
motor_abs = hardware_abs - sw_zero_ofs
```

## 4. 零位定义

当前系统存在两个不同零位，不能混淆。

### 4.1 舵机软件零位

串口命令：

```text
zero
```

作用：

```text
将当前舵机 hardware_abs 记录为 sw_zero_ofs
使当前 motor_abs 变为 0
```

公式：

```text
motor_abs = hardware_abs - sw_zero_ofs
```

因此 `zero` 后通常应看到：

```text
motor_abs = 0
hardware_abs = sw_zero_ofs
```

注意：`zero` 不会改变磁编码器角度零位。

### 4.2 磁编码器角度零位

磁编码器零位当前写在代码表中：

```text
ESP32-P4/ServoBoardMain/src/calibration/CalibrationTask.cpp
```

表名：

```cpp
kManualZeroRaw[]
```

当前版本中 J00/J01 的手动零位为：

```text
J00 raw zero = 4698
J01 raw zero = 1379
```

如果要重新定义磁编码器零位，应：

1. 将机构摆到机械零位。
2. 串口发送 `encoder`。
3. 读取对应 `Jxx raw=...`。
4. 将该 raw 填入 `kManualZeroRaw[]`。
5. 重新编译烧录。

## 5. 控制模式

当前固件主要支持三种状态/模式：

| 模式 | 命令 | 用途 |
| --- | --- | --- |
| 空闲 | `idle` | 不主动控制 |
| 直驱 | `direct` | 直接给 M00/M01 motor_abs 目标 |
| 角度控制 | `degree` | 给 J00/J01 目标角度 |

进入控制前需要：

```text
start
```

停止控制：

```text
stop
```

清空状态：

```text
reset
```

## 6. 直驱模式

直驱模式用于验证舵机方向、零位和运动是否正常。

典型流程：

```text
text
stop
zero
servo
start
direct
m0 100
m0 0
m1 100
m1 0
```

直驱目标单位为 motor_abs counts：

```text
m0 100
m0 -100
m1 200
```

直驱模式不经过角度控制器。

## 7. 角度控制模式

角度控制模式用于控制 J00/J01。

典型流程：

```text
text
stop
zero
servo
encoder
start
degree
j0 5
j0 10
j1 10
j1 20
```

当前目标限制：

```text
J00 / MCP-AA: -20° ~ +30°
J01 / MCP-FE: -20° ~ +80°
```

如果目标超过范围，下位机会进行 clamp 并打印 applied target。

## 8. 当前控制算法

当前控制思路为：

```text
绳长模型做前馈
角度误差 PID 做反馈
```

### 8.1 总体流程

```text
1. 接收 J00/J01 目标角度
2. 读取磁编码器得到 J00/J01 实际角度
3. 用 MCP 几何模型计算目标绳长
4. 用目标绳长与模型零位绳长计算前馈 motor target
5. 直接计算角度误差
6. 用 2x2 角度 PID 矩阵计算反馈修正
7. 前馈 + 反馈得到 motor_abs target
8. 经过单周期限步和安全限幅
9. 下发给 M00/M01
```

### 8.2 前馈项

前馈来自绳长模型：

```text
targetLen = tendonModel(J00_target, J01_target)
zeroLen = tendonModel(0, 0)
feedforward = (targetLen - zeroLen) * lengthToPulse
```

当前：

```text
lengthToPulse = -160 counts/mm
```

负号来自当前电机安装方向和绕线方向。

### 8.3 角度反馈项

角度误差：

```text
e00 = J00_target - J00_actual
e01 = J01_target - J01_actual
```

反馈矩阵：

```text
M00_feedback = P00*e00 + P01*e01 + I00*∫e00 + I01*∫e01 + D00*de00 + D01*de01
M01_feedback = P10*e00 + P11*e01 + I10*∫e00 + I11*∫e01 + D10*de00 + D11*de01
```

这就是 2x2 角度 PID。其含义是：

```text
两个角度误差共同决定两个电机如何补偿
```

### 8.4 当前 PID 参数

当前 P 矩阵：

```text
          J00 error   J01 error
M00/R P   -20          +30
M01/L P   +20          +30
```

当前 I 矩阵：

```text
          J00 error   J01 error
M00/R I   -1           +20
M01/L I   +1           +20
```

当前 D 矩阵：

```text
          J00 error   J01 error
M00/R D    0            0
M01/L D    0            0
```

积分上限：

```text
J00 integral = ±30 deg*s
J01 integral = ±60 deg*s
```

反馈输出限幅：

```text
M00 feedback = ±1200 counts
M01 feedback = ±1200 counts
```

## 9. 安全限制

当前主要安全限制：

| 保护 | 当前值 | 作用 |
| --- | --- | --- |
| J00 范围 | -20° ~ +30° | MCP-AA 角度保护 |
| J01 范围 | -20° ~ +80° | MCP-FE 角度保护 |
| tracking error | 30° | 目标角度与实际角度差过大则保护 |
| motor_abs guard | ±6400 counts | MCP 电机相对位置保护 |
| command step | 320 counts/update | 单周期目标限步 |
| angle feedback limit | ±1200 counts | 角度反馈输出限制 |

日志示例：

```text
[JOINT SAFETY] MCP encoder out of range ...
[JOINT SAFETY] tracking error out of range ...
[JOINT SAFETY] MCP motor abs out of range ...
```

## 10. 关于 I 项风险

当前版本增强了 J01 的积分项，目的是减少稳态误差。

但积分项有风险：

```text
如果手指遇到障碍，角度无法到达目标，误差会持续存在；
I 项会继续累积；
电机会继续增加拉力；
可能导致绳子、舵机齿轮或机构受损。
```

因此当前版本适合用于台架调试，不建议直接用于强力抓取。

后续建议加入：

1. 积分抗饱和。
2. 堵转检测。
3. 舵机 load/current/temperature 保护。
4. 遇障碍时冻结积分或进入 hold。

## 11. 日志解释

典型 MCP 日志：

```text
[MCP CTRL] J00 target=10.00 actual=10.52 J01 target=20.00 actual=16.83 M00/R targetLen=25.145 actualLen=25.597 mappedMotor=328.1 solver=328 cmd=328 M01/L targetLen=22.500 actualLen=22.797 mappedMotor=670.6 solver=670 cmd=670 [SERVO TARGET] ...
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `J00 target/actual` | J00 目标/实际角度 |
| `J01 target/actual` | J01 目标/实际角度 |
| `targetLen` | 目标角度通过模型算出的目标绳长 |
| `actualLen` | 实际角度通过模型反算的诊断绳长 |
| `mappedMotor` | 前馈 + 角度反馈后的 motor_abs 目标 |
| `solver` | solver 输出取整 |
| `cmd` | 经过限步和保护后的实际下发目标 |
| `motorTarget` | 下发给舵机层的 motor_abs 目标 |
| `swZero` | 舵机软件零位 |
| `hardwareTarget` | motorTarget + swZero |
| `motorAbsNow` | 当前 motor_abs |
| `hardwareAbsNow` | 当前硬件多圈位置 |

## 12. 当前调试建议

推荐测试顺序：

```text
text
stop
zero
servo
encoder
start
degree
j0 5
j0 10
j1 10
j1 20
```

观察重点：

1. J00/J01 actual 是否朝 target 收敛。
2. mappedMotor 是否持续变化。
3. 是否触发 JOINT SAFETY。
4. 是否出现积分导致的持续拉紧。

如果遇到不确定情况，优先执行：

```text
stop
```

