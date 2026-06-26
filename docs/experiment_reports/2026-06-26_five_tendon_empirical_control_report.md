# 2026-06-26 五绳 MCP 实测前馈与控制调参报告

版本检查点：`4aa3352 Checkpoint five-tendon empirical control tuning`  
分支：`codex/five-tendon`  
硬件口：ESP32-P4 on `COM6`

## 1. 实验目的

本轮实验目标是用实测数据替代旧机械结构 LUT，建立并验证 MCP 五绳系统的绳长前馈控制：

- 采集 `tension on` 下的关节角度、舵机相对零位位置、load、bias。
- 使用 `now - bias` 作为基础绳长估计，保留运行时 tension bias 作为保底张力。
- 在真实控制中使用“前馈舵机位置 + PID 修正 + tension bias”的方式控制 J0-J3。
- 通过阶跃响应和 J2 正弦跟踪观察稳态误差、耦合和迟滞。

## 2. 当前固件控制方式

当前固件使用实测线性前馈：

```cpp
static const bool kUseEmpiricalMcpFeedforwardCounts = true;
static const float kEmpiricalMcpMotorPerDeg[5][4] = {
    { 19.525f,  11.619f,   0.848f,  -8.994f},
    {-13.384f,  13.533f,  -7.042f,   0.826f},
    { -5.835f,  -9.028f,  38.401f, -14.173f},
    { 24.708f,  -7.853f, -18.443f,  29.339f},
    {  5.597f, -30.330f, -54.012f, -30.450f}
};
```

当前 PID 矩阵：

```cpp
static const float kMcpAngleKi[5][4] = {
    { 0.90f,  0.60f,   0.00f,   0.00f},
    {-0.90f,  0.60f,   0.00f,   0.00f},
    { 0.00f, -0.60f,   0.90f,   0.00f},
    { 0.30f, -0.525f, -0.90f,   2.40f},
    { 0.00f,  0.08f,  -0.30f,   0.00f}
};

static const float kMcpAngleKd[5][4] = {
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.02f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};
```

控制流程：

1. 先用纯 PID + `tension on` 回到实际零位附近。
2. 执行 `stop; zero`，标定舵机相对零位。
3. 执行 `ff on; tension on; start; degree`。
4. 下发阶跃或正弦目标。

## 3. 核心结论

1. 实测前馈 + tension bias 可以让单 J0 阶跃基本到位，但被动关节尤其 J3 会明显偏移。
2. 对 J3 增加 M3/M4 的积分能改善某些单独目标，但会破坏 J2/J3 组合目标，说明当前五绳全耦合 PID 矩阵存在方向冲突。
3. 当前较好的组合测试点是 `mcp_control_20260626_090614`：目标 `(0,0,30,15)`，尾部约 `(-1.05, 0.94, 32.95, 13.72)`。
4. 单独 J3 目标 `(0,0,0,15)` 在当前参数下只能到约 `7.82 deg`，说明 J3 通道仍然缺少有效的单独驱动力。
5. J2 正弦跟踪出现明显迟滞和偏置。密集采样版本 `20 + 5 sin(2*pi*0.2t)` 中，J2 实际范围为 `-0.88..31.25 deg`，RMSE `10.51 deg`。
6. 正弦测试中 J3 被拉到约 `-12 deg` 附近，说明 J2/J3 的绳路耦合仍是主要问题。
7. 下一步值得尝试“4 根绳控制 4 自由度，第 5 根仅作为冗余预紧绳”：M0-M3 控制姿态，M4 去掉前馈和角度 PID，只保留 tension bias。

## 4. 实验摘要表

表中 step 的 Tail mean 是最后 10 秒实际角度均值；Tail err = target - Tail mean。sine 的误差只统计正弦段，排除最后复位到 0 的行。

| Run | 类型 | 目标/目标范围 | 尾部均值/实际范围 | 误差摘要 | 数据量 |
|---|---|---:|---:|---:|---:|
| `mcp_control_20260626_040007` | step | `(10.00, 0.00, 0.00, 0.00)` | `(10.13, 0.13, -0.63, -4.77)` | `(-0.13, -0.13, 0.63, 4.77)` | 98 rows |
| `mcp_control_20260626_042232` | step | `(10.00, 0.00, 0.00, 0.00)` | `(8.62, 1.95, -0.36, -5.02)` | `(1.38, -1.95, 0.36, 5.02)` | 112 rows |
| `mcp_control_20260626_043004` | step | `(10.00, 0.00, 0.00, 0.00)` | `(9.26, -1.02, -0.20, 0.00)` | `(0.74, 1.02, 0.20, -0.00)` | 71 rows |
| `mcp_control_20260626_043442` | step | `(10.00, 5.00, 20.00, 0.00)` | `(7.18, 5.52, 19.78, 0.14)` | `(2.82, -0.52, 0.22, -0.14)` | 129 rows |
| `mcp_control_20260626_044310` | step | `(-10.00, 5.00, 30.00, 15.00)` | `(-12.79, 6.55, 29.53, 25.54)` | `(2.79, -1.55, 0.47, -10.54)` | 134 rows |
| `mcp_control_20260626_045436` | step | `(0.00, 0.00, 0.00, 15.00)` | `(0.04, 0.16, 0.25, 6.90)` | `(-0.04, -0.16, -0.25, 8.10)` | 143 rows |
| `mcp_control_20260626_045718` | step | `(0.00, 0.00, 30.00, 15.00)` | `(0.56, 0.53, 36.65, 9.71)` | `(-0.56, -0.53, -6.65, 5.29)` | 158 rows |
| `mcp_control_20260626_050406` | step | `(0.00, 0.00, 0.00, 15.00)` | `(-1.45, 1.84, -0.41, 8.02)` | `(1.45, -1.84, 0.41, 6.98)` | 135 rows |
| `mcp_control_20260626_050644` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-1.95, 4.60, 27.80, 15.86)` | `(1.95, -4.60, 2.20, -0.86)` | 159 rows |
| `mcp_control_20260626_051501` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-3.02, -0.98, 33.68, 22.30)` | `(3.02, 0.98, -3.68, -7.30)` | 141 rows |
| `mcp_control_20260626_051948` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-0.63, 1.06, 34.63, 12.65)` | `(0.63, -1.06, -4.63, 2.35)` | 134 rows |
| `mcp_control_20260626_084743` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-2.58, -0.39, 32.87, 19.16)` | `(2.58, 0.39, -2.87, -4.16)` | 150 rows |
| `mcp_control_20260626_085237` | step | `(0.00, 0.00, 30.00, 15.00)` | `(0.15, 0.59, 34.99, 16.19)` | `(-0.15, -0.59, -4.99, -1.19)` | 145 rows |
| `mcp_control_20260626_085703` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-1.04, 1.30, 33.10, 14.19)` | `(1.04, -1.30, -3.10, 0.81)` | 134 rows |
| `mcp_control_20260626_090134` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-1.31, 0.08, 33.06, 11.34)` | `(1.31, -0.08, -3.06, 3.66)` | 136 rows |
| `mcp_control_20260626_090614` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-1.05, 0.94, 32.95, 13.72)` | `(1.05, -0.94, -2.95, 1.28)` | 135 rows |
| `mcp_control_20260626_090928` | step | `(0.00, 0.00, 0.00, 15.00)` | `(1.06, 1.54, 0.24, 7.82)` | `(-1.06, -1.54, -0.24, 7.18)` | 136 rows |
| `mcp_control_20260626_091421` | step | `(0.00, 0.00, 0.00, 15.00)` | `(-3.28, -1.27, -0.04, 15.88)` | `(3.28, 1.27, 0.04, -0.88)` | 140 rows |
| `mcp_control_20260626_091717` | step | `(0.00, 0.00, 30.00, 15.00)` | `(-0.70, 0.93, 34.97, 11.17)` | `(0.70, -0.93, -4.97, 3.83)` | 131 rows |
| `mcp_control_20260626_095428` | sine J2 | `5.03..24.97` | `1.96..32.92` | `RMSE 6.14, mean err -3.99` | 60 rows / 59.0s |
| `mcp_control_20260626_095935` | sine J2 | `35.00..44.25` | `0.92..54.73` | `RMSE 16.74, mean err -1.79` | 20 rows / 19.0s |
| `mcp_control_20260626_100219` | sine J2 | `15.01..24.41` | `-1.38..26.19` | `RMSE 9.75, mean err 4.32` | 20 rows / 19.0s |
| `mcp_control_20260626_101201` | sine J2 | `15.01..25.00` | `-0.88..31.25` | `RMSE 10.51, mean err -3.77` | 200 rows / 19.9s |

## 5. 图表汇总

### 5.1 J0 单阶跃与早期 J3 积分调整

#### `mcp_control_20260626_040007`

![mcp_control_20260626_040007](assets/2026-06-26_five_tendon/mcp_control_20260626_040007_joint_step_response.png)

#### `mcp_control_20260626_042232`

![mcp_control_20260626_042232](assets/2026-06-26_five_tendon/mcp_control_20260626_042232_joint_step_response.png)

#### `mcp_control_20260626_043004`

![mcp_control_20260626_043004](assets/2026-06-26_five_tendon/mcp_control_20260626_043004_joint_step_response.png)

### 5.2 多关节阶跃

#### `mcp_control_20260626_043442`

![mcp_control_20260626_043442](assets/2026-06-26_five_tendon/mcp_control_20260626_043442_joint_step_response.png)

#### `mcp_control_20260626_044310`

![mcp_control_20260626_044310](assets/2026-06-26_five_tendon/mcp_control_20260626_044310_joint_step_response.png)

### 5.3 J3 单独目标与 J2/J3 组合目标

#### `mcp_control_20260626_045436`

![mcp_control_20260626_045436](assets/2026-06-26_five_tendon/mcp_control_20260626_045436_joint_step_response.png)

#### `mcp_control_20260626_045718`

![mcp_control_20260626_045718](assets/2026-06-26_five_tendon/mcp_control_20260626_045718_joint_step_response.png)

#### `mcp_control_20260626_050406`

![mcp_control_20260626_050406](assets/2026-06-26_five_tendon/mcp_control_20260626_050406_joint_step_response.png)

#### `mcp_control_20260626_050644`

![mcp_control_20260626_050644](assets/2026-06-26_five_tendon/mcp_control_20260626_050644_joint_step_response.png)

#### `mcp_control_20260626_051501`

![mcp_control_20260626_051501](assets/2026-06-26_five_tendon/mcp_control_20260626_051501_joint_step_response.png)

#### `mcp_control_20260626_051948`

![mcp_control_20260626_051948](assets/2026-06-26_five_tendon/mcp_control_20260626_051948_joint_step_response.png)

### 5.4 当前候选参数附近的组合目标扫描

#### `mcp_control_20260626_084743`

![mcp_control_20260626_084743](assets/2026-06-26_five_tendon/mcp_control_20260626_084743_joint_step_response.png)

#### `mcp_control_20260626_085237`

![mcp_control_20260626_085237](assets/2026-06-26_five_tendon/mcp_control_20260626_085237_joint_step_response.png)

#### `mcp_control_20260626_085703`

![mcp_control_20260626_085703](assets/2026-06-26_five_tendon/mcp_control_20260626_085703_joint_step_response.png)

#### `mcp_control_20260626_090134`

![mcp_control_20260626_090134](assets/2026-06-26_five_tendon/mcp_control_20260626_090134_joint_step_response.png)

#### `mcp_control_20260626_090614`

![mcp_control_20260626_090614](assets/2026-06-26_five_tendon/mcp_control_20260626_090614_joint_step_response.png)

#### `mcp_control_20260626_090928`

![mcp_control_20260626_090928](assets/2026-06-26_five_tendon/mcp_control_20260626_090928_joint_step_response.png)

#### `mcp_control_20260626_091421`

![mcp_control_20260626_091421](assets/2026-06-26_five_tendon/mcp_control_20260626_091421_joint_step_response.png)

#### `mcp_control_20260626_091717`

![mcp_control_20260626_091717](assets/2026-06-26_five_tendon/mcp_control_20260626_091717_joint_step_response.png)

### 5.5 J2 正弦跟踪

#### `mcp_control_20260626_095428`: J2 低频大振幅正弦

![mcp_control_20260626_095428](assets/2026-06-26_five_tendon/mcp_control_20260626_095428_j2_sine_response.png)

#### `mcp_control_20260626_095935`: 初始正弦尝试

![mcp_control_20260626_095935](assets/2026-06-26_five_tendon/mcp_control_20260626_095935_j2_sine_response.png)

#### `mcp_control_20260626_100219`: `J2 = 20 + 5 sin(2*pi*0.2t)`，低采样图

![mcp_control_20260626_100219](assets/2026-06-26_five_tendon/mcp_control_20260626_100219_j2_sine_response.png)

#### `mcp_control_20260626_101201`: `J2 = 20 + 5 sin(2*pi*0.2t)`，密集采样通用图

![mcp_control_20260626_101201_dense_response](assets/2026-06-26_five_tendon/mcp_control_20260626_101201_j2_sine_response_dense.png)

#### `mcp_control_20260626_101201`: `J2 = 20 + 5 sin(2*pi*0.2t)`，专用正弦跟踪图

![mcp_control_20260626_101201_dense_tracking](assets/2026-06-26_five_tendon/mcp_control_20260626_101201_j2_sine_tracking_dense.png)

## 6. 对迟滞与耦合的解释

J2 正弦测试出现的迟滞和偏置，主要不是绘图问题，而是系统本身的动态特性：

- 绳驱系统只能拉不能推，回程依赖对侧绳、张力、机构弹性和摩擦释放。
- 同一舵机位置在收绳和放绳方向上对应的关节角度不同，存在方向相关迟滞。
- tension bias 会随 load 动态改变，相当于运行时等效绳长在变化。
- 当前前馈是静态角度到舵机位置模型，没有速度项和加速度项。
- J2 动作会显著拉动 J3，说明当前矩阵把 J2/J3 的耦合分离得还不够。

## 7. 下一轮建议

1. 先实现 4+1 控制模式：M0-M3 保持姿态控制，M4 的前馈和角度 PID 清零，仅保留 tension bias。
2. 先测 `(0,0,0,15)` 和 `(0,0,30,15)`，判断 J3 单独控制和 J2/J3 组合是否改善。
3. 正弦测试改成先 settle 到 `J2=20`，再启动 `20 + 5 sin(2*pi*0.2t)`，并丢弃第一个周期。
4. 分别测试 `0.05Hz / 0.1Hz / 0.2Hz`，区分机械迟滞和控制带宽问题。
5. 如果 4+1 模式稳定，再对 M0-M3 的 4x4 前馈/PID 做新一轮矩阵辨识。
