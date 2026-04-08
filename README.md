<a id="english"></a>

# SepFOC

**Language:** English | [中文](#chinese)

Basic FOC complete edition with sensorless expansion. Current version: `0.04`

SepFOC is a self-built FOC project based on `STM32G431CBU6`, `STM32CubeMX`, `HAL`, and `Keil MDK`. It started from the basic signal chain of three-phase PWM, encoder angle reading, and phase-current sampling, and gradually evolved into a unified control framework that supports sensored single-loop control, cascaded double-loop control, triple-loop control, function-style haptic modes, and a practical sensorless FOC branch.

---

## Overview

- MCU: `STM32G431CBU6`
- Development flow: `CubeMX + HAL + Keil MDK`
- Encoder: `MT6701`
- PWM frequency: `16 kHz`
- Speed calculation frequency: `4 kHz`
- DC bus voltage config: `12 V`
- Current sensing: two-shunt, two-phase sampling with reconstructed third phase
- Debug tools: serial port + `VOFA+ FireWater`

---

## Implemented Features

- `SVPWM / inverse Park / three-phase PWM` output chain
- Encoder single-turn angle acquisition and electrical angle conversion
- Phase-current sampling, offset compensation, `Iq` calculation, and filtering
- Pure voltage torque mode
- Pure current torque mode
- Pure velocity loop
- Pure position loop
- Position-velocity cascade
- Position-current cascade
- Velocity-current cascade
- Position-velocity-current triple-loop control
- Function-style haptic modes:
  `X1` pure damping feel, `X2` detent feel
- Sensorless FOC speed mode:
  `W0`
- Runtime VOFA debug view switching:
  `V0 ~ V5`
- Overcurrent detection and latch protection

---

## Control Modes

### Sensored FOC modes

The mode order matches the `SepFocControlMode` enum:

- `M0`: output disabled
- `M1`: pure torque control in voltage mode, unit `V`
- `M2`: pure torque control in current mode, unit `A`
- `M3`: pure position loop, unit `deg`
- `M4`: pure velocity loop, unit `rad/s`
- `M5`: position-velocity cascade
- `M6`: position-current cascade
- `M7`: velocity-current cascade
- `M8`: position-velocity-current triple-loop control

All position-related sensored modes use absolute angle targets in the `0~360` range.

### Function / haptic modes

- `X0`: exit function mode
- `X1`: pure damping feel
- `X2`: detent feel

### Sensorless modes

- `W0`: sensorless FOC speed mode

---

## Loop Structure

### Single-loop modes

- Pure voltage torque: target voltage -> `Uq`
- Pure current torque: target `Iq` -> current PI -> `Uq`
- Pure velocity loop: velocity PID -> `Uq`
- Pure position loop: position PID -> `Uq`

### Double-loop cascade modes

- Position-velocity: position PID -> target velocity, velocity PID -> `Uq`
- Position-current: position PID -> target current, current PI -> `Uq`
- Velocity-current: velocity PID -> target current, current PI -> `Uq`

### Triple-loop mode

- Position-velocity-current: position PID -> target velocity, velocity PID -> target current, current PI -> `Uq`

### Sensorless branch

The current sensorless branch (`W0`) is built around:

- startup alignment
- open-loop acceleration
- `SMO + PLL` back-EMF observer
- transition preparation for closed-loop takeover
- staged target slew
- staged open-loop acceleration
- open-loop current foldback

---

## Scheduling Structure

- Current inner loop: executed inside the `ADC DMA` callback
- Velocity loop / cascade middle loop: executed by `TIM7`
- Position loop / cascade outer loop: executed by `TIM7` with divider control
- Sensorless fast loop: synchronized with ADC/PWM
- Sensorless slow loop: executed at `1 kHz`

This keeps the typical layered structure:

- fast inner loop
- slower middle loop
- slowest outer loop

---

## Serial Protocol

The project now supports runtime mode switching, function switching, sensorless switching, protection reset, and runtime VOFA view switching:

```text
M0~n   -> switch sensored control mode
Fxxx   -> write the unified target value for the current mode
X0~n   -> switch function / haptic mode
W0~n   -> switch sensorless mode
V0~n   -> switch VOFA debug view
C0     -> clear latched protection / sensorless fault
```

Example:

```text
M8
F180
X1
F0.02
W0
F80
V5
```

Meaning:

- `M8`: switch to triple-loop sensored mode
- `F180`: set target absolute angle to `180°`
- `X1`: switch to pure damping mode
- `F0.02`: set damping coefficient
- `W0`: switch to sensorless speed mode
- `F80`: set sensorless target speed to `80 rad/s`
- `V5`: switch VOFA to the sensorless diagnostic page

Typical feedback examples:

```text
# MODE OK: M8 -> PositionVelocityCurrent
# FUNC OK: X1 -> PureDamping
# SENSORLESS OK: W0 -> Speed
# VOFA OK: V5 -> Sensorless
```

---

## VOFA+ Debug

The project uses `VOFA+ FireWater` text frames. Debug content is switched at runtime by serial `V0~V5`, so recompilation is not required just to change the chart content.

Recommended pages:

- `V0`: default page, target / speed / `Iq` / `Uq` / angle
- `V1`: position-velocity cascade tuning
- `V2`: position-current cascade tuning
- `V3`: velocity-current cascade tuning
- `V4`: triple-loop tuning
- `V5`: sensorless startup and observer diagnostics

The current `V5` page is intended for startup tuning and includes:

- state
- target speed
- open-loop speed
- open-loop voltage
- staged acceleration
- event code
- reset count
- measured speed

This page is especially useful for checking:

- startup responsiveness
- staged acceleration behavior
- current foldback intervention
- whether sensorless control is still staying in open loop

---

## Sensorless FOC Notes

`W0` is the current sensorless speed mode.

Typical workflow:

```text
V5
C0
W0
F40
F60
F80
```

Key points:

- `F0` stops the sensorless branch and returns it to idle
- startup response now uses shorter alignment time plus a faster restart alignment path
- speed rise now uses staged target slew and staged open-loop acceleration
- open-loop current foldback is used to reduce direct overcurrent trips in high-speed startup

Compared with sensored FOC, low-speed sensorless behavior is naturally more conservative because the observer must first build reliable conditions before takeover becomes possible.

---

## Code Structure

### `Core/Hardware`

- `SepFOC.c / SepFOC.h`
  Main sensored FOC framework, loop implementations, parameters, and debug getters
- `MT6701.c / MT6701.h`
  Magnetic encoder reading and angle processing
- `Vofa.c / Vofa.h`
  VOFA debug frames, runtime view switching, and serial feedback
- `Function.c / Function.h`
  Damping feel and detent feel function modes
- `SensorlessFOC.c / SensorlessFOC.h`
  Sensorless startup state machine, control flow, and debug getters
- `BemfObserver.c / BemfObserver.h`
  SMO + PLL observer
- `SensorlessConfig.h`
  Sensorless configuration and tuning macros
- `config.h`
  Global parameters and mode enums

### `Core/Src`

- `main.c`
  Initialization and timed scheduling entry
- `adc.c`
  Current sampling, filtering, ADC callback, and fast-loop entry
- `usart.c`
  Serial RX/TX and `M/F/X/W/V/C` command parsing
- `tim.c`
  PWM and speed-calculation timer configuration

---

## Tuning Recommendation

For sensored modes:

1. Tune the current loop first
2. Then tune the velocity loop
3. Finally tune the position loop

Recommended order:

- `M2` -> current loop
- `M4` -> velocity loop
- `M7` -> velocity-current cascade
- `M8` -> full triple-loop control

For sensorless mode:

1. Verify startup alignment response
2. Tune low-speed startup and transition behavior
3. Tune high-speed overcurrent margin
4. Then continue improving observer takeover

---

## Version Notes

### `0.01`

- Entry-level FOC version without cascaded PID

### `0.02`

- Basic FOC complete edition
- Single-loop, double-loop cascade, and triple-loop control integrated
- Unified serial mode switching and target input completed
- Multiple VOFA debug frame types completed
- Triple-loop `M8` hold deadband added near the target position

### `0.03`

- Added overcurrent detection and latch protection
- Added runtime VOFA debug view switching

### `0.04`

- Added sensorless FOC `W0` mode
- Added `SMO + PLL` observer and sensorless startup state machine
- Added `V5` sensorless diagnostic page
- Added staged target slew, staged open-loop acceleration, fast restart alignment, and open-loop current foldback

---

<a id="chinese"></a>

# SepFOC

**语言切换：** [English](#english) | 中文

基础 FOC 完全体并扩展无感分支版本，当前版本号：`0.04`

SepFOC 是一个基于 `STM32G431CBU6`、`STM32CubeMX`、`HAL`、`Keil MDK` 搭建的自制 FOC 工程。项目从最基础的三相 PWM 输出、编码器角度读取、相电流采样开始，逐步扩展成了一个统一控制框架，同时支持有感单环、双环串级、三环控制、Function 手感模式以及可用的无感 FOC 分支。

---

## 概述

- MCU：`STM32G431CBU6`
- 开发方式：`CubeMX + HAL + Keil MDK`
- 编码器：`MT6701`
- PWM 频率：`16 kHz`
- 速度计算频率：`4 kHz`
- 当前母线电压配置：`12 V`
- 电流采样：双电阻两相采样，第三相重构
- 调试方式：串口 + `VOFA+ FireWater`

---

## 当前已实现内容

- `SVPWM / 反 Park / 三相 PWM` 输出链路
- 编码器单圈角度读取与电角度换算
- 相电流采样、零点补偿、`Iq` 计算与滤波
- 纯电压力矩模式
- 纯电流力矩模式
- 纯速度闭环
- 纯位置闭环
- 位置-速度串级环
- 位置-电流串级环
- 速度-电流串级环
- 位置-速度-电流三环控制
- Function 手感模式：
  `X1` 纯阻尼感，`X2` 定格感
- 无感 FOC 速度模式：
  `W0`
- 运行时切换的 VOFA 调试页：
  `V0 ~ V5`
- 过流检测与锁存保护

---

## 控制模式

### 有感 FOC 模式

控制模式顺序与 `SepFocControlMode` 枚举一致：

- `M0`：停止输出
- `M1`：目标电压的纯力矩环，单位 `V`
- `M2`：目标电流的纯力矩环，单位 `A`
- `M3`：纯位置闭环，单位 `deg`
- `M4`：纯速度闭环，单位 `rad/s`
- `M5`：位置-速度串级环
- `M6`：位置-电流串级环
- `M7`：速度-电流串级环
- `M8`：位置-速度-电流三环控制

所有位置类有感模式统一使用 `0~360` 的绝对角度目标。

### Function / 手感模式

- `X0`：退出 Function 模式
- `X1`：纯阻尼感
- `X2`：定格感

### 无感模式

- `W0`：无感 FOC 速度模式

---

## 环路结构

### 单环

- 纯电压力矩环：目标电压 -> `Uq`
- 纯电流力矩环：目标 `Iq` -> 电流 PI -> `Uq`
- 纯速度环：速度 PID -> `Uq`
- 纯位置环：位置 PID -> `Uq`

### 双环

- 位置-速度环：位置 PID -> 目标速度，速度 PID -> `Uq`
- 位置-电流环：位置 PID -> 目标电流，电流 PI -> `Uq`
- 速度-电流环：速度 PID -> 目标电流，电流 PI -> `Uq`

### 三环

- 位置-速度-电流环：位置 PID -> 目标速度，速度 PID -> 目标电流，电流 PI -> `Uq`

### 无感分支

当前无感分支 `W0` 内部包含：

- 对齐启动
- 开环拉升
- `SMO + PLL` 反电动势观测器
- 向闭环接管过渡的状态机
- 分段目标斜坡
- 分段开环加速度
- 开环电流折返

---

## 调度结构

- 电流内环：在 `ADC DMA` 回调中执行
- 速度环 / 串级中环：由 `TIM7` 固定节拍执行
- 位置环 / 串级外环：由 `TIM7` 通过分频执行
- 无感快环：与 ADC/PWM 同步
- 无感慢环：`1 kHz`

目标很明确：

- 最内层快
- 中间层次之
- 最外层最慢

---

## 串口协议

当前支持运行时模式切换、Function 模式、无感模式、保护清除和 VOFA 视图切换：

```text
M0~n   -> 切换有感控制模式
Fxxx   -> 给当前模式写统一目标值
X0~n   -> 切换 Function / 手感模式
W0~n   -> 切换无感模式
V0~n   -> 切换 VOFA 调试页
C0     -> 清除锁存保护 / 无感故障
```

示例：

```text
M8
F180
X1
F0.02
W0
F80
V5
```

对应含义：

- `M8`：切到三环有感模式
- `F180`：设置目标绝对角度 `180°`
- `X1`：切到纯阻尼感模式
- `F0.02`：设置阻尼系数
- `W0`：切到无感速度模式
- `F80`：设置无感目标速度 `80 rad/s`
- `V5`：切到无感专用诊断页

典型回显：

```text
# MODE OK: M8 -> PositionVelocityCurrent
# FUNC OK: X1 -> PureDamping
# SENSORLESS OK: W0 -> Speed
# VOFA OK: V5 -> Sensorless
```

---

## VOFA+ 调试

项目使用 `VOFA+ FireWater` 文本协议，调试内容通过串口 `V0~V5` 运行时切换，不需要为了换图表内容重新编译。

推荐调试页：

- `V0`：默认页，目标值 / 速度 / `Iq` / `Uq` / 角度
- `V1`：位置-速度串级调试
- `V2`：位置-电流串级调试
- `V3`：速度-电流串级调试
- `V4`：三环调试
- `V5`：无感启动与观测器诊断页

当前 `V5` 主要用于观察：

- 启动响应
- 分段加速是否生效
- 电流折返是否介入
- 无感是否仍停留在开环

---

## 无感 FOC

当前无感分支通过 `W0` 提供，是一个速度模式。

典型使用流程：

```text
V5
C0
W0
F40
F60
F80
```

几个使用要点：

- `F0` 会让无感分支停机并回到空闲态
- 启动响应现在使用更短的首次对齐时间，以及更快的重启对齐路径
- 速度上升现在使用分段目标斜坡和分段开环加速度
- 开环阶段增加了电流折返，用来减少高速启动时的直接过流

和有编码器版本相比，无感低速阶段本来就会更保守，因为观测器需要先建立可靠条件，后续才能更稳地接管。

---

## 代码结构

### `Core/Hardware`

- `SepFOC.c / SepFOC.h`
  有感 FOC 主框架、环路实现、参数和调试 getter
- `MT6701.c / MT6701.h`
  磁编码器读取与角度处理
- `Vofa.c / Vofa.h`
  VOFA 调试帧、运行时视图切换和串口回显
- `Function.c / Function.h`
  阻尼感与定格感 Function 模式
- `SensorlessFOC.c / SensorlessFOC.h`
  无感启动状态机、控制流程和调试 getter
- `BemfObserver.c / BemfObserver.h`
  `SMO + PLL` 观测器
- `SensorlessConfig.h`
  无感配置和调参宏
- `config.h`
  全局参数和模式枚举

### `Core/Src`

- `main.c`
  初始化与定时调度入口
- `adc.c`
  电流采样、滤波、ADC 回调和快环入口
- `usart.c`
  串口收发与 `M/F/X/W/V/C` 命令解析
- `tim.c`
  PWM 与速度计算相关定时器配置

---

## 调参建议

有感模式建议按由内到外的顺序调：

1. 电流环
2. 速度环
3. 位置环

推荐顺序：

- `M2` -> 电流环
- `M4` -> 速度环
- `M7` -> 速度-电流串级
- `M8` -> 完整三环

无感模式建议：

1. 先看启动响应
2. 再调低速开环与切换行为
3. 再看高速过流余量
4. 最后继续优化观测器接管

---

## 版本说明

### `0.01`

- 不带串级 PID 的初级 FOC 版本

### `0.02`

- 基础 FOC 完全体
- 单环、双环串级、三环控制接入完成
- 统一串口模式切换与目标输入完成
- 多种 VOFA 调试帧完成
- 三环 `M8` 的目标点附近保持死区加入完成

### `0.03`

- 加入过流检测与锁存保护
- 加入运行时 VOFA 调试页切换

### `0.04`

- 加入无感 FOC `W0` 模式
- 加入 `SMO + PLL` 观测器和无感启动状态机
- 加入 `V5` 无感专用诊断页
- 加入分段目标斜坡、分段开环加速度、快速重启对齐和开环限流折返
