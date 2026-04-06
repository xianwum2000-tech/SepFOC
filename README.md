<a id="english"></a>

# SepFOC

**Language:** English | [中文](#chinese)

Basic FOC complete edition, current version: `0.02`

SepFOC is a self-built FOC project based on `STM32G431CBU6`, `STM32CubeMX`, `HAL`, and `Keil MDK`. The project started from the basic FOC signal chain, including three-phase PWM output, encoder angle acquisition, and phase-current sampling, and has gradually grown into a unified control framework with single-loop, cascaded double-loop, and full triple-loop control modes.

This version focuses on making the "basic FOC complete edition" solid and extensible:

- Unified control-mode framework
- Unified serial command protocol
- Unified VOFA+ `FireWater` debug output
- Single-loop, double-loop cascade, and triple-loop cascade control

---

## Overview

- MCU: `STM32G431CBU6`
- Development flow: `CubeMX + HAL + Keil MDK`
- Encoder: `MT6701`
- PWM frequency: `16 kHz`
- Speed calculation frequency: `4 kHz`
- DC bus voltage config: `12 V`
- Current sensing: two-shunt, two-phase sampling with reconstructed `W` phase
- Debug tools: serial port + `VOFA+ FireWater`

---

## Implemented Features

- `SVPWM / inverse Park / three-phase PWM` output chain
- Encoder single-turn angle acquisition and electrical angle conversion
- Two-phase current sampling, offset compensation, `Iq` calculation, and filtering
- Pure voltage torque mode
- Pure current torque mode
- Pure velocity loop
- Pure position loop
- Position-velocity cascade
- Position-current cascade
- Velocity-current cascade
- Position-velocity-current triple-loop control
- Unified mode switching and scheduling
- Unified serial `M/F` command interface
- Multiple VOFA+ debug frame types

---

## Control Modes

The mode order matches the `SepFocControlMode` enum:

- `M0`: output disabled
- `M1`: pure torque control in voltage mode, input unit: `V`
- `M2`: pure torque control in current mode, input unit: `A`
- `M3`: pure position loop, input unit: absolute angle in `deg`
- `M4`: pure velocity loop, input unit: `rad/s`
- `M5`: position-velocity cascade
- `M6`: position-current cascade
- `M7`: velocity-current cascade
- `M8`: position-velocity-current triple-loop control

All position-related modes use absolute angle targets in the `0~360` range instead of cumulative angle.

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

The current `M8` implementation also includes a near-target hold deadband to reduce small oscillation around the final position.

---

## Scheduling Structure

The current project uses layered scheduling:

- Current inner loop: executed inside the `ADC DMA` callback
- Velocity loop / cascade middle loop: executed by `TIM7`
- Position loop / cascade outer loop: executed by `TIM7` with divider control

Default divider settings in `Core/Hardware/config.h`:

- Velocity loop: `1000 Hz`
- Position loop: `500 Hz`
- Triple-loop middle velocity loop: `1000 Hz`
- Triple-loop outer position loop: `500 Hz`

The goal is straightforward:

- Fast inner loop
- Slower middle loop
- Slowest outer loop

This makes the cascade structure more stable and easier to tune.

---

## Serial Protocol

Two command types are currently supported:

```text
M0~n   -> switch control mode
Fxxx   -> write the unified target value for the current mode
```

Example:

```text
M2
F0.20
M4
F50
M8
F180
```

Meaning:

- `M2`: switch to pure current torque mode
- `F0.20`: set target `Iq = 0.20 A`
- `M4`: switch to pure velocity mode
- `F50`: set target velocity to `50 rad/s`
- `M8`: switch to triple-loop mode
- `F180`: set target absolute angle to `180°`

Mode switch feedback is also available:

```text
# MODE OK: M8 -> PositionVelocityCurrent
# MODE ERR: M9 out of range [0,8]
```

---

## VOFA+ Debug

The project uses `VOFA+ FireWater` text frames. All debug outputs are fixed to 7 channels so that the chart layout can stay stable while switching debug views.

Available debug print types in `Core/Hardware/config.h`:

- `VOFA_DEBUG_PRINT_FULL_DATA`
- `VOFA_DEBUG_PRINT_POSVEL_DATA`
- `VOFA_DEBUG_PRINT_POSCUR_DATA`
- `VOFA_DEBUG_PRINT_SPCUR_DATA`
- `VOFA_DEBUG_PRINT_POSVCUR_DATA`

### Recommended usage

- Single-loop tuning: `FULL_DATA`
- Position-velocity tuning: `POSVEL_DATA`
- Position-current tuning: `POSCUR_DATA`
- Velocity-current tuning: `SPCUR_DATA`
- Triple-loop `M8` tuning: `POSVCUR_DATA`

The current `POSVCUR_DATA` channel layout is:

```text
ch0 = mode
ch1 = target_angle
ch2 = angle_now
ch3 = speed_target
ch4 = speed_now
ch5 = iq_target
ch6 = iq
```

---

## Code Structure

### `Core/Hardware`

- `SepFOC.c / SepFOC.h`
  Main FOC control framework, including control modes, loop implementations, cascade scheduling, parameters, and debug getters
- `MT6701.c / MT6701.h`
  Magnetic encoder reading and angle processing
- `Vofa.c / Vofa.h`
  VOFA+ debug frames, mode feedback, and debug caches
- `config.h`
  Global parameters, control-mode enum, loop divider settings, and VOFA print selection

### `Core/Src`

- `main.c`
  Main initialization and timed scheduling entry
- `adc.c`
  Current sampling, filtering, ADC callback, and fast-loop entry
- `usart.c`
  Serial RX/TX and `M/F` command parsing
- `tim.c`
  PWM and speed-calculation timer configuration

### Others

- `Drivers/`
  STM32 HAL / CMSIS
- `Middlewares/`
  Middleware and DSP-related support
- `MDK-ARM/`
  Keil project files and build outputs
- `SepFOC_Loop.ioc`
  CubeMX project file

---

## Tuning Recommendation

Tune the loops from inner to outer:

1. Tune the current loop first
2. Then tune the velocity loop
3. Finally tune the position loop

Suggested order:

- `M2`: tune pure current loop
- `M4`: tune pure velocity loop
- `M7`: verify velocity-current cascade
- `M8`: tune the full triple-loop structure last

Avoid changing all three layers at once, otherwise it becomes very hard to tell which loop is causing the problem.

---

## Recommended Tools

- `STM32CubeMX`
- `Keil MDK`
- `VOFA+`
- `VS Code`

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

---

## Repository Notes

This repository mainly keeps:

- `Core/`
- `Drivers/`
- `Middlewares/`
- `MDK-ARM/*.uvprojx`
- `SepFOC_Loop.ioc`
- `.mxproject`

Build intermediates, target files, user-local config files, and temporary logs are handled by `.gitignore` and are not recommended for direct commit.

---

<a id="chinese"></a>

# SepFOC

**语言切换：** [English](#english) | 中文

基础 FOC 的完全体版本，当前版本号：`0.02`

SepFOC 是一个基于 `STM32G431CBU6`、`STM32CubeMX`、`HAL`、`Keil MDK` 搭建的自制 FOC 工程。项目从最基础的三相 PWM 输出、编码器角度读取、相电流采样开始，逐步扩展成了一个同时支持单环、双环串级和三环串级控制的统一控制框架。

这一版的重点是把“基础 FOC 的完全体”整理扎实并做好后续扩展接口：

- 统一的控制模式框架
- 统一的串口命令协议
- 统一的 VOFA+ `FireWater` 调试输出
- 单环、双环串级、三环串级控制全部打通

---

## 概述

- MCU：`STM32G431CBU6`
- 开发方式：`CubeMX + HAL + Keil MDK`
- 编码器：`MT6701`
- PWM 频率：`16 kHz`
- 速度计算频率：`4 kHz`
- 当前母线电压配置：`12 V`
- 电流采样：双电阻两相采样，`W` 相重构
- 调试方式：串口 + `VOFA+ FireWater`

---

## 当前已实现内容

- `SVPWM / 反 Park / 三相 PWM` 输出链路
- 编码器单圈角度读取与电角度换算
- 两相电流采样、零点补偿、`Iq` 计算与滤波
- 纯电压力矩模式
- 纯电流力矩模式
- 纯速度闭环
- 纯位置闭环
- 位置-速度串级环
- 位置-电流串级环
- 速度-电流串级环
- 位置-速度-电流三环控制
- 模式统一切换与调度
- 串口 `M/F` 命令接口
- 多种 VOFA+ 调试帧输出

---

## 控制模式

控制模式顺序与 `SepFocControlMode` 枚举一致：

- `M0`：停止输出
- `M1`：目标电压的纯力矩环，输入单位 `V`
- `M2`：目标 `Q` 电流的纯力矩环，输入单位 `A`
- `M3`：纯位置闭环，输入单位为绝对角度 `deg`
- `M4`：纯速度闭环，输入单位 `rad/s`
- `M5`：位置-速度串级环
- `M6`：位置-电流串级环
- `M7`：速度-电流串级环
- `M8`：位置-速度-电流三环控制

所有位置类模式统一使用 `0~360` 的绝对角度，不是累计角度。

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

当前 `M8` 还加入了目标点附近的静止保持死区，用来减轻到位后的细小震荡。

---

## 调度结构

项目当前采用分层调度：

- 电流内环：在 `ADC DMA` 回调中同步执行
- 速度环 / 串级中环：由 `TIM7` 固定节拍执行
- 位置环 / 串级外环：由 `TIM7` 通过分频执行

当前默认分频在 `Core/Hardware/config.h` 中配置为：

- 速度环：`1000 Hz`
- 位置环：`500 Hz`
- 三环中间速度环：`1000 Hz`
- 三环最外层位置环：`500 Hz`

目标很明确：

- 最内层快
- 中间层次之
- 最外层最慢

这样串级关系更稳定，也更容易调参。

---

## 串口协议

当前统一使用两类命令：

```text
M0~n   -> 切换控制模式
Fxxx   -> 给当前模式写统一目标值
```

示例：

```text
M2
F0.20
M4
F50
M8
F180
```

对应含义：

- `M2`：切到纯电流力矩模式
- `F0.20`：目标 `Iq = 0.20 A`
- `M4`：切到纯速度模式
- `F50`：目标速度 `50 rad/s`
- `M8`：切到三环模式
- `F180`：目标绝对角度 `180°`

模式切换带有回显：

```text
# MODE OK: M8 -> PositionVelocityCurrent
# MODE ERR: M9 out of range [0,8]
```

---

## VOFA+ 调试

项目使用 `VOFA+ FireWater` 文本协议，所有调试帧都固定为 7 通道，方便保持图表模板稳定。

`Core/Hardware/config.h` 中当前可选的调试类型有：

- `VOFA_DEBUG_PRINT_FULL_DATA`
- `VOFA_DEBUG_PRINT_POSVEL_DATA`
- `VOFA_DEBUG_PRINT_POSCUR_DATA`
- `VOFA_DEBUG_PRINT_SPCUR_DATA`
- `VOFA_DEBUG_PRINT_POSVCUR_DATA`

### 常用建议

- 调单环：用 `FULL_DATA`
- 调位置-速度环：用 `POSVEL_DATA`
- 调位置-电流环：用 `POSCUR_DATA`
- 调速度-电流环：用 `SPCUR_DATA`
- 调三环 `M8`：用 `POSVCUR_DATA`

当前三环 `POSVCUR_DATA` 的通道定义是：

```text
ch0 = mode
ch1 = target_angle
ch2 = angle_now
ch3 = speed_target
ch4 = speed_now
ch5 = iq_target
ch6 = iq
```

---

## 代码结构

### `Core/Hardware`

- `SepFOC.c / SepFOC.h`
  FOC 主控制框架，包含控制模式、各类环路、串级调度、参数和调试 getter
- `MT6701.c / MT6701.h`
  磁编码器读取与角度处理
- `Vofa.c / Vofa.h`
  VOFA+ 调试帧、模式回显、调试缓存
- `config.h`
  全局参数、控制模式枚举、分频配置、VOFA 打印选择

### `Core/Src`

- `main.c`
  主初始化与定时调度入口
- `adc.c`
  电流采样、滤波、ADC 回调与快环入口
- `usart.c`
  串口收发与 `M/F` 命令解析
- `tim.c`
  PWM 与速度计算相关定时器配置

### 其他

- `Drivers/`
  STM32 HAL / CMSIS
- `Middlewares/`
  中间件与 DSP 支持
- `MDK-ARM/`
  Keil 工程文件与构建目录
- `SepFOC_Loop.ioc`
  CubeMX 工程文件

---

## 当前调参建议

建议按“由内到外”的顺序调：

1. 先调电流环
2. 再调速度环
3. 最后调位置环

对应建议顺序：

- `M2`：调纯电流环
- `M4`：调纯速度环
- `M7`：验证速度-电流双环
- `M8`：最后调完整三环

不要一开始同时改三层参数，不然很难判断到底是哪一层在放大问题。

---

## 推荐工具

- `STM32CubeMX`
- `Keil MDK`
- `VOFA+`
- `VS Code`

---

## 版本说明

### `0.01`

- 不带串级 PID 的 FOC 初级版本

### `0.02`

- 基础 FOC 完全体
- 单环、双环串级、三环控制全部接入
- 串口模式切换与统一目标输入补齐
- 多种 VOFA 调试帧补齐
- 三环 `M8` 增加目标点附近静止保持死区

---

## 仓库说明

仓库主要保存：

- `Core/`
- `Drivers/`
- `Middlewares/`
- `MDK-ARM/*.uvprojx`
- `SepFOC_Loop.ioc`
- `.mxproject`

编译中间文件、目标文件、用户本地配置和临时日志由 `.gitignore` 管理，不建议直接提交。
