# SepFOC

基础 FOC 的完全体版本，当前版本号：`0.02`

这是一个基于 `STM32G431CBU6`、`STM32CubeMX`、`HAL`、`Keil MDK` 搭建的自制 FOC 工程。项目从最基础的三相 PWM 输出、编码器角度读取、电流采样开始，逐步扩展到了纯电压、纯电流、纯速度、纯位置，以及双环、三环串级控制。

这一版的重点是把基础 FOC 控制框架补齐到“完整可扩展”的状态：

- 统一的控制模式框架
- 统一的串口命令协议
- 统一的 VOFA+ `FireWater` 调试输出
- 纯单环、双环串级、三环串级控制全部打通

---

## 概述

- MCU：`STM32G431CBU6`
- 开发方式：`CubeMX + HAL + Keil MDK`
- 编码器：`MT6701`
- PWM 频率：`16 kHz`
- 速度计算频率：`4 kHz`
- 当前母线电压配置：`12 V`
- 电流采样：双电阻双运放两相采样，`W` 相重构
- 调试方式：串口 + `VOFA+ FireWater`

---

## 当前已实现内容

- `SVPWM / 反 Park / 三相 PWM` 输出链路
- 编码器单圈角度读取与电角度换算
- 两相电流采样、零点补偿、`Iq` 计算与滤波
- 纯电压力矩环
- 纯电流力矩环
- 纯速度闭环
- 纯位置闭环
- 位置-速度串级环
- 位置-电流串级环
- 速度-电流串级环
- 位置-速度-电流三环控制
- 模式切换统一调度
- 串口 `M/F` 命令协议
- VOFA+ 多种调试帧输出

---

## 控制模式

控制模式顺序与 `SepFocControlMode` 枚举一致：

- `M0`：停止输出
- `M1`：目标电压的纯力矩环，输入单位 `V`
- `M2`：目标 `Q` 电流的纯力矩环，输入单位 `A`
- `M3`：纯位置闭环，输入绝对角度，单位 `deg`
- `M4`：纯速度闭环，输入单位 `rad/s`
- `M5`：位置-速度串级闭环
- `M6`：位置-电流串级闭环
- `M7`：速度-电流串级闭环
- `M8`：位置-速度-电流三环闭环

位置类模式统一使用绝对角度输入，范围 `0~360`，不是累计角度。

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

这一版的 `M8` 还加入了目标点附近的静止保持死区，用来减轻三环在到位后的细小震荡。

---

## 调度结构

项目当前采用分层调度：

- 电流内环：`ADC DMA` 回调同步执行
- 速度环 / 串级中环：`TIM7` 固定节拍执行
- 位置环 / 串级外环：在 `TIM7` 上按分频执行

当前默认分频配置在 `Core/Hardware/config.h` 中：

- 速度环：`1000 Hz`
- 位置环：`500 Hz`
- 三环中间速度环：`1000 Hz`
- 三环最外层位置环：`500 Hz`

这种调度方式的目标是让：

- 最内层快
- 中间层次之
- 最外层最慢

这样串级关系更稳定，也更容易调参。

---

## 串口协议

串口目前统一使用两类命令：

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

- `M2`：切到纯电流力矩环
- `F0.20`：目标 `Iq = 0.20 A`
- `M4`：切到纯速度环
- `F50`：目标速度 `50 rad/s`
- `M8`：切到三环模式
- `F180`：目标绝对角度 `180°`

模式切换带有回显，例如：

```text
# MODE OK: M8 -> PositionVelocityCurrent
# MODE ERR: M9 out of range [0,8]
```

---

## VOFA+ 调试

项目使用 `VOFA+ FireWater` 文本协议，所有调试帧都固定为 7 通道，方便直接切换图表模板。

当前在 `Core/Hardware/config.h` 里可以选择不同打印类型：

- `VOFA_DEBUG_PRINT_FULL_DATA`
- `VOFA_DEBUG_PRINT_POSVEL_DATA`
- `VOFA_DEBUG_PRINT_POSCUR_DATA`
- `VOFA_DEBUG_PRINT_SPCUR_DATA`
- `VOFA_DEBUG_PRINT_POSVCUR_DATA`

### 常用调试建议

- 调单环时：用 `FULL_DATA`
- 调位置-速度环时：用 `POSVEL_DATA`
- 调位置-电流环时：用 `POSCUR_DATA`
- 调速度-电流环时：用 `SPCUR_DATA`
- 调三环 `M8` 时：用 `POSVCUR_DATA`

三环调试帧当前通道含义为：

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
  主初始化、定时调度入口
- `adc.c`
  电流采样、滤波、ADC 回调、快环入口
- `usart.c`
  串口收发、`M/F` 命令解析
- `tim.c`
  PWM、速度计算相关定时器配置

### 其他

- `Drivers/`
  STM32 HAL / CMSIS
- `Middlewares/`
  中间件与 DSP 相关支持
- `MDK-ARM/`
  Keil 工程文件与构建目录
- `SepFOC_Loop.ioc`
  CubeMX 工程配置文件

---

## 当前调参建议

建议按“由内到外”的顺序调：

1. 先把电流环调顺
2. 再调速度环
3. 最后调位置环

对应关系：

- `M2`：调纯电流环
- `M4`：调纯速度环
- `M7`：验证速度-电流两环
- `M8`：最后调三环

不要一开始同时改三层参数，不然很难判断到底是哪一层在放大问题。

---

## 推荐开发环境

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
- 纯单环、双环串级、三环控制全部接入
- 串口模式切换与统一目标写入补齐
- VOFA 多种调试帧补齐
- 三环 `M8` 增加静止保持死区

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
