# SepFOC_Loop

这是一个基于 `STM32G431CBU6`、`STM32CubeMX` 和 `HAL` 的自制 FOC 项目，主要用于永磁同步电机 / 无刷电机的基础 FOC 控制实验、串口调试和 VOFA+ 波形观察。

当前工程已经整理出统一的控制模式框架、串口命令协议和 VOFA+ `FireWater` 输出，适合继续往位置环、速度环、电流环以及后续串级控制方向扩展。

## 工程特点

- MCU: `STM32G431CBU6`
- 开发方式: `CubeMX + HAL + Keil MDK`
- 编码器: `MT6701`
- PWM 频率: `16 kHz`
- 速度计算频率: `4 kHz`
- 支持 VOFA+ `FireWater` 串口输出
- 支持串口切换控制模式与统一写入目标值

## 当前控制模式

控制模式顺序与 `SepFocControlMode` 枚举一致：

- `M0`: 停止输出
- `M1`: 目标电压的纯力矩环（电压模式）
- `M2`: 目标 Q 电流的纯力矩环（电流模式）
- `M3`: 纯位置闭环
- `M4`: 纯速度闭环

说明：

- 位置模式输入的是绝对角度，单位为度，范围 `0~360`
- 速度模式输入单位为 `rad/s`
- 电流模式输入单位为 `A`
- 电压模式输入单位为 `V`

## 串口协议

串口目前支持两类命令：

```text
M0~n   -> 切换控制模式
Fxxx   -> 给当前模式写统一目标值
```

示例：

```text
M2
F0.10
M4
F50
M3
F90
```

含义：

- `M2` 切到电流模式
- `F0.10` 给目标 `Iq = 0.10 A`
- `M4` 切到速度模式
- `F50` 给目标速度 `50 rad/s`
- `M3` 切到位置模式
- `F90` 让电机转到绝对角度 `90°`

串口会输出模式切换回显，例如：

```text
# MODE OK: M2 -> TorqueCurrent
# MODE ERR: M9 out of range [0,4]
```

## VOFA+ 输出

VOFA+ 使用 `FireWater` 文本格式，当前固定输出 7 个通道：

```text
ch0=mode
ch1=target
ch2=speed(rad/s)
ch3=Uq(V)
ch4=Iq(A)
ch5=angle(deg, 0~360)
ch6=sample_count
```

## 工程目录

```text
Core/
  Hardware/     自定义控制、编码器、VOFA 相关代码
  Inc/          头文件
  Src/          CubeMX 生成的主工程源码
Drivers/        STM32 HAL / CMSIS
Middlewares/    DSP 等中间件
MDK-ARM/        Keil 工程与构建输出
SepFOC_Loop.ioc CubeMX 工程文件
```

## 推荐开发环境

- `STM32CubeMX`
- `Keil MDK`
- `VOFA+`

## 仓库说明

这个仓库建议主要保存：

- `Core/`
- `Drivers/`
- `Middlewares/`
- `MDK-ARM/*.uvprojx`
- `SepFOC_Loop.ioc`
- `.mxproject`

不建议上传编译中间文件、目标文件、用户本地配置和临时日志，这些内容已经在 `.gitignore` 中做了忽略。
