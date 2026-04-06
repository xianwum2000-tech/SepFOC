#include "Vofa.h"
#include <stdio.h>
#include "adc.h"
#include "MT6701.h"
#include "SepFoc.h"

// 电机速度在 main.c 中维护，这里只做只读引用用于调试输出。
extern float motor_speed;

#define VOFA_DEBUG_TICKS       (SPEED_CALCU_FREQ / 5U)  // 每 200 ms 输出一次调试窗口平均值

static volatile uint8_t vofa_debug_ready = 0U;
static volatile SepFocControlMode vofa_debug_mode = SEP_FOC_MODE_DISABLED;
static volatile uint32_t vofa_debug_samples = 0U;
static volatile float vofa_debug_target = 0.0f;
static volatile float vofa_debug_speed_avg = 0.0f;
static volatile float vofa_debug_uq_avg = 0.0f;
static volatile float vofa_debug_iq_avg = 0.0f;
static volatile float vofa_debug_angle = 0.0f;
static volatile float vofa_debug_posvel_speed_target = 0.0f;
static volatile float vofa_debug_poscur_current_target = 0.0f;
static volatile float vofa_debug_spcur_current_target = 0.0f;
static volatile float vofa_debug_posvcur_speed_target = 0.0f;
static volatile float vofa_debug_posvcur_current_target = 0.0f;

/**********************************************************************************************
 * @brief  根据当前控制模式，返回串口调试里应该显示的目标量
 * @param  mode: 当前控制模式
 * @return 目标值，单位由模式决定
 * @note   这个函数只负责给调试打印选“目标通道”，不参与控制计算。
 *********************************************************************************************/
static float Vofa_GetTargetByMode(SepFocControlMode mode)
{
    switch (mode)
    {
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
            return Sep_FOC_GetVoltageTorqueTarget();
        case SEP_FOC_MODE_TORQUE_CURRENT:
            return Sep_FOC_GetTorqueTarget();
        case SEP_FOC_MODE_POSITION:
        case SEP_FOC_MODE_POSITION_VELOCITY:
        case SEP_FOC_MODE_POSITION_CURRENT:
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            return Sep_FOC_GetPositionTarget();
        case SEP_FOC_MODE_VELOCITY:
            return Sep_FOC_GetVelocityTarget();
        case SEP_FOC_MODE_VELOCITY_CURRENT:
            return Sep_FOC_GetVelocityCurrentTarget();
        case SEP_FOC_MODE_DISABLED:
        default:
            return 0.0f;
    }
}

/**********************************************************************************************
 * @brief  在中断里累计一段时间窗口内的调试统计量
 * @note   这里只做轻量级求和与缓存，真正的 printf 放在主循环里，避免中断阻塞。
 *********************************************************************************************/
void Vofa_Debug_Update(void)
{
    static uint32_t sample_count = 0U;
    static float sum_speed = 0.0f;
    static float sum_uq = 0.0f;
    static float sum_iq = 0.0f;
    SepFocControlMode mode = Sep_FOC_GetControlMode();
    float iq_now = cal_Iq_raw(motor_i_u, motor_i_v, _electricalAngle());

    sum_speed += motor_speed;
    sum_uq += Sep_FOC_GetAppliedUq();
    sum_iq += iq_now;
    sample_count++;

    if (sample_count >= VOFA_DEBUG_TICKS)
    {
        // 如果上一帧还没打印，就直接丢弃当前统计窗口，避免前台阻塞反向拖慢中断
        if (!vofa_debug_ready)
        {
            vofa_debug_mode = mode;
            vofa_debug_target = Vofa_GetTargetByMode(mode);
            vofa_debug_speed_avg = sum_speed / (float)sample_count;
            vofa_debug_uq_avg = sum_uq / (float)sample_count;
            vofa_debug_iq_avg = sum_iq / (float)sample_count;
            vofa_debug_angle = Get_Angle();
            vofa_debug_posvel_speed_target = Sep_FOC_GetPositionVelocitySpeedTarget();
            vofa_debug_poscur_current_target = Sep_FOC_GetPositionCurrentTargetQ();
            vofa_debug_spcur_current_target = Sep_FOC_GetVelocityCurrentTargetQ();
            vofa_debug_posvcur_speed_target = Sep_FOC_GetPositionVelocityCurrentSpeedTarget();
            vofa_debug_posvcur_current_target = Sep_FOC_GetPositionVelocityCurrentTargetQ();
            vofa_debug_samples = sample_count;
            vofa_debug_ready = 1U;
        }

        sum_speed = 0.0f;
        sum_uq = 0.0f;
        sum_iq = 0.0f;
        sample_count = 0U;
    }
}

/**********************************************************************************************
 * @brief  将当前缓存好的调试窗口输出为 VOFA+ FireWater 文本帧
 * @note   固定输出 7 个通道，方便 VOFA+ 长期保持同一组通道映射。
 *********************************************************************************************/
void Vofa_PrintDebugFrame(void)
{
    SepFocControlMode mode = SEP_FOC_MODE_DISABLED;
    float target = 0.0f;
    float uq_avg = 0.0f;
#if (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_FULL_DATA)
    float speed_avg = 0.0f;
    float iq_avg = 0.0f;
    uint32_t sample_count = 0U;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVEL_DATA)
    float speed_avg = 0.0f;
    float posvel_speed_target = 0.0f;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSCUR_DATA)
    float iq_avg = 0.0f;
    float poscur_current_target = 0.0f;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_SPCUR_DATA)
    float speed_avg = 0.0f;
    float iq_avg = 0.0f;
    float spcur_current_target = 0.0f;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVCUR_DATA)
    float speed_avg = 0.0f;
    float iq_avg = 0.0f;
    float angle_now = 0.0f;
    float posvcur_speed_target = 0.0f;
    float posvcur_current_target = 0.0f;
#endif
#if (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_FULL_DATA) || \
    (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVEL_DATA) || \
    (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSCUR_DATA)
    float angle_now = 0.0f;
#endif
    uint32_t primask = __get_PRIMASK();

    if (!vofa_debug_ready)
    {
        return;
    }

    __disable_irq();
    mode = vofa_debug_mode;
    target = vofa_debug_target;
    uq_avg = vofa_debug_uq_avg;
#if (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_FULL_DATA)
    speed_avg = vofa_debug_speed_avg;
    iq_avg = vofa_debug_iq_avg;
    sample_count = vofa_debug_samples;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVEL_DATA)
    speed_avg = vofa_debug_speed_avg;
    posvel_speed_target = vofa_debug_posvel_speed_target;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSCUR_DATA)
    iq_avg = vofa_debug_iq_avg;
    poscur_current_target = vofa_debug_poscur_current_target;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_SPCUR_DATA)
    speed_avg = vofa_debug_speed_avg;
    iq_avg = vofa_debug_iq_avg;
    spcur_current_target = vofa_debug_spcur_current_target;
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVCUR_DATA)
    speed_avg = vofa_debug_speed_avg;
    iq_avg = vofa_debug_iq_avg;
    posvcur_speed_target = vofa_debug_posvcur_speed_target;
    posvcur_current_target = vofa_debug_posvcur_current_target;
#endif
#if (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_FULL_DATA) || \
    (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVEL_DATA) || \
    (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSCUR_DATA) || \
    (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVCUR_DATA)
    angle_now = vofa_debug_angle;
#endif
    vofa_debug_ready = 0U;
    if (!primask)
    {
        __enable_irq();
    }

#if (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_FULL_DATA)
    // VOFA+ FireWater 通道顺序（全量调试）：
    // ch0=mode，ch1=target，ch2=speed，ch3=Uq，ch4=Iq，ch5=angle，ch6=sample_count
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%lu\r\n",
           (unsigned int)mode,               // ch0: 当前控制模式编号
           target,                           // ch1: 当前模式对应的目标量
           speed_avg,                        // ch2: 实际平均速度，单位 rad/s
           uq_avg,                           // ch3: 当前控制器输出的平均 Uq，单位 V
           iq_avg,                           // ch4: 实际平均 Iq，单位 A
           angle_now,                        // ch5: 当前绝对角度，单位度，范围 0~360
           (unsigned long)sample_count);     // ch6: 本次统计窗口累积的采样点数
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVEL_DATA)
    // VOFA+ FireWater 通道顺序（位置-速度串级调试）：
    // ch0=mode，ch1=target_angle，ch2=angle_now，ch3=angle_error，ch4=outer_speed_target，ch5=speed，ch6=Uq
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
           (unsigned int)mode,                           // ch0: 当前控制模式编号
           target,                                       // ch1: 目标绝对角度，单位度
           angle_now,                                    // ch2: 当前绝对角度，单位度
           cycle_diff(target - angle_now, 360.0f),       // ch3: 角度误差，单位度，取最短路径
           posvel_speed_target,                          // ch4: 外环给出的速度目标，单位 rad/s
           speed_avg,                                    // ch5: 实际平均速度，单位 rad/s
           uq_avg);                                      // ch6: 速度环输出的平均 Uq，单位 V
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSCUR_DATA)
    // VOFA+ FireWater 通道顺序（位置-电流串级调试）：
    // ch0=mode，ch1=target_angle，ch2=angle_now，ch3=angle_error，ch4=iq_target，ch5=iq，ch6=Uq
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
           (unsigned int)mode,                           // ch0: 当前控制模式编号
           target,                                       // ch1: 目标绝对角度，单位度
           angle_now,                                    // ch2: 当前绝对角度，单位度
           cycle_diff(target - angle_now, 360.0f),       // ch3: 角度误差，单位度，取最短路径
           poscur_current_target,                        // ch4: 外环给出的目标电流，单位 A
           iq_avg,                                       // ch5: 实际平均 Iq，单位 A
           uq_avg);                                      // ch6: 电流环输出的平均 Uq，单位 V
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_SPCUR_DATA)
    // VOFA+ FireWater 通道顺序（速度-电流串级调试）：
    // ch0=mode，ch1=target_speed，ch2=speed_now，ch3=speed_error，ch4=iq_target，ch5=iq，ch6=Uq
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
           (unsigned int)mode,                           // ch0: 当前控制模式编号
           target,                                       // ch1: 目标速度，单位 rad/s
           speed_avg,                                    // ch2: 实际平均速度，单位 rad/s
           (target - speed_avg),                         // ch3: 速度误差，单位 rad/s
           spcur_current_target,                         // ch4: 外环给出的目标电流，单位 A
           iq_avg,                                       // ch5: 实际平均 Iq，单位 A
           uq_avg);                                      // ch6: 电流环输出的平均 Uq，单位 V
#elif (VOFA_DEBUG_PRINT_TYPE == VOFA_DEBUG_PRINT_POSVCUR_DATA)
    // VOFA+ FireWater 通道顺序（三环调试）：
    // ch0=mode，ch1=target_angle，ch2=angle_now，ch3=speed_target，ch4=speed_now，ch5=iq_target，ch6=iq
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
           (unsigned int)mode,                           // ch0: 当前控制模式编号
           target,                                       // ch1: 目标绝对角度，单位度
           angle_now,                                    // ch2: 当前绝对角度，单位度
           posvcur_speed_target,                         // ch3: 最外环位置 PID 给出的速度目标，单位 rad/s
           speed_avg,                                    // ch4: 实际平均速度，单位 rad/s
           posvcur_current_target,                       // ch5: 中间速度环给出的目标电流，单位 A
           iq_avg);                                      // ch6: 实际平均 Iq，单位 A
#else
#error "Unsupported VOFA_DEBUG_PRINT_TYPE"
#endif
}

/**********************************************************************************************
 * @brief  输出 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 * @note   这类故障日志也统一放在 Vofa 模块里，避免串口打印逻辑分散。
 *********************************************************************************************/
void Vofa_PrintAdcError(uint32_t error_code)
{
    printf("ADC1 Error! Error Code: 0x%04X\r\n", (unsigned int)error_code);
}

/**********************************************************************************************
 * @brief  输出模式切换成功回显
 * @param  mode: 已切换到的模式编号
 * @note   用于串口终端确认当前模式已生效，模式名称与 SepFocControlMode 保持一致。
 *********************************************************************************************/
void Vofa_PrintModeSwitchOk(uint32_t mode)
{
    const char *mode_name = "Unknown";

    switch ((SepFocControlMode)mode)
    {
        case SEP_FOC_MODE_DISABLED:
            mode_name = "Disabled";
            break;
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
            mode_name = "TorqueVoltage";
            break;
        case SEP_FOC_MODE_TORQUE_CURRENT:
            mode_name = "TorqueCurrent";
            break;
        case SEP_FOC_MODE_POSITION:
            mode_name = "Position";
            break;
        case SEP_FOC_MODE_VELOCITY:
            mode_name = "Velocity";
            break;
        case SEP_FOC_MODE_POSITION_VELOCITY:
            mode_name = "PositionVelocity";
            break;
        case SEP_FOC_MODE_POSITION_CURRENT:
            mode_name = "PositionCurrent";
            break;
        case SEP_FOC_MODE_VELOCITY_CURRENT:
            mode_name = "VelocityCurrent";
            break;
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            mode_name = "PositionVelocityCurrent";
            break;
        default:
            break;
    }

    printf("# MODE OK: M%lu -> %s\r\n", (unsigned long)mode, mode_name);
}

/**********************************************************************************************
 * @brief  输出非法模式编号回显
 * @param  mode: 用户输入的非法模式编号
 * @note   当串口收到超范围的 M 命令时调用，方便快速判断命令没有生效的原因。
 *********************************************************************************************/
void Vofa_PrintModeSwitchError(long mode)
{
    printf("# MODE ERR: M%ld out of range [0,%d]\r\n",
           mode,
           (int)(SEP_FOC_MODE_COUNT - 1));
}
