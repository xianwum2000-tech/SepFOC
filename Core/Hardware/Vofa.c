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
            return Sep_FOC_GetPositionTarget();
        case SEP_FOC_MODE_VELOCITY:
            return Sep_FOC_GetVelocityTarget();
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
    float speed_avg = 0.0f;
    float uq_avg = 0.0f;
    float iq_avg = 0.0f;
    float angle_now = 0.0f;
    uint32_t sample_count = 0U;
    uint32_t primask = __get_PRIMASK();

    if (!vofa_debug_ready)
    {
        return;
    }

    __disable_irq();
    mode = vofa_debug_mode;
    target = vofa_debug_target;
    speed_avg = vofa_debug_speed_avg;
    uq_avg = vofa_debug_uq_avg;
    iq_avg = vofa_debug_iq_avg;
    angle_now = vofa_debug_angle;
    sample_count = vofa_debug_samples;
    vofa_debug_ready = 0U;
    if (!primask)
    {
        __enable_irq();
    }

    // VOFA+ FireWater 通道顺序：
    // ch0=mode，当前编号与 SepFocControlMode 枚举顺序一致
    // ch1=target, ch2=speed(rad/s), ch3=Uq(V), ch4=Iq(A), ch5=angle(deg, 0~360), ch6=sample_count
    printf("%u,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%lu\r\n",
           (unsigned int)mode,               // ch0: 当前控制模式编号
           target,                           // ch1: 当前模式对应的目标量
           speed_avg,                        // ch2: 实际平均速度，单位 rad/s
           uq_avg,                           // ch3: 当前控制器输出的平均 Uq，单位 V
           iq_avg,                           // ch4: 实际平均 Iq，单位 A
           angle_now,                        // ch5: 当前绝对角度，单位度，范围 0~360
           (unsigned long)sample_count);     // ch6: 本次统计窗口累积的采样点数
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
