#include "Vofa.h"
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "MT6701.h"
#include "SensorlessFOC.h"
#include "SepFoc.h"
#include "usart.h"

// 电机机械速度在 main.c 中维护；这里仅在调试打印时读取。
extern float motor_speed;

#define VOFA_DEBUG_TICKS       (SPEED_CALCU_FREQ / 5U)  // 每 200 ms 输出一帧窗口平均值

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
static volatile float vofa_debug_sensorless_state = 0.0f;
static volatile float vofa_debug_sensorless_open_speed = 0.0f;
static volatile float vofa_debug_sensorless_open_voltage = 0.0f;
static volatile float vofa_debug_sensorless_open_accel = 0.0f;
static volatile float vofa_debug_sensorless_start_dir = 0.0f;
static volatile float vofa_debug_sensorless_event = 0.0f;
static volatile float vofa_debug_sensorless_reset_count = 0.0f;
static volatile VofaDebugView vofa_debug_view = VOFA_DEBUG_VIEW_DEFAULT;

#define VOFA_TEXT_EVENT_QUEUE_SIZE    8U
#define VOFA_TEXT_NAME_MAX_LEN        16U

typedef enum
{
    VOFA_TEXT_EVENT_NONE = 0,
    VOFA_TEXT_EVENT_ADC_ERROR,
    VOFA_TEXT_EVENT_MODE_OK,
    VOFA_TEXT_EVENT_MODE_ERR,
    VOFA_TEXT_EVENT_FUNC_OK,
    VOFA_TEXT_EVENT_FUNC_ERR,
    VOFA_TEXT_EVENT_FUNC_VALUE,
    VOFA_TEXT_EVENT_SENSORLESS_OK,
    VOFA_TEXT_EVENT_SENSORLESS_ERR,
    VOFA_TEXT_EVENT_SENSORLESS_FAULT,
    VOFA_TEXT_EVENT_VIEW_OK,
    VOFA_TEXT_EVENT_VIEW_ERR,
    VOFA_TEXT_EVENT_PROTECT_OK,
    VOFA_TEXT_EVENT_PROTECT_ERR
} VofaTextEventType;

typedef struct
{
    VofaTextEventType type;
    long integer_value;
    float float_value;
    char name[VOFA_TEXT_NAME_MAX_LEN];
} VofaTextEvent;

static volatile uint8_t vofa_text_queue_head = 0U;
static volatile uint8_t vofa_text_queue_tail = 0U;
static VofaTextEvent vofa_text_queue[VOFA_TEXT_EVENT_QUEUE_SIZE];

/**********************************************************************************************
 * @brief  向文本事件队列压入一条状态消息
 * @param  type: 事件类型
 * @param  integer_value: 整数附加参数
 * @param  name: 文本附加参数，可为空
 * @param  float_value: 浮点附加参数
 * @note   允许在中断中调用，真正的 printf 会延后到主循环执行。
 *********************************************************************************************/
static void Vofa_QueueTextEvent(VofaTextEventType type, long integer_value, const char *name, float float_value)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t next_tail = 0U;

    __disable_irq();
    next_tail = (uint8_t)((vofa_text_queue_tail + 1U) % VOFA_TEXT_EVENT_QUEUE_SIZE);
    if (next_tail != vofa_text_queue_head)
    {
        VofaTextEvent *event = &vofa_text_queue[vofa_text_queue_tail];
        event->type = type;
        event->integer_value = integer_value;
        event->float_value = float_value;
        event->name[0] = '\0';
        if (name != NULL)
        {
            strncpy(event->name, name, VOFA_TEXT_NAME_MAX_LEN - 1U);
            event->name[VOFA_TEXT_NAME_MAX_LEN - 1U] = '\0';
        }
        vofa_text_queue_tail = next_tail;
    }
    if (!primask)
    {
        __enable_irq();
    }
}

/**********************************************************************************************
 * @brief  读取当前命令目标值
 * @return 最近一次写入的 F 值
 * @note   默认调试页直接显示这个值，便于和实际反馈做对比。
 *********************************************************************************************/
static float Vofa_GetCommandTarget(void)
{
    return motor_target_val;
}

/**********************************************************************************************
 * @brief  在中断里累计一段时间窗口内的调试统计量
 * @note   这里只做求和和缓存，真正的串口输出放在主循环，避免阻塞中断。
 *********************************************************************************************/
void Vofa_Debug_Update(void)
{
    static uint32_t sample_count = 0U;
    static float sum_speed = 0.0f;
    static float sum_uq = 0.0f;
    static float sum_iq = 0.0f;
    static SensorlessFaultType last_sensorless_fault = SENSORLESS_FAULT_NONE;
    SepFocControlMode mode = Sep_FOC_GetControlMode();
    float iq_now = 0.0f;
    float speed_now = motor_speed;
    float angle_now = Get_Angle();

    if (Sensorless_FOC_IsEnabled())
    {
        SensorlessFaultType fault_now = Sensorless_FOC_GetFault();
        iq_now = Sensorless_FOC_GetMeasuredIq();
        speed_now = Sensorless_FOC_GetDebugSpeed();
        angle_now = Sensorless_FOC_GetDebugAngleDeg();
        sum_uq += Sensorless_FOC_GetOutputUq();
        if ((fault_now != SENSORLESS_FAULT_NONE) && (fault_now != last_sensorless_fault))
        {
            Vofa_RequestSensorlessFault((uint32_t)fault_now);
        }
        last_sensorless_fault = fault_now;
    }
    else
    {
        iq_now = cal_Iq_raw(motor_i_u, motor_i_v, _electricalAngle());
        sum_uq += Sep_FOC_GetAppliedUq();
        last_sensorless_fault = SENSORLESS_FAULT_NONE;
    }

    sum_speed += speed_now;
    sum_iq += iq_now;
    sample_count++;

    if (sample_count >= VOFA_DEBUG_TICKS)
    {
        // 如果上一帧还没打印，就直接丢弃当前窗口，避免前台串口拖慢控制中断。
        if (!vofa_debug_ready)
        {
            vofa_debug_mode = mode;
            vofa_debug_target = Vofa_GetCommandTarget();
            vofa_debug_speed_avg = sum_speed / (float)sample_count;
            vofa_debug_uq_avg = sum_uq / (float)sample_count;
            vofa_debug_iq_avg = sum_iq / (float)sample_count;
            vofa_debug_angle = angle_now;
            vofa_debug_posvel_speed_target = Sep_FOC_GetPositionVelocitySpeedTarget();
            vofa_debug_poscur_current_target = Sep_FOC_GetPositionCurrentTargetQ();
            vofa_debug_spcur_current_target = Sep_FOC_GetVelocityCurrentTargetQ();
            vofa_debug_posvcur_speed_target = Sep_FOC_GetPositionVelocityCurrentSpeedTarget();
            vofa_debug_posvcur_current_target = Sep_FOC_GetPositionVelocityCurrentTargetQ();
            vofa_debug_sensorless_state = (float)Sensorless_FOC_GetState();
            vofa_debug_sensorless_open_speed = Sensorless_FOC_GetOpenLoopSpeed();
            vofa_debug_sensorless_open_voltage = Sensorless_FOC_GetOpenLoopVoltage();
            vofa_debug_sensorless_open_accel = Sensorless_FOC_GetOpenLoopAcceleration();
            vofa_debug_sensorless_start_dir = Sensorless_FOC_GetStartDirection();
            vofa_debug_sensorless_event = (float)Sensorless_FOC_GetDebugEvent();
            vofa_debug_sensorless_reset_count = (float)Sensorless_FOC_GetDebugResetCount();
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
 * @brief  将当前缓存好的调试窗口按 VOFA+ FireWater 文本帧输出
 * @note   调试视图可由串口 V0~V5 运行时切换，不再依赖编译期宏。
 *********************************************************************************************/
void Vofa_PrintDebugFrame(void)
{
    SepFocControlMode mode = SEP_FOC_MODE_DISABLED;
    VofaDebugView view = VOFA_DEBUG_VIEW_DEFAULT;
    float target = 0.0f;
    float uq_avg = 0.0f;
    float speed_avg = 0.0f;
    float iq_avg = 0.0f;
    float angle_now = 0.0f;
    float posvel_speed_target = 0.0f;
    float poscur_current_target = 0.0f;
    float spcur_current_target = 0.0f;
    float posvcur_speed_target = 0.0f;
    float posvcur_current_target = 0.0f;
    float sensorless_state = 0.0f;
    float sensorless_open_speed = 0.0f;
    float sensorless_open_voltage = 0.0f;
    float sensorless_open_accel = 0.0f;
    float sensorless_start_dir = 0.0f;
    float sensorless_event = 0.0f;
    float sensorless_reset_count = 0.0f;
    uint32_t primask = __get_PRIMASK();

    if (!vofa_debug_ready)
    {
        return;
    }

    __disable_irq();
    mode = vofa_debug_mode;
    view = vofa_debug_view;
    target = vofa_debug_target;
    uq_avg = vofa_debug_uq_avg;
    speed_avg = vofa_debug_speed_avg;
    iq_avg = vofa_debug_iq_avg;
    angle_now = vofa_debug_angle;
    posvel_speed_target = vofa_debug_posvel_speed_target;
    poscur_current_target = vofa_debug_poscur_current_target;
    spcur_current_target = vofa_debug_spcur_current_target;
    posvcur_speed_target = vofa_debug_posvcur_speed_target;
    posvcur_current_target = vofa_debug_posvcur_current_target;
    sensorless_state = vofa_debug_sensorless_state;
    sensorless_open_speed = vofa_debug_sensorless_open_speed;
    sensorless_open_voltage = vofa_debug_sensorless_open_voltage;
    sensorless_open_accel = vofa_debug_sensorless_open_accel;
    sensorless_start_dir = vofa_debug_sensorless_start_dir;
    sensorless_event = vofa_debug_sensorless_event;
    sensorless_reset_count = vofa_debug_sensorless_reset_count;
    vofa_debug_ready = 0U;
    if (!primask)
    {
        __enable_irq();
    }

    (void)mode;

    switch (view)
    {
        case VOFA_DEBUG_VIEW_DEFAULT:
            // V0: 统一目标 F / 实际速度 / 实际 Iq / 实际 Uq / 当前绝对角度
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   speed_avg,
                   iq_avg,
                   uq_avg,
                   angle_now);
            break;
        case VOFA_DEBUG_VIEW_POSVEL:
            // V1: 位置-速度串级：目标角度 / 当前角度 / 角度误差 / 外环速度目标 / 实际速度 / Uq
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   cycle_diff(target - angle_now, 360.0f),
                   posvel_speed_target,
                   speed_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_POSCUR:
            // V2: 位置-电流串级：目标角度 / 当前角度 / 角度误差 / 目标 Iq / 实际 Iq / Uq
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   cycle_diff(target - angle_now, 360.0f),
                   poscur_current_target,
                   iq_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_SPCUR:
            // V3: 速度-电流串级：目标速度 / 实际速度 / 速度误差 / 目标 Iq / 实际 Iq / Uq
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   speed_avg,
                   (target - speed_avg),
                   spcur_current_target,
                   iq_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_POSVCUR:
            // V4: 位置-速度-电流三环：目标角度 / 当前角度 / 中间速度目标 / 实际速度 / 目标 Iq / 实际 Iq
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   posvcur_speed_target,
                   speed_avg,
                   posvcur_current_target,
                   iq_avg);
            break;
        case VOFA_DEBUG_VIEW_SENSORLESS:
            // V5: 无感启动诊断页：状态 / 目标速度 / 开环速度 / 开环电压 / 开环加速度 / 启动方向 / 最近事件 / 重置计数 / 实际速度
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   sensorless_state,
                   target,
                   sensorless_open_speed,
                   sensorless_open_voltage,
                   sensorless_open_accel,
                   sensorless_start_dir,
                   sensorless_event,
                   sensorless_reset_count,
                   speed_avg);
            break;
        default:
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   speed_avg,
                   iq_avg,
                   uq_avg,
                   angle_now);
            break;
    }
}

/**********************************************************************************************
 * @brief  设置当前 VOFA 调试视图
 * @param  view: 目标视图编号，串口对应 V0~V5
 *********************************************************************************************/
void Vofa_SetDebugView(VofaDebugView view)
{
    if (view >= VOFA_DEBUG_VIEW_COUNT)
    {
        view = VOFA_DEBUG_VIEW_DEFAULT;
    }

    vofa_debug_view = view;
}

/**********************************************************************************************
 * @brief  读取当前 VOFA 调试视图
 * @return 当前生效的视图编号
 *********************************************************************************************/
VofaDebugView Vofa_GetDebugView(void)
{
    return vofa_debug_view;
}

/**********************************************************************************************
 * @brief  处理文本事件队列中待输出的消息
 * @note   中断里只登记事件，真正的 printf 在主循环里统一完成。
 *********************************************************************************************/
void Vofa_ProcessPendingTextFrame(void)
{
    VofaTextEvent event = {VOFA_TEXT_EVENT_NONE, 0L, 0.0f, {0}};
    uint32_t primask = __get_PRIMASK();

    if (vofa_text_queue_head == vofa_text_queue_tail)
    {
        return;
    }

    __disable_irq();
    event = vofa_text_queue[vofa_text_queue_head];
    vofa_text_queue_head = (uint8_t)((vofa_text_queue_head + 1U) % VOFA_TEXT_EVENT_QUEUE_SIZE);
    if (!primask)
    {
        __enable_irq();
    }

    switch (event.type)
    {
        case VOFA_TEXT_EVENT_ADC_ERROR:
            Vofa_PrintAdcError((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_MODE_OK:
            Vofa_PrintModeSwitchOk((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_MODE_ERR:
            Vofa_PrintModeSwitchError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_FUNC_OK:
            Vofa_PrintFunctionSwitchOk((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_FUNC_ERR:
            Vofa_PrintFunctionSwitchError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_FUNC_VALUE:
            Vofa_PrintFunctionValueUpdate(event.name, event.float_value);
            break;
        case VOFA_TEXT_EVENT_SENSORLESS_OK:
            Vofa_PrintSensorlessSwitchOk((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_SENSORLESS_ERR:
            Vofa_PrintSensorlessSwitchError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_SENSORLESS_FAULT:
            Vofa_PrintSensorlessFault((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_VIEW_OK:
            Vofa_PrintDebugViewSwitchOk((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_VIEW_ERR:
            Vofa_PrintDebugViewSwitchError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_PROTECT_OK:
            Vofa_PrintProtectionClearOk();
            break;
        case VOFA_TEXT_EVENT_PROTECT_ERR:
            Vofa_PrintProtectionClearError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_NONE:
        default:
            break;
    }
}

/**********************************************************************************************
 * @brief  输出 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 * @note   直接打印给串口，便于定位 DMA/采样异常。
 *********************************************************************************************/
void Vofa_PrintAdcError(uint32_t error_code)
{
    printf("ADC1 Error! Error Code: 0x%04X\r\n", (unsigned int)error_code);
}

/**********************************************************************************************
 * @brief  请求缓存 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 *********************************************************************************************/
void Vofa_RequestAdcError(uint32_t error_code)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_ADC_ERROR, (long)error_code, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出有感模式切换成功回显
 * @param  mode: 已切换到的模式编号
 * @note   模式编号与 SepFocControlMode 枚举保持一致。
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
 * @brief  请求缓存有感模式切换成功信息
 * @param  mode: 需要回显的控制模式编号
 *********************************************************************************************/
void Vofa_RequestModeSwitchOk(uint32_t mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_MODE_OK, (long)mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出非法有感模式编号回显
 * @param  mode: 用户输入的非法模式编号
 * @note   有感模式编号范围由 M 模式枚举自动决定。
 *********************************************************************************************/
void Vofa_PrintModeSwitchError(long mode)
{
    printf("# MODE ERR: M%ld out of range [0,%d]\r\n",
           mode,
           (int)(SEP_FOC_MODE_COUNT - 1));
}

/**********************************************************************************************
 * @brief  请求缓存非法有感模式错误信息
 * @param  mode: 用户输入的非法模式编号
 *********************************************************************************************/
void Vofa_RequestModeSwitchError(long mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_MODE_ERR, mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出 Function 模式切换成功回显
 * @param  mode: 0=关闭，1=纯阻尼感，2=定格感
 *********************************************************************************************/
void Vofa_PrintFunctionSwitchOk(uint32_t mode)
{
    const char *mode_name = "Unknown";

    switch (mode)
    {
        case 0U:
            mode_name = "Disabled";
            break;
        case 1U:
            mode_name = "PureDamping";
            break;
        case 2U:
            mode_name = "Detent";
            break;
        default:
            break;
    }

    printf("# FUNC OK: X%lu -> %s\r\n", (unsigned long)mode, mode_name);
}

/**********************************************************************************************
 * @brief  请求缓存 Function 模式切换成功信息
 * @param  mode: 需要回显的功能模式编号
 *********************************************************************************************/
void Vofa_RequestFunctionSwitchOk(uint32_t mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_FUNC_OK, (long)mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出非法 Function 模式编号回显
 * @param  mode: 用户输入的非法功能模式编号
 *********************************************************************************************/
void Vofa_PrintFunctionSwitchError(long mode)
{
    printf("# FUNC ERR: X%ld out of range [0,2]\r\n", mode);
}

/**********************************************************************************************
 * @brief  请求缓存非法 Function 模式错误信息
 * @param  mode: 用户输入的非法功能模式编号
 *********************************************************************************************/
void Vofa_RequestFunctionSwitchError(long mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_FUNC_ERR, mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出 Function 参数更新回显
 * @param  name: 参数名
 * @param  value: 新值
 *********************************************************************************************/
void Vofa_PrintFunctionValueUpdate(const char *name, float value)
{
    printf("# FUNC SET: %s=%+.4f\r\n", name, value);
}

/**********************************************************************************************
 * @brief  请求缓存 Function 参数更新信息
 * @param  name: 参数名
 * @param  value: 参数值
 *********************************************************************************************/
void Vofa_RequestFunctionValueUpdate(const char *name, float value)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_FUNC_VALUE, 0L, name, value);
}

/**********************************************************************************************
 * @brief  输出无感模式切换成功回显
 * @param  mode: 已切换到的无感模式编号，当前从 W0 开始
 *********************************************************************************************/
void Vofa_PrintSensorlessSwitchOk(uint32_t mode)
{
    const char *mode_name = "Unknown";

    switch ((SensorlessControlMode)mode)
    {
        case SENSORLESS_CONTROL_MODE_SPEED:
            mode_name = "Speed";
            break;
        default:
            break;
    }

    printf("# SENSORLESS OK: W%lu -> %s\r\n", (unsigned long)mode, mode_name);
}

/**********************************************************************************************
 * @brief  请求缓存无感模式切换成功信息
 * @param  mode: 需要回显的无感模式编号
 *********************************************************************************************/
void Vofa_RequestSensorlessSwitchOk(uint32_t mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_SENSORLESS_OK, (long)mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出非法无感模式编号回显
 * @param  mode: 用户输入的非法无感模式编号
 *********************************************************************************************/
void Vofa_PrintSensorlessSwitchError(long mode)
{
    printf("# SENSORLESS ERR: W%ld out of range [0,%d]\r\n",
           mode,
           (int)(SENSORLESS_CONTROL_MODE_COUNT - 1));
}

/**********************************************************************************************
 * @brief  请求缓存非法无感模式错误信息
 * @param  mode: 用户输入的非法无感模式编号
 *********************************************************************************************/
void Vofa_RequestSensorlessSwitchError(long mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_SENSORLESS_ERR, mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出无感故障回显
 * @param  fault: 当前无感故障码
 *********************************************************************************************/
void Vofa_PrintSensorlessFault(uint32_t fault)
{
    const char *fault_name = "Unknown";

    switch ((SensorlessFaultType)fault)
    {
        case SENSORLESS_FAULT_NONE:
            fault_name = "None";
            break;
        case SENSORLESS_FAULT_OVERCURRENT:
            fault_name = "OverCurrent";
            break;
        case SENSORLESS_FAULT_UNDERVOLTAGE:
            fault_name = "UnderVoltage";
            break;
        case SENSORLESS_FAULT_STALL:
            fault_name = "Stall";
            break;
        case SENSORLESS_FAULT_OBSERVER_LOST:
            fault_name = "ObserverLost";
            break;
        default:
            break;
    }

    printf("# SENSORLESS FAULT: %lu -> %s\r\n", (unsigned long)fault, fault_name);
}

/**********************************************************************************************
 * @brief  请求缓存无感故障回显
 * @param  fault: 当前无感故障码
 *********************************************************************************************/
void Vofa_RequestSensorlessFault(uint32_t fault)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_SENSORLESS_FAULT, (long)fault, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出 VOFA 调试视图切换成功回显
 * @param  view: 已切换到的视图编号，对应串口 V0~V5
 *********************************************************************************************/
void Vofa_PrintDebugViewSwitchOk(uint32_t view)
{
    const char *view_name = "Unknown";

    switch ((VofaDebugView)view)
    {
        case VOFA_DEBUG_VIEW_DEFAULT:
            view_name = "Default";
            break;
        case VOFA_DEBUG_VIEW_POSVEL:
            view_name = "PosVel";
            break;
        case VOFA_DEBUG_VIEW_POSCUR:
            view_name = "PosCur";
            break;
        case VOFA_DEBUG_VIEW_SPCUR:
            view_name = "SpCur";
            break;
        case VOFA_DEBUG_VIEW_POSVCUR:
            view_name = "PosVelCur";
            break;
        case VOFA_DEBUG_VIEW_SENSORLESS:
            view_name = "Sensorless";
            break;
        default:
            break;
    }

    printf("# VOFA OK: V%lu -> %s\r\n", (unsigned long)view, view_name);
}

/**********************************************************************************************
 * @brief  请求缓存 VOFA 调试视图切换成功信息
 * @param  view: 需要回显的视图编号
 *********************************************************************************************/
void Vofa_RequestDebugViewSwitchOk(uint32_t view)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_VIEW_OK, (long)view, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出非法 VOFA 视图编号回显
 * @param  view: 用户输入的非法视图编号
 *********************************************************************************************/
void Vofa_PrintDebugViewSwitchError(long view)
{
    printf("# VOFA ERR: V%ld out of range [0,%d]\r\n",
           view,
           (int)(VOFA_DEBUG_VIEW_COUNT - 1));
}

/**********************************************************************************************
 * @brief  请求缓存非法 VOFA 视图错误信息
 * @param  view: 用户输入的非法视图编号
 *********************************************************************************************/
void Vofa_RequestDebugViewSwitchError(long view)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_VIEW_ERR, view, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出保护清除成功回显
 *********************************************************************************************/
void Vofa_PrintProtectionClearOk(void)
{
    printf("# PROTECT OK: latched trips cleared\r\n");
}

/**********************************************************************************************
 * @brief  请求缓存保护清除成功回显
 *********************************************************************************************/
void Vofa_RequestProtectionClearOk(void)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_PROTECT_OK, 0L, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出非法保护清除命令回显
 * @param  value: 用户输入的保护命令参数
 *********************************************************************************************/
void Vofa_PrintProtectionClearError(long value)
{
    printf("# PROTECT ERR: C%ld unsupported, use C0\r\n", value);
}

/**********************************************************************************************
 * @brief  请求缓存非法保护命令回显
 * @param  value: 用户输入的保护命令参数
 *********************************************************************************************/
void Vofa_RequestProtectionClearError(long value)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_PROTECT_ERR, value, NULL, 0.0f);
}
