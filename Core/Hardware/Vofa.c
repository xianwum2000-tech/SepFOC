#include "Vofa.h"
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "MT6701.h"
#include "SepFoc.h"
#include "usart.h"

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
    VOFA_TEXT_EVENT_VIEW_OK,
    VOFA_TEXT_EVENT_VIEW_ERR
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
 * @brief  向文本状态队列压入一条消息
 * @param  type: 消息类型
 * @param  integer_value: 整型附加参数
 * @param  name: 文本参数名，可为空
 * @param  float_value: 浮点附加参数
 * @note   允许在中断中调用；队列满时丢弃最新一条，优先保证控制不被打印阻塞。
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
 * @brief  获取当前串口 F 命令写入的目标值
 * @return 最近一次写入的 F 值
 * @note   默认调试页直接回显用户输入的 F 值，这样和串口命令语义保持一致。
 *********************************************************************************************/
static float Vofa_GetCommandTarget(void)
{
    return motor_target_val;
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
            vofa_debug_target = Vofa_GetCommandTarget();
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
 * @brief  将当前缓存好的调试窗口按当前视图输出为 VOFA+ FireWater 文本帧
 * @note   视图由串口 V0~V4 运行时切换，不再依赖编译期宏开关。
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
    vofa_debug_ready = 0U;
    if (!primask)
    {
        __enable_irq();
    }

    (void)mode;

    switch (view)
    {
        case VOFA_DEBUG_VIEW_DEFAULT:
            // V0: F值/速度/Iq/Uq/绝对角度
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   speed_avg,
                   iq_avg,
                   uq_avg,
                   angle_now);
            break;
        case VOFA_DEBUG_VIEW_POSVEL:
            // V1: 位置-速度串级
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   cycle_diff(target - angle_now, 360.0f),
                   posvel_speed_target,
                   speed_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_POSCUR:
            // V2: 位置-电流串级
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   cycle_diff(target - angle_now, 360.0f),
                   poscur_current_target,
                   iq_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_SPCUR:
            // V3: 速度-电流串级
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   speed_avg,
                   (target - speed_avg),
                   spcur_current_target,
                   iq_avg,
                   uq_avg);
            break;
        case VOFA_DEBUG_VIEW_POSVCUR:
            // V4: 位置-速度-电流三环
            printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                   target,
                   angle_now,
                   posvcur_speed_target,
                   speed_avg,
                   posvcur_current_target,
                   iq_avg);
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
 * @param  view: 需要切换到的视图编号
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
 * @brief  获取当前 VOFA 调试视图
 * @return 当前调试视图编号
 *********************************************************************************************/
VofaDebugView Vofa_GetDebugView(void)
{
    return vofa_debug_view;
}

/**********************************************************************************************
 * @brief  在主循环中安全输出一条文本状态消息
 * @note   该函数每次只消费一条消息，避免文本状态回显长时间占用串口发送带宽。
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
        case VOFA_TEXT_EVENT_VIEW_OK:
            Vofa_PrintDebugViewSwitchOk((uint32_t)event.integer_value);
            break;
        case VOFA_TEXT_EVENT_VIEW_ERR:
            Vofa_PrintDebugViewSwitchError(event.integer_value);
            break;
        case VOFA_TEXT_EVENT_NONE:
        default:
            break;
    }
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
 * @brief  请求缓存 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 *********************************************************************************************/
void Vofa_RequestAdcError(uint32_t error_code)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_ADC_ERROR, (long)error_code, NULL, 0.0f);
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
 * @brief  请求缓存模式切换成功信息
 * @param  mode: 需要回显的控制模式编号
 *********************************************************************************************/
void Vofa_RequestModeSwitchOk(uint32_t mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_MODE_OK, (long)mode, NULL, 0.0f);
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

/**********************************************************************************************
 * @brief  请求缓存非法模式错误信息
 * @param  mode: 用户输入的非法控制模式编号
 *********************************************************************************************/
void Vofa_RequestModeSwitchError(long mode)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_MODE_ERR, mode, NULL, 0.0f);
}

/**********************************************************************************************
 * @brief  输出 Function 模块模式切换成功回显
 * @param  mode: 功能模式编号，0=关闭，1=纯阻尼感，2=定格感
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
 * @brief  输出 Function 模块参数更新回显
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
 * @brief  输出 VOFA 调试视图切换成功回显
 * @param  view: 视图编号，和串口 V0~V4 一一对应
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
 * @brief  输出非法 VOFA 调试视图编号回显
 * @param  view: 用户输入的非法视图编号
 *********************************************************************************************/
void Vofa_PrintDebugViewSwitchError(long view)
{
    printf("# VOFA ERR: V%ld out of range [0,%d]\r\n",
           view,
           (int)(VOFA_DEBUG_VIEW_COUNT - 1));
}

/**********************************************************************************************
 * @brief  请求缓存非法 VOFA 调试视图错误信息
 * @param  view: 用户输入的非法视图编号
 *********************************************************************************************/
void Vofa_RequestDebugViewSwitchError(long view)
{
    Vofa_QueueTextEvent(VOFA_TEXT_EVENT_VIEW_ERR, view, NULL, 0.0f);
}
