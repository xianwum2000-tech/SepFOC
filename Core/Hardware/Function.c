#include "Function.h"

#include <math.h>

#include "MT6701.h"
#include "SepFOC.h"

static volatile FunctionControlMode function_control_mode = FUNCTION_CONTROL_NONE;
static volatile float function_last_target_q = 0.0f;
static volatile float function_last_detent_angle = 0.0f;

/* 默认参数尽量先给一组“能感觉到手感变化，但不至于太冲”的初值。 */
static const FunctionPureDampingConfig function_pure_damping_default_config = {0.020f, 0.80f};
static const FunctionDetentConfig function_detent_default_config = {30.0f, 5.0f, 0.10f, 0.80f, 0.020f, 0.80f, 0.060f};
static FunctionPureDampingConfig function_pure_damping_config = {0.020f, 0.80f};
static FunctionDetentConfig function_detent_config = {30.0f, 5.0f, 0.10f, 0.80f, 0.020f, 0.80f, 0.060f};

/**********************************************************************************************
 * @brief  将任意角度归一化到 0~360 度范围
 *********************************************************************************************/
static float Function_NormalizeDegree(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**********************************************************************************************
 * @brief  根据当前角度和档位间隔，寻找最近的定格点
 *********************************************************************************************/
static float Function_GetNearestDetentAngle(float angle_deg, float step_deg)
{
    float index = 0.0f;

    if (step_deg <= 0.0f)
    {
        return Function_NormalizeDegree(angle_deg);
    }

    index = floorf((angle_deg / step_deg) + 0.5f);
    return Function_NormalizeDegree(index * step_deg);
}

/**********************************************************************************************
 * @brief  纯阻尼感快环执行函数
 * @note   本质是按当前速度生成一个反向 Q 轴电流，让电机自由转但带明显黏滞感。
 *********************************************************************************************/
static void Function_RunPureDampingFastLoop(void)
{
    float target_q = -function_pure_damping_config.damping_kp * motor_speed;

    target_q = _constrain(target_q,
                          -function_pure_damping_config.current_limit,
                          function_pure_damping_config.current_limit);

    function_last_target_q = target_q;
    function_last_detent_angle = Get_Angle();
    Sep_Foc_lib_torque_control(target_q);
}

/**********************************************************************************************
 * @brief  定格感快环执行函数
 * @note   先找最近档位，再按误差动态提高吸附力度，靠近档位时会有“咔哒”吸入感。
 *********************************************************************************************/
static void Function_RunDetentFastLoop(void)
{
    float angle_now_deg = Get_Angle();
    float target_detent_deg = Function_GetNearestDetentAngle(angle_now_deg, function_detent_config.detent_step_deg);
    float error_deg = cycle_diff(target_detent_deg - angle_now_deg, 360.0f);
    float abs_error_deg = fabsf(error_deg);
    float kp_use = function_detent_config.base_kp;
    float target_q = 0.0f;

    if (abs_error_deg <= function_detent_config.snap_window_deg)
    {
        kp_use = function_detent_config.snap_kp;
    }

    target_q = kp_use * deg2rad(error_deg);
    target_q -= function_detent_config.damping_gain * motor_speed;

    if ((abs_error_deg <= function_detent_config.snap_window_deg) &&
        (abs_error_deg > 0.01f) &&
        (fabsf(target_q) < function_detent_config.min_snap_current))
    {
        target_q = (error_deg >= 0.0f) ? function_detent_config.min_snap_current
                                       : -function_detent_config.min_snap_current;
    }

    target_q = _constrain(target_q,
                          -function_detent_config.current_limit,
                          function_detent_config.current_limit);

    function_last_target_q = target_q;
    function_last_detent_angle = target_detent_deg;
    Sep_Foc_lib_torque_control(target_q);
}

/**********************************************************************************************
 * @brief  关闭 Function 模块附加控制，并清零输出
 *********************************************************************************************/
void Function_Control_Disable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    function_control_mode = FUNCTION_CONTROL_NONE;
    function_last_target_q = 0.0f;
    function_last_detent_angle = 0.0f;
    if (!primask)
    {
        __enable_irq();
    }

    setTorque(0.0f, _electricalAngle());
}

/**********************************************************************************************
 * @brief  将 Function 模块配置恢复到默认初值
 *********************************************************************************************/
void Function_Control_ResetConfigsToDefault(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    function_pure_damping_config = function_pure_damping_default_config;
    function_detent_config = function_detent_default_config;
    function_last_target_q = 0.0f;
    function_last_detent_angle = 0.0f;
    if (!primask)
    {
        __enable_irq();
    }
}

/**********************************************************************************************
 * @brief  启用纯阻尼感模式
 * @param  config: 阻尼配置
 * @note   启用时会先切到标准模式关闭态，避免和现有控制模式同时输出。
 *********************************************************************************************/
void Function_Control_EnablePureDamping(FunctionPureDampingConfig config)
{
    uint32_t primask = __get_PRIMASK();

    if (config.current_limit <= 0.0f)
    {
        config.current_limit = 0.8f;
    }
    if (config.damping_kp < 0.0f)
    {
        config.damping_kp = 0.0f;
    }

    Sep_FOC_SetControlMode(SEP_FOC_MODE_DISABLED);

    __disable_irq();
    function_pure_damping_config = config;
    function_control_mode = FUNCTION_CONTROL_PURE_DAMPING;
    function_last_target_q = 0.0f;
    function_last_detent_angle = Get_Angle();
    if (!primask)
    {
        __enable_irq();
    }
}

/**********************************************************************************************
 * @brief  用默认参数启用纯阻尼感模式
 *********************************************************************************************/
void Function_Control_EnablePureDampingDefault(void)
{
    Function_Control_ResetConfigsToDefault();
    Function_Control_EnablePureDamping(function_pure_damping_default_config);
}

/**********************************************************************************************
 * @brief  启用定格感模式
 * @param  config: 定格配置
 * @note   启用时会先切到标准模式关闭态，避免和现有控制模式同时输出。
 *********************************************************************************************/
void Function_Control_EnableDetent(FunctionDetentConfig config)
{
    uint32_t primask = __get_PRIMASK();

    if (config.detent_step_deg <= 0.0f)
    {
        config.detent_step_deg = 30.0f;
    }
    if (config.snap_window_deg < 0.0f)
    {
        config.snap_window_deg = 0.0f;
    }
    if (config.current_limit <= 0.0f)
    {
        config.current_limit = 0.8f;
    }
    if (config.min_snap_current < 0.0f)
    {
        config.min_snap_current = 0.0f;
    }

    Sep_FOC_SetControlMode(SEP_FOC_MODE_DISABLED);

    __disable_irq();
    function_detent_config = config;
    function_control_mode = FUNCTION_CONTROL_DETENT;
    function_last_target_q = 0.0f;
    function_last_detent_angle = Function_GetNearestDetentAngle(Get_Angle(), config.detent_step_deg);
    if (!primask)
    {
        __enable_irq();
    }
}

/**********************************************************************************************
 * @brief  用默认参数启用定格感模式
 *********************************************************************************************/
void Function_Control_EnableDetentDefault(void)
{
    Function_Control_ResetConfigsToDefault();
    Function_Control_EnableDetent(function_detent_default_config);
}

/**********************************************************************************************
 * @brief  查询当前是否启用了 Function 模块附加控制
 *********************************************************************************************/
uint8_t Function_Control_IsEnabled(void)
{
    return (function_control_mode != FUNCTION_CONTROL_NONE) ? 1U : 0U;
}

/**********************************************************************************************
 * @brief  读取当前 Function 模块控制模式
 *********************************************************************************************/
FunctionControlMode Function_Control_GetMode(void)
{
    return function_control_mode;
}

/**********************************************************************************************
 * @brief  Function 模块快环入口
 * @note   由 ADC 回调调用时，会根据当前启用的附加控制类型直接复用电流环输出。
 *********************************************************************************************/
void Function_Control_RunFastLoop(void)
{
    switch (function_control_mode)
    {
        case FUNCTION_CONTROL_PURE_DAMPING:
            Function_RunPureDampingFastLoop();
            break;
        case FUNCTION_CONTROL_DETENT:
            Function_RunDetentFastLoop();
            break;
        case FUNCTION_CONTROL_NONE:
        default:
            break;
    }
}

/**********************************************************************************************
 * @brief  在线修改纯阻尼感模式的阻尼系数
 * @param  damping_kp: 新的阻尼系数
 *********************************************************************************************/
void Function_Control_SetPureDampingKp(float damping_kp)
{
    if (damping_kp < 0.0f)
    {
        damping_kp = 0.0f;
    }

    function_pure_damping_config.damping_kp = damping_kp;
}

/**********************************************************************************************
 * @brief  读取当前纯阻尼感模式使用的阻尼系数
 *********************************************************************************************/
float Function_Control_GetPureDampingKp(void)
{
    return function_pure_damping_config.damping_kp;
}

/**********************************************************************************************
 * @brief  读取 Function 模块最近一次输出的目标 Q 电流
 *********************************************************************************************/
float Function_Control_GetLastTargetQ(void)
{
    return function_last_target_q;
}

/**********************************************************************************************
 * @brief  读取定格感模式当前锁定的最近档位角度
 *********************************************************************************************/
float Function_Control_GetLastDetentAngle(void)
{
    return function_last_detent_angle;
}
