#include "SensorlessFOC.h"

#include <math.h>
#include <string.h>

#include "BemfObserver.h"
#include "SensorlessConfig.h"
#include "SepFoc.h"

#ifndef SQRT3
#define SQRT3 1.7320508f
#endif

typedef struct
{
    float offset_ia;
    float offset_ib;
    float sum_ia;
    float sum_ib;
    uint16_t sample_count;
    uint16_t sample_target;
    uint8_t complete;
} CurrentCalibration_t;

typedef struct
{
    float align_voltage;
    float align_angle;
    uint32_t align_duration_ms;
    uint32_t align_timer_ms;
} AlignmentConfig_t;

typedef struct
{
    float start_voltage;
    float end_voltage;
    float target_voltage;
    float current_voltage;
    float acceleration;
    float current_angle;
    float current_speed;
    float max_speed;
    float voltage_ramp_rate;
    float current_limit;
    float current_release;
    float foldback_gain;
    float measured_current_peak;
    uint8_t foldback_active;
} OpenLoopConfig_t;

typedef struct
{
    float blend_ratio;
    float blend_rate;
    uint8_t transition_complete;
} AngleBlender_t;

typedef struct
{
    float overcurrent_threshold;
    float undervoltage_threshold;
    float stall_speed_threshold;
    uint16_t stall_timeout_ms;
    uint16_t stall_counter;
    uint16_t observer_lost_timeout_ms;
    uint16_t observer_lost_counter;
} FaultDetection_t;

/* 无感控制运行时：与现有有感 FOC 分开保存，切换 M/W 时更容易做到互不干扰。 */
static volatile uint8_t sensorless_enabled = 0U;
static volatile SensorlessControlMode sensorless_mode = SENSORLESS_CONTROL_MODE_SPEED;
static volatile SensorlessState_t sensorless_state = SENSORLESS_STATE_IDLE;
static volatile SensorlessFaultType sensorless_fault = SENSORLESS_FAULT_NONE;

static CurrentCalibration_t sensorless_current_calib;
static AlignmentConfig_t sensorless_align_config;
static OpenLoopConfig_t sensorless_openloop_config;
static AngleBlender_t sensorless_blender;
static FaultDetection_t sensorless_fault_detection;
static BemfObserver_t sensorless_observer;

static volatile float sensorless_target_speed = 0.0f;
static volatile float sensorless_target_request = 0.0f;
static volatile float sensorless_target_iq = 0.0f;
static volatile float sensorless_measured_iq = 0.0f;
static volatile float sensorless_output_uq = 0.0f;
static volatile float sensorless_last_angle = 0.0f;
static volatile float sensorless_last_ia = 0.0f;
static volatile float sensorless_last_ib = 0.0f;
static volatile float sensorless_last_i_alpha = 0.0f;
static volatile float sensorless_last_i_beta = 0.0f;

static float sensorless_speed_integral = 0.0f;
static float sensorless_speed_last_error = 0.0f;
static float sensorless_current_integral = 0.0f;
static float sensorless_start_direction = 1.0f;
static uint16_t sensorless_transition_confirm_counter = 0U;
static volatile uint32_t sensorless_debug_event = 0U;
static volatile uint32_t sensorless_debug_reset_count = 0U;
static volatile uint32_t sensorless_debug_openloop_start_count = 0U;

static uint8_t Sensorless_IsObserverUsable(float target_speed, float required_speed);

/**********************************************************************************************
 * @brief  用于内部统一的角度归一化
 *********************************************************************************************/
static float Sensorless_NormalizeAngle(float angle)
{
    return _normalizeAngle(angle);
}

/**********************************************************************************************
 * @brief  读取机械速度到电角速度的换算系数，保持 W 模式与 M 模式的正负方向一致
 *********************************************************************************************/
static float Sensorless_GetSpeedElecGain(void)
{
    return (float)DIR * (float)POLE_PAIRS;
}

/**********************************************************************************************
 * @brief  将机械角速度换算成电角速度，便于内部状态机和观测器统一使用
 *********************************************************************************************/
static float Sensorless_MechToElecSpeed(float mechanical_speed)
{
    return mechanical_speed * Sensorless_GetSpeedElecGain();
}

/**********************************************************************************************
 * @brief  将电角速度还原成机械角速度，便于保持串口 F 值和 VOFA 读数口径一致
 *********************************************************************************************/
static float Sensorless_ElecToMechSpeed(float electrical_speed)
{
    float elec_gain = Sensorless_GetSpeedElecGain();

    if (fabsf(elec_gain) < 1e-6f)
    {
        return 0.0f;
    }

    return electrical_speed / elec_gain;
}

/**********************************************************************************************
 * @brief  读取一个非零方向；目标速度接近 0 时沿用上一次启动方向
 *********************************************************************************************/
static float Sensorless_GetDirection(float target_speed)
{
    float electrical_target = Sensorless_MechToElecSpeed(target_speed);

    if (electrical_target > 0.0f)
    {
        return 1.0f;
    }
    if (electrical_target < 0.0f)
    {
        return -1.0f;
    }
    return sensorless_start_direction;
}

/**********************************************************************************************
 * @brief  按当前目标/速度区间选择分段目标斜坡
 * @note   低中速段允许更快追目标，高速段主动收缓，避免高转区电流阶跃过大。
 *********************************************************************************************/
static float Sensorless_GetTargetSlewRate(float request_speed)
{
    float speed_ref = max(fabsf(request_speed), fabsf(sensorless_target_speed));

    if (speed_ref < SENSORLESS_ACCEL_STAGE_SPEED1)
    {
        return SENSORLESS_TARGET_SLEW_RATE_FAST;
    }
    if (speed_ref < SENSORLESS_ACCEL_STAGE_SPEED2)
    {
        return SENSORLESS_TARGET_SLEW_RATE_MID;
    }
    return SENSORLESS_TARGET_SLEW_RATE_SLOW;
}

/**********************************************************************************************
 * @brief  按当前开环速度选择分段加速度
 * @note   前段加速度大一些，快速把速度拉起来；高转区自动降低加速度，配合限流折返减少过流风险。
 *********************************************************************************************/
static float Sensorless_GetOpenLoopAccelCommand(float current_speed_mech)
{
    float speed_abs = fabsf(current_speed_mech);

    if (speed_abs < SENSORLESS_ACCEL_STAGE_SPEED1)
    {
        return SENSORLESS_OPENLOOP_ACCEL_FAST;
    }
    if (speed_abs < SENSORLESS_ACCEL_STAGE_SPEED2)
    {
        return SENSORLESS_OPENLOOP_ACCEL_MID;
    }
    return SENSORLESS_OPENLOOP_ACCEL_SLOW;
}

/**********************************************************************************************
 * @brief  对外部给定的速度目标做斜坡限制，避免开环/过渡阶段因速度阶跃过大直接拉爆电流
 *********************************************************************************************/
static float Sensorless_SlewTargetSpeed(float request_speed)
{
    float delta = request_speed - sensorless_target_speed;
    float max_step = Sensorless_GetTargetSlewRate(request_speed) * SENSORLESS_SLOW_LOOP_DT;

    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }

    return sensorless_target_speed + delta;
}

/**********************************************************************************************
 * @brief  统计当前三相中的最大相电流绝对值，供开环限流折返使用
 *********************************************************************************************/
static float Sensorless_GetPhaseCurrentPeak(float ia, float ib)
{
    float ic = -(ia + ib);
    float peak = fabsf(ia);

    if (fabsf(ib) > peak)
    {
        peak = fabsf(ib);
    }
    if (fabsf(ic) > peak)
    {
        peak = fabsf(ic);
    }

    return peak;
}

/**********************************************************************************************
 * @brief  直接输出一个给定角度的 d/q 电压，适合无感对齐和开环启动阶段复用
 * @note   对齐阶段需要固定磁链方向，因此这里不能只调用 setTorque(Uq, angle)。
 *********************************************************************************************/
static void Sensorless_ApplyDQVoltage(float Ud, float Uq, float angle_el)
{
    float uq_limit = BUS_VOLTAGE / SQRT3;
    float phase_a = 0.0f;
    float phase_b = 0.0f;
    float phase_c = 0.0f;
    float phase_max = 0.0f;
    float phase_min = 0.0f;
    float common_mode = 0.0f;
    float sin_theta = sinf(angle_el);
    float cos_theta = cosf(angle_el);

    Ud = _constrain(Ud, -uq_limit, uq_limit);
    Uq = _constrain(Uq, -uq_limit, uq_limit);

    FOC.Ualpha = Ud * cos_theta - Uq * sin_theta;
    FOC.Ubeta = Ud * sin_theta + Uq * cos_theta;

    phase_a = FOC.Ualpha;
    phase_b = -0.5f * FOC.Ualpha + 0.5f * SQRT3 * FOC.Ubeta;
    phase_c = -0.5f * FOC.Ualpha - 0.5f * SQRT3 * FOC.Ubeta;
    phase_max = max(phase_a, max(phase_b, phase_c));
    phase_min = min(phase_a, min(phase_b, phase_c));
    common_mode = 0.5f * (phase_max + phase_min);

    FOC.Ua = phase_a - common_mode + BUS_VOLTAGE * 0.5f;
    FOC.Ub = phase_b - common_mode + BUS_VOLTAGE * 0.5f;
    FOC.Uc = phase_c - common_mode + BUS_VOLTAGE * 0.5f;

    setPwm(FOC.Ua, FOC.Ub, FOC.Uc);
}

/**********************************************************************************************
 * @brief  把弧度角转换成 0~360 度，方便串口和 VOFA 观察
 *********************************************************************************************/
static float Sensorless_RadToDeg360(float angle_rad)
{
    float angle_deg = rad2deg(Sensorless_NormalizeAngle(angle_rad));
    if (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

/**********************************************************************************************
 * @brief  计算开环角与观测角的最短路径混合角
 *********************************************************************************************/
static float Sensorless_GetBlendedAngle(float theta_open, float theta_obs)
{
    float angle_diff = cycle_diff(theta_obs - theta_open, _2PI);
    return Sensorless_NormalizeAngle(theta_open + (sensorless_blender.blend_ratio * angle_diff));
}

/**********************************************************************************************
 * @brief  根据当前状态决定快环实际使用哪一个控制角度
 *********************************************************************************************/
static float Sensorless_GetControlAngle(void)
{
    switch (sensorless_state)
    {
        case SENSORLESS_STATE_ALIGN:
            return sensorless_align_config.align_angle;
        case SENSORLESS_STATE_OPENLOOP:
            return sensorless_openloop_config.current_angle;
        case SENSORLESS_STATE_TRANSITION:
            return Sensorless_GetBlendedAngle(sensorless_openloop_config.current_angle,
                                              BemfObserver_GetAngle(&sensorless_observer));
        case SENSORLESS_STATE_CLOSEDLOOP:
            return BemfObserver_GetAngle(&sensorless_observer);
        case SENSORLESS_STATE_IDLE:
        case SENSORLESS_STATE_CURRENT_CALIB:
        case SENSORLESS_STATE_FAULT:
        default:
            return sensorless_last_angle;
    }
}

/**********************************************************************************************
 * @brief  重置无感运行时状态，但保留参数配置
 *********************************************************************************************/
static void Sensorless_ResetRuntime(void)
{
    sensorless_debug_reset_count++;
    sensorless_debug_event = 10U;
    sensorless_state = SENSORLESS_STATE_IDLE;
    sensorless_fault = SENSORLESS_FAULT_NONE;
    sensorless_target_speed = 0.0f;
    sensorless_target_request = 0.0f;
    sensorless_target_iq = 0.0f;
    sensorless_measured_iq = 0.0f;
    sensorless_output_uq = 0.0f;
    sensorless_last_angle = 0.0f;
    sensorless_last_ia = 0.0f;
    sensorless_last_ib = 0.0f;
    sensorless_last_i_alpha = 0.0f;
    sensorless_last_i_beta = 0.0f;
    sensorless_speed_integral = 0.0f;
    sensorless_speed_last_error = 0.0f;
    sensorless_current_integral = 0.0f;
    sensorless_start_direction = 1.0f;
    sensorless_transition_confirm_counter = 0U;

    sensorless_current_calib.offset_ia = 0.0f;
    sensorless_current_calib.offset_ib = 0.0f;
    sensorless_current_calib.sum_ia = 0.0f;
    sensorless_current_calib.sum_ib = 0.0f;
    sensorless_current_calib.sample_count = 0U;
    sensorless_current_calib.complete = 0U;

    sensorless_align_config.align_timer_ms = 0U;

    sensorless_openloop_config.current_voltage = sensorless_openloop_config.start_voltage;
    sensorless_openloop_config.current_angle = sensorless_align_config.align_angle;
    sensorless_openloop_config.current_speed = 0.0f;

    sensorless_blender.blend_ratio = 0.0f;
    sensorless_blender.transition_complete = 0U;

    sensorless_fault_detection.stall_counter = 0U;
    sensorless_fault_detection.observer_lost_counter = 0U;

    BemfObserver_Reset(&sensorless_observer);
}

/**********************************************************************************************
 * @brief  开始一次无感零漂校准
 *********************************************************************************************/
static void Sensorless_CurrentCalib_Start(void)
{
    sensorless_debug_event = 11U;
    sensorless_current_calib.offset_ia = 0.0f;
    sensorless_current_calib.offset_ib = 0.0f;
    sensorless_current_calib.sum_ia = 0.0f;
    sensorless_current_calib.sum_ib = 0.0f;
    sensorless_current_calib.sample_count = 0U;
    sensorless_current_calib.complete = 0U;
}

/**********************************************************************************************
 * @brief  在快环里累计无感零漂校准样本
 *********************************************************************************************/
static void Sensorless_CurrentCalib_Update(float ia_raw, float ib_raw)
{
    if (sensorless_current_calib.complete)
    {
        return;
    }

    sensorless_current_calib.sum_ia += ia_raw;
    sensorless_current_calib.sum_ib += ib_raw;
    sensorless_current_calib.sample_count++;

    if (sensorless_current_calib.sample_count >= sensorless_current_calib.sample_target)
    {
        sensorless_current_calib.offset_ia = sensorless_current_calib.sum_ia / (float)sensorless_current_calib.sample_count;
        sensorless_current_calib.offset_ib = sensorless_current_calib.sum_ib / (float)sensorless_current_calib.sample_count;
        sensorless_current_calib.complete = 1U;
    }
}

/**********************************************************************************************
 * @brief  对齐阶段开始：给转子一个固定角度和固定电压
 *********************************************************************************************/
static void Sensorless_Align_Start(void)
{
    if (sensorless_debug_openloop_start_count > 0U)
    {
        sensorless_align_config.align_duration_ms = SENSORLESS_ALIGN_RESTART_DURATION_MS;
        sensorless_debug_event = 24U;
    }
    else
    {
        sensorless_align_config.align_duration_ms = SENSORLESS_ALIGN_DURATION_MS;
        sensorless_debug_event = 20U;
    }

    sensorless_align_config.align_timer_ms = 0U;
}

/**********************************************************************************************
 * @brief  将无感状态机拉回空闲态，并立即撤销当前输出
 * @param  event_code: 记录到调试页的事件号，便于区分是停机、待机还是其它回退
 * @note   这个入口不重新做零漂校准，只负责把运行时积分、开环速度和电压状态清零。
 *********************************************************************************************/
static void Sensorless_EnterIdle(uint32_t event_code)
{
    sensorless_debug_event = event_code;
    sensorless_state = SENSORLESS_STATE_IDLE;
    sensorless_target_iq = 0.0f;
    sensorless_measured_iq = 0.0f;
    sensorless_output_uq = 0.0f;
    sensorless_speed_integral = 0.0f;
    sensorless_speed_last_error = 0.0f;
    sensorless_current_integral = 0.0f;
    sensorless_transition_confirm_counter = 0U;
    sensorless_blender.blend_ratio = 0.0f;
    sensorless_blender.transition_complete = 0U;
    sensorless_openloop_config.current_speed = 0.0f;
    sensorless_openloop_config.current_voltage = sensorless_openloop_config.start_voltage;
    sensorless_openloop_config.current_angle = sensorless_last_angle;
    Sensorless_ApplyDQVoltage(0.0f, 0.0f, sensorless_last_angle);
}

/**********************************************************************************************
 * @brief  更新对齐阶段计时器
 *********************************************************************************************/
static void Sensorless_Align_Update(void)
{
    if (sensorless_align_config.align_timer_ms < sensorless_align_config.align_duration_ms)
    {
        sensorless_align_config.align_timer_ms++;
    }
}

/**********************************************************************************************
 * @brief  判断对齐阶段是否已满足最小时长
 *********************************************************************************************/
static uint8_t Sensorless_Align_IsComplete(void)
{
    return (sensorless_align_config.align_timer_ms >= sensorless_align_config.align_duration_ms) ? 1U : 0U;
}

/**********************************************************************************************
 * @brief  以当前目标速度方向启动开环加速
 *********************************************************************************************/
static void Sensorless_OpenLoop_Start(float target_speed)
{
    sensorless_debug_openloop_start_count++;
    sensorless_debug_event = 30U;
    sensorless_start_direction = Sensorless_GetDirection(target_speed);
    sensorless_openloop_config.current_voltage = sensorless_openloop_config.start_voltage;
    sensorless_openloop_config.current_speed = 0.0f;
    sensorless_openloop_config.current_angle = sensorless_align_config.align_angle;
}

/**********************************************************************************************
 * @brief  根据目标速度给开环启动估算一个更合适的目标电压
 * @param  target_speed: 当前机械目标速度(rad/s)
 * @param  startup_phase: 1=仍处于起转拉升阶段，0=已经允许按目标速度继续开环运行
 * @note   低速目标不需要像高速目标那样一路灌到最大开环电压，否则很容易在起转阶段直接过流。
 *********************************************************************************************/
static float Sensorless_GetOpenLoopVoltageGoal(float target_speed, float current_speed_mech, uint8_t startup_phase)
{
    float speed_ref = fabsf(target_speed);
    float speed_span = SENSORLESS_OPENLOOP_MAX_SPEED - SENSORLESS_START_REQUEST_SPEED;
    float ratio = 0.0f;
    float voltage_goal = SENSORLESS_OPENLOOP_START_VOLTAGE;

    if (startup_phase && (speed_ref < SENSORLESS_OPENLOOP_TRANSITION_SPEED))
    {
        speed_ref = SENSORLESS_OPENLOOP_TRANSITION_SPEED;
    }
    else if (!startup_phase)
    {
        speed_ref = current_speed_mech + 0.25f * (speed_ref - current_speed_mech);
    }

    if (speed_ref > SENSORLESS_OPENLOOP_MAX_SPEED)
    {
        speed_ref = SENSORLESS_OPENLOOP_MAX_SPEED;
    }

    if (speed_span > 1e-6f)
    {
        ratio = (speed_ref - SENSORLESS_START_REQUEST_SPEED) / speed_span;
    }

    ratio = _constrain(ratio, 0.0f, 1.0f);
    voltage_goal = SENSORLESS_OPENLOOP_START_VOLTAGE +
                   ratio * (sensorless_openloop_config.end_voltage - SENSORLESS_OPENLOOP_START_VOLTAGE);

    if (startup_phase && (voltage_goal < (SENSORLESS_OPENLOOP_START_VOLTAGE + 0.4f)))
    {
        voltage_goal = SENSORLESS_OPENLOOP_START_VOLTAGE + 0.4f;
    }

    return _constrain(voltage_goal,
                      SENSORLESS_OPENLOOP_START_VOLTAGE,
                      sensorless_openloop_config.end_voltage);
}

/**********************************************************************************************
 * @brief  开环/过渡阶段的电流折返：一旦相电流过大，优先主动收一点开环电压，避免直接触发 ADC 过流保护
 *********************************************************************************************/
static void Sensorless_OpenLoop_CurrentFoldback(void)
{
    float overdrive = 0.0f;

    if ((sensorless_state != SENSORLESS_STATE_OPENLOOP) &&
        (sensorless_state != SENSORLESS_STATE_TRANSITION))
    {
        sensorless_openloop_config.foldback_active = 0U;
        return;
    }

    if (sensorless_openloop_config.measured_current_peak > sensorless_openloop_config.current_limit)
    {
        overdrive = sensorless_openloop_config.measured_current_peak - sensorless_openloop_config.current_limit;
        sensorless_openloop_config.current_voltage -= overdrive * sensorless_openloop_config.foldback_gain;
        if (sensorless_openloop_config.current_voltage < sensorless_openloop_config.start_voltage)
        {
            sensorless_openloop_config.current_voltage = sensorless_openloop_config.start_voltage;
        }
        sensorless_openloop_config.foldback_active = 1U;
        sensorless_debug_event = 35U;
    }
    else if (sensorless_openloop_config.measured_current_peak < sensorless_openloop_config.current_release)
    {
        sensorless_openloop_config.foldback_active = 0U;
    }
}

/**********************************************************************************************
 * @brief  1 kHz 下更新开环速度和电压斜坡
 *********************************************************************************************/
static void Sensorless_OpenLoop_UpdateSlow(float target_speed)
{
    float target_speed_el = fabsf(Sensorless_MechToElecSpeed(target_speed));
    float min_transition_speed_el = fabsf(Sensorless_MechToElecSpeed(SENSORLESS_OPENLOOP_TRANSITION_SPEED));
    float min_observer_speed_el = fabsf(Sensorless_MechToElecSpeed(SENSORLESS_OBSERVER_ENABLE_SPEED));
    float current_speed_mech = fabsf(Sensorless_ElecToMechSpeed(sensorless_openloop_config.current_speed));
    float accel_cmd_mech = Sensorless_GetOpenLoopAccelCommand(current_speed_mech);
    float accel_el = fabsf(Sensorless_MechToElecSpeed(accel_cmd_mech));
    float speed_goal = 0.0f;
    float voltage_goal = sensorless_openloop_config.start_voltage;
    uint8_t observer_converged = BemfObserver_IsConverged(&sensorless_observer);
    uint8_t observer_usable = Sensorless_IsObserverUsable(target_speed, SENSORLESS_OPENLOOP_TRANSITION_SPEED);
    uint8_t startup_phase = (current_speed_mech < (SENSORLESS_OPENLOOP_TRANSITION_SPEED * 0.80f)) ? 1U : 0U;

    sensorless_start_direction = Sensorless_GetDirection(target_speed);
    if (!observer_converged && !observer_usable && startup_phase)
    {
        speed_goal = max(target_speed_el, min_transition_speed_el);
        sensorless_debug_event = 30U;
    }
    else if (!observer_converged && !observer_usable)
    {
        speed_goal = target_speed_el;
        sensorless_debug_event = 32U;
    }
    else if (!observer_converged)
    {
        speed_goal = max(target_speed_el, min_observer_speed_el);
        sensorless_debug_event = 33U;
    }
    else
    {
        speed_goal = target_speed_el;
        sensorless_debug_event = 34U;
    }
    speed_goal = min(speed_goal, sensorless_openloop_config.max_speed);
    voltage_goal = Sensorless_GetOpenLoopVoltageGoal(target_speed, current_speed_mech, startup_phase);

    if (sensorless_openloop_config.foldback_active)
    {
        accel_el *= 0.35f;
    }

    sensorless_openloop_config.acceleration = Sensorless_ElecToMechSpeed(accel_el);

    if ((sensorless_start_direction > 0.0f) && (sensorless_openloop_config.current_speed < 0.0f))
    {
        sensorless_openloop_config.current_speed = 0.0f;
    }
    else if ((sensorless_start_direction < 0.0f) && (sensorless_openloop_config.current_speed > 0.0f))
    {
        sensorless_openloop_config.current_speed = 0.0f;
    }

    if (sensorless_openloop_config.current_voltage < sensorless_openloop_config.start_voltage)
    {
        sensorless_openloop_config.current_voltage = sensorless_openloop_config.start_voltage;
    }

    sensorless_openloop_config.current_speed += sensorless_start_direction * accel_el * SENSORLESS_SLOW_LOOP_DT;

    if ((fabsf(target_speed) >= SENSORLESS_START_REQUEST_SPEED) &&
        (fabsf(sensorless_openloop_config.current_speed) < 1e-4f))
    {
        sensorless_openloop_config.current_speed = sensorless_start_direction * accel_el * SENSORLESS_SLOW_LOOP_DT;
        sensorless_debug_event = 31U;
    }

    if (sensorless_start_direction > 0.0f)
    {
        if (sensorless_openloop_config.current_speed > speed_goal)
        {
            sensorless_openloop_config.current_speed = speed_goal;
        }
    }
    else if (sensorless_openloop_config.current_speed < -speed_goal)
    {
        sensorless_openloop_config.current_speed = -speed_goal;
    }

    if (sensorless_openloop_config.current_voltage < voltage_goal)
    {
        sensorless_openloop_config.current_voltage += sensorless_openloop_config.voltage_ramp_rate * SENSORLESS_SLOW_LOOP_DT;
        if (sensorless_openloop_config.current_voltage > voltage_goal)
        {
            sensorless_openloop_config.current_voltage = voltage_goal;
        }
    }
    else if (sensorless_openloop_config.current_voltage > voltage_goal)
    {
        sensorless_openloop_config.current_voltage -= sensorless_openloop_config.voltage_ramp_rate * SENSORLESS_SLOW_LOOP_DT;
        if (sensorless_openloop_config.current_voltage < voltage_goal)
        {
            sensorless_openloop_config.current_voltage = voltage_goal;
        }
    }

    sensorless_openloop_config.target_voltage = voltage_goal;
}

/**********************************************************************************************
 * @brief  计算当前允许进入过渡/闭环时所需的最低机械速度
 * @param  target_speed: 当前目标机械速度(rad/s)
 * @note   起转阶段需要至少到达切换速度；一旦观测器已经可用，则允许按更低目标进入闭环。
 *********************************************************************************************/
static float Sensorless_GetTransitionRequiredSpeed(float target_speed)
{
    float target_abs = fabsf(target_speed);
    float required_speed = 0.0f;

    if (!BemfObserver_IsConverged(&sensorless_observer))
    {
        return SENSORLESS_OPENLOOP_TRANSITION_SPEED;
    }

    required_speed = max(target_abs, SENSORLESS_OBSERVER_ENABLE_SPEED);
    if (required_speed > SENSORLESS_OPENLOOP_TRANSITION_SPEED)
    {
        required_speed = SENSORLESS_OPENLOOP_TRANSITION_SPEED;
    }

    return required_speed;
}

/**********************************************************************************************
 * @brief  判断观测器是否已经达到“可切换”的可用状态
 * @param  target_speed: 当前机械目标速度(rad/s)
 * @param  required_speed: 当前切换要求的最低机械速度(rad/s)
 * @note   这里比 BemfObserver_IsConverged() 更宽松，用来解决低速下观测器已基本可用、
 *         但因为参数模型误差仍达不到“完全收敛”判据，导致始终卡在开环的问题。
 *********************************************************************************************/
static uint8_t Sensorless_IsObserverUsable(float target_speed, float required_speed)
{
    float observer_speed_mech = Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
    float observer_speed_abs = fabsf(observer_speed_mech);
    float bemf_mag = sqrtf(sensorless_observer.e_alpha_filtered * sensorless_observer.e_alpha_filtered +
                           sensorless_observer.e_beta_filtered * sensorless_observer.e_beta_filtered);
    float current_err_mag = sqrtf(sensorless_observer.last_i_alpha_err * sensorless_observer.last_i_alpha_err +
                                  sensorless_observer.last_i_beta_err * sensorless_observer.last_i_beta_err);
    float min_speed = max(required_speed * 0.55f, SENSORLESS_OBSERVER_ENABLE_SPEED);
    float target_dir = Sensorless_GetDirection(target_speed);

    if ((observer_speed_abs > 1000000.0f) ||
        (bemf_mag > 1000000.0f) ||
        (current_err_mag > 1000000.0f))
    {
        return 0U;
    }

    if (observer_speed_abs < min_speed)
    {
        return 0U;
    }

    if (bemf_mag < SENSORLESS_SMO_USABLE_BEMF)
    {
        return 0U;
    }

    if (current_err_mag > SENSORLESS_SMO_USABLE_CURRENT_ERR)
    {
        return 0U;
    }

    if ((target_dir * observer_speed_mech) < 0.0f)
    {
        return 0U;
    }

    return 1U;
}

/**********************************************************************************************
 * @brief  16 kHz 下细分更新开环角度，让开环转子角推进更平滑
 *********************************************************************************************/
static void Sensorless_OpenLoop_UpdateFast(void)
{
    sensorless_openloop_config.current_angle =
        Sensorless_NormalizeAngle(sensorless_openloop_config.current_angle +
                                  (sensorless_openloop_config.current_speed * SENSORLESS_FAST_LOOP_DT));
}

/**********************************************************************************************
 * @brief  开始开环到闭环的角度混合阶段
 *********************************************************************************************/
static void Sensorless_Blender_Start(void)
{
    sensorless_debug_event = 40U;
    sensorless_blender.blend_ratio = 0.0f;
    sensorless_blender.transition_complete = 0U;
}

/**********************************************************************************************
 * @brief  1 kHz 下推进一次角度混合比例
 *********************************************************************************************/
static void Sensorless_Blender_Update(void)
{
    if (sensorless_blender.transition_complete)
    {
        return;
    }

    sensorless_blender.blend_ratio += sensorless_blender.blend_rate;
    if (sensorless_blender.blend_ratio >= 1.0f)
    {
        sensorless_blender.blend_ratio = 1.0f;
        sensorless_blender.transition_complete = 1U;
    }
}

/**********************************************************************************************
 * @brief  速度外环：把目标速度转换成目标 Iq
 *********************************************************************************************/
static void Sensorless_UpdateSpeedLoop(float target_speed)
{
    float measured_speed = Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
    float speed_error = target_speed - measured_speed;
    float p_term = SENSORLESS_SPEED_LOOP_KP * speed_error;
    float d_term = SENSORLESS_SPEED_LOOP_KD * (speed_error - sensorless_speed_last_error);

    sensorless_speed_integral += SENSORLESS_SPEED_LOOP_KI * speed_error;
    sensorless_speed_integral = _constrain(sensorless_speed_integral,
                                           -SENSORLESS_SPEED_LOOP_CURRENT_LIMIT,
                                           SENSORLESS_SPEED_LOOP_CURRENT_LIMIT);

    sensorless_target_iq = p_term + sensorless_speed_integral + d_term;
    sensorless_target_iq = _constrain(sensorless_target_iq,
                                      -SENSORLESS_SPEED_LOOP_CURRENT_LIMIT,
                                      SENSORLESS_SPEED_LOOP_CURRENT_LIMIT);
    sensorless_speed_last_error = speed_error;
}

/**********************************************************************************************
 * @brief  电流内环：把目标 Iq 转成 Uq，并直接驱动 setTorque
 *********************************************************************************************/
static void Sensorless_RunCurrentLoop(float angle_el)
{
    float error_q = 0.0f;
    float uq_prop = 0.0f;

    sensorless_measured_iq = cal_Iq_raw(sensorless_last_ia, sensorless_last_ib, angle_el);
    error_q = sensorless_target_iq - sensorless_measured_iq;

    uq_prop = SENSORLESS_CURRENT_LOOP_KP * error_q;
    sensorless_current_integral += SENSORLESS_CURRENT_LOOP_KI * error_q;
    sensorless_current_integral = _constrain(sensorless_current_integral,
                                             -SENSORLESS_CURRENT_LOOP_UQ_LIMIT,
                                             SENSORLESS_CURRENT_LOOP_UQ_LIMIT);

    sensorless_output_uq = _constrain(uq_prop + sensorless_current_integral,
                                      -SENSORLESS_CURRENT_LOOP_UQ_LIMIT,
                                      SENSORLESS_CURRENT_LOOP_UQ_LIMIT);
    setTorque(sensorless_output_uq, angle_el);
}

/**********************************************************************************************
 * @brief  统一进入无感故障态，并立刻清零输出
 *********************************************************************************************/
static void Sensorless_EnterFault(SensorlessFaultType fault)
{
    sensorless_debug_event = 70U + (uint32_t)fault;
    sensorless_fault = fault;
    sensorless_state = SENSORLESS_STATE_FAULT;
    sensorless_target_iq = 0.0f;
    sensorless_measured_iq = 0.0f;
    sensorless_output_uq = 0.0f;
    sensorless_speed_integral = 0.0f;
    sensorless_speed_last_error = 0.0f;
    sensorless_current_integral = 0.0f;
    Sensorless_ApplyDQVoltage(0.0f, 0.0f, sensorless_last_angle);
}

/**********************************************************************************************
 * @brief  检查当前无感控制是否触发失步/堵转等专有故障
 *********************************************************************************************/
static SensorlessFaultType Sensorless_CheckFaults(void)
{
    float closed_required_speed = max(fabsf(sensorless_target_speed), SENSORLESS_OBSERVER_ENABLE_SPEED);

    if (sensorless_fault != SENSORLESS_FAULT_NONE)
    {
        return sensorless_fault;
    }

    if ((sensorless_fault_detection.undervoltage_threshold > 0.0f) &&
        (BUS_VOLTAGE < sensorless_fault_detection.undervoltage_threshold))
    {
        return SENSORLESS_FAULT_UNDERVOLTAGE;
    }

    if (sensorless_state == SENSORLESS_STATE_CLOSEDLOOP)
    {
        if (!BemfObserver_IsConverged(&sensorless_observer) &&
            !Sensorless_IsObserverUsable(sensorless_target_speed, closed_required_speed))
        {
            sensorless_fault_detection.observer_lost_counter++;
            if (sensorless_fault_detection.observer_lost_counter >= sensorless_fault_detection.observer_lost_timeout_ms)
            {
                return SENSORLESS_FAULT_OBSERVER_LOST;
            }
        }
        else
        {
            sensorless_fault_detection.observer_lost_counter = 0U;
        }
    }
    else
    {
        sensorless_fault_detection.observer_lost_counter = 0U;
    }

    if ((sensorless_state == SENSORLESS_STATE_CLOSEDLOOP) &&
        (fabsf(sensorless_target_speed) > SENSORLESS_START_REQUEST_SPEED))
    {
        if (fabsf(Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer))) <
            sensorless_fault_detection.stall_speed_threshold)
        {
            sensorless_fault_detection.stall_counter++;
            if (sensorless_fault_detection.stall_counter >= sensorless_fault_detection.stall_timeout_ms)
            {
                return SENSORLESS_FAULT_STALL;
            }
        }
        else
        {
            sensorless_fault_detection.stall_counter = 0U;
        }
    }
    else
    {
        sensorless_fault_detection.stall_counter = 0U;
    }

    return SENSORLESS_FAULT_NONE;
}

/**********************************************************************************************
 * @brief  判断观测器是否已满足进入角度混合阶段的条件
 * @note   这里要求开环速度、观测速率和方向基本一致，避免假收敛把状态机过早拉进过渡态。
 *********************************************************************************************/
static uint8_t Sensorless_IsTransitionReady(float target_speed)
{
    float open_speed_mech = fabsf(Sensorless_ElecToMechSpeed(sensorless_openloop_config.current_speed));
    float observer_speed_mech = Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
    float observer_speed_abs = fabsf(observer_speed_mech);
    float speed_ratio = 0.0f;
    float required_speed = Sensorless_GetTransitionRequiredSpeed(target_speed);
    uint8_t observer_converged = BemfObserver_IsConverged(&sensorless_observer);
    uint8_t observer_usable = Sensorless_IsObserverUsable(target_speed, required_speed);
    float min_speed_ratio = observer_converged ? SENSORLESS_TRANSITION_MIN_SPEED_RATIO :
                                               SENSORLESS_TRANSITION_USABLE_MIN_SPEED_RATIO;
    float max_speed_ratio = observer_converged ? SENSORLESS_TRANSITION_MAX_SPEED_RATIO :
                                               SENSORLESS_TRANSITION_USABLE_MAX_SPEED_RATIO;

    if (!(observer_converged || observer_usable))
    {
        return 0U;
    }

    if (open_speed_mech < required_speed)
    {
        return 0U;
    }

    if (observer_speed_abs < (open_speed_mech * min_speed_ratio))
    {
        return 0U;
    }

    if (open_speed_mech > 1e-6f)
    {
        speed_ratio = observer_speed_abs / open_speed_mech;
        if (speed_ratio > max_speed_ratio)
        {
            return 0U;
        }
    }

    if ((sensorless_openloop_config.current_speed * BemfObserver_GetSpeed(&sensorless_observer)) < 0.0f)
    {
        return 0U;
    }

    return 1U;
}

/**********************************************************************************************
 * @brief  判断过渡阶段是否还能继续保持，不满足时直接退回开环继续拉升
 *********************************************************************************************/
static uint8_t Sensorless_IsTransitionMaintained(float target_speed)
{
    float open_speed_mech = fabsf(Sensorless_ElecToMechSpeed(sensorless_openloop_config.current_speed));
    float observer_speed_mech = Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
    float observer_speed_abs = fabsf(observer_speed_mech);
    float required_speed = max(Sensorless_GetTransitionRequiredSpeed(target_speed), SENSORLESS_START_REQUEST_SPEED);
    uint8_t observer_converged = BemfObserver_IsConverged(&sensorless_observer);
    uint8_t observer_usable = Sensorless_IsObserverUsable(target_speed, required_speed);
    float min_speed_ratio = observer_converged ? 0.30f : 0.15f;

    if (!(observer_converged || observer_usable))
    {
        return 0U;
    }

    if (open_speed_mech < required_speed)
    {
        return 0U;
    }

    if (observer_speed_abs < (open_speed_mech * min_speed_ratio))
    {
        return 0U;
    }

    if ((sensorless_openloop_config.current_speed * BemfObserver_GetSpeed(&sensorless_observer)) < 0.0f)
    {
        return 0U;
    }

    return 1U;
}

/**********************************************************************************************
 * @brief  用默认参数恢复无感配置，并顺手清空运行时状态
 *********************************************************************************************/
void Sensorless_FOC_ResetConfigsToDefault(void)
{
    sensorless_current_calib.sample_target = SENSORLESS_CURRENT_CALIB_SAMPLES;

    sensorless_align_config.align_voltage = SENSORLESS_ALIGN_VOLTAGE;
    sensorless_align_config.align_angle = SENSORLESS_ALIGN_ANGLE_RAD;
    sensorless_align_config.align_duration_ms = SENSORLESS_ALIGN_DURATION_MS;
    sensorless_align_config.align_timer_ms = 0U;

    sensorless_openloop_config.start_voltage = SENSORLESS_OPENLOOP_START_VOLTAGE;
    sensorless_openloop_config.end_voltage = SENSORLESS_OPENLOOP_END_VOLTAGE;
    sensorless_openloop_config.target_voltage = SENSORLESS_OPENLOOP_START_VOLTAGE;
    sensorless_openloop_config.current_voltage = SENSORLESS_OPENLOOP_START_VOLTAGE;
    sensorless_openloop_config.acceleration = SENSORLESS_OPENLOOP_ACCEL_FAST;
    sensorless_openloop_config.current_angle = SENSORLESS_ALIGN_ANGLE_RAD;
    sensorless_openloop_config.current_speed = 0.0f;
    sensorless_openloop_config.max_speed = fabsf(Sensorless_MechToElecSpeed(SENSORLESS_OPENLOOP_MAX_SPEED));
    sensorless_openloop_config.voltage_ramp_rate =
        (SENSORLESS_OPENLOOP_END_VOLTAGE - SENSORLESS_OPENLOOP_START_VOLTAGE) / 0.5f;
    sensorless_openloop_config.current_limit = SENSORLESS_OPENLOOP_CURRENT_LIMIT;
    sensorless_openloop_config.current_release = SENSORLESS_OPENLOOP_CURRENT_RELEASE;
    sensorless_openloop_config.foldback_gain = SENSORLESS_OPENLOOP_FOLDBACK_GAIN;
    sensorless_openloop_config.measured_current_peak = 0.0f;
    sensorless_openloop_config.foldback_active = 0U;

    sensorless_blender.blend_ratio = 0.0f;
    sensorless_blender.blend_rate = SENSORLESS_BLEND_RATE;
    sensorless_blender.transition_complete = 0U;

    sensorless_fault_detection.overcurrent_threshold = MAX_CURRENT;
    sensorless_fault_detection.undervoltage_threshold = SENSORLESS_UNDERVOLTAGE_THRESHOLD;
    sensorless_fault_detection.stall_speed_threshold = SENSORLESS_STALL_SPEED_THRESHOLD;
    sensorless_fault_detection.stall_timeout_ms = SENSORLESS_STALL_TIMEOUT_MS;
    sensorless_fault_detection.stall_counter = 0U;
    sensorless_fault_detection.observer_lost_timeout_ms = SENSORLESS_OBSERVER_LOST_TIMEOUT_MS;
    sensorless_fault_detection.observer_lost_counter = 0U;

    BemfObserver_Init(&sensorless_observer);
    Sensorless_ResetRuntime();
}

/**********************************************************************************************
 * @brief  使能无感 FOC 模式
 *********************************************************************************************/
void Sensorless_FOC_Enable(SensorlessControlMode mode)
{
    if (mode >= SENSORLESS_CONTROL_MODE_COUNT)
    {
        mode = SENSORLESS_CONTROL_MODE_SPEED;
    }

    Sep_FOC_SetControlMode(SEP_FOC_MODE_DISABLED);
    Sensorless_ResetRuntime();
    Sensorless_CurrentCalib_Start();

    sensorless_enabled = 1U;
    sensorless_mode = mode;
    sensorless_state = SENSORLESS_STATE_CURRENT_CALIB;
    sensorless_last_angle = sensorless_align_config.align_angle;
}

/**********************************************************************************************
 * @brief  关闭无感 FOC，并把输出拉回安全零值
 *********************************************************************************************/
void Sensorless_FOC_Disable(void)
{
    sensorless_enabled = 0U;
    Sensorless_ResetRuntime();
    Sensorless_ApplyDQVoltage(0.0f, 0.0f, 0.0f);
}

/**********************************************************************************************
 * @brief  查询当前是否启用了无感 FOC
 *********************************************************************************************/
uint8_t Sensorless_FOC_IsEnabled(void)
{
    return sensorless_enabled;
}

/**********************************************************************************************
 * @brief  读取当前无感控制模式
 *********************************************************************************************/
SensorlessControlMode Sensorless_FOC_GetMode(void)
{
    return sensorless_mode;
}

/**********************************************************************************************
 * @brief  读取当前无感控制模式名称
 *********************************************************************************************/
const char *Sensorless_FOC_GetModeName(void)
{
    switch (sensorless_mode)
    {
        case SENSORLESS_CONTROL_MODE_SPEED:
            return "Speed";
        default:
            return "Unknown";
    }
}

/**********************************************************************************************
 * @brief  读取当前无感状态机状态
 *********************************************************************************************/
SensorlessState_t Sensorless_FOC_GetState(void)
{
    return sensorless_state;
}

/**********************************************************************************************
 * @brief  读取当前无感状态机状态名称
 *********************************************************************************************/
const char *Sensorless_FOC_GetStateName(void)
{
    switch (sensorless_state)
    {
        case SENSORLESS_STATE_IDLE:
            return "Idle";
        case SENSORLESS_STATE_CURRENT_CALIB:
            return "CurrentCalib";
        case SENSORLESS_STATE_ALIGN:
            return "Align";
        case SENSORLESS_STATE_OPENLOOP:
            return "OpenLoop";
        case SENSORLESS_STATE_TRANSITION:
            return "Transition";
        case SENSORLESS_STATE_CLOSEDLOOP:
            return "ClosedLoop";
        case SENSORLESS_STATE_FAULT:
            return "Fault";
        default:
            return "Unknown";
    }
}

/**********************************************************************************************
 * @brief  读取当前无感故障名称，便于串口或调试页直接展示
 *********************************************************************************************/
const char *Sensorless_FOC_GetFaultName(void)
{
    switch (sensorless_fault)
    {
        case SENSORLESS_FAULT_NONE:
            return "None";
        case SENSORLESS_FAULT_OVERCURRENT:
            return "OverCurrent";
        case SENSORLESS_FAULT_UNDERVOLTAGE:
            return "UnderVoltage";
        case SENSORLESS_FAULT_STALL:
            return "Stall";
        case SENSORLESS_FAULT_OBSERVER_LOST:
            return "ObserverLost";
        default:
            return "Unknown";
    }
}

/**********************************************************************************************
 * @brief  读取某个无感模式切入时的安全保持目标
 *********************************************************************************************/
float Sensorless_FOC_GetModeHoldTarget(SensorlessControlMode mode)
{
    (void)mode;
    return 0.0f;
}

/**********************************************************************************************
 * @brief  读取无感慢环执行分频
 *********************************************************************************************/
uint8_t Sensorless_FOC_GetSlowLoopDivider(void)
{
    return (uint8_t)SENSORLESS_SLOW_LOOP_DIV;
}

/**********************************************************************************************
 * @brief  无感 FOC 快环入口
 *********************************************************************************************/
void Sensorless_FOC_RunFastLoop(float ia_raw, float ib_raw, float ia, float ib)
{
    float ia_corr = 0.0f;
    float ib_corr = 0.0f;
    float angle_use = 0.0f;
    float align_ud = 0.0f;
    uint8_t observer_active = 0U;

    if (!sensorless_enabled)
    {
        return;
    }

    if (sensorless_state == SENSORLESS_STATE_CURRENT_CALIB)
    {
        Sensorless_CurrentCalib_Update(ia_raw, ib_raw);
    }

    ia_corr = ia - sensorless_current_calib.offset_ia;
    ib_corr = ib - sensorless_current_calib.offset_ib;

    sensorless_last_ia = ia_corr;
    sensorless_last_ib = ib_corr;
    sensorless_last_i_alpha = ia_corr;
    sensorless_last_i_beta = 0.57735027f * ia_corr + 1.15470054f * ib_corr;
    sensorless_openloop_config.measured_current_peak = Sensorless_GetPhaseCurrentPeak(ia_corr, ib_corr);

    observer_active = (((sensorless_state == SENSORLESS_STATE_OPENLOOP) ||
                        (sensorless_state == SENSORLESS_STATE_TRANSITION)) &&
                       (fabsf(sensorless_openloop_config.current_speed) >=
                        fabsf(Sensorless_MechToElecSpeed(SENSORLESS_OBSERVER_ENABLE_SPEED)))) ||
                      (sensorless_state == SENSORLESS_STATE_CLOSEDLOOP);

    if (observer_active)
    {
        BemfObserver_Update(&sensorless_observer,
                            sensorless_last_i_alpha,
                            sensorless_last_i_beta,
                            FOC.Ualpha,
                            FOC.Ubeta,
                            SENSORLESS_FAST_LOOP_DT);
    }

    switch (sensorless_state)
    {
        case SENSORLESS_STATE_CURRENT_CALIB:
        case SENSORLESS_STATE_IDLE:
            sensorless_target_iq = 0.0f;
            sensorless_measured_iq = 0.0f;
            sensorless_output_uq = 0.0f;
            Sensorless_ApplyDQVoltage(0.0f, 0.0f, sensorless_last_angle);
            break;

        case SENSORLESS_STATE_ALIGN:
            align_ud = Sensorless_Align_IsComplete() ? 0.0f : sensorless_align_config.align_voltage;
            sensorless_last_angle = sensorless_align_config.align_angle;
            sensorless_target_iq = 0.0f;
            sensorless_measured_iq = cal_Iq_raw(ia_corr, ib_corr, sensorless_last_angle);
            sensorless_output_uq = 0.0f;
            Sensorless_ApplyDQVoltage(align_ud, 0.0f, sensorless_last_angle);
            break;

        case SENSORLESS_STATE_OPENLOOP:
            Sensorless_OpenLoop_UpdateFast();
            Sensorless_OpenLoop_CurrentFoldback();
            sensorless_last_angle = sensorless_openloop_config.current_angle;
            sensorless_target_iq = 0.0f;
            sensorless_measured_iq = cal_Iq_raw(ia_corr, ib_corr, sensorless_last_angle);
            sensorless_output_uq = sensorless_openloop_config.current_voltage;
            Sensorless_ApplyDQVoltage(0.0f, sensorless_openloop_config.current_voltage, sensorless_last_angle);
            break;

        case SENSORLESS_STATE_TRANSITION:
            Sensorless_OpenLoop_UpdateFast();
            Sensorless_OpenLoop_CurrentFoldback();
            angle_use = Sensorless_GetBlendedAngle(sensorless_openloop_config.current_angle,
                                                   BemfObserver_GetAngle(&sensorless_observer));
            sensorless_last_angle = angle_use;
            sensorless_target_iq = 0.0f;
            sensorless_measured_iq = cal_Iq_raw(ia_corr, ib_corr, angle_use);
            sensorless_output_uq = sensorless_openloop_config.current_voltage;
            Sensorless_ApplyDQVoltage(0.0f, sensorless_openloop_config.current_voltage, angle_use);
            break;

        case SENSORLESS_STATE_CLOSEDLOOP:
            angle_use = BemfObserver_GetAngle(&sensorless_observer);
            sensorless_last_angle = angle_use;
            Sensorless_RunCurrentLoop(angle_use);
            break;

        case SENSORLESS_STATE_FAULT:
        default:
            Sensorless_ApplyDQVoltage(0.0f, 0.0f, sensorless_last_angle);
            break;
    }
}

/**********************************************************************************************
 * @brief  无感 FOC 慢环入口
 *********************************************************************************************/
void Sensorless_FOC_RunSlowLoop(float target)
{
    float angle_error_deg = 0.0f;
    SensorlessFaultType fault = SENSORLESS_FAULT_NONE;
    float target_abs = 0.0f;

    if (!sensorless_enabled)
    {
        return;
    }

    sensorless_target_request = target;
    target_abs = fabsf(sensorless_target_request);

    if ((sensorless_state != SENSORLESS_STATE_CURRENT_CALIB) &&
        (sensorless_state != SENSORLESS_STATE_FAULT) &&
        (target_abs <= SENSORLESS_STOP_REQUEST_SPEED))
    {
        Sensorless_EnterIdle(90U);
        return;
    }

    sensorless_target_speed = Sensorless_SlewTargetSpeed(sensorless_target_request);
    target_abs = fabsf(sensorless_target_speed);

    switch (sensorless_state)
    {
        case SENSORLESS_STATE_CURRENT_CALIB:
            if (sensorless_current_calib.complete)
            {
                if (target_abs >= SENSORLESS_START_REQUEST_SPEED)
                {
                    sensorless_debug_event = 21U;
                    sensorless_state = SENSORLESS_STATE_ALIGN;
                    Sensorless_Align_Start();
                }
                else
                {
                    Sensorless_EnterIdle(12U);
                }
            }
            break;

        case SENSORLESS_STATE_IDLE:
            if (target_abs >= SENSORLESS_START_REQUEST_SPEED)
            {
                sensorless_debug_event = 22U;
                sensorless_state = SENSORLESS_STATE_ALIGN;
                Sensorless_Align_Start();
            }
            break;

        case SENSORLESS_STATE_ALIGN:
            if (target_abs < SENSORLESS_START_REQUEST_SPEED)
            {
                Sensorless_EnterIdle(23U);
                break;
            }
            Sensorless_Align_Update();
            if (Sensorless_Align_IsComplete() &&
                (target_abs >= SENSORLESS_START_REQUEST_SPEED))
            {
                Sensorless_OpenLoop_Start(sensorless_target_speed);
                BemfObserver_Seed(&sensorless_observer,
                                  sensorless_openloop_config.current_angle,
                                  sensorless_openloop_config.current_speed);
                sensorless_state = SENSORLESS_STATE_OPENLOOP;
            }
            break;

        case SENSORLESS_STATE_OPENLOOP:
            Sensorless_OpenLoop_UpdateSlow(sensorless_target_speed);
            if (Sensorless_IsTransitionReady(sensorless_target_speed))
            {
                if (sensorless_transition_confirm_counter < 0xFFFFU)
                {
                    sensorless_transition_confirm_counter++;
                }

                if (sensorless_transition_confirm_counter >= SENSORLESS_TRANSITION_CONFIRM_TICKS)
                {
                    BemfObserver_Seed(&sensorless_observer,
                                      sensorless_openloop_config.current_angle,
                                      sensorless_openloop_config.current_speed);
                    Sensorless_Blender_Start();
                    sensorless_state = SENSORLESS_STATE_TRANSITION;
                }
            }
            else
            {
                sensorless_transition_confirm_counter = 0U;
            }
            break;

        case SENSORLESS_STATE_TRANSITION:
            Sensorless_OpenLoop_UpdateSlow(sensorless_target_speed);
            if (!Sensorless_IsTransitionMaintained(sensorless_target_speed))
            {
                sensorless_debug_event = 50U;
                sensorless_transition_confirm_counter = 0U;
                sensorless_blender.blend_ratio = 0.0f;
                sensorless_blender.transition_complete = 0U;
                sensorless_state = SENSORLESS_STATE_OPENLOOP;
                break;
            }
            Sensorless_Blender_Update();
            angle_error_deg = fabsf(rad2deg(cycle_diff(BemfObserver_GetAngle(&sensorless_observer) -
                                                       sensorless_openloop_config.current_angle,
                                                       _2PI)));
            if (sensorless_blender.transition_complete &&
                (angle_error_deg <= SENSORLESS_TRANSITION_MAX_ANGLE_DEG))
            {
                sensorless_debug_event = 60U;
                sensorless_transition_confirm_counter = 0U;
                sensorless_target_iq = 0.0f;
                sensorless_current_integral = 0.0f;
                sensorless_speed_integral = 0.0f;
                sensorless_speed_last_error = 0.0f;
                sensorless_state = SENSORLESS_STATE_CLOSEDLOOP;
            }
            break;

        case SENSORLESS_STATE_CLOSEDLOOP:
            if (sensorless_mode == SENSORLESS_CONTROL_MODE_SPEED)
            {
                Sensorless_UpdateSpeedLoop(sensorless_target_speed);
            }
            break;

        case SENSORLESS_STATE_FAULT:
        default:
            break;
    }

    fault = Sensorless_CheckFaults();
    if (fault != SENSORLESS_FAULT_NONE)
    {
        Sensorless_EnterFault(fault);
    }
}

/**********************************************************************************************
 * @brief  外部强制注入一个无感故障
 *********************************************************************************************/
void Sensorless_FOC_ForceFault(SensorlessFaultType fault)
{
    if (!sensorless_enabled)
    {
        return;
    }

    Sensorless_EnterFault(fault);
}

/**********************************************************************************************
 * @brief  清除当前无感故障；若仍处于 W 模式，则直接重新从校准阶段开始
 *********************************************************************************************/
void Sensorless_FOC_ClearFault(void)
{
    if (!sensorless_enabled)
    {
        sensorless_debug_event = 80U;
        sensorless_fault = SENSORLESS_FAULT_NONE;
        return;
    }

    sensorless_debug_event = 81U;
    Sensorless_FOC_Enable(sensorless_mode);
}

/**********************************************************************************************
 * @brief  读取当前无感故障码
 *********************************************************************************************/
SensorlessFaultType Sensorless_FOC_GetFault(void)
{
    return sensorless_fault;
}

/**********************************************************************************************
 * @brief  读取当前开环电压指令
 *********************************************************************************************/
float Sensorless_FOC_GetOpenLoopVoltage(void)
{
    return sensorless_openloop_config.current_voltage;
}

/**********************************************************************************************
 * @brief  读取无感内部最近一次关键事件编号
 *********************************************************************************************/
uint32_t Sensorless_FOC_GetDebugEvent(void)
{
    return sensorless_debug_event;
}

/**********************************************************************************************
 * @brief  读取运行时重置计数
 *********************************************************************************************/
uint32_t Sensorless_FOC_GetDebugResetCount(void)
{
    return sensorless_debug_reset_count;
}

/**********************************************************************************************
 * @brief  读取开环启动计数
 *********************************************************************************************/
uint32_t Sensorless_FOC_GetDebugOpenLoopStartCount(void)
{
    return sensorless_debug_openloop_start_count;
}

/**********************************************************************************************
 * @brief  读取当前开环加速度配置（机械 rad/s^2）
 *********************************************************************************************/
float Sensorless_FOC_GetOpenLoopAcceleration(void)
{
    return sensorless_openloop_config.acceleration;
}

/**********************************************************************************************
 * @brief  读取当前无感启动方向
 *********************************************************************************************/
float Sensorless_FOC_GetStartDirection(void)
{
    return sensorless_start_direction;
}

/**********************************************************************************************
 * @brief  无感调试 getter：输出当前使用的控制角度（度）
 *********************************************************************************************/
float Sensorless_FOC_GetDebugAngleDeg(void)
{
    return Sensorless_RadToDeg360(Sensorless_GetControlAngle());
}

/**********************************************************************************************
 * @brief  无感调试 getter：输出当前使用的控制速度（rad/s）
 *********************************************************************************************/
float Sensorless_FOC_GetDebugSpeed(void)
{
    if ((sensorless_state == SENSORLESS_STATE_CLOSEDLOOP) ||
        (sensorless_state == SENSORLESS_STATE_TRANSITION))
    {
        return Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
    }

    return Sensorless_ElecToMechSpeed(sensorless_openloop_config.current_speed);
}

/**********************************************************************************************
 * @brief  读取无感模式目标速度（rad/s）
 *********************************************************************************************/
float Sensorless_FOC_GetTargetSpeed(void)
{
    return sensorless_target_speed;
}

/**********************************************************************************************
 * @brief  读取当前电流环目标 Iq（A）
 *********************************************************************************************/
float Sensorless_FOC_GetTargetIq(void)
{
    return sensorless_target_iq;
}

/**********************************************************************************************
 * @brief  读取当前实测 Iq（A）
 *********************************************************************************************/
float Sensorless_FOC_GetMeasuredIq(void)
{
    return sensorless_measured_iq;
}

/**********************************************************************************************
 * @brief  读取当前无感控制输出 Uq（V）
 *********************************************************************************************/
float Sensorless_FOC_GetOutputUq(void)
{
    return sensorless_output_uq;
}

/**********************************************************************************************
 * @brief  读取开环角度（度）
 *********************************************************************************************/
float Sensorless_FOC_GetOpenLoopAngleDeg(void)
{
    return Sensorless_RadToDeg360(sensorless_openloop_config.current_angle);
}

/**********************************************************************************************
 * @brief  读取观测器角度（度）
 *********************************************************************************************/
float Sensorless_FOC_GetObserverAngleDeg(void)
{
    return Sensorless_RadToDeg360(BemfObserver_GetAngle(&sensorless_observer));
}

/**********************************************************************************************
 * @brief  读取开环速度（rad/s）
 *********************************************************************************************/
float Sensorless_FOC_GetOpenLoopSpeed(void)
{
    return Sensorless_ElecToMechSpeed(sensorless_openloop_config.current_speed);
}

/**********************************************************************************************
 * @brief  读取观测器速度（rad/s）
 *********************************************************************************************/
float Sensorless_FOC_GetObserverSpeed(void)
{
    return Sensorless_ElecToMechSpeed(BemfObserver_GetSpeed(&sensorless_observer));
}

/**********************************************************************************************
 * @brief  读取观测到的 α 轴反电动势
 *********************************************************************************************/
float Sensorless_FOC_GetBemfAlpha(void)
{
    return sensorless_observer.e_alpha_filtered;
}

/**********************************************************************************************
 * @brief  读取观测到的 β 轴反电动势
 *********************************************************************************************/
float Sensorless_FOC_GetBemfBeta(void)
{
    return sensorless_observer.e_beta_filtered;
}
