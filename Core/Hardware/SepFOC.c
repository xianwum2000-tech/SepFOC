#include "SepFoc.h"
#include "spi.h"
#include <math.h>  // 必须包含这个，才能正确识别 fabsf()
#include <stdlib.h> // 用于 abs() 或使用 fabsf()
#include <config.h> 
#include <stdbool.h>
#include <MT6701.h>
#include <adc.h>


#define rad60  1.0471976f   // 60°转换为弧度（π/3）
#define SQRT3  1.7320508f   // 根号3

FOC_data FOC;

/**********************************************************************************************
 * @brief  控制框架运行时变量
 * @note   这里把“对外调试常看量”和“各环内部参数/状态量”分开摆放。
 *         平时调试串口、VOFA 或切模式时，优先看最前面的对外观测区；
 *         真正需要改环路手感时，再往下看各环参数区。
 *********************************************************************************************/

/* ============================== 对外观测与模式状态区 ============================== */
/* 这一组变量会被 getter、调试打印或模式切换逻辑直接使用，后面看现象时优先关注这里。 */
static volatile SepFocControlMode foc_control_mode = SEP_FOC_STARTUP_MODE;  // 当前控制模式
static volatile float foc_applied_uq = 0.0f;                                // 当前真正施加到电机上的 Uq

static volatile float voltage_torque_target = 0.0f;                         // 电压力矩模式目标 Uq

static volatile float position_target_angle = 0.0f;                         // 位置类模式目标绝对角度，单位度
static volatile float position_output_uq = 0.0f;                            // 纯位置环当前直接输出的 Uq

static volatile float speed_target = 0.0f;                                  // 纯速度环目标速度，单位 rad/s
static volatile float speed_loop_output = 0.0f;                             // 速度环当前输出的 Uq

static volatile float posvel_speed_target = 0.0f;                           // 串级模式外环给出的速度目标，单位 rad/s
static volatile float spcur_speed_target = 0.0f;                            // 速度-电流串级目标速度，单位 rad/s
static volatile float spcur_current_target_q = 0.0f;                        // 速度-电流串级外环给出的目标电流，单位 A
static volatile float posvcur_speed_target = 0.0f;                          // 三环模式最外环给出的速度目标，单位 rad/s
static volatile float posvcur_current_target_q = 0.0f;                      // 三环模式中间速度环给出的目标电流，单位 A

static volatile float poscur_current_target_q = 0.0f;                       // 位置-电流串级最终目标电流，单位 A
static volatile float poscur_pid_target_q = 0.0f;                           // 位置-电流串级外环 PID 基础输出电流，单位 A
static volatile float poscur_startup_comp_q = 0.0f;                         // 启动力矩补偿电流，单位 A
static volatile float poscur_damping_comp_q = 0.0f;                         // 速度阻尼补偿电流，单位 A
static volatile float poscur_min_comp_q = 0.0f;                             // 最小力矩补偿电流，单位 A

static volatile float torque_target_q = 0.0f;                               // 电流环目标 Q 轴电流，单位 A
static volatile float torque_measured_q = 0.0f;                             // 电流环实测 Q 轴电流，单位 A
static volatile float torque_output_uq = 0.0f;                              // 电流环输出的 Uq

/* ================================ 纯位置环参数区 ================================ */
/* 纯位置环是“角度误差直接出 Uq”的单环控制，这里调的是硬不硬、跟不跟手。 */
static float position_loop_kp = 0.05f;
static float position_loop_ki = 0.05f;
static float position_loop_kd = 0.05f;
static float position_loop_output_limit = 5.0f;

/* 纯位置环内部状态：主要用于积分与微分计算，通常调参时不用直接改。 */
static float angle_error_integral = 0.0f;
static float last_angle_error = 0.0f;

/* ================================ 纯速度环参数区 ================================ */
/* 纯速度环沿用你之前已经调过的那套参数，这里主要决定起转速度和稳速手感。 */
static float speed_loop_kp = 0.01f;
static float speed_loop_ki = 0.008f;
static float speed_loop_kd = 0.008f;
static float speed_loop_output_limit = 10.0f;

/* 纯速度环内部状态：增量式 PID 计算缓存。 */
static float speed_loop_error = 0.0f;
static float speed_loop_last_error = 0.0f;
static float speed_loop_prev_error = 0.0f;

/* ================================ 电流环参数区 ================================= */
/* 电流环目前是单 q 轴 PI，用来把目标 Iq 拉到位。 */
static float current_loop_kp = 0.8f;
static float current_loop_ki = 0.005f;

/* 电流环内部状态：这里只保留积分项。 */
static float current_loop_integral = 0.0f;

/* ============================ 位置-速度串级外环参数区 ============================ */
/* 这组参数决定“位置误差转换成速度目标”的力度。
 * 调串级模式时，优先看/调的就是这里，尤其是 Kp、Ki、Kd 和 speed_limit。 */
static float posvel_position_kp = 30.0f;
static float posvel_position_ki = 0.1f;
static float posvel_position_kd = 0.05f;
static float posvel_speed_limit = 20.0f;

/* 串级外环内部状态：外环积分、微分缓存，以及外环执行分频计数。 */
static float posvel_angle_error_integral = 0.0f;
static float posvel_last_angle_error = 0.0f;
static uint8_t posvel_outer_div_counter = 0U;

/* ============================ 速度-电流串级外环参数区 ============================ */
/* 这组参数把速度误差转换成目标电流，适合在“要速度、也要力矩感”的场景下使用。
 * 调这套环时，优先关注目标电流是否经常打满，再决定是改 Kp/Ki/Kd 还是改 current_limit。 */
static float spcur_speed_kp = 0.03f;
static float spcur_speed_ki = 0.20f;
static float spcur_speed_kd = 0.0f;
static float spcur_current_limit = 0.8f;

/* 速度-电流串级外环内部状态：速度积分与微分缓存。 */
static float spcur_speed_error_integral = 0.0f;
static float spcur_last_speed_error = 0.0f;

/* ======================= 位置-速度-电流三环参数与状态区 ======================== */
/* 这是最完整的三环控制：位置环出速度目标，速度环出电流目标，最后交给电流 PI 出 Uq。
 * 调参时建议严格按“位置外环 -> 速度中环 -> 电流内环”的顺序来，不要三层一起动。 */
static float posvcur_position_kp = 30.0f;
static float posvcur_position_ki = 0.0f;
static float posvcur_position_kd = 0.05f;
static float posvcur_speed_limit = 20.0f;
static float posvcur_hold_angle_deadband_deg = 0.05f;
static float posvcur_hold_speed_deadband = 0.05f;

static float posvcur_speed_kp = 0.03f;
static float posvcur_speed_ki = 0.20f;
static float posvcur_speed_kd = 0.0f;
static float posvcur_current_limit = 0.8f;

/* 三环模式内部状态：位置外环按分频执行，速度中环每次慢环执行，电流内环仍走 ADC 快环。 */
static float posvcur_angle_error_integral = 0.0f;
static float posvcur_last_angle_error = 0.0f;
static float posvcur_speed_error_integral = 0.0f;
static float posvcur_last_speed_error = 0.0f;
static uint8_t posvcur_outer_div_counter = 0U;

/* ============================ 位置-电流串级外环参数区 ============================ */
/* 这组参数把位置误差转换成目标电流，适合做“有力、能扛负载”的位置控制。
 * 这里额外引入启动力矩补偿、速度阻尼和最小力矩补偿，用来对抗静摩擦和低速发软。 */
static float poscur_position_kp = 0.8f;
static float poscur_position_ki = 0.0f;
static float poscur_position_kd = 0.02f;
static float poscur_current_limit = 0.8f;
static float poscur_startup_boost_q = 0.18f;
static float poscur_startup_error_threshold_deg = 3.0f;
static float poscur_startup_speed_threshold = 0.5f;
static float poscur_damping_gain = 0.02f;
static float poscur_min_torque_comp_q = 0.08f;
static float poscur_min_comp_error_threshold_deg = 1.0f;

/* 位置-电流串级外环内部状态：位置积分、微分缓存。
 * 内环电流 PI 继续复用 current_loop_*，这里只保存外环位置侧状态。 */
static float poscur_angle_error_integral = 0.0f;
static float poscur_last_angle_error = 0.0f;

static void Sep_FOC_ResetLoopStates(void);
static float Sep_FOC_NormalizeDegreeTarget(float target_deg);
static uint8_t Sep_FOC_GetPosVelOuterRatio(void);
static uint8_t Sep_FOC_GetPosVCurOuterRatio(void);


// 启动PWM输出 *******************************************************************************************
void PWM_Start(void)
{
	// 1. 初始化 DRV8313 状态
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // RESET# 拉高，退出复位
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // EN 拉高，开启半桥驱动
	
    HAL_TIM_PWM_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start_IT(&Pwm_Control_Timer, TIM_CHANNEL_3);
	
}

// 停止PWM输出 *******************************************************************************************
void PWM_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop_IT(&Pwm_Control_Timer, TIM_CHANNEL_3);
}

// FOC数据的初始化 *******************************************************************************************
void FOC_Data_Init(FOC_data *FOC_Data_init)
{
	// 三相输出电压
	FOC_Data_init->Ua = 0;
	FOC_Data_init->Ub = 0;
	FOC_Data_init->Uc = 0;
	// 三相输出电压中间值
	FOC_Data_init->dc_a = 0;
	FOC_Data_init->dc_b = 0;
	FOC_Data_init->dc_c = 0;
	// 静止坐标系电压
	FOC_Data_init->Ualpha = 0;
	FOC_Data_init->Ubeta = 0;
	// 电角度零位偏移
	FOC_Data_init->zero_electric_angle = 0;
	// Q 轴电流
	FOC_Data_init ->Iq_filtered  = 0;
}
	



float dc_a, dc_b, dc_c;         // 三相输出电压中间值
/****************************************************** 占空比 ****************************************************************************** */
// 设置Pwm函数 ******************************************************************************************************************
/**
 * @brief  底层 PWM 占空比写入
 * @param  Ua, Ub, Uc: 三相目标电压值
 */
void setPwm(float Ua, float Ub, float Uc) 
{
//    // 1. 计算归一化占空比并限幅 (0.01~0.99 保护死区)
//    // 这里的除法将电压值转为 0.0 ~ 1.0 的比例
//    FOC.dc_a = _constrain(Ua / BUS_VOLTAGE, 0.01f, 0.99f);
//    FOC.dc_b = _constrain(Ub / BUS_VOLTAGE, 0.01f, 0.99f);
//    FOC.dc_c = _constrain(Uc / BUS_VOLTAGE, 0.01f, 0.99f);
//	
//    __disable_irq();
//    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_1, (uint32_t)(FOC.dc_a * Pwm_Control_Timer.Instance->ARR)); // HAL库的PWM占空比设置宏(修改定时器捕获比较寄存器（CCR）)
//    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_2, (uint32_t)(FOC.dc_b * Pwm_Control_Timer.Instance->ARR));
//    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_3, (uint32_t)(FOC.dc_c * Pwm_Control_Timer.Instance->ARR));
//    __enable_irq();
	
    // 1. 计算归一化占空比并限幅 (0.01~0.99 保护死区)
    // 这里的除法将电压值转为 0.0 ~ 1.0 的比例
    FOC.dc_a = _constrain(Ua / BUS_VOLTAGE, 0.01f, 0.99f);
    FOC.dc_b = _constrain(Ub / BUS_VOLTAGE, 0.01f, 0.99f);
    FOC.dc_c = _constrain(Uc / BUS_VOLTAGE, 0.01f, 0.99f);
	
    // 2. 映射到硬件寄存器 (STM32 HAL 库)
    // 4999.0f 对应你的 ARR 值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(FOC.dc_c * Pwm_Control_Timer.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(FOC.dc_b * Pwm_Control_Timer.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(FOC.dc_a * Pwm_Control_Timer.Instance->ARR));
	
}

/****************************************************** 设置力矩 ****************************************************************************** */
// 设置力矩（输出电压向量） ******************************************************************************************************************
/**
 * @brief  设置力矩（输出电压向量）
 * @param  Uq: 交轴电压 (控制电流/力矩)
 * @param  angle_el: 电角度 (弧度)
 */
void setTorque(float Uq, float angle_el) 
{
    // 1. 限制 Uq 输出范围，防止超过电源电压的一半（SPWM 的线性区限制）
    Uq = _constrain(Uq, - BUS_VOLTAGE/2.0f, BUS_VOLTAGE/2.0f);
    foc_applied_uq = Uq;
    
    float Ud = 0; // 通常 D 轴设为 0
    
    // 2. 角度归一化与零位补偿
    //angle_el = _normalizeAngle(angle_el);

    // 3. 逆帕克变换 (使用 sinf/cosf 触发 G431 硬件加速)
    float _sin = sinf(angle_el);
    float _cos = cosf(angle_el);
    
    // 这里的负号取决于你的电机相序定义，通常 Uq 对应垂直磁场
    FOC.Ualpha = Ud * _cos - Uq * _sin;  
    FOC.Ubeta  = Ud * _sin + Uq * _cos;  

    // 4. 逆克拉克变换 (SPWM 模式，带电源中值偏移)
    // 这种写法等价于：Ua = Ualpha + Vbus/2
    float v_offset = BUS_VOLTAGE / 2.0f;
    const float sqrt3 = 1.73205081f;

    FOC.Ua = FOC.Ualpha + v_offset;
    FOC.Ub = (sqrt3 * FOC.Ubeta - FOC.Ualpha) / 2.0f + v_offset;
    FOC.Uc = (-FOC.Ualpha - sqrt3 * FOC.Ubeta) / 2.0f + v_offset;

    // 5. 调用底层硬件写入
    setPwm(FOC.Ua, FOC.Ub, FOC.Uc);
}



/****************************************************** SVPWM ******************************************************************************** */
// SVPWM（空间矢量脉宽调制）函数，将FOC控制中的 d/q 轴电压转换为 U/V/W 三相的 PWM 占空比 *******************************************************
// phi：转子电角度（弧度）
// d/q: d/q 轴电压
// Udc: 母线电压
// d_u/d_v/d_w: u/v/w 相占空比
//static void Ud_Uq2SVPWM(float phi, float d, float q, float Udc, float *d_u, float *d_v, float *d_w)
//{
//    // 母线电压有效性判断（避免除0/负数）
//    if (Udc <= 2.0f) 
//    { 
//        // Udc小于2V视为无效，输出0占空比
//        *d_u = *d_v = *d_w = 0;
//        return;
//    }

//    // 电压归一化
//    // SVPWM中，α/β轴最大线性输出电压幅值为Udc/2（三相桥的固有特性）
//    float d_norm = d * 2.0f / Udc; 
//    float q_norm = q * 2.0f / Udc; 

//    // 限幅，避免过调制
//    d_norm = min(d_norm, MAX_NORM_V);
//    d_norm = max(d_norm, -MAX_NORM_V);
//    q_norm = min(q_norm, MAX_NORM_V);
//    q_norm = max(q_norm, -MAX_NORM_V);

//    // 6个有效矢量对应 U/V/W 三相的 PWM 状态（比如v[0]={1,0,0}表示 U 相上桥臂导通，V/W 下桥臂导通）
//    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
//    // 扇区查找表，通过3个布尔值组合成8种状态（K=0~7），快速定位电压矢量所在的扇区（1~6），避免条件判断，提高运行效率
//    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

//    // Park 逆变换（d/q 轴 → α/β 轴），旋转坐标系到静止坐标系
//    // alpha = d*cosφ - q*sinφ
//    // beta  = d*sinφ + q*cosφ
//    float sin_phi = arm_sin_f32(phi);
//    float cos_phi = arm_cos_f32(phi);
//    float alpha = 0;
//    float beta = 0;
//    arm_inv_park_f32(d_norm, q_norm, &alpha, &beta, sin_phi, cos_phi);

//    // 扇区判断
//    bool A = beta > 0;                                 // β轴电压是否大于0
//    bool B = fabs(beta) > SQRT3 * fabs(alpha);         // |β| > √3|α|
//    bool C = alpha > 0;                                // α轴电压是否大于0
//    int K = 4 * A + 2 * B + C;                         // 组合成0~7的索引值
//    int sector = K_to_sector[K];                       // 查表得到扇区（1~6）

//    // 计算矢量作用时间
//    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
//    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
//    float t_0 = 1 - t_m - t_n;

//    // 过调处理
//    float t_sum = t_m + t_n;
//    if (t_sum > 1.0f)
//    {
//        t_m /= t_sum;  // 归一化到总和为1
//        t_n /= t_sum;
//        t_0 = 0;       // 零矢量时间为0，进入过调制模式
//    }

//    // 分配三相占空比
//    // 第一个有效矢量对 x 相的作用时间 + 第二个有效矢量对 x 相的作用时间 + 零矢量平分到 000 和 111，各占 t_0/2
//    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
//    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
//    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;

//    // 占空比最终限幅（防止浮点精度问题导致超0~1）
//    *d_u = max(0.0f, min(*d_u, 1.0f));
//    *d_v = max(0.0f, min(*d_v, 1.0f));
//    *d_w = max(0.0f, min(*d_w, 1.0f));
//}


/****************************************************** 电机转动调用 ****************************************************************************** */
//// d/q 轴电压 --- 反帕克变换 --- SVPWM --- 三相PWM设置 ******************************************************************************
//// d/q: d/q 轴电压
//// electrical_rad: 转子电角度（弧度）
//void foc_forward(float d, float q, float electrical_rad)
//{
//    float d_u = 0;
//    float d_v = 0;
//    float d_w = 0;

//    //Ud_Uq2SVPWM(electrical_rad, d, q, BUS_VOLTAGE, &d_u, &d_v, &d_w);
//    setPwm(d_u, d_v, d_w);
//    //printf("PWM duty: U = %.2f, V = %.2f, W = %.2f\r\n", d_u, d_v, d_w);
//    //printf("encoder_angle:%.3f, rotor_zero_angle:%.3f, rotor_electrical_angle:%.3f\r\n\n", encoder_angle, rotor_zero_angle, rotor_electrical_angle);
//        
//}



/****************************************************** 周期差值最短路径 **************************************************************************** */
// 周期差值归一化函数，确保差值始终是周期内的最短路径差值 ******************************************************************************
float cycle_diff(float diff, float cycle)
{
    // 情况1：差值大于周期的一半 → 减去周期，得到负的最短差值
    if (diff > (cycle / 2))
        diff -= cycle;
    // 情况2：差值小于负的周期一半 → 加上周期，得到正的最短差值
    else if (diff < (-cycle / 2))
        diff += cycle;
    // 情况3：差值在[-cycle/2, cycle/2]范围内 → 直接返回（无需处理）
    return diff;
}


/****************************************************** 低通滤波 ************************************************************************** */
// 低通滤波器 ***********************************************************************************************************************
float low_pass_filter(float input, float last_output, float alpha)
{
	float _filtered = alpha * input + (1.0f - alpha) * last_output;
    return _filtered;
}


/**********************************************************************************************
 * @brief  将角度目标统一折算到 0~360 度范围内
 * @param  target_deg: 任意输入角度，单位度
 * @return 归一化后的单圈绝对角度
 *********************************************************************************************/
static float Sep_FOC_NormalizeDegreeTarget(float target_deg)
{
    target_deg = fmodf(target_deg, 360.0f);
    if (target_deg < 0.0f)
    {
        target_deg += 360.0f;
    }
    return target_deg;
}

/**********************************************************************************************
 * @brief  计算串级模式中外环相对内环的执行倍率
 * @return 外环每隔多少次内环执行一次，最小返回 1
 * @note   推荐将 SEP_FOC_POSVEL_OUTER_DIV 设为 SEP_FOC_POSVEL_INNER_DIV 的整数倍。
 *         如果不是整数倍，这里会向上取整，保证外环频率不会高于内环。
 *********************************************************************************************/
static uint8_t Sep_FOC_GetPosVelOuterRatio(void)
{
    uint8_t ratio = 1U;

    if (SEP_FOC_POSVEL_INNER_DIV == 0U)
    {
        return 1U;
    }

    if (SEP_FOC_POSVEL_OUTER_DIV > SEP_FOC_POSVEL_INNER_DIV)
    {
        ratio = (uint8_t)(SEP_FOC_POSVEL_OUTER_DIV / SEP_FOC_POSVEL_INNER_DIV);
        if ((SEP_FOC_POSVEL_OUTER_DIV % SEP_FOC_POSVEL_INNER_DIV) != 0U)
        {
            ratio++;
        }
    }

    return (ratio == 0U) ? 1U : ratio;
}

/**********************************************************************************************
 * @brief  计算三环模式中位置外环相对速度中环的执行倍率
 * @return 最外层位置环每隔多少次速度环执行一次，最小返回 1
 * @note   推荐将 SEP_FOC_POSVCUR_POSITION_DIV 设为 SEP_FOC_POSVCUR_SPEED_DIV 的整数倍。
 *         如果不是整数倍，这里会向上取整，保证最外环频率不会高于中间速度环。
 *********************************************************************************************/
static uint8_t Sep_FOC_GetPosVCurOuterRatio(void)
{
    uint8_t ratio = 1U;

    if (SEP_FOC_POSVCUR_SPEED_DIV == 0U)
    {
        return 1U;
    }

    if (SEP_FOC_POSVCUR_POSITION_DIV > SEP_FOC_POSVCUR_SPEED_DIV)
    {
        ratio = (uint8_t)(SEP_FOC_POSVCUR_POSITION_DIV / SEP_FOC_POSVCUR_SPEED_DIV);
        if ((SEP_FOC_POSVCUR_POSITION_DIV % SEP_FOC_POSVCUR_SPEED_DIV) != 0U)
        {
            ratio++;
        }
    }

    return (ratio == 0U) ? 1U : ratio;
}



// 定义时直接计算好
const float _elec_gain = (float)DIR * (float)POLE_PAIRS;
extern float encoder_angle;           		// 当前机械角度 (弧度 0~2PI)

/****************************************************** 零点校准 ************************************************************************** */
// 转子电气零点校准 ******************************************************************************************************************
void set_rotor_zero_angle(void)
{
	// 1. 强行拉向 D 轴 0 度位（注意：setTorque 里的 angle_el 设为 0）
    // 如果你的 setTorque 内部是 Q 轴映射，给 Uq 且角度 0，通常会拉到电角度 0 或 90 度
    // 最稳妥的方法是直接给 Ud=3, Uq=0, angle=0。
    // 这里沿用你的函数，将矢量锁定在电角度 0
    setPwm(2.0f , 0 , 0); 
    
    HAL_Delay(3000); 
    
    FOC.zero_electric_angle = _normalizeAngle(encoder_angle * _elec_gain);
    
    // 3. 停止输出
    setPwm(0 , 0 , 0); 
    HAL_Delay(500);
}


/****************************************************** 归一化 ************************************************************************** */
// 归一化角度到 [0,2PI] ******************************************************************************************************************
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}




/****************************************************** 机角转电角 ************************************************************************** */
// 将机械角度转换为电角度 ******************************************************************************************************************
float _electricalAngle(void) 
{
	// 当前电角度 = 总电角度 - 电气零位偏移
    return _normalizeAngle((encoder_angle * _elec_gain) - FOC.zero_electric_angle);
}

/****************************************************** 	计算Q轴电流	************************************************************************** */
//********************************************************************************************************************************************

/**
 * @brief  计算原始 Q 轴电流
 * @param  current_a: A 相电流 (A)
 * @param  current_b: B 相电流 (A)
 * @param  angle_el:  当前电角度 (rad)
 * @return 原始 Q 轴电流
 */
float cal_Iq_raw(float current_a, float current_b, float angle_el)
{
    // 1. 克拉克变换 (Clarke)
    float I_alpha = current_a;
    float I_beta  = 0.57735027f * current_a + 1.15470054f * current_b;

    // 2. 帕克变换 (Park) - 使用 G431 硬件加速单精度浮点运算
    float _st = sinf(angle_el);
    float _ct = cosf(angle_el);
    
    return -I_alpha * _st + I_beta * _ct;
}

/**
 * @brief  计算并滤波 Q 轴电流 (适应 G431 硬件加速)
 * @param  current_a: A 相电流 (A)
 * @param  current_b: B 相电流 (A)
 * @param  angle_el:  当前电角度 (rad)
 * @return 滤波后的 Q 轴电流
 */
float cal_Iq_filtered(float current_a, float current_b, float angle_el)
{
    float Iq_raw = cal_Iq_raw(current_a, current_b, angle_el);

    // 3. 应用低通滤波 (使用你提供的函数)
    // 注意：FOC.Iq_last 需在结构体中定义并初始化
    // LPF_ALPHA 通常取值 0.01~0.1，取决于你的采样频率
    FOC.Iq_filtered = low_pass_filter(Iq_raw, FOC.Iq_filtered, 0.1f);
    
    return FOC.Iq_filtered;
}


/****************************************************** 	PID 	************************************************************************** */
//********************************************************************************************************************************************









/****************************************************** 核心调用函数 ************************************************************************** */
//********************************************************************************************************************************************


/**
 * @brief 纯力矩控制
 * @param Target: 目标电压 (V)
 */
void Sep_FOC_M0_setTorque(float Target) 
{
    voltage_torque_target = Target;
    setTorque(Target, _electricalAngle());
}



/**
 * @brief 纯位置闭环控制 (力位控制)
 * @param Target: 目标绝对角度 (单位: 度，范围 0~360)
 */
void Sep_FOC_M0_set_Force_Angle(float Target)
{
    // 1. 将目标和反馈统一转换到单圈绝对角度，位置误差取最短路径
    float target_angle_deg = Sep_FOC_NormalizeDegreeTarget(Target);
    float current_angle_deg = Get_Angle();
    float error = 0.0f;

    error = cycle_diff(target_angle_deg - current_angle_deg, 360.0f);

    // 2. 积分计算 (增加积分上限，让它有足够的“劲”)
    angle_error_integral += error;
    // 积分限幅 (Anti-Windup)，防止电机疯转
    if(angle_error_integral > 2.0f) angle_error_integral = 2.0f;
    if(angle_error_integral < -2.0f) angle_error_integral = -2.0f;

    // 3. 微分计算
    float d_error = error - last_angle_error;
    last_angle_error = error;

    // 4. PID 总输出
    float output_torque = (position_loop_kp * error) +
                          (position_loop_ki * angle_error_integral) +
                          (position_loop_kd * d_error);

    // 5. 输出限幅 (非常重要！没有这个可能会烧管子或者手被打疼)
    if(output_torque > position_loop_output_limit)  output_torque = position_loop_output_limit;
    if(output_torque < -position_loop_output_limit) output_torque = -position_loop_output_limit;

    position_target_angle = target_angle_deg;
    position_output_uq = output_torque;
    setTorque(output_torque, _electricalAngle());
}





/** 
 * @brief 速度闭环控制
 * @param Target: 目标速度 (单位: rad/s)
 */
void Sep_FOC_M0_setVelocity(float Target) 
{
    speed_target = Target;

    // 计算当前误差
    speed_loop_error = Target - motor_speed;

    // 增量式 PID 计算公式：delta_u = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    float delta_output = speed_loop_kp * (speed_loop_error - speed_loop_last_error) + 
                         speed_loop_ki * speed_loop_error + 
                         speed_loop_kd * (speed_loop_error - 2.0f * speed_loop_last_error + speed_loop_prev_error);

    speed_loop_output += delta_output;

    // 输出限幅（防止积分饱和及硬件过载）
    if (speed_loop_output > speed_loop_output_limit)  speed_loop_output = speed_loop_output_limit;
    if (speed_loop_output < -speed_loop_output_limit) speed_loop_output = -speed_loop_output_limit;

    // 更新误差状态
    speed_loop_prev_error = speed_loop_last_error;
    speed_loop_last_error = speed_loop_error;

    // 执行力矩输出（电流环/占空比映射）
    setTorque(speed_loop_output, _electricalAngle()); 
}

/**********************************************************************************************
 * @brief  速度-电流串级闭环控制
 * @param  Target: 目标速度，单位 rad/s
 * @note   外环速度 PID 输出目标 Iq，内环复用现有电流 PI 输出 Uq。
 *         这层只更新目标电流，不直接写 PWM，真正的电流控制放在 ADC 快环里。
 *********************************************************************************************/
void Sep_FOC_M0_set_VelocityCurrent(float Target)
{
    float error = Target - motor_speed;
    float dt = (float)SEP_FOC_SPCUR_OUTER_DIV / (float)SPEED_CALCU_FREQ;
    float derivative = 0.0f;
    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;
    float integral_candidate = 0.0f;
    float iq_cmd_unsat = 0.0f;
    float iq_cmd = 0.0f;

    if (dt <= 0.0f)
    {
        dt = 1.0f / (float)SPEED_CALCU_FREQ;
    }

    derivative = (error - spcur_last_speed_error) / dt;
    p_term = spcur_speed_kp * error;
    d_term = spcur_speed_kd * derivative;
    integral_candidate = spcur_speed_error_integral + error * dt;

    if (spcur_speed_ki > 0.0f)
    {
        float integral_limit = spcur_current_limit / spcur_speed_ki;
        integral_candidate = _constrain(integral_candidate, -integral_limit, integral_limit);
        iq_cmd_unsat = p_term + (spcur_speed_ki * integral_candidate) + d_term;

        // 条件积分：未饱和时正常积；已饱和时，只保留能把输出拉回线性区的积分项。
        if (((iq_cmd_unsat < spcur_current_limit) && (iq_cmd_unsat > -spcur_current_limit)) ||
            ((iq_cmd_unsat >= spcur_current_limit) && (error < 0.0f)) ||
            ((iq_cmd_unsat <= -spcur_current_limit) && (error > 0.0f)))
        {
            spcur_speed_error_integral = integral_candidate;
        }
    }

    i_term = spcur_speed_ki * spcur_speed_error_integral;
    iq_cmd = p_term + i_term + d_term;
    iq_cmd = _constrain(iq_cmd, -spcur_current_limit, spcur_current_limit);

    spcur_speed_target = Target;
    spcur_current_target_q = iq_cmd;
    spcur_last_speed_error = error;
}

/**********************************************************************************************
 * @brief  位置-速度-电流三环闭环控制
 * @param  Target: 目标绝对角度，单位度，范围 0~360
 * @note   最外环位置 PID 输出速度目标，中间速度 PID 输出目标 Iq，
 *         最内环继续复用现有电流 PI 输出 Uq，是当前框架下最完整的三环控制。
 *********************************************************************************************/
void Sep_FOC_M0_set_PositionVelocityCurrent(float Target)
{
    float target_angle_deg = Sep_FOC_NormalizeDegreeTarget(Target);
    float current_angle_deg = Get_Angle();
    float error_deg = cycle_diff(target_angle_deg - current_angle_deg, 360.0f);
    float error_rad = deg2rad(error_deg);
    uint8_t outer_ratio = Sep_FOC_GetPosVCurOuterRatio();

    // 静止保持死区：到位且速度已经很小后，直接把中间层和内层目标清零，
    // 避免编码器量化与速度差分噪声在目标点附近不断来回找点。
    if ((fabsf(error_deg) <= posvcur_hold_angle_deadband_deg) &&
        (fabsf(motor_speed) <= posvcur_hold_speed_deadband))
    {
        position_target_angle = target_angle_deg;
        posvcur_speed_target = 0.0f;
        posvcur_current_target_q = 0.0f;
        posvcur_angle_error_integral = 0.0f;
        posvcur_speed_error_integral = 0.0f;
        posvcur_last_angle_error = error_rad;
        posvcur_last_speed_error = -motor_speed;
        posvcur_outer_div_counter = 0U;
        return;
    }

    if (posvcur_outer_div_counter == 0U)
    {
        float dt_pos = (float)SEP_FOC_POSVCUR_POSITION_DIV / (float)SPEED_CALCU_FREQ;
        float derivative = 0.0f;
        float p_term = 0.0f;
        float i_term = 0.0f;
        float d_term = 0.0f;
        float integral_candidate = 0.0f;
        float speed_cmd_unsat = 0.0f;
        float speed_cmd = 0.0f;

        if (dt_pos <= 0.0f)
        {
            dt_pos = 1.0f / (float)SPEED_CALCU_FREQ;
        }

        derivative = (error_rad - posvcur_last_angle_error) / dt_pos;
        p_term = posvcur_position_kp * error_rad;
        d_term = posvcur_position_kd * derivative;
        integral_candidate = posvcur_angle_error_integral + error_rad * dt_pos;

        if (posvcur_position_ki > 0.0f)
        {
            float integral_limit = posvcur_speed_limit / posvcur_position_ki;
            integral_candidate = _constrain(integral_candidate, -integral_limit, integral_limit);
            speed_cmd_unsat = p_term + (posvcur_position_ki * integral_candidate) + d_term;

            if (((speed_cmd_unsat < posvcur_speed_limit) && (speed_cmd_unsat > -posvcur_speed_limit)) ||
                ((speed_cmd_unsat >= posvcur_speed_limit) && (error_rad < 0.0f)) ||
                ((speed_cmd_unsat <= -posvcur_speed_limit) && (error_rad > 0.0f)))
            {
                posvcur_angle_error_integral = integral_candidate;
            }
        }

        i_term = posvcur_position_ki * posvcur_angle_error_integral;
        speed_cmd = p_term + i_term + d_term;
        speed_cmd = _constrain(speed_cmd, -posvcur_speed_limit, posvcur_speed_limit);

        position_target_angle = target_angle_deg;
        posvcur_speed_target = speed_cmd;
        posvcur_last_angle_error = error_rad;
    }

    posvcur_outer_div_counter++;
    if (posvcur_outer_div_counter >= outer_ratio)
    {
        posvcur_outer_div_counter = 0U;
    }

    {
        float error = posvcur_speed_target - motor_speed;
        float dt_speed = (float)SEP_FOC_POSVCUR_SPEED_DIV / (float)SPEED_CALCU_FREQ;
        float derivative = 0.0f;
        float p_term = 0.0f;
        float i_term = 0.0f;
        float d_term = 0.0f;
        float integral_candidate = 0.0f;
        float iq_cmd_unsat = 0.0f;
        float iq_cmd = 0.0f;

        if (dt_speed <= 0.0f)
        {
            dt_speed = 1.0f / (float)SPEED_CALCU_FREQ;
        }

        derivative = (error - posvcur_last_speed_error) / dt_speed;
        p_term = posvcur_speed_kp * error;
        d_term = posvcur_speed_kd * derivative;
        integral_candidate = posvcur_speed_error_integral + error * dt_speed;

        if (posvcur_speed_ki > 0.0f)
        {
            float integral_limit = posvcur_current_limit / posvcur_speed_ki;
            integral_candidate = _constrain(integral_candidate, -integral_limit, integral_limit);
            iq_cmd_unsat = p_term + (posvcur_speed_ki * integral_candidate) + d_term;

            if (((iq_cmd_unsat < posvcur_current_limit) && (iq_cmd_unsat > -posvcur_current_limit)) ||
                ((iq_cmd_unsat >= posvcur_current_limit) && (error < 0.0f)) ||
                ((iq_cmd_unsat <= -posvcur_current_limit) && (error > 0.0f)))
            {
                posvcur_speed_error_integral = integral_candidate;
            }
        }

        i_term = posvcur_speed_ki * posvcur_speed_error_integral;
        iq_cmd = p_term + i_term + d_term;
        iq_cmd = _constrain(iq_cmd, -posvcur_current_limit, posvcur_current_limit);

        posvcur_current_target_q = iq_cmd;
        posvcur_last_speed_error = error;
    }
}

/**********************************************************************************************
 * @brief  位置-速度串级闭环控制
 * @param  Target: 目标绝对角度，单位度，范围 0~360
 * @note   外环位置 PID 输出速度目标，内环复用现有速度环输出 Uq。
 *         建议外环频率低于或等于内环频率，并使用配置宏单独调节。
 *********************************************************************************************/
void Sep_FOC_M0_set_PositionVelocity(float Target)
{
    float target_angle_deg = Sep_FOC_NormalizeDegreeTarget(Target);
    uint8_t outer_ratio = Sep_FOC_GetPosVelOuterRatio();

    if (posvel_outer_div_counter == 0U)
    {
        float current_angle_deg = Get_Angle();
        float error_deg = cycle_diff(target_angle_deg - current_angle_deg, 360.0f);
        float error_rad = deg2rad(error_deg);
        float dt = (float)SEP_FOC_POSVEL_OUTER_DIV / (float)SPEED_CALCU_FREQ;
        float derivative = 0.0f;
        float p_term = 0.0f;
        float i_term = 0.0f;
        float d_term = 0.0f;
        float integral_candidate = 0.0f;
        float speed_cmd_unsat = 0.0f;
        float speed_cmd = 0.0f;

        if (dt <= 0.0f)
        {
            dt = 1.0f / (float)SPEED_CALCU_FREQ;
        }

        derivative = (error_rad - posvel_last_angle_error) / dt;
        p_term = posvel_position_kp * error_rad;
        d_term = posvel_position_kd * derivative;
        integral_candidate = posvel_angle_error_integral + error_rad * dt;

        if (posvel_position_ki > 0.0f)
        {
            float integral_limit = posvel_speed_limit / posvel_position_ki;
            integral_candidate = _constrain(integral_candidate, -integral_limit, integral_limit);
            speed_cmd_unsat = p_term + (posvel_position_ki * integral_candidate) + d_term;

            // 条件积分：只有在未饱和，或积分有助于把输出从饱和区拉回时，才更新积分。
            if (((speed_cmd_unsat < posvel_speed_limit) && (speed_cmd_unsat > -posvel_speed_limit)) ||
                ((speed_cmd_unsat >= posvel_speed_limit) && (error_rad < 0.0f)) ||
                ((speed_cmd_unsat <= -posvel_speed_limit) && (error_rad > 0.0f)))
            {
                posvel_angle_error_integral = integral_candidate;
            }
        }

        i_term = posvel_position_ki * posvel_angle_error_integral;
        speed_cmd = p_term + i_term + d_term;
        speed_cmd = _constrain(speed_cmd, -posvel_speed_limit, posvel_speed_limit);

        posvel_last_angle_error = error_rad;
        position_target_angle = target_angle_deg;
        posvel_speed_target = speed_cmd;
    }

    posvel_outer_div_counter++;
    if (posvel_outer_div_counter >= outer_ratio)
    {
        posvel_outer_div_counter = 0U;
    }

    Sep_FOC_M0_setVelocity(posvel_speed_target);
}

/**********************************************************************************************
 * @brief  位置-电流串级闭环控制
 * @param  Target: 目标绝对角度，单位度，范围 0~360
 * @note   外环位置 PID 输出目标 Iq，内环复用电流 PI 输出 Uq。
 *         这里额外叠加了启动力矩补偿、速度阻尼和最小力矩补偿，优先解决低速发软、
 *         静摩擦起不来以及接近目标时容易“推不动”的问题。
 *********************************************************************************************/
void Sep_FOC_M0_set_PositionCurrent(float Target)
{
    float target_angle_deg = Sep_FOC_NormalizeDegreeTarget(Target);
    float current_angle_deg = Get_Angle();
    float error_deg = cycle_diff(target_angle_deg - current_angle_deg, 360.0f);
    float error_rad = deg2rad(error_deg);
    float dt = (float)SEP_FOC_POSCUR_OUTER_DIV / (float)SPEED_CALCU_FREQ;
    float derivative = 0.0f;
    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;
    float integral_candidate = 0.0f;
    float pid_target_q_unsat = 0.0f;
    float pid_target_q = 0.0f;
    float startup_comp_q = 0.0f;
    float damping_comp_q = 0.0f;
    float min_comp_q = 0.0f;
    float target_q = 0.0f;

    if (dt <= 0.0f)
    {
        dt = 1.0f / (float)SPEED_CALCU_FREQ;
    }

    derivative = (error_rad - poscur_last_angle_error) / dt;
    p_term = poscur_position_kp * error_rad;
    d_term = poscur_position_kd * derivative;
    integral_candidate = poscur_angle_error_integral + error_rad * dt;

    if (poscur_position_ki > 0.0f)
    {
        float integral_limit = poscur_current_limit / poscur_position_ki;
        integral_candidate = _constrain(integral_candidate, -integral_limit, integral_limit);
        pid_target_q_unsat = p_term + (poscur_position_ki * integral_candidate) + d_term;

        if (((pid_target_q_unsat < poscur_current_limit) && (pid_target_q_unsat > -poscur_current_limit)) ||
            ((pid_target_q_unsat >= poscur_current_limit) && (error_rad < 0.0f)) ||
            ((pid_target_q_unsat <= -poscur_current_limit) && (error_rad > 0.0f)))
        {
            poscur_angle_error_integral = integral_candidate;
        }
    }

    i_term = poscur_position_ki * poscur_angle_error_integral;
    pid_target_q = p_term + i_term + d_term;
    pid_target_q = _constrain(pid_target_q, -poscur_current_limit, poscur_current_limit);

    // 速度阻尼：速度越大，给的阻尼越大，用来减轻过冲和靠近目标时的摆动。
    damping_comp_q = -poscur_damping_gain * motor_speed;

    // 启动力矩补偿：大误差且转子几乎没动时，额外补一点电流帮助克服静摩擦。
    if ((fabsf(error_deg) > poscur_startup_error_threshold_deg) &&
        (fabsf(motor_speed) < poscur_startup_speed_threshold))
    {
        startup_comp_q = (error_rad >= 0.0f) ? poscur_startup_boost_q : -poscur_startup_boost_q;
    }

    target_q = pid_target_q + damping_comp_q + startup_comp_q;

    // 最小力矩补偿：误差还明显时，保证最终目标电流不低于一个最小可推动值。
    if ((fabsf(error_deg) > poscur_min_comp_error_threshold_deg) &&
        (fabsf(target_q) < poscur_min_torque_comp_q))
    {
        float min_target_q = (error_rad >= 0.0f) ? poscur_min_torque_comp_q : -poscur_min_torque_comp_q;
        min_comp_q = min_target_q - target_q;
        target_q = min_target_q;
    }

    target_q = _constrain(target_q, -poscur_current_limit, poscur_current_limit);

    position_target_angle = target_angle_deg;
    poscur_pid_target_q = pid_target_q;
    poscur_damping_comp_q = damping_comp_q;
    poscur_startup_comp_q = startup_comp_q;
    poscur_min_comp_q = min_comp_q;
    poscur_current_target_q = target_q;
    poscur_last_angle_error = error_rad;
}

/**********************************************************************************************
 * @brief  读取当前真正施加到逆 Park / PWM 前的 Uq
 * @note   这个值适合调试当前控制器最后给出的电压指令，不区分模式
 *********************************************************************************************/
float Sep_FOC_GetAppliedUq(void)
{
    return foc_applied_uq;
}

/**********************************************************************************************
 * @brief  读取纯电压力矩模式下的目标 Uq
 *********************************************************************************************/
float Sep_FOC_GetVoltageTorqueTarget(void)
{
    return voltage_torque_target;
}

/**********************************************************************************************
 * @brief  读取位置模式下的目标角度
 *********************************************************************************************/
float Sep_FOC_GetPositionTarget(void)
{
    return position_target_angle;
}

/**********************************************************************************************
 * @brief  读取位置环当前输出的 Uq
 *********************************************************************************************/
float Sep_FOC_GetPositionOutput(void)
{
    return position_output_uq;
}

/**********************************************************************************************
 * @brief  读取速度模式下的目标速度
 *********************************************************************************************/
float Sep_FOC_GetVelocityTarget(void)
{
    return speed_target;
}

/**********************************************************************************************
 * @brief  读取速度环当前输出的 Uq
 *********************************************************************************************/
float Sep_FOC_GetVelocityOutput(void)
{
    return speed_loop_output;
}

/**********************************************************************************************
 * @brief  读取串级模式外环给出的速度目标
 * @note   该值是位置环输出、速度环输入，便于观察串级环是否已经开始限速或跟踪。
 *********************************************************************************************/
float Sep_FOC_GetPositionVelocitySpeedTarget(void)
{
    return posvel_speed_target;
}

/**********************************************************************************************
 * @brief  读取速度-电流串级模式下的目标速度
 *********************************************************************************************/
float Sep_FOC_GetVelocityCurrentTarget(void)
{
    return spcur_speed_target;
}

/**********************************************************************************************
 * @brief  读取速度-电流串级模式外环给出的目标电流
 *********************************************************************************************/
float Sep_FOC_GetVelocityCurrentTargetQ(void)
{
    return spcur_current_target_q;
}

/**********************************************************************************************
 * @brief  读取三环模式最外环给出的速度目标
 *********************************************************************************************/
float Sep_FOC_GetPositionVelocityCurrentSpeedTarget(void)
{
    return posvcur_speed_target;
}

/**********************************************************************************************
 * @brief  读取三环模式中间速度环给出的目标电流
 *********************************************************************************************/
float Sep_FOC_GetPositionVelocityCurrentTargetQ(void)
{
    return posvcur_current_target_q;
}

/**********************************************************************************************
 * @brief  读取位置-电流串级模式外环给出的最终目标电流
 *********************************************************************************************/
float Sep_FOC_GetPositionCurrentTargetQ(void)
{
    return poscur_current_target_q;
}

/**********************************************************************************************
 * @brief  读取位置-电流串级模式外环 PID 的基础输出电流
 * @note   这个值还没叠加启动力矩补偿、速度阻尼和最小力矩补偿。
 *********************************************************************************************/
float Sep_FOC_GetPositionCurrentPidTargetQ(void)
{
    return poscur_pid_target_q;
}

/**********************************************************************************************
 * @brief  读取位置-电流串级模式中的启动力矩补偿电流
 *********************************************************************************************/
float Sep_FOC_GetPositionCurrentStartupCompQ(void)
{
    return poscur_startup_comp_q;
}

/**********************************************************************************************
 * @brief  读取位置-电流串级模式中的速度阻尼补偿电流
 *********************************************************************************************/
float Sep_FOC_GetPositionCurrentDampingCompQ(void)
{
    return poscur_damping_comp_q;
}

/**********************************************************************************************
 * @brief  读取位置-电流串级模式中的最小力矩补偿电流
 *********************************************************************************************/
float Sep_FOC_GetPositionCurrentMinCompQ(void)
{
    return poscur_min_comp_q;
}

/**********************************************************************************************
 * @brief  读取电流模式下的目标 Q 轴电流
 *********************************************************************************************/
float Sep_FOC_GetTorqueTarget(void)
{
    return torque_target_q;
}

/**********************************************************************************************
 * @brief  读取电流模式下的实际 Q 轴电流
 *********************************************************************************************/
float Sep_FOC_GetTorqueCurrent(void)
{
    return torque_measured_q;
}

/**********************************************************************************************
 * @brief  读取电流环当前输出的 Uq
 *********************************************************************************************/
float Sep_FOC_GetTorqueOutput(void)
{
    return torque_output_uq;
}


/**********************************************************************************************
 * @brief  力矩（电流）环控制函数
 * @param  current_q: q 轴目标电流，单位 A
 * @note   当前实现的是 q 轴 PI 电流环，d 轴默认保持为 0
 *********************************************************************************************/
float Sep_Foc_lib_torque_control(float current_q)
{
	const float uq_limit = BUS_VOLTAGE / 2.0f;
    float angle_el = _electricalAngle();
    float error_q = 0.0f;
    float uq_prop = 0.0f;
    float uq_cmd = 0.0f;
    float measured_q = 0.0f;

    // 目标 Q 轴电流先限幅，避免指令超过硬件允许范围
    current_q = _constrain(current_q, -MAX_CURRENT, MAX_CURRENT);

    // 用当前采样值和电角度计算实际 Q 轴电流
    measured_q = cal_Iq_filtered(motor_i_u, motor_i_v, angle_el);
    error_q = current_q - measured_q;

    // PI 控制器：比例项给快速响应，积分项消除稳态误差
    uq_prop = current_loop_kp * error_q;
    current_loop_integral += current_loop_ki * error_q;
    current_loop_integral = _constrain(current_loop_integral, -uq_limit, uq_limit);

    uq_cmd = uq_prop + current_loop_integral;

    // 输出限幅并做简单抗积分饱和
    if (uq_cmd > uq_limit)
    {
        uq_cmd = uq_limit;
        if (error_q > 0.0f)
        {
            current_loop_integral = uq_cmd - uq_prop;
        }
    }
    else if (uq_cmd < -uq_limit)
    {
        uq_cmd = -uq_limit;
        if (error_q < 0.0f)
        {
            current_loop_integral = uq_cmd - uq_prop;
        }
    }

    setTorque(uq_cmd, angle_el);

    torque_target_q = current_q;
    torque_measured_q = measured_q;
    torque_output_uq = uq_cmd;

    return measured_q;
}

/**********************************************************************************************
 * @brief  重置所有控制模式共用的内部状态
 * @note   在切换模式或重新初始化控制框架时调用，避免把上一个模式的积分项、
 *         输出缓存、目标值残留到下一个模式里。
 *********************************************************************************************/
static void Sep_FOC_ResetLoopStates(void)
{
    voltage_torque_target = 0.0f;
    position_target_angle = 0.0f;
    position_output_uq = 0.0f;
    speed_target = 0.0f;
    posvel_speed_target = 0.0f;
    spcur_speed_target = 0.0f;
    spcur_current_target_q = 0.0f;
    posvcur_speed_target = 0.0f;
    posvcur_current_target_q = 0.0f;
    poscur_current_target_q = 0.0f;
    poscur_pid_target_q = 0.0f;
    poscur_startup_comp_q = 0.0f;
    poscur_damping_comp_q = 0.0f;
    poscur_min_comp_q = 0.0f;
    speed_loop_error = 0.0f;
    speed_loop_last_error = 0.0f;
    speed_loop_prev_error = 0.0f;
    speed_loop_output = 0.0f;
    torque_target_q = 0.0f;
    torque_measured_q = 0.0f;
    torque_output_uq = 0.0f;
    current_loop_integral = 0.0f;
    angle_error_integral = 0.0f;
    last_angle_error = 0.0f;
    posvel_angle_error_integral = 0.0f;
    posvel_last_angle_error = 0.0f;
    posvel_outer_div_counter = 0U;
    spcur_speed_error_integral = 0.0f;
    spcur_last_speed_error = 0.0f;
    posvcur_angle_error_integral = 0.0f;
    posvcur_last_angle_error = 0.0f;
    posvcur_speed_error_integral = 0.0f;
    posvcur_last_speed_error = 0.0f;
    posvcur_outer_div_counter = 0U;
    poscur_angle_error_integral = 0.0f;
    poscur_last_angle_error = 0.0f;
    foc_applied_uq = 0.0f;
    FOC.Iq_filtered = 0.0f;
}

/**********************************************************************************************
 * @brief  初始化 FOC 控制模式
 * @param  mode: 上电后需要立即进入的控制模式
 *********************************************************************************************/
void Sep_FOC_Control_Init(SepFocControlMode mode)
{
    Sep_FOC_SetControlMode(mode);
}

/**********************************************************************************************
 * @brief  切换控制模式并清空内部状态
 * @param  mode: 新的控制模式
 * @note   这里会先复位各环状态，再把输出拉到 0，减少模式切换时的突变。
 *********************************************************************************************/
void Sep_FOC_SetControlMode(SepFocControlMode mode)
{
    foc_control_mode = mode;
    Sep_FOC_ResetLoopStates();
    position_target_angle = Sep_FOC_GetModeHoldTarget(mode);
    setTorque(0.0f, _electricalAngle());
}

/**********************************************************************************************
 * @brief  读取当前控制模式枚举值
 *********************************************************************************************/
SepFocControlMode Sep_FOC_GetControlMode(void)
{
    return foc_control_mode;
}

/**********************************************************************************************
 * @brief  返回某个模式切入时推荐的安全保持目标
 * @param  mode: 目标控制模式
 * @return 该模式下建议写入的初始目标值
 * @note   位置类模式默认锁定当前位置，避免一切模式就被旧目标或 0 度拉走。
 *********************************************************************************************/
float Sep_FOC_GetModeHoldTarget(SepFocControlMode mode)
{
    switch (mode)
    {
        case SEP_FOC_MODE_POSITION:
        case SEP_FOC_MODE_POSITION_VELOCITY:
        case SEP_FOC_MODE_POSITION_CURRENT:
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            return Get_Angle();
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
        case SEP_FOC_MODE_TORQUE_CURRENT:
        case SEP_FOC_MODE_VELOCITY:
        case SEP_FOC_MODE_VELOCITY_CURRENT:
        case SEP_FOC_MODE_DISABLED:
        default:
            return 0.0f;
    }
}

/**********************************************************************************************
 * @brief  读取当前控制模式的字符串名称
 * @note   主要用于日志、调试或后面做上位机显示时的人类可读输出。
 *********************************************************************************************/
const char *Sep_FOC_GetControlModeName(void)
{
    switch (foc_control_mode)
    {
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
            return "TorqueVoltage";
        case SEP_FOC_MODE_TORQUE_CURRENT:
            return "TorqueCurrent";
        case SEP_FOC_MODE_POSITION:
            return "Position";
        case SEP_FOC_MODE_VELOCITY:
            return "Velocity";
        case SEP_FOC_MODE_POSITION_VELOCITY:
            return "PositionVelocity";
        case SEP_FOC_MODE_POSITION_CURRENT:
            return "PositionCurrent";
        case SEP_FOC_MODE_VELOCITY_CURRENT:
            return "VelocityCurrent";
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            return "PositionVelocityCurrent";
        case SEP_FOC_MODE_DISABLED:
        default:
            return "Disabled";
    }
}

/**********************************************************************************************
 * @brief  根据当前控制模式返回慢环分频
 * @return 0 表示当前模式不需要在 TIM7 慢环里执行
 * @note   纯电流环走 ADC 同步快环；速度/位置/电压力矩走 TIM7 慢环；
 *         带电流内环的串级模式则由 TIM7 跑外层/中层，ADC 回调跑最内层电流环。
 *********************************************************************************************/
uint8_t Sep_FOC_GetSlowLoopDivider(void)
{
    switch (foc_control_mode)
    {
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
            return SEP_FOC_VOLTAGE_LOOP_DIV;
        case SEP_FOC_MODE_POSITION:
            return SEP_FOC_POSITION_LOOP_DIV;
        case SEP_FOC_MODE_VELOCITY:
            return SEP_FOC_VELOCITY_LOOP_DIV;
        case SEP_FOC_MODE_POSITION_VELOCITY:
            return SEP_FOC_POSVEL_INNER_DIV;
        case SEP_FOC_MODE_POSITION_CURRENT:
            return SEP_FOC_POSCUR_OUTER_DIV;
        case SEP_FOC_MODE_VELOCITY_CURRENT:
            return SEP_FOC_SPCUR_OUTER_DIV;
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            return SEP_FOC_POSVCUR_SPEED_DIV;
        case SEP_FOC_MODE_TORQUE_CURRENT:
        case SEP_FOC_MODE_DISABLED:
        default:
            return 0U;
    }
}

/**********************************************************************************************
 * @brief  快环统一入口
 * @param  target: 当前模式对应的目标值
 * @note   当前由 ADC 回调驱动。纯电流模式直接使用串口目标电流，
 *         各种带电流内环的串级模式则使用对应外层/中层刚算好的目标电流。
 *********************************************************************************************/
void Sep_FOC_RunFastLoop(float target)
{
    if (foc_control_mode == SEP_FOC_MODE_TORQUE_CURRENT)
    {
        Sep_Foc_lib_torque_control(target);
    }
    else if (foc_control_mode == SEP_FOC_MODE_POSITION_CURRENT)
    {
        Sep_Foc_lib_torque_control(poscur_current_target_q);
    }
    else if (foc_control_mode == SEP_FOC_MODE_VELOCITY_CURRENT)
    {
        Sep_Foc_lib_torque_control(spcur_current_target_q);
    }
    else if (foc_control_mode == SEP_FOC_MODE_POSITION_VELOCITY_CURRENT)
    {
        Sep_Foc_lib_torque_control(posvcur_current_target_q);
    }
}

/**********************************************************************************************
 * @brief  慢环统一入口
 * @param  target: 当前模式对应的目标值
 * @note   由 TIM7 固定节拍调度，不同模式只是在这里分发到不同控制函数。
 *********************************************************************************************/
void Sep_FOC_RunSlowLoop(float target)
{
    switch (foc_control_mode)
    {
        case SEP_FOC_MODE_TORQUE_VOLTAGE:
            Sep_FOC_M0_setTorque(target);
            break;
        case SEP_FOC_MODE_POSITION:
            Sep_FOC_M0_set_Force_Angle(target);
            break;
        case SEP_FOC_MODE_VELOCITY:
            Sep_FOC_M0_setVelocity(target);
            break;
        case SEP_FOC_MODE_POSITION_VELOCITY:
            Sep_FOC_M0_set_PositionVelocity(target);
            break;
        case SEP_FOC_MODE_POSITION_CURRENT:
            Sep_FOC_M0_set_PositionCurrent(target);
            break;
        case SEP_FOC_MODE_VELOCITY_CURRENT:
            Sep_FOC_M0_set_VelocityCurrent(target);
            break;
        case SEP_FOC_MODE_POSITION_VELOCITY_CURRENT:
            Sep_FOC_M0_set_PositionVelocityCurrent(target);
            break;
        case SEP_FOC_MODE_DISABLED:
            setTorque(0.0f, _electricalAngle());
            break;
        case SEP_FOC_MODE_TORQUE_CURRENT:
        default:
            break;
    }
}
