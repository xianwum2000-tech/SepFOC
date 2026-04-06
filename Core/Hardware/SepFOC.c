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

// 控制框架运行时变量：
// 这里集中保存当前模式、各环目标值、各环输出值以及 PID/PI 状态，
// 方便后面继续扩展位置-速度-电流串级控制或运行时切换模式。
static volatile SepFocControlMode foc_control_mode = SEP_FOC_STARTUP_MODE;
static volatile float foc_applied_uq = 0.0f;
static volatile float voltage_torque_target = 0.0f;
static volatile float position_target_angle = 0.0f;
static volatile float position_output_uq = 0.0f;
static volatile float speed_target = 0.0f;
static volatile float torque_target_q = 0.0f;
static volatile float torque_measured_q = 0.0f;
static volatile float torque_output_uq = 0.0f;

static float speed_loop_kp = 0.01f;          // 保持原来的速度环参数
static float speed_loop_ki = 0.008f;
static float speed_loop_kd = 0.008f;
static float speed_loop_error = 0.0f;
static float speed_loop_last_error = 0.0f;
static float speed_loop_prev_error = 0.0f;
static float speed_loop_output = 0.0f;
static float speed_loop_output_limit = 10.0f;

static float current_loop_kp = 0.8f;         // 单独给电流环一套参数
static float current_loop_ki = 0.005f;
static float current_loop_integral = 0.0f;

static float angle_error_integral = 0.0f;    // 位置环积分项
static float last_angle_error = 0.0f;        // 位置环上次误差

static void Sep_FOC_ResetLoopStates(void);


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
	// --- 这里的参数是让它变“硬”的关键 ---
    float Kp = 0.05f;    // 显著增大！15.0 意味着偏离 1 rad 输出 15A/V 的力矩
    float Ki = 0.05f;     // 增大！消除静态偏差，让你掰不动
    float Kd = 0.05f;     // 增大！配合 Kp 防止乱抖
    float Max_Torque = 5.0f; // 这里的数值取决于你的驱动器能承受的最大电流/电压

    // 1. 将目标和反馈统一转换到单圈绝对角度，位置误差取最短路径
    float target_angle_deg = fmodf(Target, 360.0f);
    float current_angle_deg = Get_Angle();
    float error = 0.0f;

    if (target_angle_deg < 0.0f)
    {
        target_angle_deg += 360.0f;
    }

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
    float output_torque = (Kp * error) + (Ki * angle_error_integral) + (Kd * d_error);

    // 5. 输出限幅 (非常重要！没有这个可能会烧管子或者手被打疼)
    if(output_torque > Max_Torque)  output_torque = Max_Torque;
    if(output_torque < -Max_Torque) output_torque = -Max_Torque;

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
        case SEP_FOC_MODE_DISABLED:
        default:
            return "Disabled";
    }
}

/**********************************************************************************************
 * @brief  根据当前控制模式返回慢环分频
 * @return 0 表示当前模式不需要在 TIM7 慢环里执行
 * @note   电流环走 ADC 同步快环，速度/位置/电压力矩模式走 TIM7 慢环。
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
        case SEP_FOC_MODE_TORQUE_CURRENT:
        case SEP_FOC_MODE_DISABLED:
        default:
            return 0U;
    }
}

/**********************************************************************************************
 * @brief  快环统一入口
 * @param  target: 当前模式对应的目标值
 * @note   目前只有电流模式会在 ADC 回调里执行，后面如果加 dq 电流双环也继续放这里。
 *********************************************************************************************/
void Sep_FOC_RunFastLoop(float target)
{
    if (foc_control_mode == SEP_FOC_MODE_TORQUE_CURRENT)
    {
        Sep_Foc_lib_torque_control(target);
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
        case SEP_FOC_MODE_DISABLED:
            setTorque(0.0f, _electricalAngle());
            break;
        case SEP_FOC_MODE_TORQUE_CURRENT:
        default:
            break;
    }
}
