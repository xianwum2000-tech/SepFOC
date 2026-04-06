#ifndef __CONFIG_H__
#define __CONFIG_H__

// FOC 控制模式枚举
typedef enum
{
    SEP_FOC_MODE_DISABLED = 0,      // 停止输出
    SEP_FOC_MODE_TORQUE_VOLTAGE,    // 目标电压的纯力矩环（电压模式）
    SEP_FOC_MODE_TORQUE_CURRENT,    // 目标 Q 电流的纯力矩环（电流模式）
    SEP_FOC_MODE_POSITION,          // 纯位置闭环
    SEP_FOC_MODE_VELOCITY,          // 纯速度闭环
    SEP_FOC_MODE_COUNT              // 当前已实现的模式数量，供串口等范围检查使用
} SepFocControlMode;

// 电路参数
#define MAX_CURRENT 2.5f        // 额定最大电流
#define BUS_VOLTAGE 12.0f       // 母线电压，单位 V
//#define MAX_SPEED 36.5f       // 额定转速，单位 rad/s
#define MAX_SPEED 58.0f         // 最大转速，单位 rad/s

// 控制模式配置
#define SEP_FOC_STARTUP_MODE     SEP_FOC_MODE_TORQUE_CURRENT  // 上电默认进入的控制模式
#define SEP_FOC_VOLTAGE_LOOP_DIV 1U                           // 电压力矩环分频，4000 / 1 = 4000 Hz
#define SEP_FOC_VELOCITY_LOOP_DIV 4U                          // 速度环分频，4000 / 4 = 1000 Hz
#define SEP_FOC_POSITION_LOOP_DIV 8U                          // 位置环分频，4000 / 8 = 500 Hz


// 常用换算宏
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define motor2load(x) (x / REDUCTION_RATIO)
#define load2motor(x) (x * REDUCTION_RATIO)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

// 电机机械参数
#define POLE_PAIRS 7          			// 极对数
#define DIR 1          					// 电机方向
#define REDUCTION_RATIO 1     			// 减速比

// 定时器配置参数
#define 	PWM_FREQ 16000          	// TIM1 三相 PWM 频率，单位 Hz
#define 	SPEED_CALCU_FREQ 4000		// 速度计算频率，单位 Hz


// 常用数学常量
#define PI 3.14159265358979f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f


// 卡尔曼滤波参数：R 越大越平滑，Q 越大响应越快，详见 kalman_filter.c
#define KF_R 1.0f
#define KF_Q 0.001f

// 一阶低通滤波系数：一般可在 0.1~0.3 之间调试
#define motor_speed_low_pass_filter_alpha 0.1f
#define i_d_low_pass_filter_alpha 0.1f
#define i_q_low_pass_filter_alpha 0.1f

// SVPWM 参数
#define MAX_NORM_V  0.8f            	// 归一化电压上限，理论极限约为 0.866
	
// 电流采样参数
#define ADC_VOLTAGE 3.3f            	// ADC 参考电压，单位 V
#define ADC_RESOLUTION 4096         	// ADC 分辨率
#define R_SHUNT 0.01f               	// 采样电阻，单位欧姆
#define OP_GAIN 50                  	// 运放放大倍数
#define BUS_VREF_CORRECT   0.0045f  	// 母线采样 Vref 修正值，(1.65 - 1.635) / 3.3 = 0.0045

//#define MOTOR_I_U_CORRECT  0.00318f 	// 早期调试的 U 相偏置补偿
//#define MOTOR_I_V_CORRECT  0.00015f 	// 早期调试的 V 相偏置补偿

#define MOTOR_I_U_CORRECT  0.0048f  // 根据 PWM 关闭静态均值回推的 U 相零点补偿
#define MOTOR_I_V_CORRECT  0.0022f  // 根据 PWM 关闭静态均值回推的 V 相零点补偿


#define V_GAIN_SCALE    2.092f  		// 增益缩放系数，由实测 U_peak / V_peak = 1.13 / 0.54 得到

#endif

