#ifndef __SEPFOC_H
#define __SEPFOC_H

#include "main.h"
#include "config.h"
#include "arm_math.h"   	// ARM DSP库（针对arm架构优化的数学库），提供sin/cos/inv_park等函数
#include "tim.h"   			

#define Pwm_Control_Timer htim1

// 结构体：用于存储FOC数据
typedef struct
{
	float zero_electric_angle;		// 电角度零位偏移
	float Ualpha, Ubeta;           	// 静止坐标系电压
    float Ua, Ub, Uc;              	// 三相输出电压
	float dc_a, dc_b, dc_c;         // 三相输出电压中间值
	float Iq_filtered;				//  Q 轴电流
}FOC_data;


extern FOC_data FOC;
extern float encoder_init_angle; // 刚上电时编码器角度


/**
 * @brief 限幅函数 (内联版本，防止多次触发计算副作用)
 */
static inline float _constrain(float amt, float low, float high) {
    if (amt < low) return low;
    if (amt > high) return high;
    return amt;
}

void PWM_Start(void);														// 开启PWM输出 
void PWM_Stop(void);														// 停止PWM输出 
void setPwm(float Ua, float Ub, float Uc);									// 设置Pwm占空比函数 
float cycle_diff(float diff, float cycle);									// 周期差值归一化函数，确保差值始终是周期内的最短路径差值 
float low_pass_filter(float input, float last_output, float alpha);			// 低通滤波器 
void set_rotor_zero_angle(void);											// 转子电气零点校准
void foc_forward(float d, float q, float electrical_rad);					// d/q 轴电压 --- 反帕克变换 --- SVPWM --- 三相PWM设置 
float _electricalAngle(void);												// 将机械角度转换为电角度 
float _normalizeAngle(float angle);											// 归一化
void setTorque(float Uq, float angle_el);									// 设置力矩
void FOC_Data_Init(FOC_data *FOC_Data);										// FOC数据的初始化
float cal_Iq_raw(float current_a, float current_b, float angle_el);		// 计算原始 Q 轴电流
float cal_Iq_filtered(float current_a, float current_b, float angle_el);	// 计算并滤波 Q 轴电流
void Sep_FOC_Control_Init(SepFocControlMode mode);							// 控制模式初始化
void Sep_FOC_SetControlMode(SepFocControlMode mode);						// 切换控制模式并重置环路状态
SepFocControlMode Sep_FOC_GetControlMode(void);								// 读取当前控制模式
const char *Sep_FOC_GetControlModeName(void);								// 读取当前控制模式名称
float Sep_FOC_GetModeHoldTarget(SepFocControlMode mode);                    // 读取某个模式切入时的安全保持目标
uint8_t Sep_FOC_GetSlowLoopDivider(void);									// 读取慢环执行分频
void Sep_FOC_RunFastLoop(float target);										// 快环入口，适合 ADC 同步任务
void Sep_FOC_RunSlowLoop(float target);										// 慢环入口，适合定时器任务


// 核心FOC调用
void Sep_FOC_M0_setTorque(float Target);							// 纯力矩控制
void Sep_FOC_M0_set_Force_Angle(float Target);						// 纯位置闭环控制，输入绝对角度(度)
void Sep_FOC_M0_setVelocity(float Target);							// 速度闭环控制
void Sep_FOC_M0_set_PositionVelocity(float Target);                 // 位置-速度串级闭环，输入绝对角度(度)
void Sep_FOC_M0_set_PositionCurrent(float Target);                  // 位置-电流串级闭环，输入绝对角度(度)
void Sep_FOC_M0_set_VelocityCurrent(float Target);                  // 速度-电流串级闭环，输入目标速度(rad/s)
void Sep_FOC_M0_set_PositionVelocityCurrent(float Target);          // 位置-速度-电流三环闭环，输入绝对角度(度)
float Sep_FOC_GetAppliedUq(void);									// 读取当前实际施加的 Uq
float Sep_FOC_GetVoltageTorqueTarget(void);							// 读取电压模式目标 Uq
float Sep_FOC_GetPositionTarget(void);								// 读取位置模式目标绝对角度(度)
float Sep_FOC_GetPositionOutput(void);								// 读取位置环当前输出
float Sep_FOC_GetVelocityTarget(void);								// 读取速度环当前目标
float Sep_FOC_GetVelocityOutput(void);								// 读取速度环当前输出
float Sep_FOC_GetPositionVelocitySpeedTarget(void);                 // 读取串级模式外环给出的速度目标
float Sep_FOC_GetVelocityCurrentTarget(void);                       // 读取速度-电流串级模式目标速度
float Sep_FOC_GetVelocityCurrentTargetQ(void);                      // 读取速度-电流串级模式外环给出的目标电流
float Sep_FOC_GetPositionVelocityCurrentSpeedTarget(void);          // 读取三环模式最外环给出的速度目标
float Sep_FOC_GetPositionVelocityCurrentTargetQ(void);              // 读取三环模式中间速度环给出的目标电流
float Sep_FOC_GetPositionCurrentTargetQ(void);                      // 读取位置-电流串级外环给出的最终目标电流
float Sep_FOC_GetPositionCurrentPidTargetQ(void);                   // 读取位置-电流串级外环 PID 基础输出电流
float Sep_FOC_GetPositionCurrentStartupCompQ(void);                 // 读取启动力矩补偿电流
float Sep_FOC_GetPositionCurrentDampingCompQ(void);                 // 读取速度阻尼补偿电流
float Sep_FOC_GetPositionCurrentMinCompQ(void);                     // 读取最小力矩补偿电流
float Sep_Foc_lib_torque_control(float current_q);					// Q 轴电流环控制，返回实测 Q 轴电流
float Sep_FOC_GetTorqueTarget(void);								// 读取 Q 轴目标电流
float Sep_FOC_GetTorqueCurrent(void);								// 读取 Q 轴实测电流
float Sep_FOC_GetTorqueOutput(void);								// 读取电流环当前 Uq 输出

#endif
