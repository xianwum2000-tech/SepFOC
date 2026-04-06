#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

/**
 * @brief  在中断里累计一段时间窗口内的调试量
 * @note   建议放在固定周期中断中调用，函数本身不做 printf。
 */
void Vofa_Debug_Update(void);

/**
 * @brief  将当前缓存好的调试窗口输出为 VOFA+ FireWater 文本帧
 * @note   建议放在主循环里调用，避免串口打印阻塞中断。
 */
void Vofa_PrintDebugFrame(void);

/**
 * @brief  输出 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 */
void Vofa_PrintAdcError(uint32_t error_code);

/**
 * @brief  输出模式切换成功回显
 * @param  mode: 已切换到的模式编号
 */
void Vofa_PrintModeSwitchOk(uint32_t mode);

/**
 * @brief  输出非法模式编号回显
 * @param  mode: 用户输入的非法模式编号
 */
void Vofa_PrintModeSwitchError(long mode);

#endif 
