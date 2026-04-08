#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

typedef enum
{
    VOFA_DEBUG_VIEW_DEFAULT = 0,   // F值/速度/Iq/Uq/绝对角度
    VOFA_DEBUG_VIEW_POSVEL,        // 位置-速度串级调试
    VOFA_DEBUG_VIEW_POSCUR,        // 位置-电流串级调试
    VOFA_DEBUG_VIEW_SPCUR,         // 速度-电流串级调试
    VOFA_DEBUG_VIEW_POSVCUR,       // 位置-速度-电流三环调试
    VOFA_DEBUG_VIEW_COUNT
} VofaDebugView;

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
 * @brief  处理缓存的文本状态回显
 * @note   仅在主循环调用，把中断中登记的文本消息安全发送到串口。
 */
void Vofa_ProcessPendingTextFrame(void);

/**
 * @brief  设置当前 VOFA 调试视图
 * @param  view: 需要切换到的调试视图编号，和串口 V0~V4 一一对应
 */
void Vofa_SetDebugView(VofaDebugView view);

/**
 * @brief  获取当前 VOFA 调试视图
 * @return 当前视图编号
 */
VofaDebugView Vofa_GetDebugView(void);

/**
 * @brief  输出 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 */
void Vofa_PrintAdcError(uint32_t error_code);

/**
 * @brief  请求缓存 ADC 错误信息
 * @param  error_code: HAL ADC 错误码
 * @note   允许在中断里调用，实际打印放到主循环执行。
 */
void Vofa_RequestAdcError(uint32_t error_code);

/**
 * @brief  输出模式切换成功回显
 * @param  mode: 已切换到的模式编号
 */
void Vofa_PrintModeSwitchOk(uint32_t mode);

/**
 * @brief  请求缓存模式切换成功信息
 * @param  mode: 需要回显的控制模式编号
 */
void Vofa_RequestModeSwitchOk(uint32_t mode);

/**
 * @brief  输出非法模式编号回显
 * @param  mode: 用户输入的非法模式编号
 */
void Vofa_PrintModeSwitchError(long mode);

/**
 * @brief  请求缓存非法模式错误信息
 * @param  mode: 用户输入的非法控制模式编号
 */
void Vofa_RequestModeSwitchError(long mode);

/**
 * @brief  输出 Function 模块模式切换成功回显
 * @param  mode: 已切换到的功能模式编号，0=关闭，1=纯阻尼感，2=定格感
 */
void Vofa_PrintFunctionSwitchOk(uint32_t mode);

/**
 * @brief  请求缓存 Function 模式切换成功信息
 * @param  mode: 需要回显的功能模式编号
 */
void Vofa_RequestFunctionSwitchOk(uint32_t mode);

/**
 * @brief  输出非法 Function 模式编号回显
 * @param  mode: 用户输入的非法功能模式编号
 */
void Vofa_PrintFunctionSwitchError(long mode);

/**
 * @brief  请求缓存非法 Function 模式错误信息
 * @param  mode: 用户输入的非法功能模式编号
 */
void Vofa_RequestFunctionSwitchError(long mode);

/**
 * @brief  输出 Function 模块参数更新回显
 * @param  name: 参数名
 * @param  value: 新值
 */
void Vofa_PrintFunctionValueUpdate(const char *name, float value);

/**
 * @brief  请求缓存 Function 参数更新信息
 * @param  name: 参数名
 * @param  value: 参数值
 */
void Vofa_RequestFunctionValueUpdate(const char *name, float value);

/**
 * @brief  输出 VOFA 调试视图切换成功回显
 * @param  view: 已切换到的视图编号
 */
void Vofa_PrintDebugViewSwitchOk(uint32_t view);

/**
 * @brief  请求缓存 VOFA 调试视图切换成功信息
 * @param  view: 需要回显的视图编号
 */
void Vofa_RequestDebugViewSwitchOk(uint32_t view);

/**
 * @brief  输出非法 VOFA 调试视图编号回显
 * @param  view: 用户输入的非法视图编号
 */
void Vofa_PrintDebugViewSwitchError(long view);

/**
 * @brief  请求缓存非法 VOFA 调试视图错误信息
 * @param  view: 用户输入的非法视图编号
 */
void Vofa_RequestDebugViewSwitchError(long view);

#endif 
