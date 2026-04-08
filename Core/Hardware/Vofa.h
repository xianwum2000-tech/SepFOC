#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

typedef enum
{
    VOFA_DEBUG_VIEW_DEFAULT = 0,   // F 值 / 实际速度 / Iq / Uq / 绝对角度
    VOFA_DEBUG_VIEW_POSVEL,        // 位置-速度串级调试页
    VOFA_DEBUG_VIEW_POSCUR,        // 位置-电流串级调试页
    VOFA_DEBUG_VIEW_SPCUR,         // 速度-电流串级调试页
    VOFA_DEBUG_VIEW_POSVCUR,       // 位置-速度-电流三环调试页
    VOFA_DEBUG_VIEW_SENSORLESS,    // 无感 FOC 启动诊断页
    VOFA_DEBUG_VIEW_COUNT
} VofaDebugView;

/**
 * @brief  在中断中累计一段时间窗口内的调试统计量
 * @note   这里只做轻量级缓存，真正的串口输出放在主循环执行。
 */
void Vofa_Debug_Update(void);

/**
 * @brief  将当前缓存的数据按 VOFA+ FireWater 格式输出
 * @note   建议在主循环里调用，避免串口打印阻塞中断。
 */
void Vofa_PrintDebugFrame(void);

/**
 * @brief  处理待输出的文本状态消息
 * @note   用于把中断里登记的回显信息安全地放到主循环输出。
 */
void Vofa_ProcessPendingTextFrame(void);

/**
 * @brief  设置当前 VOFA 调试视图
 * @param  view: 目标视图编号，对应串口命令 V0~V5
 */
void Vofa_SetDebugView(VofaDebugView view);

/**
 * @brief  读取当前 VOFA 调试视图
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
 */
void Vofa_RequestAdcError(uint32_t error_code);

/**
 * @brief  输出有感模式切换成功回显
 * @param  mode: 已切换到的模式编号
 */
void Vofa_PrintModeSwitchOk(uint32_t mode);

/**
 * @brief  请求缓存有感模式切换成功信息
 * @param  mode: 需要回显的控制模式编号
 */
void Vofa_RequestModeSwitchOk(uint32_t mode);

/**
 * @brief  输出非法有感模式编号回显
 * @param  mode: 用户输入的非法模式编号
 */
void Vofa_PrintModeSwitchError(long mode);

/**
 * @brief  请求缓存非法有感模式错误信息
 * @param  mode: 用户输入的非法模式编号
 */
void Vofa_RequestModeSwitchError(long mode);

/**
 * @brief  输出 Function 模式切换成功回显
 * @param  mode: 0=关闭，1=纯阻尼感，2=定格感
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
 * @brief  输出 Function 参数更新回显
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
 * @brief  输出无感模式切换成功回显
 * @param  mode: 已切换到的无感模式编号
 */
void Vofa_PrintSensorlessSwitchOk(uint32_t mode);

/**
 * @brief  请求缓存无感模式切换成功信息
 * @param  mode: 需要回显的无感模式编号
 */
void Vofa_RequestSensorlessSwitchOk(uint32_t mode);

/**
 * @brief  输出非法无感模式编号回显
 * @param  mode: 用户输入的非法无感模式编号
 */
void Vofa_PrintSensorlessSwitchError(long mode);

/**
 * @brief  请求缓存非法无感模式错误信息
 * @param  mode: 用户输入的非法无感模式编号
 */
void Vofa_RequestSensorlessSwitchError(long mode);

/**
 * @brief  输出无感故障回显
 * @param  fault: 当前无感故障码
 */
void Vofa_PrintSensorlessFault(uint32_t fault);

/**
 * @brief  请求缓存无感故障回显
 * @param  fault: 当前无感故障码
 */
void Vofa_RequestSensorlessFault(uint32_t fault);

/**
 * @brief  输出 VOFA 调试视图切换成功回显
 * @param  view: 视图编号
 */
void Vofa_PrintDebugViewSwitchOk(uint32_t view);

/**
 * @brief  请求缓存 VOFA 调试视图切换成功信息
 * @param  view: 需要回显的视图编号
 */
void Vofa_RequestDebugViewSwitchOk(uint32_t view);

/**
 * @brief  输出非法 VOFA 视图编号回显
 * @param  view: 用户输入的非法视图编号
 */
void Vofa_PrintDebugViewSwitchError(long view);

/**
 * @brief  请求缓存非法 VOFA 视图错误信息
 * @param  view: 用户输入的非法视图编号
 */
void Vofa_RequestDebugViewSwitchError(long view);

/**
 * @brief  输出保护清除成功回显
 */
void Vofa_PrintProtectionClearOk(void);

/**
 * @brief  请求缓存保护清除成功回显
 */
void Vofa_RequestProtectionClearOk(void);

/**
 * @brief  输出非法保护清除命令回显
 * @param  value: 用户输入的保护命令参数
 */
void Vofa_PrintProtectionClearError(long value);

/**
 * @brief  请求缓存非法保护命令回显
 * @param  value: 用户输入的保护命令参数
 */
void Vofa_RequestProtectionClearError(long value);

#endif
