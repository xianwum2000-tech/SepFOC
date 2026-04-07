/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdlib.h>
#include "SepFoc.h"
#include "Vofa.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA2_Channel1;
    hdma_usart3_tx.Init.Request = DMA_REQUEST_USART3_TX;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#include "stdio.h"
#include "stdlib.h"
#include "Function.h"

uint8_t uart_tx_buf[512]; 
uint16_t uart_tx_len = 0;
volatile uint8_t uart_tx_busy_p = 0; 

int fputc(int ch, FILE *f)
{
    // 如果上一帧 DMA 还没发完，这里会等待；不要在高频中断里使用 printf
    while (uart_tx_busy_p); 

    if (uart_tx_len < sizeof(uart_tx_buf) - 2) {
        if (ch == '\n') uart_tx_buf[uart_tx_len++] = '\r';
        uart_tx_buf[uart_tx_len++] = ch;
    }

    // 仅在遇到换行时启动一次 DMA，把当前缓存整包发出去
    if (ch == '\n') {
        uart_tx_busy_p = 1;
        HAL_UART_Transmit_DMA(&huart3, uart_tx_buf, uart_tx_len);
    }
    return ch;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
    // 仅处理 USART3 的发送完成事件
    if (huart->Instance == USART3) 
    {
        uart_tx_busy_p = 0;
        uart_tx_len = 0; // 清空发送缓冲区
    }
}



// 串口控制相关变量
volatile float motor_target_val = 0.0f;
uint8_t g_rx_byte;                 // 接收单个字节的缓冲
uint8_t temp_buf[20];              // 解析数字时的临时缓存
uint8_t buf_idx = 0;               // 缓冲索引
uint8_t start_flag = 0;            // 接收状态位
uint8_t rx_cmd_type = 0;           // 当前接收命令类型：0=无，'F'=目标值，'M'=模式切换，'X'=Function 模式

void USART3_Rx_Init(void)
{
    // 1. 清除所有可能的错误标志，防止一启动就进入错误状态
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    
    // 2. 开启单字节中断接收
    // 每收到一个字符，都会进入 HAL_UART_RxCpltCallback
    HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        uint8_t ch = g_rx_byte;

        // 串口协议：
        // Fxxx -> 给当前模式写统一目标值，例如 F0.10 / F50
        // Mx   -> 切换控制模式，模式编号顺序与 SepFocControlMode 保持一致
        // Xx   -> 切换 Function 模式，X0=关闭，X1=纯阻尼感，X2=定格感
        if(ch == 'F' || ch == 'f') {
            start_flag = 1;
            rx_cmd_type = 'F';
            buf_idx = 0;
        }
        else if(ch == 'M' || ch == 'm') {
            start_flag = 1;
            rx_cmd_type = 'M';
            buf_idx = 0;
        }
        else if(ch == 'X' || ch == 'x') {
            start_flag = 1;
            rx_cmd_type = 'X';
            buf_idx = 0;
        }
        else if(start_flag) {
            if(rx_cmd_type == 'F')
            {
                // F 命令允许数字、小数点和负号进入缓冲
                if((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') {
                    if(buf_idx < 18) {
                        temp_buf[buf_idx++] = ch;
                    }
                }
                // 收到回车换行后，把缓存转成浮点数
                else if(ch == '\r' || ch == '\n') {
                    if(buf_idx > 0) {
                        float value = 0.0f;
                        temp_buf[buf_idx] = '\0'; // 字符串结束
                        value = (float)atof((char*)temp_buf);

                        if(Function_Control_GetMode() == FUNCTION_CONTROL_PURE_DAMPING) {
                            Function_Control_SetPureDampingKp(value);
                            motor_target_val = Function_Control_GetPureDampingKp();
                            Vofa_RequestFunctionValueUpdate("DampingKp", motor_target_val);
                        }
                        else if(Function_Control_GetMode() == FUNCTION_CONTROL_DETENT) {
                            // X2 定格感模式下不接收 F 参数，直接忽略
                        }
                        else {
                            motor_target_val = value;
                        }
                    }
                    start_flag = 0;
                    rx_cmd_type = 0;
                    buf_idx = 0;
                }
            }
            else if(rx_cmd_type == 'M')
            {
                // M 命令只接收整数模式编号
                if(ch >= '0' && ch <= '9') {
                    if(buf_idx < 18) {
                        temp_buf[buf_idx++] = ch;
                    }
                }
                else if(ch == '\r' || ch == '\n') {
                    if(buf_idx > 0) {
                        long mode_val = 0;
                        SepFocControlMode new_mode = SEP_FOC_MODE_DISABLED;
                        temp_buf[buf_idx] = '\0';
                        mode_val = strtol((char*)temp_buf, NULL, 10);
                        if((mode_val >= 0) && (mode_val < (long)SEP_FOC_MODE_COUNT)) {
                            new_mode = (SepFocControlMode)mode_val;
                            Function_Control_ResetConfigsToDefault();
                            Function_Control_Disable();
                            // 位置类模式默认锁当前位置，其余模式默认目标清零，避免切换瞬间误动作
                            motor_target_val = Sep_FOC_GetModeHoldTarget(new_mode);
                            Sep_FOC_SetControlMode(new_mode);
                            Vofa_RequestModeSwitchOk((uint32_t)mode_val);
                        }
                        else {
                            Vofa_RequestModeSwitchError(mode_val);
                        }
                    }
                    start_flag = 0;
                    rx_cmd_type = 0;
                    buf_idx = 0;
                }
            }
            else if(rx_cmd_type == 'X')
            {
                // X 命令只接收整数功能模式编号
                if(ch >= '0' && ch <= '9') {
                    if(buf_idx < 18) {
                        temp_buf[buf_idx++] = ch;
                    }
                }
                else if(ch == '\r' || ch == '\n') {
                    if(buf_idx > 0) {
                        long func_mode = 0;
                        temp_buf[buf_idx] = '\0';
                        func_mode = strtol((char*)temp_buf, NULL, 10);

                        // 每次进入或退出 X 模式前都恢复默认配置
                        Function_Control_ResetConfigsToDefault();

                        if(func_mode == 0) {
                            Function_Control_Disable();
                            motor_target_val = 0.0f;
                            Vofa_RequestFunctionSwitchOk((uint32_t)func_mode);
                        }
                        else if(func_mode == 1) {
                            Function_Control_EnablePureDampingDefault();
                            motor_target_val = Function_Control_GetPureDampingKp();
                            Vofa_RequestFunctionSwitchOk((uint32_t)func_mode);
                        }
                        else if(func_mode == 2) {
                            Function_Control_EnableDetentDefault();
                            motor_target_val = 0.0f;
                            Vofa_RequestFunctionSwitchOk((uint32_t)func_mode);
                        }
                        else {
                            Vofa_RequestFunctionSwitchError(func_mode);
                        }
                    }
                    start_flag = 0;
                    rx_cmd_type = 0;
                    buf_idx = 0;
                }
            }
        }

        // 重新挂起下一次单字节接收中断
        HAL_UART_Receive_IT(&huart3, &g_rx_byte, 1);
    }
}



/* USER CODE END 1 */
