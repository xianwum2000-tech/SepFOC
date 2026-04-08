/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"
#include "Function.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

// STM32G431 的 ADC 转换时间 = 采样周期 + 12.5 个 ADC 时钟周期。
// 当前配置下采样时间为 ADC_SAMPLETIME_47CYCLES_5，折算单次转换时间约 1.6 us。

#include "arm_math.h"   // ARM DSP 数学库，后续可用于滤波或坐标变换等运算。
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <config.h>
#include "SensorlessFOC.h"
#include "SepFoc.h"
#include "usart.h"
#include "Vofa.h"

uint16_t adc1_buf[ADC1_CH_NUM] = {0};  // ADC1 DMA 原始采样缓冲区
float motor_i_u = 0;          // U 相滤波电流
float motor_i_v = 0;          // V 相滤波电流
float motor_i_w = 0;          // W 相重构电流
float motor_i_bus = 0;        // 母线电流，当前未使用


// 相电流滑动平均滤波缓存
float i_u_filter_buf[FILTER_TIMES] = {0};
float i_v_filter_buf[FILTER_TIMES] = {0};
float i_bus_filter_buf[FILTER_TIMES] = {0};
uint8_t filter_index = 0;  // 当前滑动窗口写入位置
uint8_t over_curr_cnt_con = 0; // 持续过流计数
uint8_t over_curr_cnt_ins = 0; // 瞬时过流计数
// 当前滤波结果与 ADC 零点
float i_u_filt = 0, i_v_filt = 0, i_bus_filt = 0;
uint32_t adc_offset_u = 2048, adc_offset_v = 2048, adc_offset_bus = 2048;

#define ADC_OVERCURRENT_CONT_THRESHOLD_A   (MAX_CURRENT * 1.10f)
#define ADC_OVERCURRENT_INST_THRESHOLD_A   (MAX_CURRENT * 1.60f)
#define ADC_OVERCURRENT_CONT_COUNT_LIMIT   8U
#define ADC_OVERCURRENT_INST_COUNT_LIMIT   1U

static volatile uint8_t over_current_trip_latched = 0U;
static volatile uint8_t over_current_shutdown_done = 0U;

/**********************************************************************************************
 * @brief  过流检测
 * @param  i_u_raw/i_v_raw: 当前 ADC 换算得到的两相原始电流
 * @note   瞬时过流使用原始电流判定，持续过流使用滤波电流判定，任一满足都会锁存保护。
 *********************************************************************************************/
static void Over_Current_Detect(float i_u_raw, float i_v_raw)
{
    float i_w_raw = -(i_u_raw + i_v_raw);
    float i_w_filt = -(i_u_filt + i_v_filt);
    float inst_abs_max = fabsf(i_u_raw);
    float cont_abs_max = fabsf(i_u_filt);

    if (fabsf(i_v_raw) > inst_abs_max) inst_abs_max = fabsf(i_v_raw);
    if (fabsf(i_w_raw) > inst_abs_max) inst_abs_max = fabsf(i_w_raw);

    if (fabsf(i_v_filt) > cont_abs_max) cont_abs_max = fabsf(i_v_filt);
    if (fabsf(i_w_filt) > cont_abs_max) cont_abs_max = fabsf(i_w_filt);

    if (inst_abs_max >= ADC_OVERCURRENT_INST_THRESHOLD_A)
    {
        if (over_curr_cnt_ins < 255U)
        {
            over_curr_cnt_ins++;
        }
    }
    else
    {
        over_curr_cnt_ins = 0U;
    }

    if (cont_abs_max >= ADC_OVERCURRENT_CONT_THRESHOLD_A)
    {
        if (over_curr_cnt_con < 255U)
        {
            over_curr_cnt_con++;
        }
    }
    else
    {
        over_curr_cnt_con = 0U;
    }

    if ((over_curr_cnt_ins >= ADC_OVERCURRENT_INST_COUNT_LIMIT) ||
        (over_curr_cnt_con >= ADC_OVERCURRENT_CONT_COUNT_LIMIT))
    {
        over_current_trip_latched = 1U;
    }
}

/**********************************************************************************************
 * @brief  执行过流保护
 * @note   触发后统一拉低目标值、关闭 Function 模式、停用有感主控并给无感注入过流故障。
 *         这是锁存式保护，除非手动清除，否则不会自动恢复输出。
 *********************************************************************************************/
static void Over_Current_Protect(void)
{
    if (!over_current_trip_latched)
    {
        return;
    }

    if (over_current_shutdown_done)
    {
        return;
    }

    motor_target_val = 0.0f;
    Function_Control_Disable();
    Sensorless_FOC_ForceFault(SENSORLESS_FAULT_OVERCURRENT);
    Sep_FOC_SetControlMode(SEP_FOC_MODE_DISABLED);
    setTorque(0.0f, _electricalAngle());

    over_current_shutdown_done = 1U;
}

/**********************************************************************************************
 * @brief  清除过流锁存
 * @note   用于串口 C0 或上层调试恢复；只清状态，不主动恢复任何控制模式。
 *********************************************************************************************/
void ADC_ClearOverCurrentLatch(void)
{
    over_curr_cnt_con = 0U;
    over_curr_cnt_ins = 0U;
    over_current_trip_latched = 0U;
    over_current_shutdown_done = 0U;
}

/**********************************************************************************************
 * @brief  查询当前是否存在过流锁存
 *********************************************************************************************/
uint8_t ADC_GetOverCurrentLatched(void)
{
    return over_current_trip_latched;
}



// ADC DMA 采样完成回调
// 在这里完成电流换算、滤波、保护判断以及快环调度
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {

		// 把 ADC 采样值换算为相电流，运放链路近似满足 Vadc = 1.65 + 0.01 * I * 50
		float u_1 = ADC_VOLTAGE * ((float)(adc1_buf[0]) / (ADC_RESOLUTION - 1) - 0.5f);
		float u_2 = ADC_VOLTAGE * ((float)(adc1_buf[1]) / (ADC_RESOLUTION - 1) - 0.5f);
		float i_1 = u_1 / R_SHUNT / OP_GAIN + MOTOR_I_U_CORRECT;
		float i_2 = u_2 / R_SHUNT / OP_GAIN + MOTOR_I_V_CORRECT;
		
		// 更新相电流滑动平均结果
		Current_Slide_Filter(i_1, i_2);

		motor_i_u = i_u_filt;                  // 实测 U 相
		motor_i_v = i_v_filt;                  // 实测 V 相
		motor_i_w = -(motor_i_u + motor_i_v);  // 由两相重构 W 相

		// 先做保护，再决定本次快环是否继续输出 PWM
		Over_Current_Detect(i_1, i_2);
		Over_Current_Protect();
		if (over_current_trip_latched)
		{
			HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);
			return;
		}

        // 快环调度优先级：无感 > Function 手感模式 > 原有有感 FOC
        if (Sensorless_FOC_IsEnabled())
        {
            Sensorless_FOC_RunFastLoop(i_1, i_2, motor_i_u, motor_i_v);
        }
        else if (Function_Control_IsEnabled())
        {
            Function_Control_RunFastLoop();
        }
		else
		{
			// 默认有感 FOC 快环：继续运行当前模式对应的最内层控制
			Sep_FOC_RunFastLoop(motor_target_val);
		}

		HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);  // 重新挂起下一次 DMA 采样
    }
}

// ADC 错误回调
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // 仅登记错误消息，避免在错误回调里直接阻塞串口
        Vofa_RequestAdcError(hadc->ErrorCode);
    }
    
    HAL_ADC_Stop_DMA(hadc);
    if(hadc->Instance == ADC1)
    {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);
    }
}

// 电流滑动平均滤波
// 用固定长度窗口抑制采样噪声，得到更稳定的相电流反馈
static void Current_Slide_Filter(float i_u, float i_v)
{
    // 写入最新采样
    i_u_filter_buf[filter_index] = i_u;
    i_v_filter_buf[filter_index] = i_v;
    // 更新循环缓冲索引
    filter_index = (filter_index + 1) % FILTER_TIMES;
    
    // 重新求窗口和
    float sum_u = 0, sum_v = 0;
    for (uint8_t i = 0; i < FILTER_TIMES; i++)
    {
        sum_u += i_u_filter_buf[i];
        sum_v += i_v_filter_buf[i];
    }
    // 输出平均值
    i_u_filt = sum_u / FILTER_TIMES;
    i_v_filt = sum_v / FILTER_TIMES;
}


// 电流零点标定
// 建议在 PWM 关闭、电机静止时调用
void Motor_Current_Calibration(void)
{
    uint32_t sum_u = 0, sum_v = 0, sum_bus = 0;
    uint16_t sample_counts = 1024; // 累计 1024 次采样作为零点基准

    // 逐次读取静态 ADC 结果并求平均
    for(int i = 0; i < sample_counts; i++)
    {
        // 这里直接等待下一次 DMA 刷新，简化零点标定流程
        HAL_Delay(1); 
        sum_u += adc1_buf[0];
        sum_v += adc1_buf[1];
    }

    // 保存零点 ADC 计数
    adc_offset_u = sum_u / sample_counts;
    adc_offset_v = sum_v / sample_counts;
    adc_offset_bus = sum_bus / sample_counts;
}




/* USER CODE END 1 */
