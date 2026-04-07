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

// 在stm32g431里，转换时间 = 采样时间 + 12.5个ADC时钟周期(cubemx配置为150MHz4分频)
// 三相电流配置采样时间 = ADC_SAMPLETIME_47CYCLES_5 对应 转换时间 = (47.5+12.5)/(150M/4）= 1.6us

#include "arm_math.h"   // ARM DSP库（针对arm架构优化的数学库），提供sin/cos/park等函数
#include <stdlib.h>
#include <stdio.h>
#include <config.h>
#include "SepFoc.h"
#include "usart.h"
#include "Vofa.h"

uint16_t adc1_buf[ADC1_CH_NUM] = {0};  // ADC1-三相电流采样值
float motor_i_u = 0;          // W 相采样电流，安培
float motor_i_v = 0;          // V 相采样电流，安培
float motor_i_w = 0;          // W 相采样电流，安培
float motor_i_bus = 0;        // 低侧总线电流，安培


// 滑动均值滤波缓冲区
float i_u_filter_buf[FILTER_TIMES] = {0};
float i_v_filter_buf[FILTER_TIMES] = {0};
float i_bus_filter_buf[FILTER_TIMES] = {0};
uint8_t filter_index = 0;  // 滤波缓冲区索引（循环覆盖）
uint8_t over_curr_cnt_con = 0; // 持续过流计数（连续超过阈值的次数）
uint8_t over_curr_cnt_ins = 0; // 瞬间大电流过流计数
// 滤波后的电流值
float i_u_filt = 0, i_v_filt = 0, i_bus_filt = 0;
uint32_t adc_offset_u = 2048, adc_offset_v = 2048, adc_offset_bus = 2048;



// ADC DMA传输完成回调函数 ***********************************************************************************
// 获得3相电流，通过克拉克变换、帕克变换得到 d/q 轴电流
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {

		// 根据电路，电流计算公式 Vadc = 1.65+0.01*I*50
   	    float u_1 = ADC_VOLTAGE * ((float)(adc1_buf[0]) / (ADC_RESOLUTION - 1) - 0.5f);
   	    float u_2 = ADC_VOLTAGE * ((float)(adc1_buf[1]) / (ADC_RESOLUTION - 1) - 0.5f);
   	    float i_1 = u_1 / R_SHUNT / OP_GAIN + MOTOR_I_U_CORRECT;
   	    float i_2 = u_2 / R_SHUNT / OP_GAIN + MOTOR_I_V_CORRECT;
		
		// 滑动均值滤波
		Current_Slide_Filter(i_1, i_2);

//      // 过流检测（带延时确认）
//      Over_Current_Detect(); 
//      // 过流保护执行
//      Over_Current_Protect();

		motor_i_u = i_u_filt;                  // 对应MA
		motor_i_v = i_v_filt;                  // 对应MB
		motor_i_w = -(motor_i_u + motor_i_v);  // 对应MC

        // Function 模块启用时，由它接管快环输出；否则继续走原有 FOC 快环。
        if (Function_Control_IsEnabled())
        {
            Function_Control_RunFastLoop();
        }
        else
        {
            // 电流环模式下，Q 轴电流环与采样节拍同步执行
            Sep_FOC_RunFastLoop(motor_target_val);
        }

      // 测试
      // motor_i_u = i_1;                  
      // motor_i_v = i_2;                 
      // motor_i_w = -(motor_i_u + motor_i_v); 
      // motor_i_bus = i_3;  

//      float i_alpha = 0;
//      float i_beta = 0;
//      // 克拉克变换
//      // i_alpha = i_u
//      // i_beta  = (1/sqrt(3)) * i_u + (2/sqrt(3)) * i_v
//      arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);

//      float sin_value = arm_sin_f32(rotor_electrical_angle);
//      float cos_value = arm_cos_f32(rotor_electrical_angle);
//      float _motor_i_d = 0;
//      float _motor_i_q = 0;
//      // 帕克变换
//      // id ?= iα * ?cosθ + iβ * ?sinθ
//      // iq ?= ?iα * ?sinθ + iβ * ?cosθ
//      // i_d：直轴电流（励磁电流），控制电机磁链，FOC 通常设为 0（弱磁控制除外）
//      // i_q：交轴电流（转矩电流），直接决定电机输出转矩，是转速环的控制目标
//      arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);

//      // 一阶低通滤波
//      motor_i_d = low_pass_filter(_motor_i_d, motor_i_d, i_d_low_pass_filter_alpha);
//      motor_i_q = low_pass_filter(_motor_i_q, motor_i_q, i_q_low_pass_filter_alpha);

      // //printf("pos:%.2f, speed:%.2f, electrical_angle:%.2f \r\n", cumulative_encoder_angle, motor_speed, rotor_electrical_angle);   
      // //printf("target_position:%.2f, target_speed:%.2f \r\n", motor_control_context.position, motor_control_context.speed);


//      // 运行电机相应闭环控制
//      switch (motor_control_context.type)
//      {
//        case control_type_null:
//          set_pwm_duty(0, 0, 0, 0.9f);
//          pid_i_reset();
//          break;
//        case control_type_position:
//          set_motor_pid_single(&pid_position, Position_P, Position_I, Position_D);
//          lib_position_control(motor_control_context.position);
//          break;
//        case control_type_speed:
//          set_motor_pid_single(&pid_speed, Speed_P, Speed_I, Speed_D);
//          lib_speed_control(motor_control_context.speed);
//          break;
//        case control_type_torque:
//          set_motor_pid_single(&pid_torque_q, Torque_Q_P, Torque_Q_I, Torque_Q_D);
//          lib_torque_control(motor_control_context.current_d, motor_control_context.current_q);
//          break;
//        case control_type_position_speed:
//          set_motor_pid_single(&pid_position_PS_Ctrl, Position_P, Position_I, Position_D);
//          set_motor_pid_single(&pid_speed_PS_Ctrl, Speed_P, Speed_I, Speed_D);
//          lib_position_speed_control(motor_control_context.position, motor_control_context.speed);
//          break;
//        case control_type_position_torque:
//          set_motor_pid_single(&pid_position_PT_Ctrl, Position_P, Position_I, Position_D);
//          set_motor_pid_single(&pid_torque_q_PT_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
//          lib_position_torque_control(motor_control_context.position, motor_control_context.current_q);
//          break;
//        case control_type_speed_torque:
//          set_motor_pid_single(&pid_speed_ST_Ctrl, Speed_P, Speed_I, Speed_D);
//          set_motor_pid_single(&pid_torque_q_ST_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
//          lib_speed_torque_control(motor_control_context.speed, motor_control_context.current_q);
//          break;
//        case control_type_position_speed_torque:
//          set_motor_pid_single(&pid_position_PST_Ctrl, Position_P, Position_I, Position_D);
//          set_motor_pid_single(&pid_speed_PST_Ctrl, Speed_P, Speed_I, Speed_D);
//          set_motor_pid_single(&pid_torque_q_PST_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
//          lib_position_speed_torque_control(motor_control_context.position, motor_control_context.speed, motor_control_context.current_q);
//          break;
//        default:
//          break;
//      }

      HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);  //开启下一次
    }
}

// ADC错误回调
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // ADC 错误回显也改成前台打印，避免中断里等待串口发送
        Vofa_RequestAdcError(hadc->ErrorCode);
    }
    
    HAL_ADC_Stop_DMA(hadc);
    if(hadc->Instance == ADC1)
    {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);
    }
}

// 滑动均值滤波,对三相电流做滑动均值滤波，输出平滑的电流值 ***************************************************
// 滑动均值：每次新增一个采样值，剔除最旧的，计算平均值（兼顾实时性和滤波效果）
static void Current_Slide_Filter(float i_u, float i_v)
{
    // 循环覆盖滤波缓冲区
    i_u_filter_buf[filter_index] = i_u;
    i_v_filter_buf[filter_index] = i_v;
    // 索引自增，超过次数则清零（循环）
    filter_index = (filter_index + 1) % FILTER_TIMES;
    
    // 计算滑动平均值
    float sum_u = 0, sum_v = 0;
    for (uint8_t i = 0; i < FILTER_TIMES; i++)
    {
        sum_u += i_u_filter_buf[i];
        sum_v += i_v_filter_buf[i];
    }
    // 存储滤波后的电流值
    i_u_filt = sum_u / FILTER_TIMES;
    i_v_filt = sum_v / FILTER_TIMES;
}


// ADC DMA传输校准函数 ***********************************************************************************
// 开机校准
void Motor_Current_Calibration(void)
{
    uint32_t sum_u = 0, sum_v = 0, sum_bus = 0;
    uint16_t sample_counts = 1024; // 采样1024次取平均，滤除高频噪声

    // 确保此时 PWM 为关闭状态，无电流流经电阻
    for(int i = 0; i < sample_counts; i++)
    {
        // 触发一次采样（取决于你的配置，如果是DMA循环模式直接读取即可）
        HAL_Delay(1); 
        sum_u += adc1_buf[0];
        sum_v += adc1_buf[1];
    }

    // 得到平均零点 Counts
    adc_offset_u = sum_u / sample_counts;
    adc_offset_v = sum_v / sample_counts;
    adc_offset_bus = sum_bus / sample_counts;
}




/* USER CODE END 1 */
