#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include <stdio.h>
#include <adc.h>

#include "MT6701.h"
#include "Config.h"
#include "kalman_filter.h"
#include "SensorlessFOC.h"
#include "SepFoc.h"
#include "Vofa.h"

void SystemClock_Config(void);

// FOC 运行时变量
float motor_speed = 0;                  // 电机转速，单位 rad/s

// 速度计算复位标志，只在首次进入时生效
uint8_t speed_cal_reset_flag = 1;

int main(void)
{

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_FDCAN1_Init();
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM7_Init();

    // 初始化卡尔曼滤波器
    kalman_filter_init(&motor_speed_kalman_filter, KF_R, KF_Q);
    // 初始化 FOC 数据结构
    FOC_Data_Init(&FOC);
    // 初始化无感 FOC 默认参数，后续 W0~n 切换时从统一默认值起步
    Sensorless_FOC_ResetConfigsToDefault();
	
    // 启动 PWM 输出
    PWM_Start();
    // 启动 TIM1 更新中断，用来同步触发编码器 SPI DMA 读取
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_Delay(5);

    // 转子零电角校准
    set_rotor_zero_angle();
    // 初始化当前控制模式，上电时就确定跑哪一种环
    Sep_FOC_Control_Init(SEP_FOC_STARTUP_MODE);
    motor_target_val = Sep_FOC_GetModeHoldTarget(SEP_FOC_STARTUP_MODE);

    // ADC 校准
    HAL_ADCEx_Calibration_Start(&hadc1, 0);
    // 启动 ADC1 + DMA 电流采样
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC1_CH_NUM);

    // 初始化串口接收：
    // Fxxx 表示给当前模式写统一目标值，M0~n / W0~n / X0~n 分别切换有感/无感/功能模式
    USART3_Rx_Init();

    // 启动速度计算定时器
    HAL_TIM_Base_Start_IT(&htim7);

    while (1)
    {
        // 先把中断里登记的文本状态安全打印出去，避免接收回显卡在中断里
        Vofa_ProcessPendingTextFrame();

        // 串口打印可能阻塞前台，但不会影响定时器/ADC 里的闭环调度
        Vofa_PrintDebugFrame();

        // 当前模式由 SEP_FOC_STARTUP_MODE 决定，也可以后续用串口 M0~n 切换
        // 串口输入 Fxxx 时，motor_target_val 会按当前模式被解释成对应目标量
    }
}


// TIM1 PWM 周期：62.5 us，对应 16 kHz
// TIM7 速度计算周期：250 us，对应 4 kHz

// 定时器周期中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    // 使用 DMA 读取磁编码器角度值
    if (htim->Instance == TIM1)
    {
        // 让编码器读取与 PWM / ADC 采样节拍保持同步
        MT6701_Start_DMA_Read();
    }

    // 计算电机转速，并为前台调试打印准备统计量
    if (htim->Instance == TIM7)
    {
        static float encoder_angle_last = 0.0f;   // 存储上一次编码器角度
        static uint8_t slow_loop_div_cnt = 0U;
        static uint8_t sensorless_slow_loop_div_cnt = 0U;
        uint8_t slow_loop_div = 0U;
        uint8_t sensorless_slow_loop_div = 0U;

        // 限制编码器角度范围，防止异常值影响速度计算
        encoder_angle = min(encoder_angle, 2 * PI);
        encoder_angle = max(encoder_angle, -2 * PI);

        // 首次进入时先完成初始化，避免第一次计算出现跳变
        if (speed_cal_reset_flag)
        {
            // 先把当前角度记为上一时刻角度，保证首次 diff_angle 为 0
            encoder_angle_last = encoder_angle;
            motor_speed = 0.0f;
            speed_cal_reset_flag = 0;
        }

        // 处理角度跨 2PI 回绕的情况，例如 6.27 跳到 0.01 时应得到小角度差
        float diff_angle = cycle_diff(encoder_angle - encoder_angle_last, 2 * PI);
//      diff_angle = -diff_angle;
        encoder_angle_last = encoder_angle;

        // 速度 = 角度增量 / 时间 = 角度增量 * 采样频率，单位 rad/s
        float _motor_speed = diff_angle * SPEED_CALCU_FREQ;
        // 一阶低通滤波
        motor_speed = low_pass_filter(_motor_speed, motor_speed, motor_speed_low_pass_filter_alpha);
        // 卡尔曼滤波版本
        //motor_speed = kalman_filter_calc(&motor_speed_kalman_filter, _motor_speed);

        // 在中断里做轻量级调试量累计，打印仍留在主循环里做
        Vofa_Debug_Update();

        // 无感模式启用时，TIM7 只调度无感状态机/速度外环；否则继续走原有 FOC 慢环。
        if (Sensorless_FOC_IsEnabled())
        {
            sensorless_slow_loop_div = Sensorless_FOC_GetSlowLoopDivider();
            if (sensorless_slow_loop_div > 0U)
            {
                sensorless_slow_loop_div_cnt++;
                if (sensorless_slow_loop_div_cnt >= sensorless_slow_loop_div)
                {
                    sensorless_slow_loop_div_cnt = 0U;
                    Sensorless_FOC_RunSlowLoop(motor_target_val);
                }
            }
            else
            {
                sensorless_slow_loop_div_cnt = 0U;
            }

            slow_loop_div_cnt = 0U;
        }
        else
        {
            sensorless_slow_loop_div_cnt = 0U;

            // 根据当前模式决定是否执行慢环，以及慢环的固定周期
            slow_loop_div = Sep_FOC_GetSlowLoopDivider();
            if (slow_loop_div > 0U)
            {
                slow_loop_div_cnt++;
                if (slow_loop_div_cnt >= slow_loop_div)
                {
                    slow_loop_div_cnt = 0U;
                    Sep_FOC_RunSlowLoop(motor_target_val);
                }
            }
            else
            {
                slow_loop_div_cnt = 0U;
            }
        }

        // 如有需要，也可以在这里再次触发一次角度采样
//      MT6701_Start_DMA_Read();
    }
}









/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
