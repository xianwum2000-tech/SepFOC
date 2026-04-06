#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

#define ADC1_CH_NUM  		2  			// ADC1：三相电流采样
#define FILTER_TIMES      	5      		// 滑动均值滤波次数（5次，兼顾精度和响应）

extern uint16_t adc1_buf[ADC1_CH_NUM];  // ADC1-三相电流采样值
extern float motor_i_u;        			// W 相采样电流，安培
extern float motor_i_v;        			// V 相采样电流，安培
extern float motor_i_w;        			// W 相采样电流，安培
extern float motor_i_bus;      			// 低侧总线电流，安培








static void Current_Slide_Filter(float i_u, float i_v);
void Motor_Current_Calibration(void);




/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

