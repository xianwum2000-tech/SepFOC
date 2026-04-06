#include "mt6701.h"
#include "spi.h"
#include <math.h>  // 必须包含这个，才能正确识别 fabsf()
#include <stdlib.h> // 用于 abs() 或使用 fabsf()
#include <config.h> 


/* 变量定义 */
float encoder_angle = 0;           			// 当前机械角度 (弧度 0~2PI)
float cumulative_encoder_angle = 0; 		// 多圈累加角度
uint8_t motor_angle_reset_flag = 1;			// 复位标志，用于只赋一次值

uint32_t raw_angle = 0;
// MT6701 读取起始地址 0x03 (也可以直接连续读取，取决于具体模式)
// 这里发送 0x03 后连续读取 3 个字节
uint8_t Spi_TxData[3] = {0x83, 0x00, 0x00}; 
uint8_t Spi_RxData[3] = {0};

#define MT6701_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MT6701_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define MT6701_ANGLE_MAX  16383.0f   										// 14位 (2^14 - 1)



static void multi_circle_angle_accumulation(void);

/**
 * @brief 启动一次精确的 SPI 采样
 */
void MT6701_Start_DMA_Read(void)
{
    // 1. 强行清除可能的 Busy 标志（G4 系列 HAL 库常见坑）
	if(hspi1.State != HAL_SPI_STATE_READY) return;
    
    MT6701_CS_LOW();
    // 2. 同时启动 TX 和 RX DMA，时钟由 TX 产生，同步接收
    HAL_SPI_TransmitReceive_DMA(&hspi1, Spi_TxData, Spi_RxData, 3);
}

/**
 * @brief SPI 全双工传输完成回调
 * @brief MT6701 修正版数据解析（解决 1-bit 位移问题）
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        MT6701_CS_HIGH();

        /* 补偿逻辑：
           由于硬件多发一个脉冲，Spi_RxData[0] 的最高位其实是丢弃位，
           我们需要将其左移 1 位补偿回来。
        */
        // 这里的拼接逻辑对应你测试中的 Case 1
        uint16_t corrected_raw = ((uint16_t)Spi_RxData[0] << 7) | (Spi_RxData[1] >> 1);
        
        // 强制约束到 14位 范围 (0 ~ 16383)
        raw_angle = corrected_raw & 0x3FFF;

        // 转换为弧度
        encoder_angle = (raw_angle / 16383.0f) * 2.0f * PI;

        // 多圈累加计算
        multi_circle_angle_accumulation();
    }
}


/**
 * @brief 电机多圈累加弧度值计算
 */
static void multi_circle_angle_accumulation(void)
{
    static float encoder_angle_last = 0;

    if (motor_angle_reset_flag)
    {
        encoder_angle_last = encoder_angle;
        // 第一次复位时，累加值应设为 0 或当前绝对值
        cumulative_encoder_angle = 0; 
        motor_angle_reset_flag = 0;
        return; // 跳过第一次差值计算
    }

    // 计算当前与上一帧的差值
    float diff = encoder_angle - encoder_angle_last;

    // 过零处理 (处理从 6.28 跳变到 0 或反向跳变)
    if (diff > PI)  diff -= (2.0f * PI);
    else if (diff < -PI) diff += (2.0f * PI);

    // 累加总角度
    cumulative_encoder_angle += diff;
    
    // 更新上一帧记录
    encoder_angle_last = encoder_angle;
}



/**
 * @brief 将弧度转换为 0-360 度
 */
float Get_Angle(void)
{
    // 1. 弧度转角度
    float deg = encoder_angle * (180.0f / PI);
    
    // 2. 取模（防止角度超过 360 或低于 0）
    // fmodf 处理浮点数取模，例如 370.0 -> 10.0
    deg = fmodf(deg, 360.0f);
    
    // 3. 处理负数情况 (例如 -10.0 -> 350.0)
    if (deg < 0)
    {
        deg += 360.0f;
    }
    
    return deg;
}



/**
 * @brief SPI读取数据有问题，这个可以打印各种左移或右移什么的，然后给AI分析，是出什问题了
 */
//void Case_Bit(void)
//{
//	// 1. 触发读取
//    MT6701_Start_DMA_Read();

//    // 2. 延时确保 DMA 完成且不刷屏太快
//    HAL_Delay(100); 

//    // 3. 计算四种情况 (注意：MT6701 是 14 位，最大值是 16383)
//    uint16_t case0 = ((uint16_t)Spi_RxData[0] << 6) | (Spi_RxData[1] >> 2);
//    uint16_t case1 = ((uint16_t)Spi_RxData[0] << 7) | (Spi_RxData[1] >> 1);
//    uint16_t case2 = ((uint16_t)Spi_RxData[0] << 5) | (Spi_RxData[1] >> 3);
//    uint16_t case3 = ((uint16_t)(Spi_RxData[0] & 0x7F) << 7) | (Spi_RxData[1] >> 1);

//    // 4. 格式化打印
//    printf("HEX:[%02X %02X] | C0:%5u | C1:%5u | C2:%5u | C3:%5u\r\n", 
//            Spi_RxData[0], Spi_RxData[1], 
//            (uint16_t)(case0 & 0x3FFF), 
//            (uint16_t)(case1 & 0x3FFF), 
//            (uint16_t)(case2 & 0x3FFF), 
//            (uint16_t)(case3 & 0x3FFF));
//}

