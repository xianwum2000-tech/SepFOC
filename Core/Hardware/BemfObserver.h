#ifndef __BEMF_OBSERVER_H__
#define __BEMF_OBSERVER_H__

#include "main.h"

/* 反电动势观测器：把 SMO 与 PLL 放在一套结构里，方便无感状态机直接管理。 */
typedef struct
{
    float Rs;
    float Ls;
    float Rs_Ls;
    float inv_Ls;

    float k_slide;
    float lpf_alpha;
    float sat_epsilon;
    float converge_bemf_threshold;
    float converge_current_err_threshold;

    float pll_kp;
    float pll_ki;
    float pll_omega_limit;
    float pll_speed_lpf_alpha;

    float i_alpha_hat;
    float i_beta_hat;
    float z_alpha;
    float z_beta;

    float e_alpha;
    float e_beta;
    float e_alpha_filtered;
    float e_beta_filtered;

    float theta_est;
    float omega_est;
    float omega_filtered;
    float pll_integral;

    float last_i_alpha_err;
    float last_i_beta_err;
} BemfObserver_t;

/**********************************************************************************************
 * @brief  用默认参数初始化反电动势观测器
 * @param  observer: 观测器对象
 *********************************************************************************************/
void BemfObserver_Init(BemfObserver_t *observer);

/**********************************************************************************************
 * @brief  清空观测器运行时状态，但保留参数配置
 * @param  observer: 观测器对象
 *********************************************************************************************/
void BemfObserver_Reset(BemfObserver_t *observer);

/**********************************************************************************************
 * @brief  在开环转闭环前，用当前开环角度和速度给观测器一个较平顺的初值
 * @param  observer: 观测器对象
 * @param  angle_est:  角度初值，单位 rad
 * @param  speed_est:  速度初值，单位 rad/s
 *********************************************************************************************/
void BemfObserver_Seed(BemfObserver_t *observer, float angle_est, float speed_est);

/**********************************************************************************************
 * @brief  更新一次 SMO + PLL
 * @param  observer: 观测器对象
 * @param  i_alpha/i_beta: αβ 轴电流
 * @param  u_alpha/u_beta: αβ 轴电压
 * @param  dt: 快环周期，单位 s
 *********************************************************************************************/
void BemfObserver_Update(BemfObserver_t *observer,
                         float i_alpha,
                         float i_beta,
                         float u_alpha,
                         float u_beta,
                         float dt);

/**********************************************************************************************
 * @brief  判断当前观测器是否基本收敛
 * @param  observer: 观测器对象
 * @return 1=收敛，0=未收敛
 *********************************************************************************************/
uint8_t BemfObserver_IsConverged(const BemfObserver_t *observer);

/**********************************************************************************************
 * @brief  读取观测角度
 *********************************************************************************************/
float BemfObserver_GetAngle(const BemfObserver_t *observer);

/**********************************************************************************************
 * @brief  读取滤波后的观测速度
 *********************************************************************************************/
float BemfObserver_GetSpeed(const BemfObserver_t *observer);

#endif
