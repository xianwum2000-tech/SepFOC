#include "BemfObserver.h"

#include <math.h>
#include <string.h>

#include "SensorlessConfig.h"
#include "SepFoc.h"

/**********************************************************************************************
 * @brief  限幅符号函数：误差很小时走线性段，降低滑模抖振
 *********************************************************************************************/
static float BemfObserver_Sat(float value, float epsilon)
{
    if (epsilon <= 0.0f)
    {
        return (value >= 0.0f) ? 1.0f : -1.0f;
    }

    if (fabsf(value) < epsilon)
    {
        return value / epsilon;
    }

    return (value >= 0.0f) ? 1.0f : -1.0f;
}

/**********************************************************************************************
 * @brief  用默认参数初始化反电动势观测器
 *********************************************************************************************/
void BemfObserver_Init(BemfObserver_t *observer)
{
    if (observer == NULL)
    {
        return;
    }

    memset(observer, 0, sizeof(BemfObserver_t));

    observer->Rs = SENSORLESS_RS;
    observer->Ls = SENSORLESS_LS;
    observer->Rs_Ls = SENSORLESS_RS / SENSORLESS_LS;
    observer->inv_Ls = 1.0f / SENSORLESS_LS;

    observer->k_slide = SENSORLESS_SMO_K_SLIDE;
    observer->lpf_alpha = SENSORLESS_SMO_LPF_ALPHA;
    observer->sat_epsilon = SENSORLESS_SMO_SAT_EPSILON;
    observer->converge_bemf_threshold = SENSORLESS_SMO_CONVERGED_BEMF;
    observer->converge_current_err_threshold = SENSORLESS_SMO_CONVERGED_CURRENT_ERR;

    observer->pll_kp = SENSORLESS_PLL_KP;
    observer->pll_ki = SENSORLESS_PLL_KI;
    observer->pll_omega_limit = SENSORLESS_PLL_OMEGA_LIMIT;
    observer->pll_speed_lpf_alpha = SENSORLESS_PLL_SPEED_LPF_ALPHA;
}

/**********************************************************************************************
 * @brief  清空观测器运行时状态，但保留参数配置
 *********************************************************************************************/
void BemfObserver_Reset(BemfObserver_t *observer)
{
    if (observer == NULL)
    {
        return;
    }

    observer->i_alpha_hat = 0.0f;
    observer->i_beta_hat = 0.0f;
    observer->z_alpha = 0.0f;
    observer->z_beta = 0.0f;
    observer->e_alpha = 0.0f;
    observer->e_beta = 0.0f;
    observer->e_alpha_filtered = 0.0f;
    observer->e_beta_filtered = 0.0f;
    observer->theta_est = 0.0f;
    observer->omega_est = 0.0f;
    observer->omega_filtered = 0.0f;
    observer->pll_integral = 0.0f;
    observer->last_i_alpha_err = 0.0f;
    observer->last_i_beta_err = 0.0f;
}

/**********************************************************************************************
 * @brief  在开环转闭环前给观测器一个角度/速度初值，减少切换瞬间的跳变
 *********************************************************************************************/
void BemfObserver_Seed(BemfObserver_t *observer, float angle_est, float speed_est)
{
    if (observer == NULL)
    {
        return;
    }

    observer->theta_est = _normalizeAngle(angle_est);
    observer->omega_est = _constrain(speed_est, -observer->pll_omega_limit, observer->pll_omega_limit);
    observer->omega_filtered = observer->omega_est;
    observer->pll_integral = observer->omega_est;
}

/**********************************************************************************************
 * @brief  更新一次 SMO + PLL
 *********************************************************************************************/
void BemfObserver_Update(BemfObserver_t *observer,
                         float i_alpha,
                         float i_beta,
                         float u_alpha,
                         float u_beta,
                         float dt)
{
    float i_alpha_err = 0.0f;
    float i_beta_err = 0.0f;
    float di_alpha_hat = 0.0f;
    float di_beta_hat = 0.0f;
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    float pll_error = 0.0f;

    if ((observer == NULL) || (dt <= 0.0f))
    {
        return;
    }

    i_alpha_err = observer->i_alpha_hat - i_alpha;
    i_beta_err = observer->i_beta_hat - i_beta;

    observer->z_alpha = observer->k_slide * BemfObserver_Sat(i_alpha_err, observer->sat_epsilon);
    observer->z_beta = observer->k_slide * BemfObserver_Sat(i_beta_err, observer->sat_epsilon);

    di_alpha_hat = (-observer->Rs_Ls * observer->i_alpha_hat) + (observer->inv_Ls * u_alpha) - (observer->inv_Ls * observer->z_alpha);
    di_beta_hat = (-observer->Rs_Ls * observer->i_beta_hat) + (observer->inv_Ls * u_beta) - (observer->inv_Ls * observer->z_beta);

    observer->i_alpha_hat += di_alpha_hat * dt;
    observer->i_beta_hat += di_beta_hat * dt;

    observer->e_alpha = observer->z_alpha;
    observer->e_beta = observer->z_beta;
    observer->e_alpha_filtered = low_pass_filter(observer->e_alpha, observer->e_alpha_filtered, observer->lpf_alpha);
    observer->e_beta_filtered = low_pass_filter(observer->e_beta, observer->e_beta_filtered, observer->lpf_alpha);

    sin_theta = sinf(observer->theta_est);
    cos_theta = cosf(observer->theta_est);
    pll_error = (observer->e_alpha_filtered * sin_theta) - (observer->e_beta_filtered * cos_theta);

    observer->pll_integral += observer->pll_ki * pll_error * dt;
    observer->pll_integral = _constrain(observer->pll_integral, -observer->pll_omega_limit, observer->pll_omega_limit);

    observer->omega_est = (observer->pll_kp * pll_error) + observer->pll_integral;
    observer->omega_est = _constrain(observer->omega_est, -observer->pll_omega_limit, observer->pll_omega_limit);

    observer->theta_est = _normalizeAngle(observer->theta_est + (observer->omega_est * dt));
    observer->omega_filtered = low_pass_filter(observer->omega_est, observer->omega_filtered, observer->pll_speed_lpf_alpha);

    observer->last_i_alpha_err = i_alpha_err;
    observer->last_i_beta_err = i_beta_err;
}

/**********************************************************************************************
 * @brief  判断当前观测器是否基本收敛
 *********************************************************************************************/
uint8_t BemfObserver_IsConverged(const BemfObserver_t *observer)
{
    if (observer == NULL)
    {
        return 0U;
    }

    if ((fabsf(observer->e_alpha_filtered) > observer->converge_bemf_threshold ||
         fabsf(observer->e_beta_filtered) > observer->converge_bemf_threshold) &&
        (fabsf(observer->last_i_alpha_err) < observer->converge_current_err_threshold &&
         fabsf(observer->last_i_beta_err) < observer->converge_current_err_threshold))
    {
        return 1U;
    }

    return 0U;
}

/**********************************************************************************************
 * @brief  读取观测角度
 *********************************************************************************************/
float BemfObserver_GetAngle(const BemfObserver_t *observer)
{
    return (observer != NULL) ? observer->theta_est : 0.0f;
}

/**********************************************************************************************
 * @brief  读取滤波后的观测速度
 *********************************************************************************************/
float BemfObserver_GetSpeed(const BemfObserver_t *observer)
{
    return (observer != NULL) ? observer->omega_filtered : 0.0f;
}
