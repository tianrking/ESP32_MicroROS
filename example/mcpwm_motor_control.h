#ifndef MCPWM_CONTROL_H
#define MCPWM_CONTROL_H

#include "driver/mcpwm.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 定义MCPWM操作的GPIO
#define MCPWM_GPIO_A 18
#define MCPWM_GPIO_B 19

/**
 * @brief MCPWM 初始化配置。
 */
void mcpwm_example_initialize(void);

/**
 * @brief 设置MCPWM的两个输出的占空比。
 * 
 * @param duty_cycle_a 第一个输出的占空比百分比，范围为0到100。
 * @param duty_cycle_b 第二个输出的占空比百分比，范围为0到100。
 */
void set_dual_duty_cycle(float duty_cycle_a, float duty_cycle_b);

#ifdef __cplusplus
}
#endif

#endif // MCPWM_CONTROL_H
