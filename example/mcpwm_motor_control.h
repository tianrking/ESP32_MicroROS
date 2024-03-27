// 假设源文件命名为 mcpwm_motor_control.h
#ifndef MCPWM_MOTOR_CONTROL_H
#define MCPWM_MOTOR_CONTROL_H

#include "driver/mcpwm_prelude.h" // 引用MCPWM的新API头文件
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化MCPWM配置，设置GPIO和MCPWM单元。
 */
void mcpwm_example_initialize(void);

/**
 * @brief 设置MCPWM的两个输出的占空比。
 * 
 * @param duty_cycle_a 第一个输出的占空比百分比，范围为0到100。
 * @param duty_cycle_b 第二个输出的占空比百分比，范围为0到100。
 */
void set_dual_duty_cycle(float duty_cycle_a, float duty_cycle_b);

/**
 * @brief 电机控制任务，不断设置MCPWM的占空比。
 * 
 * @param pvParameters 传递给任务的参数，未使用。
 */
void motor_control_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // MCPWM_MOTOR_CONTROL_H
