#ifndef LEDC_MOTOR_CONTROL_H
#define LEDC_MOTOR_CONTROL_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_random.h" // 包含esp_random()

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化LEDC定时器和通道，用于控制电机。
 */
void ledc_motor_init(void);

/**
 * FreeRTOS任务，用于控制电机的占空比。
 * 
 * @param pvParameters 传递给任务的参数，未使用。
 */
void ledc_motor_control_task(void *pvParameters);
void set_motor_speed(int speed);
void set_motor_speed_test(int speed);

#ifdef __cplusplus
}
#endif

#endif // LEDC_MOTOR_CONTROL_H
