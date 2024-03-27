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

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    // 如果有更多电机，可以继续添加
} motor_id_t;

// 定义电机旋转状态
typedef enum {
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_direction_t;

void ledc_motor_control_task(void *pvParameters);
void set_motor_speed(int speed, motor_id_t motor_id);
void set_motor_pwm_internal(int pwm, motor_id_t motor_id);

#ifdef __cplusplus
}
#endif

#endif // LEDC_MOTOR_CONTROL_H
