#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

// 定义MCPWM操作的GPIO
#define MCPWM_GPIO_A 18
#define MCPWM_GPIO_B 19

/**
 * @brief MCPWM 初始化配置
 */
void mcpwm_example_initialize(void) {
    // 初始化MCPWM单元0的定时器0
    printf("Initializing MCPWM for GPIO %d and %d...\n", MCPWM_GPIO_A, MCPWM_GPIO_B);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MCPWM_GPIO_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MCPWM_GPIO_B);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;  // 设置PWM频率为1000Hz
    pwm_config.cmpr_a = 0;        // 初始占空比A为0
    pwm_config.cmpr_b = 0;        // 初始占空比B为0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

/**
 * @brief 设置MCPWM的两个输出的占空比
 * 
 * @param duty_cycle_a 第一个输出的占空比百分比，范围为0到100
 * @param duty_cycle_b 第二个输出的占空比百分比，范围为0到100
 */
void set_dual_duty_cycle(float duty_cycle_a, float duty_cycle_b) {
    // float duty_a = duty_cycle_a * (1 << MCPWM_TIMER_0) / 100.0;
    // float duty_b = duty_cycle_b * (1 << MCPWM_TIMER_0) / 100.0;
    
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_a);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle_b);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void app_main(void) {
    // 初始化MCPWM
    mcpwm_example_initialize();

    // 设置第一个输出的占空比为40%，第二个输出的占空比为60%
    set_dual_duty_cycle(40.0, 60.0);

    // 在这里，电机将按照设定的占空比运行
    // 您可以根据需要添加更多的逻辑或控制代码
}