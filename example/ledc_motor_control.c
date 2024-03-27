#include "ledc_motor_control.h"

// 电机控制的GPIO定义
#define MOTOR_A_GPIO 18
#define MOTOR_B_GPIO 19

// LEDC配置参数
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (MOTOR_A_GPIO)
#define LEDC_HS_CH1_GPIO       (MOTOR_B_GPIO)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_FREQ_HZ           (1000)   // PWM频率为1kHz
#define LEDC_RESOLUTION        LEDC_TIMER_10_BIT // 分辨率为10位

/**
 * 初始化LEDC定时器和通道，用于控制电机
 */
void ledc_motor_init(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ_HZ,
        .speed_mode = LEDC_HS_MODE,
        .timer_num = LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK, // 使用自动时钟配置
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_0 = {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel_0);

    ledc_channel_config_t ledc_channel_1 = {
        .channel    = LEDC_HS_CH1_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH1_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel_1);
}

/**
 * FreeRTOS任务，用于控制电机的占空比
 */
void ledc_motor_control_task(void *pvParameters) {
    // 电机PWM占空比初始值
    uint32_t duty_a = 1024 * 40 / 100; // 40%
    uint32_t duty_b = 1024 * 60 / 100; // 60%

    // 初始化LEDC
    ledc_motor_init();

    while (1) {
        // 设置电机A的占空比
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty_a);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);

        // 设置电机B的占空比
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, duty_b);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);

        // 简单的占空比调整逻辑，用于示范
        duty_a = (duty_a + 102) % 1024;
        duty_b = (duty_b + 204) % 1024;

        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒更新一次
    }
}

// void app_main(void) {
//     xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
// }
