#include "driver/ledc.h"
#include "esp_err.h"

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

void app_main(void) {
    // 设置LEDC定时器配置
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_RESOLUTION, // 分辨率
        .freq_hz = LEDC_FREQ_HZ,             // PWM信号频率
        .speed_mode = LEDC_HS_MODE,          // 速度模式
        .timer_num = LEDC_HS_TIMER           // 定时器编号
    };
    // 初始化LEDC定时器
    ledc_timer_config(&ledc_timer);

    // 设置LEDC通道配置 - 电机A
    ledc_channel_config_t ledc_channel_0 = {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    // 设置LEDC通道配置 - 电机B
    ledc_channel_config_t ledc_channel_1 = {
        .channel    = LEDC_HS_CH1_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH1_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };

    // 初始化LEDC通道
    ledc_channel_config(&ledc_channel_0);
    ledc_channel_config(&ledc_channel_1);

    // 设置电机A的占空比为40%
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, 1024 * 40 / 100);
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);

    // 设置电机B的占空比为60%
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, 1024 * 60 / 100);
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);
}
