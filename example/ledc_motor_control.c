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


// 定义电机旋转状态
typedef enum {
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_direction_t;

// 声明电机当前旋转方向
static motor_direction_t current_direction = MOTOR_FORWARD;

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

void set_motor_speed_test(int speed) {
    uint32_t abs_speed = abs(speed); // 获取速度的绝对值

    // 确保速度值不超出范围
    if (abs_speed > 100) {
        abs_speed = 100;
    }

    uint32_t duty_cycle = 1024 * abs_speed / 100; // 转换为对应的占空比值

    // 根据速度正负判断方向
    if (speed >= 0) {
        current_direction = MOTOR_FORWARD;
        // 设置正转
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
        // 停止另一通道
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, 0);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);
    } else {
        current_direction = MOTOR_REVERSE;
        // 设置反转
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);
        // 停止另一通道
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, 0);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
    }
}

/**
 * 设置电机速度和方向。
 * 
 * @param speed 正值表示正转速度的占空比，负值表示反转速度的占空比。
 */
void set_motor_speed(int speed) {
    uint32_t abs_speed = abs(speed); // 获取速度的绝对值

    // 确保速度值不超出范围
    if (abs_speed > 100) {
        abs_speed = 100;
    }

    uint32_t duty_cycle = 1024 * abs_speed / 100; // 转换为对应的占空比值

    // 根据速度正负判断方向
    if (speed >= 0) {
        current_direction = MOTOR_FORWARD;
        // 设置正转
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
        // 停止另一通道
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, 0);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);
    } else {
        current_direction = MOTOR_REVERSE;
        // 设置反转
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH1_CHANNEL);
        // 停止另一通道
        ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, 0);
        ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
    }
}

/**
 * FreeRTOS任务，用于控制电机的占空比
 */
void ledc_motor_control_task(void *pvParameters) {

    // 初始化LEDC
    ledc_motor_init();

    while (1) {

        uint32_t random_speed = esp_random() % 101; // esp_random()返回一个uint32_t随机数，% 101确保结果在0到100范围内

        // 随机决定电机方向：正转或反转
        motor_direction_t direction = (esp_random() % 2) ? MOTOR_FORWARD : MOTOR_REVERSE;
        int speed = (direction == MOTOR_FORWARD) ? random_speed : -random_speed; // 如果方向是反转，速度值取负

        set_motor_speed(speed);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 每2秒更新一次方向
    }
}

// void app_main(void) {
//     xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
// }
