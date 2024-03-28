#include "ledc_motor_control.h"
#include "microros.h"

// 电机控制的GPIO定义
#define MOTOR_A_GPIO 18
#define MOTOR_B_GPIO 19

// LEDC配置参数
#define LEDC_HS_CH0_GPIO       (MOTOR_A_GPIO)
#define LEDC_HS_CH1_GPIO       (MOTOR_B_GPIO)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define MOTOR_C_GPIO 21
#define MOTOR_D_GPIO 22

// 新增LEDC通道定义
#define LEDC_HS_CH2_GPIO       (MOTOR_C_GPIO)
#define LEDC_HS_CH3_GPIO       (MOTOR_D_GPIO)
#define LEDC_HS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_HS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_FREQ_HZ           (1000)   // PWM频率为1kHz
#define LEDC_RESOLUTION        LEDC_TIMER_10_BIT // 分辨率为10位

extern RobotState robot_state;
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


    // 配置第二个电机的LED控制器通道
    ledc_channel_config_t ledc_channel_2 = {
        .channel    = LEDC_HS_CH2_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH2_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel_2);

    ledc_channel_config_t ledc_channel_3 = {
        .channel    = LEDC_HS_CH3_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH3_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel_3);
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

        // set_motor_speed(speed,0);
        // set_motor_speed(speed,1);
    
        set_motor_speed(robot_state.RL,0);
        set_motor_speed(robot_state.RR,1);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 每2秒更新一次方向
    }
}

void set_motor_pwm_internal(int pwm, motor_id_t motor_id) {
    // 校正PWM值，确保其在合理范围
    uint32_t abs_pwm = abs(pwm) > 1023 ? 1023 : abs(pwm);

    ledc_channel_t channel_forward, channel_reverse;

    // 根据电机ID选择正确的通道
    switch (motor_id) {
        case MOTOR_1:
            channel_forward = LEDC_HS_CH0_CHANNEL;
            channel_reverse = LEDC_HS_CH1_CHANNEL;
            break;
        case MOTOR_2:
            channel_forward = LEDC_HS_CH2_CHANNEL;
            channel_reverse = LEDC_HS_CH3_CHANNEL;
            break;
        // 如果有更多电机，继续在这里添加case
        default:
            return; // 如果传入了无效的电机ID，直接返回
    }

    // 设置PWM
    if (pwm >= 0) {
        ledc_set_duty(LEDC_HS_MODE, channel_forward, abs_pwm);
        ledc_update_duty(LEDC_HS_MODE, channel_forward);
        ledc_set_duty(LEDC_HS_MODE, channel_reverse, 0);
        ledc_update_duty(LEDC_HS_MODE, channel_reverse);
    } else {
        ledc_set_duty(LEDC_HS_MODE, channel_reverse, abs_pwm);
        ledc_update_duty(LEDC_HS_MODE, channel_reverse);
        ledc_set_duty(LEDC_HS_MODE, channel_forward, 0);
        ledc_update_duty(LEDC_HS_MODE, channel_forward);
    }
}

void set_motor_speed(int speed, motor_id_t motor_id) {
    // 将速度转换为PWM值
    uint32_t pwm_value = 1024 * abs(speed) / 100;
    set_motor_pwm_internal(pwm_value, motor_id);
}

// void app_main(void) {
//     xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
// }
