#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pcnt.h"

#define PCNT_INPUT_SIG_IO 18 // Pulse Input GPIO，连接到旋转编码器A信号线
#define PCNT_INPUT_CTRL_IO 19 // Control Input GPIO，连接到旋转编码器B信号线
#define PCNT_HIGH_LIMIT 10000
#define PCNT_LOW_LIMIT -10000

static const char *TAG = "Rotary Encoder";

// 初始化PCNT功能，设置两个通道分别处理A、B两路信号
void pcnt_example_init(void) {
    // 通道0配置，用于正转计数
    pcnt_config_t pcnt_config_a = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC, // 上升沿增加计数
        .neg_mode = PCNT_COUNT_DEC, // 下降沿减少计数
        .lctrl_mode = PCNT_MODE_REVERSE, // 低电平反向
        .hctrl_mode = PCNT_MODE_KEEP, // 高电平保持
        .counter_h_lim = PCNT_HIGH_LIMIT,
        .counter_l_lim = PCNT_LOW_LIMIT,
    };
    pcnt_unit_config(&pcnt_config_a);

    // 通道1配置，用于反转计数（与通道0相反）
    pcnt_config_t pcnt_config_b = pcnt_config_a;
    pcnt_config_b.channel = PCNT_CHANNEL_1;
    pcnt_config_b.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_b.neg_mode = PCNT_COUNT_INC;
    pcnt_config_b.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_b.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_unit_config(&pcnt_config_b);

    // 初始化PCNT单元
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void measure_speed_task(void *arg) {
    int16_t count;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500)); // 每500毫秒读取一次

        // 获取当前PCNT的计数值
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        ESP_LOGI(TAG, "Speed: %d pulses/sec", count);

        // 重置计数器
        pcnt_counter_clear(PCNT_UNIT_0);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Rotary Encoder Speed Measurement");

    // 初始化PCNT单元
    pcnt_example_init();

    // 创建一个任务来定期测量速度
    xTaskCreate(measure_speed_task, "measure_speed_task", 2048, NULL, 10, NULL);
}
