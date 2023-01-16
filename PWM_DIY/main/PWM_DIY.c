#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include "driver/ledc.h"

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
// #if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (19)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
// #endif

#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)


void app_main(void)
{

    int ch;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    ledc_fade_func_install(0);

    int dduu = 0 ;

    while (1) {
        printf("1. LEDC 18 fade up to duty = %d\n", LEDC_TEST_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
        //     ledc_fade_start(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        // }
        // ledc_set_fade_with_time(ledc_channel[2].speed_mode,
        //             ledc_channel[2].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);

        // ledc_fade_start(ledc_channel[2].speed_mode,
        //             ledc_channel[2].channel, LEDC_FADE_NO_WAIT);
        
        while(dduu < 10000 ){
            ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, dduu);
            dduu = dduu + 500 ;
            ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);
            vTaskDelay(200/ portTICK_PERIOD_MS);
        }
        while(dduu > 0){
            ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, dduu);
            dduu = dduu - 500 ;
            ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

    }

    // printf("Testing brushed motor...\n");
    // xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, NULL, 5, NULL);
}