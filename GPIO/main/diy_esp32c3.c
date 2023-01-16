#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/gpio.h"

#define D4 2

void app_main(void)
{
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName,"GG");

    gpio_reset_pin(D4);
    gpio_set_direction(D4,GPIO_MODE_OUTPUT);

    while(1){
        gpio_set_level(D4,1);
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
        gpio_set_level(D4,0);
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
    }
}
