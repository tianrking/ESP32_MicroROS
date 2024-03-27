#include <string.h>
#include <stdio.h>
#include <unistd.h> 
#include <inttypes.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include <uros_network_interfaces.h>

#include "wit_c_sdk.h"
#include "wit_task.h"
#include "microros.h"
#include "http_handle.h"
#include "websocket_handle.h"
#include "mqtt_handle.h"
#include "pcnt_rotary_encoder.h"

// #include "mcpwm_motor_control.h"
#include "ledc_motor_control.h"

#define TAG "MEM_CHECK"

// 内存检查任务
static void memory_check_task(void *pvParameter);

void app_main(void)
{
	xTaskCreate(Usart0_task, "Usart0_task", 4096, NULL, 5, NULL);
	xTaskCreate(Usart1_task, "Usart1_task", 4096, NULL, 5, NULL);

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
            ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    // CONFIG_MICRO_ROS_APP_STACK,
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            20480,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
        
        xTaskCreate(http_request_task, "http_request_task", 8192, NULL, 5, NULL);
        xTaskCreate(memory_check_task, "memory_check_task", 5120, NULL, 5, NULL);
        // xTaskCreate(mqtt_app_task, "mqtt_app_task", 4096, NULL, 5, NULL);
        // xTaskCreate(ws_server_task, "ws_server_task", 4096, NULL, 5, NULL);

        
    // xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);

    xTaskCreate(ledc_motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);

    xTaskCreate(measure_speed_task, "measure_speed_task", 2048, NULL, 10, NULL);

	wit_init_all();
	while (1)
	{
		wit_debug();
	}
}

// static void memory_check_task(void *pvParameter) {
//     while (1) {
//         // 获取内存统计信息
//         multi_heap_info_t info;
//         heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);

//         // 打印内存统计信息
//         ESP_LOGI(TAG, "Total internal memory: %d", info.total_free_bytes + info.total_allocated_bytes);
//         ESP_LOGI(TAG, "Total free memory: %d", info.total_free_bytes);
//         ESP_LOGI(TAG, "Largest free block: %d", info.largest_free_block);
//         ESP_LOGI(TAG, "Minimum free memory ever: %d", info.minimum_free_bytes);

//         // 每5秒钟检查一次
//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
// }

static const char *TAG_MMM = "MemCheck";
static void memory_check_task(void *pvParameter) {
    while (1) {
        // 获取并打印内存统计信息
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);

        ESP_LOGI(TAG_MMM, "Total internal memory: %zu", info.total_free_bytes + info.total_allocated_bytes);
        ESP_LOGI(TAG_MMM, "Total free memory: %zu", info.total_free_bytes);
        ESP_LOGI(TAG_MMM, "Largest free block: %zu", info.largest_free_block);
        ESP_LOGI(TAG_MMM, "Minimum free memory ever: %zu", info.minimum_free_bytes);

        // 假设系统中不会有超过20个任务同时运行
        const uint32_t taskArraySize = 20;
        TaskStatus_t taskStatusArray[taskArraySize];
        uint32_t totalRunTime;

        // 获取系统中所有任务的状态
        uint32_t activeTasks = uxTaskGetSystemState(taskStatusArray, taskArraySize, &totalRunTime);

        //逐个打印任务的状态信息
        for (uint32_t i = 0; i < activeTasks; i++) {
            ESP_LOGI(TAG_MMM, "Task: %s, Remaining Stack: %" PRIu32,
                     taskStatusArray[i].pcTaskName,
                     taskStatusArray[i].usStackHighWaterMark);
        }

        // 每5秒钟执行一次
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


