#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wit_task.h"
#include "microros.h"
#include <unistd.h> 
#include <uros_network_interfaces.h>

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
            10240,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);


	wit_init_all();
	while (1)
	{
		wit_debug();
	}
}

