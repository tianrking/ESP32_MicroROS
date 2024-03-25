#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wit_task.h"
#include "microros.h"

void app_main(void)
{
	xTaskCreate(Usart0_task, "Usart0_task", 4096, NULL, 5, NULL);
	xTaskCreate(Usart1_task, "Usart1_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "Usart1_task", 4096, NULL, 5, NULL);

	wit_init_all();
	while (1)
	{
		wit_debug();
	}
}

