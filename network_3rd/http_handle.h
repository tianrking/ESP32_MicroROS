#ifndef __HTTP_C_SDK_H
#define __HTTP_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_err.h"

void http_get_request_to_baidu(void);
esp_err_t http_event_handler(esp_http_client_event_t *evt);
void http_request_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

#endif