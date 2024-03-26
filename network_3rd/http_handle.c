#include "http_handle.h"

#define TAG "HTTP_CLIENT"

// HTTP事件处理器
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
                printf("%.*s\n", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT: // 处理重定向事件
            ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}


// 发送HTTP GET请求到百度并输出响应
void http_get_request_to_baidu() {
    esp_http_client_config_t config = {
        .url = "http://www.baidu.com",
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// HTTP请求任务
void http_request_task(void *pvParameters) {
    while(1) {
        ESP_LOGI(TAG, "Sending HTTP GET request to baidu.com");
        // http_get_request_to_baidu();

        // 每次请求后等待一段时间
        vTaskDelay(pdMS_TO_TICKS(10000)); // 等待10秒
    }
}