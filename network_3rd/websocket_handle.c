// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "cJSON.h"
// #include "libwebsockets.h"

// static const char *TAG = "ws_server";
// extern float latitude, longitude, altitude;

// // 将GPS数据转换为JSON字符串
// char* format_gps_data() {
//     cJSON *root = cJSON_CreateObject();
//     if (!root) {
//         return NULL;
//     }

//     cJSON_AddNumberToObject(root, "latitude", latitude);
//     cJSON_AddNumberToObject(root, "longitude", longitude);
//     cJSON_AddNumberToObject(root, "altitude", altitude);

//     char *formatted_data = cJSON_Print(root);
//     cJSON_Delete(root);

//     return formatted_data;
// }

// // 发送GPS数据给所有已连接的客户端
// void send_gps_data(struct lws_context *context) {
//     char *gps_data = format_gps_data();
//     if (!gps_data) {
//         ESP_LOGE(TAG, "Failed to format GPS data");
//         return;
//     }

//     // 遍历所有连接，发送GPS数据
//     struct lws_protocols *protocol = lws_get_protocol(context);
//     struct lws *wsi = NULL;
//     while ((wsi = lws_get_next_wsi(context, wsi))) {
//         // 确保wsi有效且使用了正确的协议
//         if (lws_get_protocol(wsi) == protocol) {
//             size_t len = strlen(gps_data);
//             unsigned char *buf = malloc(LWS_PRE + len);
//             memcpy(buf + LWS_PRE, gps_data, len);
//             lws_write(wsi, buf + LWS_PRE, len, LWS_WRITE_TEXT);
//             free(buf);
//         }
//     }

//     free(gps_data);
// }

// // WebSocket服务器任务
// void ws_server_task(void *pvParameters) {
//     struct lws_context_creation_info info;
//     memset(&info, 0, sizeof(info));
//     info.port = 8080;

//     struct lws_context *context = lws_create_context(&info);
//     if (!context) {
//         ESP_LOGE(TAG, "WebSocket context creation failed");
//         vTaskDelete(NULL);
//         return;
//     }

//     while (true) {
//         lws_service(context, 1000); // 以1秒的间隔运行WebSocket服务逻辑
//         // 定期发送GPS数据
//         send_gps_data(context);
//     }

//     lws_context_destroy(context);
//     vTaskDelete(NULL);
// }

// // void app_main() {
// //     xTaskCreate(ws_server_task, "ws_server_task", 4096, NULL, 5, NULL);
// // }
