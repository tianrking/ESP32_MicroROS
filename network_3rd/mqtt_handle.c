// #include "mqtt_handle.h"

// static const char *TAG = "MQTT_APP";

// static void log_error_if_nonzero(const char *message, int error_code) {
//     if (error_code != 0) {
//         ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
//     }
// }

// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     switch ((esp_mqtt_event_id_t)event_id) {
//     case MQTT_EVENT_CONNECTED:
//         ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//         break;
//     case MQTT_EVENT_DISCONNECTED:
//         ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
//         break;
//     case MQTT_EVENT_DATA:
//         ESP_LOGI(TAG, "MQTT_EVENT_DATA");
//         printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
//         printf("DATA=%.*s\r\n", event->data_len, event->data);
//         break;
//     case MQTT_EVENT_ERROR:
//         ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
//         break;
//     default:
//         ESP_LOGI(TAG, "Other event id:%d", event_id);
//         break;
//     }
// }


// void mqtt_app_start(void) {
//     const esp_mqtt_client_config_t mqtt_cfg = {
//         .url = "broker.hivemq.com", 
//     };

//     esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
//     esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
//     esp_mqtt_client_start(client);
// }

// // void app_main(void) {
// //     ESP_LOGI(TAG, "Starting MQTT client");
// //     mqtt_app_start();
// // }
