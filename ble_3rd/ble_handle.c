// #include "ble_handle.h"

// static const char *TAG = "BLE_GATTS";

// // BLE GATTS 事件处理器
// void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
//     // 处理GATTS事件
// }

// // 初始化蓝牙
// void ble_init(void) {
//     esp_err_t ret;

//     // 初始化NVS
//     ret = nvs_flash_init();
//     ESP_ERROR_CHECK(ret);

//     // 初始化蓝牙控制器
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     ESP_ERROR_CHECK(ret);

//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     ESP_ERROR_CHECK(ret);

//     // 初始化Bluedroid
//     ret = esp_bluedroid_init();
//     ESP_ERROR_CHECK(ret);

//     ret = esp_bluedroid_enable();
//     ESP_ERROR_CHECK(ret);

//     // 注册GATTS事件处理器
//     esp_ble_gatts_register_callback(gatts_event_handler);
// }

// // BLE 任务
// void ble_task(void *pvParameter) {
//     // 在这里添加BLE任务逻辑，如初始化GATT服务等
//     ESP_LOGI(TAG, "BLE Task Started");

//     for (;;) {
//         // 任务循环，可以通过队列接收消息等
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

// // void app_main(void) {
// //     // 初始化蓝牙
// //     ble_init();

// //     // 创建BLE任务
// //     xTaskCreate(ble_task, "ble_task", 4096, NULL, 5, NULL);
// // }
