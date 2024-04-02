#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>

#define TX_PIN 17
#define RX_PIN 16
#define UART_NUM UART_NUM_2

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void setupWiFi() {
  const char* ssid = "OOOO"; // 替换为你的WiFi SSID
  const char* password = "oooooooo"; // 替换为你的WiFi密码
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
}

void uartTask(void *pvParameters) {
    uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uartConfig);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0);

    char numStr[4]; // Enough for 3 digits and null terminator
    while (true) {
        for (int i = 1; i <= 100; i++) {
            snprintf(numStr, sizeof(numStr), "%d\n", i);
            uart_write_bytes(UART_NUM, numStr, strlen(numStr));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void microROSTask(void *pvParameters) {
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;

    IPAddress agent_ip;
    agent_ip.fromString("192.168.0.118");
    // micro-ROS setup
    set_microros_wifi_transports("OOOO", "oooooooo", agent_ip, 8888);
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    rcl_node_t node;
    rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support);

    // Initialize publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "lucky_number"
    );

    while (true) {
        for (int i = 1; i <= 100; i++) {
            msg.data = i;
            rcl_publish(&publisher, &msg, NULL);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void setup() {
  Serial.begin(115200);
  setupWiFi();

  // Create tasks for UART communication and micro-ROS publisher
  xTaskCreate(uartTask, "uartTask", 2048, NULL, 1, NULL);
  xTaskCreate(microROSTask, "microROSTask", 4096, NULL, 1, NULL);
}

void loop() {
  // Main loop does nothing, tasks handle everything
}
