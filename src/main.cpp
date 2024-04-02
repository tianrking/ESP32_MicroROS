#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void publisherTask(void *pvParameters) {
  while (1) {
    // 生成1到100之间的随机数
    msg.data = esp_random() % 100 + 1;

    // 发布消息
    rcl_publish(&publisher, &msg, NULL);
    Serial.print("Published: ");
    Serial.println(msg.data);

    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待一秒钟再发布下一个数字
  }
}

char ssid[] = "OOOO";
char psk[]= "oooooooo";

void setup() {
  Serial.begin(115200);
  // 初始化WiFi连接
  WiFi.begin(ssid, psk);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  set_microros_wifi_transports(ssid, psk, IPAddress(192,168,0,118), 8888);

  delay(2000); // 延时等待micro-ROS配置完成

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "hello_microros", "", &support);

  // 初始化publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lucky_number");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // 创建FreeRTOS任务
  xTaskCreate(publisherTask, "PublishTask", 2048, NULL, 1, NULL);
}

void loop() {
  // 空循环，所有操作在FreeRTOS任务中处理
}
