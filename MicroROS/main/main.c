#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
// #if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (19)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
// #endif

#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

int ch;

ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
	{
		.channel    = LEDC_LS_CH0_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_LS_CH0_GPIO,
		.speed_mode = LEDC_LS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_LS_TIMER
	},
	{
		.channel    = LEDC_LS_CH1_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_LS_CH1_GPIO,
		.speed_mode = LEDC_LS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_LS_TIMER
	},
	{
		.channel    = LEDC_LS_CH2_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_LS_CH2_GPIO,
		.speed_mode = LEDC_LS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_LS_TIMER
	},
	{
		.channel    = LEDC_LS_CH3_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_LS_CH3_GPIO,
		.speed_mode = LEDC_LS_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_LS_TIMER
	},
};

ledc_timer_config_t ledc_timer = {
	.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
	.freq_hz = 5000,                      // frequency of PWM signal
	.speed_mode = LEDC_LS_MODE,           // timer mode
	.timer_num = LEDC_LS_TIMER,            // timer index
	.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
};

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

int kk = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		send_msg.data = kk;
		printf("Sent: %d\n", send_msg.data);
		send_msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	kk = msg->data;
	ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, (int)msg->data);
            ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);

	printf("Received: %d\n", msg->data);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "int32_publisher_subscriber_rclc", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_subscriber"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	send_msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	ledc_timer_config(&ledc_timer);
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
	ledc_fade_func_install(0);

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}