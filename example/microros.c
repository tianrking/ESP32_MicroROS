#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

std_msgs__msg__Int32 recv_msg;

rcl_publisher_t range_publisher;
sensor_msgs__msg__Range range_msg;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t temperature_publisher;
sensor_msgs__msg__Temperature temperature_msg;

int kk = 1 ;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		//printf("Sent: %d\n", send_msg.data);
		
		range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND ;
		range_msg.min_range = 1.0;
		range_msg.max_range = 5.0 ;
		range_msg.field_of_view = 0.3 ;
		range_msg.header.frame_id = micro_ros_string_utilities_set(range_msg.header.frame_id, "/ultrasonic_sensor_test_link");
		
		if(range_msg.range > range_msg.max_range){
			kk = -1 ;
		}
		if(range_msg.range < range_msg.min_range){
			kk = 1 ;
		}

		range_msg.range += kk * 0.1 ;


		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

		imu_msg.header.frame_id = micro_ros_string_utilities_set(range_msg.header.frame_id, "/imu_link");
		// imu_msg.orientation = 
		// imu_msg.orientation_covariance[9] =
		// imu_msg.angular_velocity =
		// imu_msg.angular_velocity_covariance[9] =
		// imu_msg.linear_acceleration =
		// imu_msg.linear_acceleration_covariance[9]=

		// msg.linear_acceleration.x = ax;
        // msg.linear_acceleration.y = ay;
        // msg.linear_acceleration.z = az;
		imu_msg.linear_acceleration.x = 1;
        imu_msg.linear_acceleration.y = 1;
        imu_msg.linear_acceleration.z = 1;

        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[1] = 0;
        imu_msg.linear_acceleration_covariance[2] = 0;

        imu_msg.linear_acceleration_covariance[3] = 0;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[5] = 0;

        imu_msg.linear_acceleration_covariance[6] = 0;
        imu_msg.linear_acceleration_covariance[7] = 0;
        imu_msg.linear_acceleration_covariance[8] = 0.04;


        // imu_msg.angular_velocity.x = gx;
        // imu_msg.angular_velocity.y = gy;
        // imu_msg.angular_velocity.z = gz;
		imu_msg.angular_velocity.x = 1;
        imu_msg.angular_velocity.y = 1;
        imu_msg.angular_velocity.z = 1;


        imu_msg.angular_velocity_covariance[0] = 0.02;
        imu_msg.angular_velocity_covariance[1] = 0;
        imu_msg.angular_velocity_covariance[2] = 0;

        imu_msg.angular_velocity_covariance[3] = 0;
        imu_msg.angular_velocity_covariance[4] = 0.02;
        imu_msg.angular_velocity_covariance[5] = 0;

        imu_msg.angular_velocity_covariance[6] = 0;
        imu_msg.angular_velocity_covariance[7] = 0;
        imu_msg.angular_velocity_covariance[8] = 0.02;

        // imu_msg.orientation.w = ahrs.getW();
        // imu_msg.orientation.x = ahrs.getX();
        // imu_msg.orientation.y = ahrs.getY();
        // imu_msg.orientation.z = ahrs.getZ();
		imu_msg.orientation.w = 1;
        imu_msg.orientation.x = 1;
        imu_msg.orientation.y = 1;
        imu_msg.orientation.z = 1;

        imu_msg.orientation_covariance[0] = 0.0025;
        imu_msg.orientation_covariance[1] = 0;
        imu_msg.orientation_covariance[2] = 0;

        imu_msg.orientation_covariance[3] = 0;
        imu_msg.orientation_covariance[4] = 0.0025;
        imu_msg.orientation_covariance[5] = 0;

        imu_msg.orientation_covariance[6] = 0;
        imu_msg.orientation_covariance[7] = 0;
        imu_msg.orientation_covariance[8] = 0.0025;

		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

		temperature_msg.header.frame_id = micro_ros_string_utilities_set(temperature_msg.header.frame_id, "/temperature_link");
		temperature_msg.temperature = send_msg.data ;
		temperature_msg.variance = send_msg.data ;
		RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));

		send_msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
//	printf("Received: %d\n", msg->data);
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
	
	// Create range publisher.
	RCCHECK(rclc_publisher_init_default(
		&range_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
		"range_publisher"));

	// Create imu publisher.
	RCCHECK(rclc_publisher_init_default(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"imu_publisher"));
	
	// Create imu temperature
	RCCHECK(rclc_publisher_init_default(
		&temperature_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
		"temperature_publisher"));

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
	unsigned int rcl_wait_timeout = 500;   // in ms
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
	RCCHECK(rcl_publisher_fini(&range_publisher, &node));
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
	RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18) // 第一个电机的GPIO
#define LEDC_HS_CH1_GPIO       (19) // 第二个电机的GPIO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_FREQ_HZ           (5000) // PWM信号频率
#define LEDC_RESOLUTION        LEDC_TIMER_13_BIT // PWM分辨率

void ledc_init(void) {
    // 设置定时器配置
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ_HZ,
        .speed_mode = LEDC_HS_MODE,
        .timer_num = LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // 设置通道配置
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        }
    };

    // 配置LED控制器通道
    for (int ch = 0; ch < 2; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
}

// 设置电机速度百分比，范围为0到100
void set_motor_speed_percentage(int motor, int percentage) {
    if (percentage < 0 || percentage > 100) {
        printf("Percentage out of range (0-100)\n");
        return;
    }
    // 将百分比转换为具体的PWM占空比值
    uint32_t duty = (percentage * ((1 << LEDC_RESOLUTION) - 1)) / 100;
    
    // 根据电机选择对应的通道
    ledc_channel_t channel = (motor == 1) ? LEDC_HS_CH0_CHANNEL : LEDC_HS_CH1_CHANNEL;
    
    ledc_set_duty(LEDC_HS_MODE, channel, duty);
    ledc_update_duty(LEDC_HS_MODE, channel);
}

void app_main(void) {
    // 初始化PWM
    ledc_init();

    // 设置电机速度为50%，作为示例
    set_motor_speed_percentage(1, 50); // 第一个电机
    // set_motor_speed_percentage(2, 75); // 第二个电机

    // 以下代码可以根据需要加入其他逻辑
}

#include "driver/ledc.h"
#include "esp_err.h"

void app_main(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
	#include "driver/ledc.h"
#include "esp_err.h"

void app_main(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = 19,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    // 设置占空比为50%
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 4096);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

}