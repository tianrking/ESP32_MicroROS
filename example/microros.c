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
#include <sensor_msgs/msg/nav_sat_fix.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t gps_publisher;
sensor_msgs__msg__NavSatFix gps_msg;

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
extern float fAcc[3], fGyro[3], fAngle[3];

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        // 填充IMU消息
        // imu_msg.header.stamp = rcl_node_init(&node); // 获取当前时间戳
        imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_frame");

        // 加速度
        imu_msg.linear_acceleration.x = fAcc[0];
        imu_msg.linear_acceleration.y = fAcc[1];
        imu_msg.linear_acceleration.z = fAcc[2];

        // 陀螺仪
        imu_msg.angular_velocity.x = fGyro[0];
        imu_msg.angular_velocity.y = fGyro[1];
        imu_msg.angular_velocity.z = fGyro[2];

        // 姿态角（假设你的fAngle数组包含roll, pitch, yaw的值）
        // 注意：ROS IMU消息的姿态是四元数。如果fAngle是欧拉角，你需要将它们转换为四元数。
        // 这里假设fAngle直接表示四元数
        imu_msg.orientation.x = fAngle[0];
        imu_msg.orientation.y = fAngle[1];
        imu_msg.orientation.z = fAngle[2];
        imu_msg.orientation.w = 1.0; // 假设为单位四元数

        // 发布IMU消息
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

        // RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		send_msg.data++;

        // 填充并发布GPS消息
        
        // gps_msg.header.stamp = rcl_node_now(&node); // 获取当前时间戳
        gps_msg.header.frame_id = micro_ros_string_utilities_set(gps_msg.header.frame_id, "gps_frame");
        gps_msg.latitude = rand() % 90; // 示例: 生成随机纬度
        gps_msg.longitude = rand() % 180; // 示例: 生成随机经度
        gps_msg.altitude = rand() % 100; // 示例: 生成随机高度

        // 发布GPS消息
        RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
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
	// RCCHECK(rclc_publisher_init_default(
	// 	&publisher,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"int32_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_subscriber"));
	
	// Create GPS publisher.
    RCCHECK(rclc_publisher_init_default(
        &gps_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "/gps_data"));

    // Create imu publisher.
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu_data"));

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
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
	unsigned int rcl_wait_timeout = 500;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	// RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
	
	// Spin forever.
	send_msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	// RCCHECK(rcl_publisher_fini(&range_publisher, &node));
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
	RCCHECK(rcl_publisher_fini(&gps_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

