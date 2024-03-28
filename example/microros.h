#ifndef MICROROS_TASK_H
#define MICROROS_TASK_H

#include <stdio.h>
#include <string.h>
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
#include <geometry_msgs/msg/twist.h>
 
// error: unknown type name 'rcl_timer_t'
// void micro_ros_task(void * arg);
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
// void subscription_callback(const void * msgin);
void micro_ros_task(void * arg);

void velocity_callback(const void *msgin);

// 定义一个包含机器人状态的结构体
typedef struct {
    int mode,last_mode;
    int state,last_state;
    float acc[3];          // 加速度计数据
    float gyro[3];         // 陀螺仪数据
    float angle[3];        // 角度数据
    float latitude;        // 纬度
    float longitude;       // 经度
    float altitude;        // 高度
    float linear_velocity_except; // 期望线速度 microros下发
    float angular_velocity_except;// 期望角速度 microros下发
    float linear_velocity; // 真实线速度
    float angular_velocity;// 真实角速度
    float RR,RL,FR,FL;
    float RR_ex,RL_ex,FR_ex,FL_ex;
} RobotState;

// 声明一个全局变量用于存储机器人状态
extern RobotState robot_state;

#endif // MICROROS_TASK_H

