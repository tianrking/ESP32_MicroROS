#ifndef MICROROS_TASK_H
#define MICROROS_TASK_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#include <rclc/executor.h>  
// error: unknown type name 'rcl_timer_t'
void micro_ros_task(void * arg);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback(const void * msgin);

#endif // MICROROS_TASK_H
