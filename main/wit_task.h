#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern float fAcc[3], fGyro[3], fAngle[3];
void imu_init(void);
void imu_task(void *pvParameters);
void Usart0_task(void *pvParameters);
void Usart1_task(void *pvParameters);
void wit_init_all(void);
void wit_debug(void);
#endif // IMU_TASK_H
