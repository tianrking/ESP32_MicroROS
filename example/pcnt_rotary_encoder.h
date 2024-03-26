#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pcnt.h"

// #define PCNT_INPUT_SIG_IO 18  // Pulse Input GPIO, connect to rotary encoder A signal
// #define PCNT_INPUT_CTRL_IO 19 // Control Input GPIO, connect to rotary encoder B signal
// #define PCNT_HIGH_LIMIT 10000
// #define PCNT_LOW_LIMIT -10000

/**
 * @brief Initialize PCNT for rotary encoder
 */
void pcnt_example_init(void);

/**
 * @brief Task to measure rotary encoder speed
 * @param arg Pointer to task arguments (unused)
 */
void measure_speed_task(void *arg);

#endif // ROTARY_ENCODER_H