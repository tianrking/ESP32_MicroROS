#include "mcpwm_motor_control.h"

static const char *TAG_DC = "mcpwm_example";

// 定义MCPWM操作的GPIO
#define MCPWM_GPIO_A 18
#define MCPWM_GPIO_B 19
#define EXAMPLE_TIMER_PERIOD 1000 // 假设的定时器周期

// 全局变量，存储比较器句柄
static mcpwm_cmpr_handle_t comparator_a = NULL, comparator_b = NULL;
static mcpwm_gen_handle_t generator_a = NULL, generator_b = NULL; // 正确声明全局变量

/**
 * @brief 将占空比百分比转换为比较值
 * 
 * @param duty_cycle 占空比百分比，范围为0到100
 * @return 比较值，用于设置MCPWM的占空比
 */
static uint32_t duty_cycle_to_compare_value(float duty_cycle) {
    // 将占空比百分比转换为在定时器周期内的比较值
    return (uint32_t)((duty_cycle / 100.0) * (float)EXAMPLE_TIMER_PERIOD);
}

void mcpwm_example_initialize(void) {
    ESP_LOGI(TAG_DC, "Initializing MCPWM for GPIO %d and %d...", MCPWM_GPIO_A, MCPWM_GPIO_B);

    // 初始化定时器
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz, 1us per tick
        .period_ticks = EXAMPLE_TIMER_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 初始化操作器
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // 初始化发生器和比较器
    mcpwm_generator_config_t generator_config_a = {.gen_gpio_num = MCPWM_GPIO_A};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_a, &generator_a));

    mcpwm_generator_config_t generator_config_b = {.gen_gpio_num = MCPWM_GPIO_B};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_b, &generator_b));

}

void set_dual_duty_cycle(float duty_cycle_a, float duty_cycle_b) {
    // 计算比较值
    uint32_t compare_value_a = duty_cycle_to_compare_value(duty_cycle_a);
    uint32_t compare_value_b = duty_cycle_to_compare_value(duty_cycle_b);

    // 确保比较器句柄在使用前已经创建
    if (comparator_a == NULL || comparator_b == NULL) {
        ESP_LOGE(TAG_DC, "Comparator handle(s) not initialized");
        return; // Early return or handle the error as needed
    }

    // 检查比较值是否在合理范围内
    if (compare_value_a >= EXAMPLE_TIMER_PERIOD || compare_value_b >= EXAMPLE_TIMER_PERIOD) {
        ESP_LOGE(TAG_DC, "Compare value out of range");
        return; // Early return or handle the error as needed
    }

    // 设置比较器的比较值以调整占空比
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, compare_value_a));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, compare_value_b));
}

void motor_control_task(void *pvParameters) {
    mcpwm_example_initialize();

    while (1) {
        set_dual_duty_cycle(40, 60);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 根据需要调整延时
    }
}

// void app_main(void) {
//     xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
// }
