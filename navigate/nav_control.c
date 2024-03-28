#include <math.h>
#include <stdio.h>

// 假设定义了这些全局变量来存储当前和期望的GPS位置
extern float latitude, longitude, altitude;
float expect_latitude, expect_longitude, expect_altitude;

// 假设定义了这些常量来表示不同的速度等级
#define MAX_SPEED 10.0
#define MEDIUM_SPEED 5.0
#define LOW_SPEED 1.0

// 函数原型声明
void calculate_distance_and_direction(float lat1, float lon1, float lat2, float lon2, float *distance, float *direction);
void set_vehicle_direction(float direction);
void set_vehicle_speed(float speed);
void approach_target(void);

int main()
{
    // 初始化当前位置和期望位置
    latitude = 40.712776; // NY
    longitude = -74.005974;
    altitude = 0.0;

    expect_latitude = 42.360081; // 
    expect_longitude = -71.058884;
    expect_altitude = 0.0;

    // 调用函数，使小车逐步逼近目标点
    approach_target();

    return 0;
}

void calculate_distance_and_direction(float lat1, float lon1, float lat2, float lon2, float *distance, float *direction)
{
    float R = 6371e3; // 地球半径，单位：米
    float phi1 = lat1 * M_PI / 180;
    float phi2 = lat2 * M_PI / 180;
    float delta_phi = (lat2 - lat1) * M_PI / 180;
    float delta_lambda = (lon2 - lon1) * M_PI / 180;

    float a = sin(delta_phi / 2) * sin(delta_phi / 2) +
              cos(phi1) * cos(phi2) *
                  sin(delta_lambda / 2) * sin(delta_lambda / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    *distance = R * c; // 计算两点间的距离

    *direction = atan2(sin(delta_lambda) * cos(phi2),
                       cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda));
    *direction = fmod((*direction * 180 / M_PI + 360), 360); // 转换为度
}

void set_vehicle_direction(float direction)
{
    // 假设的函数，用来调整小车的方向
    printf("Setting vehicle direction to %f degrees\n", direction);
}

void set_vehicle_speed(float speed)
{
    // 假设的函数，用来调整小车的速度
    printf("Setting vehicle speed to %f\n", speed);
}

void approach_target()
{
    float distance, direction;
    calculate_distance_and_direction(latitude, longitude, expect_latitude, expect_longitude, &distance, &direction);

    // 调整小车方向
    set_vehicle_direction(direction);

    // 根据距离调整速度
    float speed;
    if (distance > 1000)
    { // 大于1000米时使用最高速度
        speed = MAX_SPEED;
    }
    else if (distance > 100)
    { // 100米到1000米之间使用中速
        speed = MEDIUM_SPEED;
    }
    else
    { // 小于100米时使用低速，提高精度
        speed = LOW_SPEED;
    }

    set_vehicle_speed(speed);
    // 这里的逻辑简化了实际控制小车的过程
    // 在实际应用中，你需要根据小车的具体反馈来动态更新车辆的位置
    // 并反复调用 `approach_target` 或其核心逻辑，直到达到目标位置
    if (distance < 1)
    {                         // 如果认为小车已经足够接近目标位置
        set_vehicle_speed(0); // 停车
        printf("Target reached.\n");
    }
    else
    {
        // 在实际应用中，这里可以是一个循环或者基于定时器的重复调用
        // 来不断更新小车的状态并适应动态变化的环境条件
        printf("Approaching target...\n");
    }
}