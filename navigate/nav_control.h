#ifndef NAV_CONTROL_H
#define NAV_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// 声明外部使用的变量，这里假设这些全局变量在其他地方定义和更新
// extern float latitude, longitude, altitude;
extern float expect_latitude, expect_longitude, expect_altitude;

// 定义速度等级的常量
#define MAX_SPEED 10.0
#define MEDIUM_SPEED 5.0
#define LOW_SPEED 1.0

/**
 * 计算两个GPS点之间的距离和方向。
 * 
 * @param lat1 第一个点的纬度
 * @param lon1 第一个点的经度
 * @param lat2 第二个点的纬度
 * @param lon2 第二个点的经度
 * @param distance 指向存储计算出的距离的指针
 * @param direction 指向存储计算出的方向的指针
 */
void calculate_distance_and_direction(float lat1, float lon1, float lat2, float lon2, float* distance, float* direction);

/**
 * 设置小车的行进方向。
 * 
 * @param direction 行进方向（度），以正北为0度，顺时针增加
 */
void set_vehicle_direction(float direction);

/**
 * 设置小车的行进速度。
 * 
 * @param speed 行进速度
 */
void set_vehicle_speed(float speed);

/**
 * 使小车逐步逼近目标GPS点的主控制函数。
 */
void approach_target(void);

#ifdef __cplusplus
}
#endif

#endif // NAV_CONTROL_H
