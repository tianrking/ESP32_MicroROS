#define WHEEL_BASE 0.5f // 轮间距，单位为米，根据您的机器人实际调整

#include "microros.h"

extern RobotState robot_state;
// 根据后轮速度计算并更新线速度和角速度的函数
#define WHEEL_BASE 0.5f // 轮间距，单位为米，根据您的机器人实际调整


//正解
void calculateWheelSpeedsFromVelocity(float linear_velocity, float angular_velocity) {
    // 根据线速度和角速度计算后轮速度
    robot_state.RL = linear_velocity - (WHEEL_BASE / 2.0f) * angular_velocity;
    robot_state.RR = linear_velocity + (WHEEL_BASE / 2.0f) * angular_velocity;

    // 可选：更新前轮速度，如果前轮也是驱动轮
    // robot_state.FL = robot_state.RL;
    // robot_state.FR = robot_state.RR;
    
    // 打印或进一步处理后轮速度
    printf("Rear Left Wheel Speed: %f m/s\n", robot_state.RL);
    printf("Rear Right Wheel Speed: %f m/s\n", robot_state.RR);
}

//逆解算
void calculateVelocityFromWheelSpeeds() {
    // 根据后轮速度计算线速度和角速度
    robot_state.linear_velocity = (robot_state.RR + robot_state.RL) / 2.0f;
    robot_state.angular_velocity = (robot_state.RR - robot_state.RL) / WHEEL_BASE;

    // 打印或进一步处理线速度和角速度
    printf("Linear Velocity: %f m/s\n", robot_state.linear_velocity);
    printf("Angular Velocity: %f rad/s\n", robot_state.angular_velocity);
}
