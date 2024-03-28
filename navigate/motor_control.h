#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H


void calculateWheelSpeedsFromVelocity(float linear_velocity, float angular_velocity);
void calculateVelocityFromWheelSpeeds(void) ;

#endif // MOTOR_CONTROL_H