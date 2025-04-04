#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

extern uint16_t pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;

extern float depth_integral, depth_error, depth;

void PWM_FOR_SINGLE_MOTOR_CONTROL(int pwm1, int pwm2, int pwm3, int pwm4,  //FOR STOP OR DRIVE ENGINES ONE BY ONE
                                  int pwm5, int pwm6, int pwm7,int pwm8);


void yaw_update_motor(float desired_yaw, float dt);

float pid_control(float setpoint,
                         float measured,
                         float *integral,
                         float *prev_error,
                         float dt);




#ifdef __cplusplus
}
#endif

#endif // IMU_H
