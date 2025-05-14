#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

extern uint16_t pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;
extern uint8_t is_armed;
extern float led_aktif_main;

extern float depth_integral, depth_error, desired_yaw;
extern float KP;
extern float KI;
extern float KD;
extern float ALPHA;
extern float ALPHA1;
extern float ALPHA2;
extern float alpha_for_stabilize;
void PWM_FOR_SINGLE_MOTOR_CONTROL(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4,  //FOR STOP OR DRIVE ENGINES ONE BY ONE
								  uint16_t pwm5, uint16_t pwm6, uint16_t pwm7,uint16_t pwm8);

float pid_control(float setpoint,
                         float measured,
                         float *integral,
                         float *prev_error,
                         float dt);



void try_to_engine(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_H
