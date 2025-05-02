#ifndef ALLSERVICE_H
#define ALLSERVICE_H

#include <stdint.h>
#include "Core/Inc/rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

void send_calibrate_pid(float kp, float ki, float kd);
/*
    * @brief  Send new PID gains to the MCU.
    * @param  kp  Proportional gain
    * @param  ki  Integral gain
    * @param  kd  Derivative gain
    * @return 0 on success, -1 on error    
*/

void send_set_degrees_of_yaw(float degrees);
/*
    * @brief  Send a new desired yaw angle to the MCU.
    * @param  degrees  Yaw in degrees
    * @return 0 on success, -1 on error
*/

void send_set_depth_cm(float cm);
/*
    * @brief  Send a new desired depth (in cm) to the MCU.
    * @param  cm  Depth in centimeters
    * @return 0 on success, -1 on error
*/

void send_coefficient_complementary_filters(float alpha_pitch,
                                           float alpha_yaw,
                                           float alpha_roll,
                                           float alpha_stabilize);
/*
    * @brief  Send complementary filter coefficients.
    * @param  alpha_pitch      Filter α for pitch
    * @param  alpha_yaw        Filter α for yaw
    * @param  alpha_roll       Filter α for roll
    * @param  alpha_stabilize  Filter α for stabilize
    * @return 0 on success, -1 on error
*/

void send_pwm_motors_for_drive_one_by_one(uint16_t pwm1, uint16_t pwm2,
                                         uint16_t pwm3, uint16_t pwm4,
                                         uint16_t pwm5, uint16_t pwm6,
                                         uint16_t pwm7, uint16_t pwm8);
/*
    * @brief  Send PWM setpoints for all 8 motors.
    * @param  pwm1…pwm8  1–8. motor PWM values
    * @return 0 on success, -1 on error
*/

void send_for_arm(uint8_t arm_or_disarm);
/*
    * @brief  Arm or disarm the system.
    * @param  arm_or_disarm  non-zero = arm, zero = disarm
    * @return 0 on success, -1 on error
*/

void send_led_control(float led_aktif);
/*
    * @brief  Turn the LED on or off.
    * @param  led_aktif  non-zero = LED on, zero = LED off
    * @return 0 on success, -1 on error
*/

#ifdef __cplusplus
}
#endif

#endif // ALLSERVICE_H
