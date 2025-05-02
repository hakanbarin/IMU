// allservice.c

#include "allservice.h"
#include <stdio.h>
#include <string.h>
#include <windows.h>

#define COM_PORT  "COM5"
#define BAUD_RATE 115200

// Public API fonksiyonları

int calibrate_PID(float kp, float ki, float kd) {
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service    = CALIBRATE_PID;
    msg.data.p1.kp = kp;
    msg.data.p1.ki = ki;
    msg.data.p1.kd = kd;
    return send_message(&msg);
}

int set_degrees_of_yaw(float degrees) {
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service         = SET_DEGREES_OF_YAW;
    msg.data.p2.degrees = degrees;
    return send_message(&msg);
}

int set_depth_cm(float cm) {
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service    = SET_DEPTH_CM;
    msg.data.p3.cm = cm;
    return send_message(&msg);
}

int set_complementary_filters(float alpha_pitch,
                              float alpha_yaw,
                              float alpha_roll,
                              float alpha_stabilize)
{
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service                     = COEFFICIENT_COMPLEMENTARY_FILTERS;
    msg.data.p4.alpha_for_pitch     = alpha_pitch;
    msg.data.p4.alpha_for_yaw       = alpha_yaw;
    msg.data.p4.alpha_for_roll      = alpha_roll;
    msg.data.p4.alpha_for_stabilize = alpha_stabilize;
    return send_message(&msg);
}

int pwm_motors_for_drive_one_by_one(uint16_t pwm1, uint16_t pwm2,
                                    uint16_t pwm3, uint16_t pwm4,
                                    uint16_t pwm5, uint16_t pwm6,
                                    uint16_t pwm7, uint16_t pwm8)
{
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service      = PWM_MOTORS_FOR_DRIVE_ONE_BY_ONE;
    msg.data.p5.pwm1 = pwm1;
    msg.data.p5.pwm2 = pwm2;
    msg.data.p5.pwm3 = pwm3;
    msg.data.p5.pwm4 = pwm4;
    msg.data.p5.pwm5 = pwm5;
    msg.data.p5.pwm6 = pwm6;
    msg.data.p5.pwm7 = pwm7;
    msg.data.p5.pwm8 = pwm8;
    return send_message(&msg);
}

int for_arm(uint8_t arm_or_disarm) {
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service            = FOR_ARM;
    msg.data.p6.arm_or_disarm = arm_or_disarm;
    return send_message(&msg);
}

int led_control(uint8_t led_aktif) {
    rpc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.service       = LED_CONTROL;
    msg.data.p7.led_aktif = led_aktif;
    return send_message(&msg);
}
