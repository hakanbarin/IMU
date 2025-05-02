// client.c

#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <string.h>
#include "Core/Inc/rpc.h"

#define COM_PORT   "COM5"
#define BAUD_RATE  115200

// Open and configure the serial port.
// Returns a valid HANDLE, or INVALID_HANDLE_VALUE on error.
HANDLE open_serial(void) {
    HANDLE h = CreateFileA(
        COM_PORT,
        GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error: could not open port %s\n", COM_PORT);
        return INVALID_HANDLE_VALUE;
    }
    DCB dcb = { .DCBlength = sizeof(dcb) };
    if (!GetCommState(h, &dcb)) {
        fprintf(stderr, "Error: GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = BAUD_RATE;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(h, &dcb)) {
        fprintf(stderr, "Error: SetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }
    return h;
}

// Close a previously opened serial port.
void close_serial(HANDLE h) {
    if (h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
    }
}




// Send an rpc_message_t packet over the serial port.
void send_message(const rpc_message_t *msg) {
    HANDLE h = open_serial();
    if (h == INVALID_HANDLE_VALUE) {
        return;
    }
    DWORD written = 0;
    WriteFile(h, msg, sizeof(*msg), &written, NULL);
    close_serial(h);
}


// Send new PID gains to the MCU.
void calibrate_PID(float kp, float ki, float kd) {
    rpc_message_t msg = { .service = CALIBRATE_PID };
    msg.data.p1.kp = kp;
    msg.data.p1.ki = ki;
    msg.data.p1.kd = kd;
    send_message(&msg);
}

// Send a new desired yaw angle to the MCU.
void set_degrees_of_yaw(float degrees) {
    rpc_message_t msg = { .service = SET_DEGREES_OF_YAW };
    msg.data.p2.degrees = degrees;
    send_message(&msg);
}

// Send a new desired depth (in cm) to the MCU.
void set_depth_cm(float cm) {
    rpc_message_t msg = { .service = SET_DEPTH_CM };
    msg.data.p3.cm = cm;
    send_message(&msg);
}

// Send complementary filter coefficients to the MCU.
void set_complementary_filters(float alpha_pitch,
                               float alpha_yaw,
                               float alpha_roll,
                               float alpha_stabilize)
{
    rpc_message_t msg = { .service = COEFFICIENT_COMPLEMANTARY_FILTER};
    msg.data.p4.alpha_for_pitch     = alpha_pitch;
    msg.data.p4.alpha_for_yaw       = alpha_yaw;
    msg.data.p4.alpha_for_roll      = alpha_roll;
    msg.data.p4.alpha_for_stabilize = alpha_stabilize;
    send_message(&msg);
}

// Send PWM setpoints for all 8 motors.
void pwm_motors_for_drive_one_by_one(uint16_t pwm1, uint16_t pwm2,
                                     uint16_t pwm3, uint16_t pwm4,
                                     uint16_t pwm5, uint16_t pwm6,
                                     uint16_t pwm7, uint16_t pwm8)
{
    rpc_message_t msg = { .service = PWM_MOTORS_FOR_STOP };
    msg.data.p5.pwm1 = pwm1;
    msg.data.p5.pwm2 = pwm2;
    msg.data.p5.pwm3 = pwm3;
    msg.data.p5.pwm4 = pwm4;
    msg.data.p5.pwm5 = pwm5;
    msg.data.p5.pwm6 = pwm6;
    msg.data.p5.pwm7 = pwm7;
    msg.data.p5.pwm8 = pwm8;
    send_message(&msg);
}

// Arm or disarm the system.
void for_arm(uint8_t arm_or_disarm) {
    rpc_message_t msg = { .service = FOR_ARM };
    msg.data.p6.arm_or_disarm = arm_or_disarm;
    send_message(&msg);
}

// Turn the LED on or off.
void led_control(float led_aktif) {
    rpc_message_t msg = { .service = LED_CONTROL };
    msg.data.p7.led_aktif = led_aktif;
    send_message(&msg);
}







int main(void) {
    // Example usage:

    led_control(2.5f);

    printf("LED turned on\n");

    calibrate_PID(12.5f, 0.8f, 0.02f);

    printf("PID gains sent: Kp=12.5, Ki=0.8, Kd=0.02\n");

    set_degrees_of_yaw(90.0f);

    printf("Desired yaw set to 45°\n");

    set_depth_cm(150.0f);

    printf("Desired depth set to 150 cm\n");

    set_complementary_filters(0.02f, 0.02f, 0.02f, 0.5f);

    printf("Complementary filter coefficients sent\n");

    pwm_motors_for_drive_one_by_one(
        1200,1200,1200,1200,
        1200,1200,1200,1200
    );
    printf("PWM values for 8 motors sent\n");

    for_arm(2);
    printf("System armed\n");

    return 0;
}
