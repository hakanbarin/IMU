// client_linux.c

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "rpc.h"

#define SERIAL_PORT "/dev/ttyACM2"
#define BAUD_RATE B115200

int open_serial(void) {
    int fd = open(SERIAL_PORT, O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    return fd;
}

void send_message(const rpc_message_t *msg) {
    int fd = open_serial();
    if (fd < 0) return;

    write(fd, msg, sizeof(*msg));
    close(fd);
}

// Di�er fonksiyonlar (kalibrasyon, led kontrol, pwm vs.) ayn� kalabilir:
void calibrate_PID(float kp, float ki, float kd) {
    rpc_message_t msg = { .service = CALIBRATE_PID };
    msg.data.p1.kp = kp;
    msg.data.p1.ki = ki;
    msg.data.p1.kd = kd;
    send_message(&msg);
}

void set_degrees_of_yaw(float degrees) {
    rpc_message_t msg = { .service = SET_DEGREES_OF_YAW };
    msg.data.p2.degrees = degrees;
    send_message(&msg);
}

void set_depth_cm(float cm) {
    rpc_message_t msg = { .service = SET_DEPTH_CM };
    msg.data.p3.cm = cm;
    send_message(&msg);
}

void set_complementary_filters(float a1, float a2, float a3, float a4) {
    rpc_message_t msg = { .service = COEFFICIENT_COMPLEMANTARY_FILTER };
    msg.data.p4.alpha_for_pitch     = a1;
    msg.data.p4.alpha_for_yaw       = a2;
    msg.data.p4.alpha_for_roll      = a3;
    msg.data.p4.alpha_for_stabilize = a4;
    send_message(&msg);
}

void pwm_motors_for_drive_one_by_one(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4,
                                     uint16_t p5, uint16_t p6, uint16_t p7, uint16_t p8) {
    rpc_message_t msg = { .service = PWM_MOTORS_FOR_STOP };
    msg.data.p5.pwm1 = p1;
    msg.data.p5.pwm2 = p2;
    msg.data.p5.pwm3 = p3;
    msg.data.p5.pwm4 = p4;
    msg.data.p5.pwm5 = p5;
    msg.data.p5.pwm6 = p6;
    msg.data.p5.pwm7 = p7;
    msg.data.p5.pwm8 = p8;
    send_message(&msg);
}

void for_arm(uint8_t arm) {
    rpc_message_t msg = { .service = FOR_ARM };
    msg.data.p6.arm_or_disarm = arm;
    send_message(&msg);
}

void led_control(float led) {
    rpc_message_t msg = { .service = LED_CONTROL };
    msg.data.p7.led_aktif = led;
    send_message(&msg);
}

int main(void) {
    printf("LED turned on\n");
    calibrate_PID(12.5f, 0.8f, 0.02f);
    printf("PID gains sent\n");
    set_degrees_of_yaw(90.0f);
    set_depth_cm(150.0f);
    set_complementary_filters(0.02f, 0.02f, 0.02f, 0.5f);
    pwm_motors_for_drive_one_by_one(1200,1200,1200,1200,1200,1200,1200,1200);
    for_arm(0);
    led_control(2.5f);
    return 0;
}
