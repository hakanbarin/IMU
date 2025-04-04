#ifndef RPC_H
#define RPC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

    typedef enum
    {
        CALIBRATE_PID,
        SET_DEGREES_OF_YAW,
        SET_DEPTH_CM,
        COEFFICIENT_COMPLEMANTARY_FILTER,
        PWM_MOTORS_FOR_STOP,
        NUM_OF_RPC_SERVICES
    } rpc_service_t;

    typedef struct
    {
        float kp;
        float ki;
        float kd;
    } calibrate_pid_t;

    typedef struct
    {
        float degrees;
    } set_degrees_of_yaw_t;

    typedef struct
    {
        float cm;
    } set_depth_cm_t;

    typedef struct
    {
        uint16_t pwm1;
        uint16_t pwm2;
        uint16_t pwm3;  // bunları uint16_t olarak tanımlasam olur mu
        uint16_t pwm4;
        uint16_t pwm5; // SOL ÖN 1, SAĞ ÖN 2, SAĞ ARKA 3, SOL ARKA 4, SOL ÜST 5, SAĞ ÜST 6, SAĞ ALT 7, SOL ALT 8. MOTOR
        uint16_t pwm6; // AYNI ZAMANDA MOTORLARI AYRI AYRI SÜRMEK İÇİN
        uint16_t pwm7; // BU KOMUTTAN SONRA TEKRARDAN MOTORLARI ÇALIŞTIRMAM GEREKİYOR
        uint16_t pwm8;
    } pwm_motors_for_stop_t;

    typedef struct
    {
        rpc_service_t service;
        union
		{
			set_degrees_of_yaw_t p1;
			set_depth_cm_t p2;
			pwm_motors_for_stop_t p3;
		} data;

    } rpc_message_t;

    extern volatile uint8_t rpc_rx_buffer[sizeof(rpc_message_t)];

#ifdef __cplusplus
}
#endif

#endif // RPC_H
