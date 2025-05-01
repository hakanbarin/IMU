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
		FOR_ARM,
		LED_CONTROL,
        NUM_OF_RPC_SERVICES
    } rpc_service_t;

//#pragma pack(push, 0)

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
    	float alpha_for_pitch;
    	float alpha_for_yaw;
    	float alpha_for_roll;
    	float alpha_for_stabilize;
	} coefficient_complemantary_t;

    typedef struct
    {
        uint16_t pwm1;
        uint16_t pwm2;
        uint16_t pwm3;  // bunları uint16_t olarak tanımlasam olur mu
        uint16_t pwm4;
        uint16_t pwm5; 	// SOL ÖN 1, SAĞ ÖN 2, SAĞ ARKA 3, SOL ARKA 4, SOL ÜST 5, SAĞ ÜST 6, SAĞ ALT 7, SOL ALT 8. MOTOR
        uint16_t pwm6; 	// AYNI ZAMANDA MOTORLARI AYRI AYRI SÜRMEK İÇİN
        uint16_t pwm7; 	// BU KOMUTTAN SONRA TEKRARDAN MOTORLARI ÇALIŞTIRMAM GEREKİYOR
        uint16_t pwm8;
    } pwm_motors_for_stop_t;

    typedef struct
    {
    	uint8_t arm_or_disarm;
    } for_arm_t;

    typedef struct
    {
    	uint8_t led_aktif;
    }led_kontrol;


    typedef struct
    {
        rpc_service_t service;
        union
		{
        	calibrate_pid_t p1;
			set_degrees_of_yaw_t p2;
        	set_depth_cm_t p3;
			set_depth_cm_t p4;
			coefficient_complemantary_t p5;
			pwm_motors_for_stop_t p6;
			led_kontrol p7;
		} data;

    } rpc_message_t;

//#pragma pack(pop)



#ifdef __cplusplus
}
#endif

#endif // RPC_H
