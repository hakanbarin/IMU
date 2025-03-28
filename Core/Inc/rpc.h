#ifndef RPC_H
#define RPC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))

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
        rpc_service_t service;
        uint8_t data;
    } rpc_header_t;

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
           int pwm1;
           int pwm2;
           int pwm3;
           int pwm4;
           int pwm5;		// SOL ÖN 1, SAĞ ÖN 2, SAĞ ARKA 3, SOL ARKA 4, SOL ÜST 5, SAĞ ÜST 6, SAĞ ALT 7, SOL ALT 8. MOTOR
           int pwm6;		// AYNI ZAMANDA MOTORLARI AYRI AYRI SÜRMEK İÇİN
           int pwm7;		// BU KOMUTTAN SONRA TEKRARDAN MOTORLARI ÇALIŞTIRMAM GEREKİYOR
           int pwm8;

        } pwm_motors_for_stop;



//extern bool motorlar_durduruldu = false;
//
//void handle_rpc_command(rpc_header_t *command) {
//
//		if(command->service == PWM_MOTORS_FOR_STOP){
//			motorlar_durduruldu = true;
//			motorlari_durdur();  // Motorları durdurma fonksiyonunu çağır
//		}
//		else{
//			motorlar_durduruldu = false;
//		}
//
//}


#ifdef __cplusplus
}
#endif

#endif // RPC_H
