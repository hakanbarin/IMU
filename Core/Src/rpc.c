#include <stdio.h>
#include <string.h>
#include <cmsis_os.h>
#include <task.h>
#include <semphr.h>
#include "rpc.h"
#include "gpio.h"
#include "usart.h"
#include "queue.h"
#include "imu.h"
#include "depth.h"

extern osThreadId_t imu_taskHandle;

volatile uint8_t rpc_rx_buffer[sizeof(rpc_message_t)];

extern uint16_t pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;

// FUNCTION POINTER TANIMLAMASI
typedef void (*rpc_handler_t)(void*);

static void calibrate_PID(void *raw_input);
static void set_degrees_of_yaw(void *raw_input);
static void set_depth_cm(void *raw_input);
static void pwm_motors_for_drive_one_by_one(void *raw_input);
static void coefficient_complemantary_filters(void *raw_input);
static void for_arm(void *raw_input);
static void led_control(void *raw_input);
static void motor_arm(void *raw_input);


//FUNCTION POINTER KULLANIYOR
static const rpc_handler_t RPC_SERVICE_MAP[NUM_OF_RPC_SERVICES] = {calibrate_PID,
													 set_degrees_of_yaw,
                                                     set_depth_cm,
													 pwm_motors_for_drive_one_by_one,
													 coefficient_complemantary_filters,
													 for_arm,
													 led_control,
													 motor_arm};

//extern DMA_HandleTypeDef hdma_usart2_rx;

void rpc_thread(void *argument)
{
    (void)argument;
    rpc_message_t msg;
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)rpc_rx_buffer, sizeof(rpc_rx_buffer));


    while (1)
    {
        xQueueReceive(rpc_queueHandle, &msg, portMAX_DELAY);
        RPC_SERVICE_MAP[msg.service](&msg.data);

    }
}

//void rpc_thread(void *argument)
//{
//    (void)argument;
//    rpc_message_t msg;
//
//    printf("rpc_thread başladı. rpc_message_t boyutu: %d\n", sizeof(rpc_message_t));
//    HAL_UART_Receive_DMA(&huart2,(uint8_t *)rpc_rx_buffer, sizeof(rpc_rx_buffer));
//
//
//    while (1)
//    {
//        // Kuyruktan veri gelene kadar bekle
//        xQueueReceive(rpc_queueHandle, &msg, portMAX_DELAY);
//        RPC_SERVICE_MAP[msg.service](&msg.data);
////        uint8_t *bytes = (uint8_t *)&msg;
////        led_kontrol* input = &msg.data;
////        printf("kp = %f, ki = %f, kd = %f \r\n", input->kp, input->ki, input->kd);
////        printf("kp = %d\r\n", input->led_aktif);
//    }
//}

//void rpc_thread(void *argument) {
//	osDelay(20000);
//    rpc_message_t msg;
//
//    printf("rpc_thread başladı (rpc_message_t size = %u)\r\n",
//           (unsigned)sizeof(rpc_message_t));
//
//    // İlk DMA alımını başlat
//    HAL_UART_Receive_DMA(&huart2,
//                         (uint8_t*)rpc_rx_buffer,
//                         sizeof(rpc_message_t));
//
//    for (;;) {
//        // Mesaj gelene kadar tamamen blokla (CPU başka thread’e gider)
//        if (osMessageQueueGet(rpc_queueHandle,
//                              &msg,
//                              NULL,
//                              osWaitForever) == osOK) {
//
//		RPC_SERVICE_MAP[msg.service](&msg.data);
//
//        }
//    }
//}




static void calibrate_PID(void *raw_input){

    osThreadSuspend(imu_taskHandle);

    calibrate_pid_t *input = raw_input;


    KP = input->kp;
    KI = input->ki;
    KD = input->kd;

    osThreadResume(imu_taskHandle);
}


static void set_degrees_of_yaw(void *raw_input) // abi burada yaw açısını da updatemotorda değiştiriyorum ondan dolayı fonksiyonu çağırmam gerekiyor
{                                               // onun yerine ayrı bir fonksiyon olarak mı çağırayım
    osThreadSuspend(imu_taskHandle);

    set_degrees_of_yaw_t *input = raw_input;


    desired_yaw = input->degrees;

    osThreadResume(imu_taskHandle);
}

static void set_depth_cm(void *raw_input)
{

    osThreadSuspend(imu_taskHandle);

    set_depth_cm_t *input = raw_input;
//    xSemaphoreTake(depth_mutexHandle, portMAX_DELAY);

    desired_depth = input->cm;
//    xSemaphoreGive(depth_mutexHandle);

    osThreadResume(imu_taskHandle);
}

static void coefficient_complemantary_filters(void *raw_input){

    osThreadSuspend(imu_taskHandle);

	coefficient_complemantary_t *input = raw_input;


	ALPHA  = input->alpha_for_pitch;
	ALPHA1 = input->alpha_for_yaw;
	ALPHA2 = input->alpha_for_roll;
	alpha_for_stabilize = input->alpha_for_stabilize;

    osThreadResume(imu_taskHandle);
}


static void pwm_motors_for_drive_one_by_one(void *raw_input)
{

    pwm_motors_for_stop_t *input = raw_input;
    // SOL ÖN 1, SAĞ ÖN 2, SAĞ ARKA 3, SOL ARKA 4, SOL ÜST 5, SAĞ ÜST 6, SAĞ ALT 7, SOL ALT 8. MOTOR
    pwm1 = input->pwm1;
    pwm2 = input->pwm2;
    pwm3 = input->pwm3;
    pwm4 = input->pwm4;
    pwm5 = input->pwm5;
    pwm6 = input->pwm6;
    pwm7 = input->pwm7;
    pwm8 = input->pwm8;

//   1               2
//       5        6

//     MOTOR DİZİLİMİ

//       8        7
//   4               3
}

static void for_arm(void *raw_input){

    osThreadSuspend(imu_taskHandle);

	for_arm_t *input = raw_input;
	is_armed = input->arm_or_disarm;

    osThreadResume(imu_taskHandle);
}

static void led_control(void *raw_input){

    osThreadSuspend(imu_taskHandle);

    led_kontrol *input = raw_input;

    led_aktif_main = input->led_aktif;

//    printf("LED aktif değeri alındı: %f\n", led_aktif_main);
    osThreadResume(imu_taskHandle);
}

static void motor_arm(void *raw_input){
	try_to_engine();
}




