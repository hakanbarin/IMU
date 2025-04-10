#include "rpc.h"
#include "gpio.h"
#include "usart.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "imu.h"
#include "depth.h"
#include "semphr.h"

volatile uint8_t rpc_rx_buffer[sizeof(rpc_message_t)];

typedef void (*rpc_handler_t)(void*);

static void calibrate_PID(void *raw_input);
static void set_degrees_of_yaw(void *raw_input);
static void set_depth_cm(void *raw_input);
static void pwm_motors_for_drive_one_by_one(void *raw_input);
static void coefficient_complemantary_filters(void *raw_input);
static void for_arm(void *raw_input);

static const rpc_handler_t RPC_SERVICE_MAP[NUM_OF_RPC_SERVICES] = {calibrate_PID,
													 set_degrees_of_yaw,
                                                     set_depth_cm,
													 coefficient_complemantary_filters,
													 pwm_motors_for_drive_one_by_one,
													 for_arm};

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

static void calibrate_PID(void *raw_input){

	calibrate_pid_t *input = raw_input;
	KP = input->kp;
	KI = input->ki;
	KD = input->kd;
}


static void set_degrees_of_yaw(void *raw_input) // abi burada yaw açısını da updatemotorda değiştiriyorum ondan dolayı fonksiyonu çağırmam gerekiyor
{                                               // onun yerine ayrı bir fonksiyon olarak mı çağırayım

    set_degrees_of_yaw_t *input = raw_input;

    desired_yaw = input->degrees;

}

static void set_depth_cm(void *raw_input)
{
    set_depth_cm_t *input = raw_input;
    xSemaphoreTake(depth_mutexHandle, portMAX_DELAY);
    desired_depth = input->cm;
    xSemaphoreGive(depth_mutexHandle);
}

static void coefficient_complemantary_filters(void *raw_input){

	coefficient_complemantary_t *input = raw_input;

	ALPHA  = input->alpha_for_pitch;
	ALPHA1 = input->alpha_for_yaw;
	ALPHA2 = input->alpha_for_roll;
	alpha_for_stabilize = input->alpha_for_stabilize;
}


static void pwm_motors_for_drive_one_by_one(void *raw_input)
{
    pwm_motors_for_stop_t *input = raw_input;
    // SOL ÖN 1, SAĞ ÖN 2, SAĞ ARKA 3, SOL ARKA 4, SOL ÜST 5, SAĞ ÜST 6, SAĞ ALT 7, SOL ALT 8. MOTOR
    PWM_FOR_SINGLE_MOTOR_CONTROL(input->pwm1, input->pwm2,input->pwm3,input->pwm4
                                 ,input->pwm5,input->pwm6,input->pwm7,input->pwm8);


//   1               2
//       5        6

//     MOTOR DİZİLİMİ

//       8        7
//   4               3
}

static void for_arm(void *raw_input){
	for_arm_t *input = raw_input;

	is_armed = input->arm_or_disarm;
}




