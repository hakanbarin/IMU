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

static const rpc_handler_t RPC_SERVICE_MAP[NUM_OF_RPC_SERVICES] = {calibrate_PID,
													 set_degrees_of_yaw,
                                                     set_depth_cm,
													 pwm_motors_for_drive_one_by_one};

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

//	calibrate_pid_t *input = raw_input;

}


static void set_degrees_of_yaw(void *raw_input) // abi burada yaw açısını da updatemotorda değiştiriyorum ondan dolayı fonksiyonu çağırmam gerekiyor
{                                               // onun yerine ayrı bir fonksiyon olarak mı çağırayım
	static uint32_t last_time = 0;
    set_degrees_of_yaw_t *input = raw_input;
    // // todo
     last_time = xTaskGetTickCount();
     const uint32_t now = xTaskGetTickCount();

     const float dt = (now - last_time) / 1000.0f; //saniye
     last_time = now;

     yaw_update_motor(input->degrees, dt);
}

static void set_depth_cm(void *raw_input)
{
    set_depth_cm_t *input = raw_input;
    xSemaphoreTake(depth_mutexHandle, portMAX_DELAY);
    desired_depth = input->cm;
    xSemaphoreGive(depth_mutexHandle);
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
