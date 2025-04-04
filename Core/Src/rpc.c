#include "rpc.h"
#include "gpio.h"
#include "usart.h"
#include "cmsis_os.h"

volatile uint8_t rpc_rx_buffer[sizeof(rpc_message_t)];

static void set_degrees_of_yaw();
static void set_depth_cm();
static void pwm_motors_for_stop();

static const RPC_SERVICE_MAP[NUM_OF_RPC_SERVICES] = {set_degrees_of_yaw,
                                                     set_depth_cm,
                                                     pwm_motors_for_stop};

void rpc_thread(void *argument)
{
    (void)argument;

    HAL_UART_Receive_DMA(&huart2, rpc_rx_buffer, sizeof(rpc_rx_buffer));

    while (1)
    {
        xSemaphoreTake(rpc_rx_semaphore, portMAX_DELAY);
        HAL_UART_Receive_DMA(&huart2, rpc_rx_buffer, sizeof(rpc_rx_buffer));

        rpc_message_t *msg = rpc_rx_buffer;

        RPC_SERVICE_MAP[msg->service](msg->data);
    }
}

static void set_degrees_of_yaw(void *raw_input) // abi burada yaw açısını da updatemotorda değiştiriyorum ondan dolayı fonksiyonu çağırmam gerekiyor
{                                               // onun yerine ayrı bir fonksiyon olarak mı çağırayım
    set_degrees_of_yaw_t *input = raw_input;
    // // todo
    // static uint32_t last_time = HAL_GetTick();
    // uint32_t now = HAL_GetTick();

    // float dt = (now - last_time) / 1000.0f; //saniye
    // last_time = now;
}

static void set_depth_cm(void *raw_input)
{
    set_depth_cm_t *input = raw_input;
    // todo
    static uint32_t last_time = HAL_GetTick();
    uint32_t now = HAL_GetTick();

    float dt = (now - last_time) / 1000.0f; // saniye
    last_time = now;

    // PID kontrolünü çağırarak PWM güncelle
    change_PWM_for_depth(input->cm, dt);
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
