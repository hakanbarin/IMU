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

static void set_degrees_of_yaw(void *raw_input)
{
    set_degrees_of_yaw_t *input = raw_input;
    // todo
}

static void set_depth_cm(void *raw_input)
{
    set_depth_cm_t *input = raw_input;
    // todo
}

static void pwm_motors_for_stop(void *raw_input)
{
    pwm_motors_for_stop_t *input = raw_input;
    // todo
}
