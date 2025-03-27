#include "rpc.h"
#include "cmsis_os.h"
#include "gpio.h"

void rpc_thread(void* argument)
{
    while (1)
    {
    	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	osDelay(100);
    }
}
