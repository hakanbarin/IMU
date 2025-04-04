#include "imu.h"
#include "tim.h"
#include "depth.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"

float desired_depth = 0;


void depth_thread(void *input){
	// THREAD EKLENECEK

	float desired_depth_local = 0;
	uint32_t last_time = xTaskGetTickCount();
	while(1){

		xSemaphoreTake(depth_mutexHandle, portMAX_DELAY);
		desired_depth_local = desired_depth;
		xSemaphoreGive(depth_mutexHandle);

		const uint32_t now = xTaskGetTickCount();
		const float dt = (now - last_time) / 1000.0f;
		const float PWM = pid_control(desired_depth_local, depth, &depth_integral, &depth_error, dt);
		last_time = now;


		xSemaphoreTake(pwm_mutexHandle, portMAX_DELAY);
		pwm1 = 1500 + PWM; // Motor 1 - TIM1 CH1		//sol ön üst
		pwm2 = 1500 + PWM; // Motor 2 - TIM1 CH2		//sağ ön üst
		pwm3 = 1500 + PWM; // Motor 3 - TIM1 CH3		//sağ arka üst
		pwm4 = 1500 + PWM; // Motor 4 - TIM1 CH4		//sol arka üst
		xSemaphoreGive(pwm_mutexHandle);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
	}
}




