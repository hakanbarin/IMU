#include <stdio.h>
#include <tim.h>
#include <math.h>
#include <semphr.h>
#include <cmsis_os.h>
#include "sensors.h"
#include "task.h"
#include "imu.h"
#include "ms5837.h"

MS5837_HandleTypeDef depthSensor;
float                surfacePressure;
extern I2C_HandleTypeDef hi2c1;				//eğer main.c de kütüphaneleri başlatacaksan bu satırı yoruma al yukarı ikiliyi extern yap maindekini yorumları da aç

#define CALIBRATION_SAMPLES (200)
//#define CALIBRATION_SAMPLES_MAG (1000)

#define FOR_STABILIZE_MODE 2
#define FOR_ONE_BY_ONE_MOTOR_DRIVE 1
#define FOR_STOP 0
#define FOR_ARM 3
#define ARM_SUCCESFUL 1
#define ARM_UNSUCCESFUL 0

float ALPHA = 0.1f;
float ALPHA1 = 0.92f;
float ALPHA2 = 0.2f;
float KP = 4.f;
float KI = 0.02f;
float KD = 0.05f;
float alpha_for_stabilize = 0.6f;
uint8_t is_armed = 0;
float led_aktif_main = 2.5;

float angle_pitch = 0, angle_roll = 0;

float desired_depth = 0, depth = 0;
uint16_t pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;
float pitch_error_prev = 0, roll_error_prev = 0, yaw_error_prev = 0, depth_error_prev = 0;
float pitch_integral = 0, roll_integral = 0, yaw_integral = 0, depth_integral = 0;
float pitch_adjust, roll_adjust, yaw_adjust, depth_adjust, desired_yaw = 0;

//static void esc_calibration_bidirectional(void);



static void update_motors(float pitch_setpoint,
                          float roll_setpoint,
                          float current_pitch,
                          float current_roll,
						  float current_yaw,
						  float current_depth,
                          float dt);

static void calibrate_gyro(float *offsetX, float *offsetY, float *offsetZ);

void change_PWM_for_depth(float depth, float dt);



void led_yaniyor(float led_deneme)
{


    if (fabs(led_deneme - 2.5) < 0.01)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // led surekli yanik
    }
    else
    {

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(2000);

    }
}

uint8_t protection_function(){


    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int16_t temp;

    ITG3205_Read(&temp, &gx, &gy, &gz);
    ADXL345_Read(&ax, &ay, &az);
    uint32_t last_time = osKernelGetTickCount();

    uint32_t current_time = osKernelGetTickCount();
    const float delta_time = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    const float gyro_x = -((gx / 14.375f));
    const float gyro_y = (gy / 14.375f);

    const float acc_angle_pitch = -(atan2f(ay, az) * 180.0f / M_PI);
    const float acc_angle_roll = -(atan2f(ax, az) * 180.0f / M_PI);


    angle_pitch = (1 - ALPHA) * (angle_pitch + gyro_x * delta_time) + ALPHA * acc_angle_pitch;
    angle_roll = (1 - ALPHA2) * (angle_roll + gyro_y * delta_time) + ALPHA2 * acc_angle_roll;


    if((fabs(angle_roll) < 8) && (fabs(angle_pitch) < 8)){
    	return ARM_SUCCESFUL;
    }
    else{
    	return ARM_UNSUCCESFUL;
    }
}


void imu_thread(void *argument)
{

    float yaw = 0, angle_yaw = 0;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    int16_t temp;
    int16_t cal_mx, cal_my, cal_mz;


    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    // Sonra sensor init’ler
    ITG3205_Init();
    printf("– ITG init bitti\r\n");

    ADXL345_Init();
    printf("– ADXL init bitti\r\n");

    HMC5883L_Init();
    printf("– HMC init bitti\r\n");

    MS5837_Init(&depthSensor, &hi2c1, 0);
    MS5837_SetDensity(&depthSensor, 0.99802f);

    calibrate_gyro(&gyro_x_offset, &gyro_y_offset, &gyro_z_offset);
    printf("– kalibrasyon bitti, try_to_engine giriyor\r\n");



//    while(protection_function() != ARM_SUCCESFUL);

    uint32_t last_time = osKernelGetTickCount();
    try_to_engine();

    printf(" try_to_engine tamamlandı\r\n");

//    	int pulsee = 1050;
//    	while(pulsee <= 1850){
//
//    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulsee);
//    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulsee);
//    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulsee);
//    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulsee);
//    	printf("%d\n",pulsee);
//    	pulsee++;
//    	osDelay(100);
//
//    	}

    while (1)
    {

    	if(is_armed == FOR_STABILIZE_MODE){

			ITG3205_Read(&temp, &gx, &gy, &gz);
			ADXL345_Read(&ax, &ay, &az);
//			BAR30_Read(&depth);

			HMC5883L_Read(&mx, &my, &mz);
			apply_mag_calibration(mx, my, mz, &cal_mx, &cal_my, &cal_mz);

			MS5837_Read(&depthSensor, 13);

			depth = MS5837_GetDepth(&depthSensor, surfacePressure);

		    printf(" okudum hepsini\r\n");
			uint32_t current_time = osKernelGetTickCount();
			const float delta_time = (current_time - last_time) / 1000.0f;
			last_time = current_time;

			const float gyro_x = -((gx / 14.375f) - gyro_x_offset);
			const float gyro_y = (gy / 14.375f) - gyro_y_offset;
			const float gyro_z = -((gz / 14.375f) - gyro_z_offset);

			const float acc_angle_pitch = -(atan2f(ay, az) * 180.0f / M_PI);
			const float acc_angle_roll = -(atan2f(ax, az) * 180.0f / M_PI);
			yaw += gyro_z * delta_time;

		 // Tilt-compensated yaw hesapla
			const float pitchRad = acc_angle_pitch * M_PI / 180.0f;
			const float rollRad  = acc_angle_roll  * M_PI / 180.0f;

			const float mx_comp = cal_mx * cos(pitchRad) + cal_mz * sin(pitchRad);
			const float my_comp = cal_mx * sin(rollRad) * sin(pitchRad) +
							cal_my * cos(rollRad) -
							cal_mz * sin(rollRad) * cos(pitchRad);

			float magYaw = atan2f(my_comp, mx_comp) * 180.0f / M_PI;

			if (magYaw < 0) magYaw += 360.0f;


			angle_pitch = (1 - ALPHA) * (angle_pitch + gyro_x * delta_time) + ALPHA * acc_angle_pitch;
			angle_roll = (1 - ALPHA2) * (angle_roll + gyro_y * delta_time) + ALPHA2 * acc_angle_roll;
			angle_yaw   = (1 - ALPHA1) * yaw + ALPHA1 * magYaw;


			printf(" motor update olcakr\n");
			update_motors(0, 0, angle_pitch, angle_roll, angle_yaw, depth, delta_time);
//			osDelay(50);

		}

        else if (is_armed == FOR_ONE_BY_ONE_MOTOR_DRIVE){//arm kesilsin
			PWM_FOR_SINGLE_MOTOR_CONTROL(pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8);
		}
        else if(is_armed == FOR_STOP){
        	led_yaniyor(led_aktif_main);
        	PWM_FOR_SINGLE_MOTOR_CONTROL(0, 0, 0, 0, 0, 0, 0, 0);
        }
        else{
        	try_to_engine();
        }

}

}


void try_to_engine(void)
{

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    osDelay(500);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    osDelay(500);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    osDelay(500);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    osDelay(500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
	osDelay(3000);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2100);
	osDelay(3000);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
	osDelay(3000);


}

float pid_control(float setpoint,
                         float measured,
                         float *integral,
                         float *prev_error,
                         float dt)
{
    if (dt < 0.001f)
        dt = 0.001f;

    float error = setpoint - measured;

    *integral += error * dt;

    float MAX_INTEGRAL = 50.0f;
    if (*integral > MAX_INTEGRAL)
        *integral = MAX_INTEGRAL;
    if (*integral < -MAX_INTEGRAL)
        *integral = -MAX_INTEGRAL;

    return (KP * error) + (KI * (*integral));
}



static void update_motors(float pitch_setpoint,
                          float roll_setpoint,
                          float current_pitch,
                          float current_roll,
						  float current_yaw,
						  float current_depth,
                          float dt)
{

	pitch_adjust = pid_control(0, current_pitch, &pitch_integral, &pitch_error_prev, dt);
	roll_adjust = pid_control(0, current_roll, &roll_integral, &roll_error_prev, dt);
	yaw_adjust = pid_control(desired_yaw, current_yaw, &yaw_integral, &yaw_error_prev, dt);
//	depth_adjust = pid_control(desired_depth, current_depth, &depth_integral, &depth_error_prev, dt);

//	pwm1 = 1500 + (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust - roll_adjust)); // Motor 1 - TIM1 CH1sol ön üst
//	pwm2 = 1500 + (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 2 - TIM1 CH2		//sağ ön üst
//	pwm3 = 1500 - (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 3 - TIM1 CH3		//sağ arka üst
//	pwm4 = 1500 - (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust - roll_adjust)); // Motor 4 - TIM1 CH4		//sol arka üst
////
//								8 MOTORLU ROV İÇİN
//	pwm1 = 1500 + (depth_adjust) + ((pitch_adjust - roll_adjust) / (alpha_for_stabilize)); 		// Motor 1 - TIM1 CH1		//sol ön üst
//	pwm2 = 1500 + (depth_adjust) + ((pitch_adjust + roll_adjust) / (alpha_for_stabilize)); 		// Motor 2 - TIM1 CH2		//sağ ön üst
//	pwm3 = 1500 - (depth_adjust) + ((pitch_adjust + roll_adjust) / (alpha_for_stabilize)); 		// Motor 3 - TIM1 CH3		//sağ arka üst
//	pwm4 = 1500 - (depth_adjust) + ((pitch_adjust - roll_adjust) / (alpha_for_stabilize)); 		// Motor 4 - TIM1 CH4		//sol arka üst
//	pwm5 = 1500 + yaw_adjust;																		// Motor 5 - TIM1 CH1		//sol ön
//	pwm6 = 1500 - yaw_adjust; 																		// Motor 6 - TIM1 CH2		//sağ ön
//	pwm7 = 1500 - yaw_adjust; 																		// Motor 7 - TIM1 CH3		//sağ arka
//	pwm8 = 1500 + yaw_adjust; 																		// Motor 8 - TIM1 CH4		//sol arka
//
//	// TIMERLARI BELLİ DEĞİL
//
//	pwm1 = fmax(1100, fmin(1900, pwm1));
//	pwm2 = fmax(1100, fmin(1900, pwm2));
//	pwm3 = fmax(1100, fmin(1900, pwm3));
//	pwm4 = fmax(1100, fmin(1900, pwm4));
//	pwm5 = fmax(1100, fmin(1900, pwm5));
//	pwm6 = fmax(1100, fmin(1900, pwm6));
//	pwm7 = fmax(1100, fmin(1900, pwm7));
//	pwm8 = fmax(1100, fmin(1900, pwm8));
//
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
//
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm2);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4);



//								4 MOTORLU ROV İÇİN
	pwm1 = 1500 + (depth_adjust) + (roll_adjust) / (alpha_for_stabilize); 		// Motor 1 - TIM1 CH1		//sol üst
	pwm2 = 1500 + (depth_adjust) + (roll_adjust) / (alpha_for_stabilize); 		// Motor 2 - TIM1 CH2		//sağ üst
	pwm3 = 1500 + yaw_adjust;													// Motor 3 - TIM1 CH3		//sol forward
	pwm4 = 1500 - yaw_adjust; 													// Motor 4 - TIM1 CH4		//sağ forward


	// TIMERLARI BELLİ DEĞİL

	pwm1 = fmax(1100, fmin(1200, pwm1));
	pwm2 = fmax(1100, fmin(1200, pwm2));
	pwm3 = fmax(1100, fmin(1200, pwm3));
	pwm4 = fmax(1100, fmin(1200, pwm4));

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);

	printf(" motor update oldur\n");
         // xSemaphoreTake(pwm_mutexHandle, portMAX_DELAY);		 //MUTEX KALDIRILABİLİR BENCE
		 //    pwm4 = 1500 + (alpha_for_stabilize * 300 ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 4 - TIM1 CH4
		 //    pwm4 = fmax(1000, fmin(2000, pwm4));
		 //	xSemaphoreGive(pwm_mutexHandle);


}

static void calibrate_gyro(float *offsetX, float *offsetY, float *offsetZ)
{
    int16_t gx, gy, gz;
    int16_t temp;
    float sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        ITG3205_Read(&temp, &gx, &gy, &gz); // Jiroskop verilerini oku
        sumX += gx / 14.375f;               // X ekseni ölçeklendirme
        sumY += gy / 14.375f;               // Y ekseni ölçeklendirme
        sumZ += gz / 14.375f;               // Z ekseni ölçeklendirme
        osDelay(10);                      // 10ms bekle
    }

    // Ortalama ofset değerlerini hesapla
    *offsetX = sumX / CALIBRATION_SAMPLES;
    *offsetY = sumY / CALIBRATION_SAMPLES;
    *offsetZ = sumZ / CALIBRATION_SAMPLES;
}

void PWM_FOR_SINGLE_MOTOR_CONTROL(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4,  //FOR STOP OR DRIVE ENGINES ONE BY ONE
								  uint16_t pwm5, uint16_t pwm6, uint16_t pwm7,uint16_t pwm8)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
//    osDelay(1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
//    osDelay(1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
//    osDelay(1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
//    osDelay(1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm5);
//    osDelay(1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6);
//    osDelay(1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm7);
//    osDelay(1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm8);
//    osDelay(1);
}
