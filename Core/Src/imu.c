#include "tim.h"
#include "math.h"
#include "cmsis_os.h"
#include "sensors.h"
#include "task.h"
#include "semphr.h"
#include "imu.h"
#include "stdio.h"


//KP KI KD değerleri atomic yapılması gerekebilir veya mutex kullanılabilir.
//FreeRTOS'ta atomic nasıl yapılıyor bak.

#define CALIBRATION_SAMPLES (200)
#define CALIBRATION_SAMPLES_MAG (1000)

float ALPHA = 0.1f;
float ALPHA1 = 0.92f;
float ALPHA2 = 0.2f;
float KP = 4.f;
float KI = 0.02f;
float KD = 0.05f;
float alpha_for_stabilize = 0.6f;
uint8_t is_armed = 0;
float led_aktif_main = 2.5;


float desired_depth = 0, depth = 0;

float pitch_error_prev = 0, roll_error_prev = 0, yaw_error_prev = 0, depth_error_prev = 0;
float pitch_integral = 0, roll_integral = 0, yaw_integral = 0, depth_integral = 0;
float pitch_adjust, roll_adjust, yaw_adjust, depth_adjust, desired_yaw = 0;
uint16_t pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;

//static void esc_calibration_bidirectional(void);

static void start_engine(void);

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
		osDelay(500);

    }
}

void imu_thread(void *argument)
{

    float angle_pitch = 0, angle_roll = 0, angle_yaw = 0, yaw = 0;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int16_t temp;


    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    ITG3205_Init();
    ADXL345_Init();
    HMC5883L_Init();
    BAR30_init();
    calibrate_gyro(&gyro_x_offset, &gyro_y_offset, &gyro_z_offset);

    uint32_t last_time = osKernelGetTickCount();
//    start_engine();
//    esc_calibration_bidirectional();

    while (1)
    {
    	if(is_armed == 2){
        ITG3205_Read(&temp, &gx, &gy, &gz);
        ADXL345_Read(&ax, &ay, &az);
        BAR30_Read(&depth);

        uint32_t current_time = osKernelGetTickCount();
        const float delta_time = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        const float gyro_x = -((gx / 14.375f) - gyro_x_offset);
        const float gyro_y = (gy / 14.375f) - gyro_y_offset;
        const float gyro_z = -((gz / 14.375f) - gyro_z_offset);

        const float acc_angle_pitch = -(atan2f(ay, az) * 180.0f / M_PI);
        const float acc_angle_roll = -(atan2f(ax, az) * 180.0f / M_PI);
        yaw += gyro_z * delta_time;

        angle_pitch = (1 - ALPHA) * (angle_pitch + gyro_x * delta_time) + ALPHA * acc_angle_pitch;
        angle_roll = (1 - ALPHA2) * (angle_roll + gyro_y * delta_time) + ALPHA2 * acc_angle_roll;
        angle_yaw = (1 - ALPHA1) * (yaw + gyro_z * delta_time) + ALPHA1 * yaw;

        update_motors(0, 0, angle_pitch, angle_roll, angle_yaw, depth, delta_time);

        osDelay(10);
		}

        else if (is_armed == 1){//arm kesilsin
			PWM_FOR_SINGLE_MOTOR_CONTROL(1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600);
		}
        else{
        	led_yaniyor(led_aktif_main);
        }
}

}

//static void esc_calibration_bidirectional(void)
//{
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//    osDelay(6000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000); // Motor 4
//    osDelay(5000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000); // Motor 4
//    osDelay(2000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500); // Motor 4
//    osDelay(2000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000); // Motor 4
//    osDelay(2000);
//}

static void start_engine(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
    osDelay(3000);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000);
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
    depth_adjust = pid_control(desired_depth, current_depth, &depth_integral, &depth_error_prev, dt);

    //	8 MOTORLU ROV İÇİN
         pwm1 = 1500 + (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust - roll_adjust)); // Motor 1 - TIM1 CH1sol ön üst
         pwm2 = 1500 + (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 2 - TIM1 CH2		//sağ ön üst
         pwm3 = 1500 - (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 3 - TIM1 CH3		//sağ arka üst
         pwm4 = 1500 - (alpha_for_stabilize * depth_adjust ) + ((1- alpha_for_stabilize)*(pitch_adjust - roll_adjust)); // Motor 4 - TIM1 CH4		//sol arka üst
    // TIMERLARI BELLİ DEĞİL
    //     pwm5 = 1500 + yaw_adjust;					// Motor 5 - TIM1 CH1		//sol ön
    //     pwm6 = 1500 - yaw_adjust; 				// Motor 6 - TIM1 CH2		//sağ ön
    //     pwm7 = 1500 - yaw_adjust; 				// Motor 7 - TIM1 CH3		//sağ arka
    //     pwm8 = 1500 + yaw_adjust; 				// Motor 8 - TIM1 CH4		//sol arka
//	xSemaphoreTake(pwm_mutexHandle, portMAX_DELAY);
//    pwm4 = 1500 + (alpha_for_stabilize * 300 ) + ((1- alpha_for_stabilize)*(pitch_adjust + roll_adjust)); // Motor 4 - TIM1 CH4
//    pwm4 = fmax(1000, fmin(2000, pwm4));
//	xSemaphoreGive(pwm_mutexHandle);

        pwm1 = fmax(1100, fmin(1900, pwm1));
        pwm2 = fmax(1100, fmin(1900, pwm2));
        pwm3 = fmax(1100, fmin(1900, pwm3));
        pwm4 = fmax(1100, fmin(1900, pwm4));
    //    pwm5 = fmax(1100, fmin(1900, pwm5));
    //    pwm6 = fmax(1100, fmin(1900, pwm6));
    //    pwm7 = fmax(1100, fmin(1900, pwm7));
    //    pwm8 = fmax(1100, fmin(1900, pwm8));


        printf("%.2d, %.2d, %.2d, %.2d\n", pwm1, pwm2, pwm3, pwm4);
        printf("%.2f, %.2f\n", pitch_adjust, roll_adjust);

    //    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
    //    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
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

// THREAD EKLENECEK
//void change_PWM_for_depth(float desired_depth, float dt){
//
////    while(1){
//
//
//	pwm1 = 1500 + PWM; // Motor 1 - TIM1 CH1		//sol ön üst
//	pwm2 = 1500 + PWM; // Motor 2 - TIM1 CH2		//sağ ön üst
//	pwm3 = 1500 + PWM; // Motor 3 - TIM1 CH3		//sağ arka üst
//	pwm4 = 1500 + PWM; // Motor 4 - TIM1 CH4		//sol arka üst
//
////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm2);
////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm3);
////	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
////    }
//}

void PWM_FOR_SINGLE_MOTOR_CONTROL(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4,  //FOR STOP OR DRIVE ENGINES ONE BY ONE
								  uint16_t pwm5, uint16_t pwm6, uint16_t pwm7,uint16_t pwm8)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm5);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm7);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm8);
}
