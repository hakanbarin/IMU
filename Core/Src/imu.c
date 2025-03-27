#include "tim.h"
#include <math.h>
#include "cmsis_os.h"
#include "sensors.h"

#define CALIBRATION_SAMPLES (200)
#define CALIBRATION_SAMPLES_MAG (1000)

float ALPHA = 0.1f;
float ALPHA1 = 0.92f;
float ALPHA2 = 0.2f;
float KP = 4.f;
float KI = 0.02f;
float KD = 0.05f;

// PID Değişkenleri
float integralX = 0, integralY = 0, pwmPITCH, pwmROLL;
float prevErrorX = 0, prevErrorY = 0;

float pitch_error_prev = 0, roll_error_prev = 0;
float pitch_integral = 0, roll_integral = 0;


static void esc_calibration_bidirectional(void);

static void start_engine(void);

static float pid_control(float setpoint,
                         float measured,
                         float* integral,
                         float* prev_error,
                         float dt);

static void update_motors(float pitch_setpoint,
                          float roll_setpoint,
                          float current_pitch,
                          float current_roll,
                          float dt);

static void calibrate_gyro(float* offsetX, float* offsetY, float* offsetZ);


void imu_thread(void* argument)
{
    float angle_pitch = 0, angle_roll = 0, angle_yaw = 0, yaw = 0;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int16_t temp;

    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    ITG3205_Init();
    ADXL345_Init();
    HMC5883L_Init();

    calibrate_gyro(&gyro_x_offset, &gyro_y_offset, &gyro_z_offset);

    uint32_t last_time = HAL_GetTick();

    esc_calibration_bidirectional();

    while (1)
    {
        ITG3205_Read(&temp, &gx, &gy, &gz);
        ADXL345_Read(&ax, &ay, &az);

        uint32_t current_time = HAL_GetTick();
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

        update_motors(0, 0, angle_pitch, angle_roll, delta_time);

        osDelay(50);
    }
}

static void esc_calibration_bidirectional(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_Delay(6000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000); // Motor 4
    HAL_Delay(5000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000); // Motor 4
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500); // Motor 4
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000); // Motor 4
    HAL_Delay(2000);
}

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
    HAL_Delay(3000);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000);
    HAL_Delay(3000);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
    HAL_Delay(3000);
}

static float pid_control(float setpoint,
                         float measured,
                         float* integral,
                         float* prev_error,
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
                          float dt)
{
    float pitch_adjust = pid_control(0, current_pitch, &pitch_integral, &pitch_error_prev, dt);
    float roll_adjust = pid_control(0, current_roll, &roll_integral, &roll_error_prev, dt);

    int pwm4 = 1500 + roll_adjust;

    pwm4 = fmax(1000, fmin(2000, pwm4));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4);
}

static void calibrate_gyro(float* offsetX, float* offsetY, float* offsetZ)
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
        HAL_Delay(10);                      // 10ms bekle
    }

    // Ortalama ofset değerlerini hesapla
    *offsetX = sumX / CALIBRATION_SAMPLES;
    *offsetY = sumY / CALIBRATION_SAMPLES;
    *offsetZ = sumZ / CALIBRATION_SAMPLES;
}