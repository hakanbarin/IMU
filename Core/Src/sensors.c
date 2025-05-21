#include "i2c.h"
#include "sensors.h"

#define ITG3205_ADDR (0x68 << 1) // AD0 LOW ise 0x68, HIGH ise 0x69
#define ADXL345_ADDR (0x53 << 1) // SDO LOW ise 0x53
#define HMC5883L_ADDR (0x1E << 1)
#define BAR30_I2C_ADDR (0x76 << 1)

// MS5837 register/command definitions
#define MS5837_CMD_RESET        0x1E
#define MS5837_CMD_ADC_READ     0x00 // Her okumada bu adresi kullan
#define MS5837_CMD_CONVERT_D1_8192 0x4A
#define MS5837_CMD_CONVERT_D2_8192 0x5A
#define MS5837_CMD_PROM_READ    0xA0 // PROM okuma başlangıcı

#define HARD_IRON_X   (294.3650406f)
#define HARD_IRON_Y   (-1240.36481506f)
#define HARD_IRON_Z   (-566.6978993f)

const float soft_iron[3][3] = {
    {0.018215949f,  -0.0000906315f,  0.0001179308f},
    {-0.0000906315f,  0.018266505f,  0.0001862537f},
    {0.0001179308f,   0.0001862537f, 0.018167255f}
};



float offset_x = 0, offset_y = 0, offset_z = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;

void ITG3205_Init(void)
{
    uint8_t data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x3E, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x16, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void ADXL345_Init(void)
{
    uint8_t data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    data = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void apply_mag_calibration(int16_t raw_x, int16_t raw_y, int16_t raw_z,
		int16_t *cal_x, int16_t *cal_y, int16_t *cal_z)
{
    float x = raw_x - HARD_IRON_X;
    float y = raw_y - HARD_IRON_Y;
    float z = raw_z - HARD_IRON_Z;

    *cal_x = soft_iron[0][0]*x + soft_iron[0][1]*y + soft_iron[0][2]*z;
    *cal_y = soft_iron[1][0]*x + soft_iron[1][1]*y + soft_iron[1][2]*z;
    *cal_z = soft_iron[2][0]*x + soft_iron[2][1]*y + soft_iron[2][2]*z;
}


void HMC5883L_Init(void) {
    uint8_t data;


    data = 0xF0;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x00, 1, &data, 1, 100);

    data = 0x20;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x00, 1, &data, 1, 100);


    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x02, 1, &data, 1, 100);

    osDelay(10);
}


void HMC5883L_Read(int16_t  *mx, int16_t  *my, int16_t  *mz) {
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, 0x3D, 0x03, 1, buffer, 6, 100); // 6 byte okuyor kaydıra kaydıra


    *mx = (buffer[0] << 8) | buffer[1];  // X ekseni
    *my = (buffer[4] << 8) | buffer[5];  // Y ekseni
    *mz = (buffer[2] << 8) | buffer[3];  // Z ekseni
    *mx = *mx * 0.092f;
    *my = *my * 0.092f;
    *mz = *mz * 0.092f;

//    printf("%d,%d,%d\r\n", *mx, *my, *mz);
    osDelay(15); // BU KALKABİLİR


}


void ADXL345_Read(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);
    *ax = (buffer[1] << 8) | buffer[0];
    *ay = (buffer[3] << 8) | buffer[2];
    *az = (buffer[5] << 8) | buffer[4];
}

void ITG3205_Read(int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[8];
    HAL_I2C_Mem_Read(&hi2c1, ITG3205_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, buffer, 8, 100);
    *temp = (buffer[0] << 8) | buffer[1];
    *gx   = (buffer[2] << 8) | buffer[3];
    *gy   = (buffer[4] << 8) | buffer[5];
    *gz   = (buffer[6] << 8) | buffer[7];
}

void HMC5883L_Read_Calibrated(float *mx, float *my, float *mz)
{
    int16_t mxx, myy, mzz;
    HMC5883L_Read(&mxx, &myy, &mzz);

    *mx = (mxx - offset_x) / scale_x;
    *my = (myy - offset_y) / scale_y;
    *mz = (mzz - offset_z) / scale_z;
}
