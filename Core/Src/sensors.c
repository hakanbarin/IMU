#include "i2c.h"
#include "sensors.h"

#define ITG3205_ADDR (0x68 << 1) // AD0 LOW ise 0x68, HIGH ise 0x69
#define ADXL345_ADDR (0x53 << 1) // SDO LOW ise 0x53
#define HMC5883L_ADDR (0x1E << 1)

// Manyetometre ofset ve ölçek faktörleri
float offset_x = 0, offset_y = 0, offset_z = 0;
float offset_x_mg = 0, offset_y_mg = 0, offset_z_mg = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;

void ITG3205_Init(void)
{
    uint8_t data[2];
    // Power Management: PLL, X ekseni referans
    data[0] = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x3E, 1, data, 1, 100);
    // DLPF ve Full Scale Ayarları (2000dps, 256Hz)
    data[0] = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x16, 1, data, 1, 100);
}

void ADXL345_Init(void)
{
    uint8_t data[2];
    // Measurement Mode
    data[0] = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, 1, data, 1, 100);
    // +/-4g, Full Resolution
    data[0] = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31, 1, data, 1, 100);
}
bir şey oldu abi usbler çalışmıyor şu an :D
abi bi bilgisayarı restleyip geliyorum sesini de duyamıyorum şu an
void HMC5883L_Init(void)
{
    uint8_t data[2];
    // Ölçüm Sıklığı ve Mod (75Hz, Sürekli)
    data[0] = 0xF0; // 8 örnek, 75Hz
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, data, 1, 100);
    data[0] = 0x00; // Sürekli ölçüm
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, data, 1, 100);
}

void HMC5883L_Read(int16_t* mx, int16_t* my, int16_t* mz)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, 100);
    *mx = (buffer[0] << 8) | buffer[1]; // X ekseni
    *my = (buffer[4] << 8) | buffer[5]; // Y ekseni (HMC5883L veri sayfasına göre sıra X, Z, Y)
    *mz = (buffer[2] << 8) | buffer[3]; // Z ekseni
}

void ADXL345_Read(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, 1, buffer, 6, 100);
    *ax = (buffer[1] << 8) | buffer[0];
    *ay = (buffer[3] << 8) | buffer[2];
    *az = (buffer[5] << 8) | buffer[4];
}

void ITG3205_Read(int16_t* temp, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buffer[8];
    HAL_I2C_Mem_Read(&hi2c1, ITG3205_ADDR, 0x1B, 1, buffer, 8, 100);
    *temp = (buffer[0] << 8) | buffer[1];
    *gx = (buffer[2] << 8) | buffer[3];
    *gy = (buffer[4] << 8) | buffer[5];
    *gz = (buffer[6] << 8) | buffer[7];
}

void HMC5883L_Read_Calibrated(float* mx, float* my, float* mz)
{
    int16_t mxx, myy, mzz;
    HMC5883L_Read(&mxx, &myy, &mzz);

    *mx = (mxx - offset_x) / scale_x;
    *my = (myy - offset_y) / scale_y;
    *mz = (mzz - offset_z) / scale_z;
}
