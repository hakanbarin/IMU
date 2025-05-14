// Core/Inc/ms5837.h
#ifndef MS5837_H
#define MS5837_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"   // F0 serisi için doğru HAL başlığı

// sensör tipleri
#define MS5803_TYPE_01   1
#define MS5837_TYPE_02   2
#define MS5837_TYPE_30  30

// default I²C adresi
#define MS5837_DEFAULT_ADDRESS  0x76



typedef struct {
    I2C_HandleTypeDef *hi2c;   // HAL I2C handle
    uint8_t  address;
    uint8_t  type;
    float    C[8];
    float    pressure;
    float    temperature;
    float    density;
    int      error;
    uint32_t lastRead;
} MS5837_HandleTypeDef;

// prototipler
bool   MS5837_Init      (MS5837_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t mathMode);
bool   MS5837_Read      (MS5837_HandleTypeDef *dev, uint8_t bits);
float  MS5837_GetPressure   (MS5837_HandleTypeDef *dev);
float  MS5837_GetTemperature(MS5837_HandleTypeDef *dev);
float  MS5837_GetAltitude   (MS5837_HandleTypeDef *dev, float airPressure);
void   MS5837_SetDensity    (MS5837_HandleTypeDef *dev, float density);
float  MS5837_GetDepth      (MS5837_HandleTypeDef *dev, float airPressure);
int    MS5837_GetLastError  (MS5837_HandleTypeDef *dev);

#endif // MS5837_H
