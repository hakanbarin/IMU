// Core/Src/ms5837.c

#include "ms5837.h"
#include <math.h>   // for powf

// MS5837 command definitions
#define MS5837_CMD_RESET        0x1E
#define MS5837_CMD_READ_PROM    0xA0
#define MS5837_CMD_CONVERT_D1   0x4A
#define MS5837_CMD_CONVERT_D2   0x5A
#define MS5837_CMD_READ_ADC     0x00



// Helper: send a single‐byte command over I²C
static HAL_StatusTypeDef _cmd(MS5837_HandleTypeDef *dev, uint8_t c) {
    return HAL_I2C_Master_Transmit(dev->hi2c,
                                   (uint16_t)(dev->address << 1),
                                   &c, 1,
                                   HAL_MAX_DELAY);
}

// Helper: read 24‐bit ADC result
static uint32_t _readADC(MS5837_HandleTypeDef *dev) {
    uint8_t buf[3];
    if (HAL_I2C_Master_Receive(dev->hi2c,
                               (uint16_t)(dev->address << 1),
                               buf, 3,
                               HAL_MAX_DELAY) == HAL_OK) {
        return ((uint32_t)buf[0] << 16)
             | ((uint32_t)buf[1] << 8)
             |  (uint32_t)buf[2];
    }
    dev->error = -2;
    return 0;
}

// Helper: initialize the C[] constants from PROM and mathMode adjustments
static void _initConstants(MS5837_HandleTypeDef *dev, uint8_t mathMode) {
    // baseline constants (from datasheet / C++ library)
    dev->C[0] = 1.0f;
    dev->C[1] = 32768.0f;
    dev->C[2] = 65536.0f;
    dev->C[3] = 3.90625e-3f;
    dev->C[4] = 7.8125e-3f;
    dev->C[5] = 256.0f;
    dev->C[6] = 1.1920928955e-7f;
    dev->C[7] = 1.220703125e-4f;

    // mathMode adjustments
    if (mathMode == 1) {
        dev->C[1] *= 2.0f;
        dev->C[2] *= 2.0f;
        dev->C[3] *= 2.0f;
        dev->C[4] *= 2.0f;
        dev->C[7] /= 4.0f;
    } else if (mathMode == 2) {
        dev->C[7] /= 4.0f;
    }

    // read PROM words 0..6 and multiply
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t cmd = MS5837_CMD_READ_PROM + (i * 2);
        HAL_I2C_Master_Transmit(dev->hi2c,
                                (uint16_t)(dev->address << 1),
                                &cmd, 1,
                                HAL_MAX_DELAY);
        uint8_t buf[2];
        if (HAL_I2C_Master_Receive(dev->hi2c,
                                   (uint16_t)(dev->address << 1),
                                   buf, 2,
                                   HAL_MAX_DELAY) == HAL_OK) {
            uint16_t val = ((uint16_t)buf[0] << 8) | buf[1];
            dev->C[i] *= (float)val;
        }
    }
}

bool MS5837_Init(MS5837_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t mathMode) {
    dev->hi2c    = hi2c;
    dev->address = MS5837_DEFAULT_ADDRESS;
    dev->density = 0.99802f;   // default: freshwater at 20 °C
    dev->error   = 0;

    // send RESET
    if (_cmd(dev, MS5837_CMD_RESET) != HAL_OK) {
        dev->error = -1;
        return false;
    }
    HAL_Delay(10);

    // read PROM & init constants
    _initConstants(dev, mathMode);

    // set sensor type
    if (mathMode == 1)        dev->type = MS5837_TYPE_02;
    else if (mathMode == 2)   dev->type = MS5803_TYPE_01;
    else                       dev->type = MS5837_TYPE_30;

    return true;
}

bool MS5837_Read(MS5837_HandleTypeDef *dev, uint8_t bits) {
    if (bits < 8)  bits = 8;
    if (bits > 13) bits = 13;
    uint8_t osr = bits - 8;
    // approximate conversion time in ms for OSR = 256,512,1024,2048,4096,8192
    const uint8_t wait[6] = {1,2,3,5,9,18};

    // D1 (pressure)
    if (_cmd(dev, MS5837_CMD_CONVERT_D1 + osr * 2) != HAL_OK) {
        dev->error = -3;
        return false;
    }
    HAL_Delay(wait[osr]);
    uint32_t D1 = _readADC(dev);

    // D2 (temperature)
    if (_cmd(dev, MS5837_CMD_CONVERT_D2 + osr * 2) != HAL_OK) {
        dev->error = -4;
        return false;
    }
    HAL_Delay(wait[osr]);
    uint32_t D2 = _readADC(dev);

    // calculate temperature
    float dT    = (float)D2 - dev->C[5];
    float temp  = 2000.0f + dT * dev->C[6];

    // calculate offset & sensitivity
    float offset = dev->C[2] + dT * dev->C[4];
    float sens   = dev->C[1] + dT * dev->C[3];

    // second‐order compensation if below 20 °C
    if (temp < 2000.0f) {
        float t2      = (temp - 2000.0f) * (temp - 2000.0f);
        offset       -= t2 * (31.0f * 0.125f);
        sens         -= t2 * (63.0f * 0.03125f);
        temp         -= (dT * dT * 2.91038304567e-11f);
    }

    // final pressure & temperature
    dev->pressure    = (((float)D1 * sens * 4.76837158203e-7f) - offset)
                       * dev->C[7] * 0.01f;
    dev->temperature = temp * 0.01f;
    dev->lastRead    = HAL_GetTick();

    return true;
}

float MS5837_GetPressure(MS5837_HandleTypeDef *dev) {
    return dev->pressure;
}

float MS5837_GetTemperature(MS5837_HandleTypeDef *dev) {
    return dev->temperature;
}

float MS5837_GetAltitude(MS5837_HandleTypeDef *dev, float airPressure) {
    float ratio = dev->pressure / airPressure;
    return 44330.0f * (1.0f - powf(ratio, 0.190294957f));
}

void MS5837_SetDensity(MS5837_HandleTypeDef *dev, float density) {
    dev->density = density;
}

float MS5837_GetDepth(MS5837_HandleTypeDef *dev, float airPressure) {
    // h = (P - P0) / (ρ·g·10), with ρ in g/cm³ and P in mbar
    return (dev->pressure - airPressure) / (dev->density * 9.80665f * 10.0f);
}

int MS5837_GetLastError(MS5837_HandleTypeDef *dev) {
    int e = dev->error;
    dev->error = 0;
    return e;
}
