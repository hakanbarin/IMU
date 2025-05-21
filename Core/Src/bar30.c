#include "bar30.h"
#include <string.h>
#include <stdio.h>

static uint16_t C[8];

static uint32_t D1, D2;


static int32_t TEMP;
static int32_t P;

static float surfacePressurePa = 0.0f;
float pressure_mbar,temp_c;

static uint8_t crc4(uint16_t *prom);
static void    calculate(void);



HAL_StatusTypeDef BAR30_Init()
{
    HAL_StatusTypeDef s;
    uint8_t cmd, buf[2];


    cmd = MS5837_CMD_RESET;
    s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    HAL_Delay(3);


    for (uint8_t i = 0; i < 8; i++) {
        cmd = MS5837_CMD_PROM_READ + (i * 2);
        s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
        if (s != HAL_OK) return s;
        s = HAL_I2C_Master_Receive(&hi2c1, BAR30_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
        if (s != HAL_OK) return s;
        C[i] = (buf[0] << 8) | buf[1];
    }


    uint8_t crc_read = C[0] >> 12;
    if (crc4(C) != crc_read) return HAL_ERROR;

    return HAL_OK;
}



HAL_StatusTypeDef BAR30_Read(float *depth_m)
{
    HAL_StatusTypeDef s;
    uint8_t cmd, buf3[3];


    cmd = MS5837_CMD_CONVERT_D1_8192;
    s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    HAL_Delay(20);

    /* Read D1 */
    cmd = MS5837_CMD_ADC_READ;
    s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    s = HAL_I2C_Master_Receive(&hi2c1, BAR30_I2C_ADDR, buf3, 3, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    D1 = ((uint32_t)buf3[0] << 16) | ((uint32_t)buf3[1] << 8) | buf3[2];


    cmd = MS5837_CMD_CONVERT_D2_8192;
    s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    HAL_Delay(20);


    cmd = MS5837_CMD_ADC_READ;
    s = HAL_I2C_Master_Transmit(&hi2c1, BAR30_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    s = HAL_I2C_Master_Receive(&hi2c1, BAR30_I2C_ADDR, buf3, 3, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    D2 = ((uint32_t)buf3[0] << 16) | ((uint32_t)buf3[1] << 8) | buf3[2];


    calculate();


    if (pressure_mbar) pressure_mbar = (float)P;
    if (temp_c)        temp_c        = (float)TEMP / 100.0f;
    if (depth_m) {
        float p_pa = (float)P * 100.0f;
        float rho  = BAR30_DENSITY_FRESHWATER;
        float g    = 9.80665f;
        *depth_m = (p_pa - surfacePressurePa) / (rho * g);
        *depth_m = *depth_m - 11.2055155f;
        printf("%.6f\r\n", *depth_m);

        	if(*depth_m < 0){
        		*depth_m = 0;
        	}
    }

    return HAL_OK;
}

static uint8_t crc4(uint16_t *prom)
{
    uint16_t n_rem = 0;

    prom[0] &= 0x0FFF;
    prom[7] = 0;

    for (uint8_t i = 0; i < 16; i++) {
        n_rem ^= (i & 1) ? (prom[i >> 1] & 0x00FF) : (prom[i >> 1] >> 8);
        for (uint8_t bit = 8; bit > 0; bit--) {
            n_rem = (n_rem & 0x8000) ? (n_rem << 1) ^ 0x3000 : (n_rem << 1);
        }
    }
    return (n_rem >> 12) & 0x0F;
}

static void calculate(void)
{
    int32_t dT = (int32_t)D2 - ((int32_t)C[5] << 8);
    TEMP = 2000 + (int64_t)dT * C[6] / 8388608LL;

    int64_t OFF  = (int64_t)C[2] << 16 | ((int64_t)C[4] * dT >> 7);
    int64_t SENS = (int64_t)C[1] << 15 | ((int64_t)C[3] * dT >> 8);

    int32_t Ti = 0;
    int64_t OFFi = 0, SENSi = 0;
    if (TEMP < 2000) {
        int32_t Tlow = TEMP - 2000;
        Ti    = (int64_t)3 * dT * dT >> 33;
        OFFi  = (int64_t)3 * Tlow * Tlow >> 1;
        SENSi = (int64_t)5 * Tlow * Tlow >> 3;
        if (TEMP < -1500) {
            int32_t Tvl = TEMP + 1500;
            OFFi  += (int64_t)7 * Tvl * Tvl;
            SENSi += (int64_t)4 * Tvl * Tvl;
        }
    } else {
        int32_t Thigh = TEMP - 2000;
        Ti    = (int64_t)2 * dT * dT >> 37;
        OFFi  = (int64_t)Thigh * Thigh >> 4;
        SENSi = 0;
    }

    TEMP -= Ti;
    OFF  -= OFFi;
    SENS -= SENSi;

    int64_t P_calc = (((int64_t)D1 * SENS >> 21) - OFF) >> 13;
    P = (int32_t)(P_calc / 10);
}
