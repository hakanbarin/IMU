#ifndef BAR30_H
#define BAR30_H

#include "stm32f0xx_hal.h"  // MCU serinize göre değiştirin
#include <stdint.h>
#include "i2c.h"

#define BAR30_I2C_ADDR             (0x76 << 1)
#define MS5837_CMD_RESET           0x1E
#define MS5837_CMD_PROM_READ       0xA0
#define MS5837_CMD_CONVERT_D1_8192 0x48
#define MS5837_CMD_CONVERT_D2_8192 0x58
#define MS5837_CMD_ADC_READ        0x00

#define BAR30_DENSITY_FRESHWATER   997.0f

#ifdef __cplusplus
extern "C" {
#endif




HAL_StatusTypeDef BAR30_Init();
HAL_StatusTypeDef BAR30_Read(float *);

#ifdef __cplusplus
}
#endif

#endif /* BAR30_H */
