// Non-blocking DMA-backed driver for Bosch BME280/BMP280 I2C sensors (0x76, 0x77).

#ifndef BME280_H
#define BME280_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define BME280_BASE_ADDR_7BIT   0x76U
#define BME280_MAX_ADDR_7BIT    0x77U
#define BME280_MAX_SENSORS      2U
#define BME280_POLL_PERIOD_MS   100U

void BME280_ResetAll(void);
bool BME280_EnsureInit(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit);
bool BME280_GetCached(uint8_t addr7Bit, float *p_tempC, float *p_humPct, float *p_pressHpa, bool *p_ok);
void BME280_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs);
bool BME280_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                 float *p_tempC, float *p_humPct, float *p_pressHpa, bool *p_ok);
bool BME280_IsDmaBusy(void);
void BME280_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c);
void BME280_OnI2cError(I2C_HandleTypeDef *p_hi2c);

#ifdef __cplusplus
}
#endif

#endif // BME280_H
