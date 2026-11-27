// Non-blocking DMA-backed driver for Sensirion SHT40 I2C temperature/humidity sensors (0x44, 0x45).

#ifndef SHT40_H
#define SHT40_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define SHT40_BASE_ADDR_7BIT   0x44U
#define SHT40_MAX_ADDR_7BIT    0x45U
#define SHT40_MAX_SENSORS      2U
#define SHT40_POLL_PERIOD_MS   100U

void SHT40_ResetAll(void);
bool SHT40_GetCached(uint8_t addr7Bit, float *p_tempC, float *p_humPct, bool *p_ok);
void SHT40_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs);
bool SHT40_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                float *p_tempC, float *p_humPct, bool *p_ok);
bool SHT40_IsDmaBusy(void);
void SHT40_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c);
void SHT40_OnI2cError(I2C_HandleTypeDef *p_hi2c);

#ifdef __cplusplus
}
#endif

#endif // SHT40_H
