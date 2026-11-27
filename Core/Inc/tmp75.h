// Non-blocking DMA-backed driver for TMP75 I2C temperature sensors (0x48..0x4F).

#ifndef TMP75_H
#define TMP75_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define TMP75_BASE_ADDR_7BIT   0x48U
#define TMP75_MAX_ADDR_7BIT    0x4FU
#define TMP75_MAX_SENSORS      8U
#define TMP75_POLL_PERIOD_MS   50U

// Reset all TMP75 lazy-init states (called on PLC_HAL_Init / VM program reload).
void TMP75_ResetAll(void);

// Lazily initialize a TMP75 sensor at 7-bit address (0x48..0x4F) in 12-bit mode.
bool TMP75_EnsureInit(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit);

// Return cached temperature (deg C) immediately and register the sensor address for background polling.
bool TMP75_GetCached(uint8_t addr7Bit, float *p_tempC, bool *p_ok);

// Background I2C1 DMA poller (called from commIoTask).
void TMP75_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs);

// Return cached temperature (deg C) and trigger a background DMA read if due and I2C1 is idle.
bool TMP75_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                float *p_tempC, bool *p_ok);

// Check whether a TMP75 DMA transfer is currently active on the I2C bus.
bool TMP75_IsDmaBusy(void);

// HAL I2C DMA completion & error hooks (called from HAL_I2C_MemRxCpltCallback / ErrorCallback).
void TMP75_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c);
void TMP75_OnI2cError(I2C_HandleTypeDef *p_hi2c);

#ifdef __cplusplus
}
#endif

#endif // TMP75_H
