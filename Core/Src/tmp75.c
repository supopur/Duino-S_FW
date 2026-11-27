// Non-blocking DMA-backed driver for TMP75 I2C temperature sensors (0x48..0x4F).

#include "tmp75.h"
#include <string.h>

#define TMP75_REG_TEMP         0x00U
#define TMP75_REG_CONFIG       0x01U
#define TMP75_CFG_12BIT_CONT   0x00U

typedef struct {
    bool     active;
    bool     initialized;
    bool     ok;
    float    tempC;
    uint32_t lastPollMs;
    uint32_t lastRetryMs;
    uint8_t  rxBuf[2];
} Tmp75Slot_t;

static Tmp75Slot_t   g_tmp75_slots[TMP75_MAX_SENSORS];
static volatile int8_t g_tmp75_activeDmaIdx = -1;

static int8_t addr_to_idx(uint8_t addr7Bit)
{
    if (addr7Bit < TMP75_BASE_ADDR_7BIT || addr7Bit > TMP75_MAX_ADDR_7BIT) {
        return -1;
    }
    return (int8_t)(addr7Bit - TMP75_BASE_ADDR_7BIT);
}

void TMP75_ResetAll(void)
{
    memset(g_tmp75_slots, 0, sizeof(g_tmp75_slots));
    g_tmp75_activeDmaIdx = -1;
}

bool TMP75_IsDmaBusy(void)
{
    return g_tmp75_activeDmaIdx >= 0;
}

bool TMP75_EnsureInit(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (p_hi2c == NULL || idx < 0) {
        return false;
    }
    Tmp75Slot_t *p_slot = &g_tmp75_slots[idx];
    if (p_slot->initialized) {
        return p_slot->ok;
    }

    uint16_t addr8Bit = (uint16_t)(addr7Bit << 1U);
    uint8_t cfg = TMP75_CFG_12BIT_CONT;
    if (HAL_I2C_Mem_Write(p_hi2c, addr8Bit, TMP75_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &cfg, 1U, 10U) == HAL_OK) {
        uint8_t initBuf[2] = {0U, 0U};
        if (HAL_I2C_Mem_Read(p_hi2c, addr8Bit, TMP75_REG_TEMP, I2C_MEMADD_SIZE_8BIT, initBuf, 2U, 10U) == HAL_OK) {
            int16_t raw12 = (int16_t)((uint16_t)initBuf[0] << 8U | initBuf[1]) >> 4;
            p_slot->tempC = (float)raw12 * 0.0625f;
            p_slot->ok = true;
        }
        p_slot->initialized = true;
        p_slot->lastPollMs  = HAL_GetTick();
    } else {
        p_slot->ok = false;
    }
    return p_slot->ok;
}

bool TMP75_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                float *p_tempC, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (p_hi2c == NULL || idx < 0) {
        if (p_tempC != NULL) { *p_tempC = 0.0f; }
        if (p_ok != NULL)    { *p_ok = false; }
        return false;
    }

    Tmp75Slot_t *p_slot = &g_tmp75_slots[idx];
    if (!p_slot->initialized) {
        if ((nowMs - p_slot->lastRetryMs) >= 1000U &&
            HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY && g_tmp75_activeDmaIdx < 0) {
            p_slot->lastRetryMs = nowMs;
            TMP75_EnsureInit(p_hi2c, addr7Bit);
        }
    } else if ((nowMs - p_slot->lastPollMs) >= TMP75_POLL_PERIOD_MS) {
        if (g_tmp75_activeDmaIdx < 0 && HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY) {
            uint16_t addr8Bit = (uint16_t)(addr7Bit << 1U);
            p_slot->lastPollMs = nowMs;
            if (HAL_I2C_Mem_Read(p_hi2c, addr8Bit, TMP75_REG_TEMP, I2C_MEMADD_SIZE_8BIT,
                                 p_slot->rxBuf, 2U, 10U) == HAL_OK) {
                int16_t raw12 = (int16_t)(((uint16_t)p_slot->rxBuf[0] << 8U) | p_slot->rxBuf[1]) >> 4;
                p_slot->tempC = (float)raw12 * 0.0625f;
                p_slot->ok = true;
            } else {
                p_slot->ok = false;
            }
        }
    }

    if (p_tempC != NULL) { *p_tempC = p_slot->tempC; }
    if (p_ok != NULL)    { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

bool TMP75_GetCached(uint8_t addr7Bit, float *p_tempC, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (idx < 0) {
        if (p_tempC != NULL) { *p_tempC = 0.0f; }
        if (p_ok != NULL)    { *p_ok = false; }
        return false;
    }
    Tmp75Slot_t *p_slot = &g_tmp75_slots[idx];
    p_slot->active = true;
    if (p_tempC != NULL) { *p_tempC = p_slot->tempC; }
    if (p_ok != NULL)    { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

void TMP75_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs)
{
    if (p_hi2c == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < TMP75_MAX_SENSORS; i++) {
        if (g_tmp75_slots[i].active) {
            TMP75_ReadCachedOrStartDma(p_hi2c, (uint8_t)(TMP75_BASE_ADDR_7BIT + i), nowMs, NULL, NULL);
        }
    }
}

void TMP75_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_tmp75_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)TMP75_MAX_SENSORS) {
        Tmp75Slot_t *p_slot = &g_tmp75_slots[idx];
        int16_t raw12 = (int16_t)(((uint16_t)p_slot->rxBuf[0] << 8U) | p_slot->rxBuf[1]) >> 4;
        p_slot->tempC = (float)raw12 * 0.0625f;
        p_slot->ok = true;
    }
    g_tmp75_activeDmaIdx = -1;
}

void TMP75_OnI2cError(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_tmp75_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)TMP75_MAX_SENSORS) {
        g_tmp75_slots[idx].ok = false;
    }
    g_tmp75_activeDmaIdx = -1;
}
