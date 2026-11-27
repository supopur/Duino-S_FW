// Non-blocking DMA-backed driver for Sensirion SHT40 I2C temperature/humidity sensors (0x44, 0x45).

#include "sht40.h"
#include <string.h>

#define SHT40_CMD_MEAS_HIGH    0xFDU
#define SHT40_MEAS_DELAY_MS    12U

typedef enum {
    SHT40_STATE_IDLE = 0,
    SHT40_STATE_WAIT_MEAS = 1
} Sht40State_t;

typedef struct {
    bool         active;
    Sht40State_t state;
    bool         ok;
    float        tempC;
    float        humPct;
    uint32_t     lastActionMs;
    uint8_t      txCmd;
    uint8_t      rxBuf[6];
} Sht40Slot_t;

static Sht40Slot_t     g_sht40_slots[SHT40_MAX_SENSORS];
static volatile int8_t g_sht40_activeDmaIdx = -1;

static int8_t addr_to_idx(uint8_t addr7Bit)
{
    if (addr7Bit < SHT40_BASE_ADDR_7BIT || addr7Bit > SHT40_MAX_ADDR_7BIT) {
        return -1;
    }
    return (int8_t)(addr7Bit - SHT40_BASE_ADDR_7BIT);
}

static uint8_t sht40_crc8(const uint8_t *p_data, uint8_t len)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= p_data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x31U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

void SHT40_ResetAll(void)
{
    memset(g_sht40_slots, 0, sizeof(g_sht40_slots));
    g_sht40_activeDmaIdx = -1;
}

bool SHT40_IsDmaBusy(void)
{
    return g_sht40_activeDmaIdx >= 0;
}

bool SHT40_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                float *p_tempC, float *p_humPct, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (p_hi2c == NULL || idx < 0) {
        if (p_tempC != NULL)  { *p_tempC = 0.0f; }
        if (p_humPct != NULL) { *p_humPct = 0.0f; }
        if (p_ok != NULL)     { *p_ok = false; }
        return false;
    }

    Sht40Slot_t *p_slot = &g_sht40_slots[idx];
    uint16_t addr8Bit = (uint16_t)(addr7Bit << 1U);

    if (p_slot->state == SHT40_STATE_IDLE) {
        if ((nowMs - p_slot->lastActionMs) >= SHT40_POLL_PERIOD_MS &&
            g_sht40_activeDmaIdx < 0 && HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY) {
            p_slot->txCmd = SHT40_CMD_MEAS_HIGH;
            p_slot->lastActionMs = nowMs;
            if (HAL_I2C_Master_Transmit_IT(p_hi2c, addr8Bit, &p_slot->txCmd, 1U) == HAL_OK) {
                p_slot->state = SHT40_STATE_WAIT_MEAS;
            } else {
                p_slot->ok = false;
            }
        }
    } else if (p_slot->state == SHT40_STATE_WAIT_MEAS) {
        if ((nowMs - p_slot->lastActionMs) >= SHT40_MEAS_DELAY_MS &&
            g_sht40_activeDmaIdx < 0 && HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY) {
            p_slot->state = SHT40_STATE_IDLE;
            p_slot->lastActionMs = nowMs;
            g_sht40_activeDmaIdx = idx;
            if (HAL_I2C_Master_Receive_DMA(p_hi2c, addr8Bit, p_slot->rxBuf, 6U) != HAL_OK) {
                g_sht40_activeDmaIdx = -1;
                p_slot->ok = false;
            }
        }
    }

    if (p_tempC != NULL)  { *p_tempC = p_slot->tempC; }
    if (p_humPct != NULL) { *p_humPct = p_slot->humPct; }
    if (p_ok != NULL)     { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

bool SHT40_GetCached(uint8_t addr7Bit, float *p_tempC, float *p_humPct, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (idx < 0) {
        if (p_tempC != NULL)  { *p_tempC = 0.0f; }
        if (p_humPct != NULL) { *p_humPct = 0.0f; }
        if (p_ok != NULL)     { *p_ok = false; }
        return false;
    }
    Sht40Slot_t *p_slot = &g_sht40_slots[idx];
    p_slot->active = true;
    if (p_tempC != NULL)  { *p_tempC = p_slot->tempC; }
    if (p_humPct != NULL) { *p_humPct = p_slot->humPct; }
    if (p_ok != NULL)     { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

void SHT40_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs)
{
    if (p_hi2c == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < SHT40_MAX_SENSORS; i++) {
        if (g_sht40_slots[i].active) {
            SHT40_ReadCachedOrStartDma(p_hi2c, (uint8_t)(SHT40_BASE_ADDR_7BIT + i), nowMs, NULL, NULL, NULL);
        }
    }
}

void SHT40_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_sht40_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)SHT40_MAX_SENSORS) {
        Sht40Slot_t *p_slot = &g_sht40_slots[idx];
        if (sht40_crc8(&p_slot->rxBuf[0], 2U) == p_slot->rxBuf[2] &&
            sht40_crc8(&p_slot->rxBuf[3], 2U) == p_slot->rxBuf[5]) {
            uint16_t rawT  = (uint16_t)(((uint16_t)p_slot->rxBuf[0] << 8U) | p_slot->rxBuf[1]);
            uint16_t rawRH = (uint16_t)(((uint16_t)p_slot->rxBuf[3] << 8U) | p_slot->rxBuf[4]);
            p_slot->tempC  = -45.0f + 175.0f * ((float)rawT / 65535.0f);
            float rh       = -6.0f + 125.0f * ((float)rawRH / 65535.0f);
            if (rh < 0.0f)   { rh = 0.0f; }
            if (rh > 100.0f) { rh = 100.0f; }
            p_slot->humPct = rh;
            p_slot->ok     = true;
        } else {
            p_slot->ok = false;
        }
    }
    g_sht40_activeDmaIdx = -1;
}

void SHT40_OnI2cError(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_sht40_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)SHT40_MAX_SENSORS) {
        g_sht40_slots[idx].ok = false;
        g_sht40_slots[idx].state = SHT40_STATE_IDLE;
    }
    g_sht40_activeDmaIdx = -1;
}
