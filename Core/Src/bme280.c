// Non-blocking DMA-backed driver for Bosch BME280/BMP280 I2C sensors (0x76, 0x77).

#include "bme280.h"
#include <string.h>

#define BME280_REG_DIG_T1     0x88U
#define BME280_REG_CHIP_ID    0xD0U
#define BME280_REG_RESET      0xE0U
#define BME280_REG_DIG_H2     0xE1U
#define BME280_REG_CTRL_HUM   0xF2U
#define BME280_REG_CTRL_MEAS  0xF4U
#define BME280_REG_CONFIG     0xF5U
#define BME280_REG_DATA_START 0xF7U

#define BME280_CHIP_ID        0x60U
#define BMP280_CHIP_ID        0x58U

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} Bme280Calib_t;

typedef struct {
    bool         active;
    bool         initialized;
    bool         hasHumidity;
    bool         ok;
    float        tempC;
    float        humPct;
    float        pressHpa;
    uint32_t     lastPollMs;
    uint32_t     lastRetryMs;
    uint8_t      rxBuf[8];
    Bme280Calib_t calib;
} Bme280Slot_t;

static Bme280Slot_t    g_bme280_slots[BME280_MAX_SENSORS];
static volatile int8_t g_bme280_activeDmaIdx = -1;

static int8_t addr_to_idx(uint8_t addr7Bit)
{
    if (addr7Bit < BME280_BASE_ADDR_7BIT || addr7Bit > BME280_MAX_ADDR_7BIT) {
        return -1;
    }
    return (int8_t)(addr7Bit - BME280_BASE_ADDR_7BIT);
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8U) | p[0]);
}

static int16_t read_s16_le(const uint8_t *p)
{
    return (int16_t)read_u16_le(p);
}

static void compensate_raw(Bme280Slot_t *p_slot, const uint8_t *p_raw)
{
    int32_t adc_P = (int32_t)(((uint32_t)p_raw[0] << 12U) | ((uint32_t)p_raw[1] << 4U) | ((uint32_t)p_raw[2] >> 4U));
    int32_t adc_T = (int32_t)(((uint32_t)p_raw[3] << 12U) | ((uint32_t)p_raw[4] << 4U) | ((uint32_t)p_raw[5] >> 4U));
    int32_t adc_H = (int32_t)(((uint32_t)p_raw[6] << 8U) | (uint32_t)p_raw[7]);

    const Bme280Calib_t *c = &p_slot->calib;

    // Temperature compensation (Bosch datasheet 4.2.3)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) * ((int32_t)c->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)c->dig_T1)) * ((adc_T >> 4) - ((int32_t)c->dig_T1))) >> 12) * ((int32_t)c->dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    int32_t T_x100 = (t_fine * 5 + 128) >> 8;
    p_slot->tempC = (float)T_x100 / 100.0f;

    // Pressure compensation (64-bit integer, Q24.8 Pa)
    int64_t p_v1 = (int64_t)t_fine - 128000;
    int64_t p_v2 = p_v1 * p_v1 * (int64_t)c->dig_P6;
    p_v2 = p_v2 + ((p_v1 * (int64_t)c->dig_P5) << 17);
    p_v2 = p_v2 + (((int64_t)c->dig_P4) << 35);
    p_v1 = ((p_v1 * p_v1 * (int64_t)c->dig_P3) >> 8) + ((p_v1 * (int64_t)c->dig_P2) << 12);
    p_v1 = (((((int64_t)1) << 47) + p_v1) * ((int64_t)c->dig_P1)) >> 33;
    if (p_v1 != 0) {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - p_v2) * 3125) / p_v1;
        p_v1 = (((int64_t)c->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_v2 = (((int64_t)c->dig_P8) * p) >> 19;
        p = ((p + p_v1 + p_v2) >> 8) + (((int64_t)c->dig_P7) << 4);
        p_slot->pressHpa = (float)p / 25600.0f;
    }

    // Humidity compensation (Bosch datasheet 4.2.3)
    if (p_slot->hasHumidity) {
        int32_t v_x1 = t_fine - 76800;
        v_x1 = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20) - (((int32_t)c->dig_H5) * v_x1)) + 16384) >> 15) *
                (((((((v_x1 * ((int32_t)c->dig_H6)) >> 10) * (((v_x1 * ((int32_t)c->dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                  ((int32_t)c->dig_H2) + 8192) >> 14));
        v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)c->dig_H1)) >> 4);
        if (v_x1 < 0) { v_x1 = 0; }
        if (v_x1 > 419430400) { v_x1 = 419430400; }
        p_slot->humPct = (float)(v_x1 >> 12) / 1024.0f;
    } else {
        p_slot->humPct = 0.0f;
    }
}

void BME280_ResetAll(void)
{
    memset(g_bme280_slots, 0, sizeof(g_bme280_slots));
    g_bme280_activeDmaIdx = -1;
}

bool BME280_IsDmaBusy(void)
{
    return g_bme280_activeDmaIdx >= 0;
}

bool BME280_EnsureInit(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (p_hi2c == NULL || idx < 0) {
        return false;
    }
    Bme280Slot_t *p_slot = &g_bme280_slots[idx];
    if (p_slot->initialized) {
        return p_slot->ok;
    }

    uint16_t addr8Bit = (uint16_t)(addr7Bit << 1U);
    uint8_t chipId = 0U;
    if (HAL_I2C_Mem_Read(p_hi2c, addr8Bit, BME280_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chipId, 1U, 10U) != HAL_OK) {
        p_slot->ok = false;
        return false;
    }
    if (chipId != BME280_CHIP_ID && chipId != BMP280_CHIP_ID) {
        p_slot->ok = false;
        return false;
    }
    p_slot->hasHumidity = (chipId == BME280_CHIP_ID);

    uint8_t c1[26] = {0};
    if (HAL_I2C_Mem_Read(p_hi2c, addr8Bit, BME280_REG_DIG_T1, I2C_MEMADD_SIZE_8BIT, c1, 26U, 15U) != HAL_OK) {
        p_slot->ok = false;
        return false;
    }
    p_slot->calib.dig_T1 = read_u16_le(&c1[0]);
    p_slot->calib.dig_T2 = read_s16_le(&c1[2]);
    p_slot->calib.dig_T3 = read_s16_le(&c1[4]);
    p_slot->calib.dig_P1 = read_u16_le(&c1[6]);
    p_slot->calib.dig_P2 = read_s16_le(&c1[8]);
    p_slot->calib.dig_P3 = read_s16_le(&c1[10]);
    p_slot->calib.dig_P4 = read_s16_le(&c1[12]);
    p_slot->calib.dig_P5 = read_s16_le(&c1[14]);
    p_slot->calib.dig_P6 = read_s16_le(&c1[16]);
    p_slot->calib.dig_P7 = read_s16_le(&c1[18]);
    p_slot->calib.dig_P8 = read_s16_le(&c1[20]);
    p_slot->calib.dig_P9 = read_s16_le(&c1[22]);
    p_slot->calib.dig_H1 = c1[25];

    if (p_slot->hasHumidity) {
        uint8_t c2[7] = {0};
        if (HAL_I2C_Mem_Read(p_hi2c, addr8Bit, BME280_REG_DIG_H2, I2C_MEMADD_SIZE_8BIT, c2, 7U, 15U) != HAL_OK) {
            p_slot->ok = false;
            return false;
        }
        p_slot->calib.dig_H2 = read_s16_le(&c2[0]);
        p_slot->calib.dig_H3 = c2[2];
        p_slot->calib.dig_H4 = (int16_t)(((int16_t)(int8_t)c2[3] << 4) | (c2[4] & 0x0F));
        p_slot->calib.dig_H5 = (int16_t)(((int16_t)(int8_t)c2[5] << 4) | ((c2[4] >> 4) & 0x0F));
        p_slot->calib.dig_H6 = (int8_t)c2[6];

        uint8_t ctrlHum = 0x01U; // osrs_h = x1
        HAL_I2C_Mem_Write(p_hi2c, addr8Bit, BME280_REG_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &ctrlHum, 1U, 10U);
    }

    uint8_t cfg = 0x20U;      // t_sb = 125ms, filter off
    uint8_t ctrlMeas = 0x27U; // osrs_t=x1, osrs_p=x1, normal mode
    HAL_I2C_Mem_Write(p_hi2c, addr8Bit, BME280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &cfg, 1U, 10U);
    if (HAL_I2C_Mem_Write(p_hi2c, addr8Bit, BME280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrlMeas, 1U, 10U) != HAL_OK) {
        p_slot->ok = false;
        return false;
    }

    p_slot->initialized = true;
    p_slot->lastPollMs  = HAL_GetTick() - BME280_POLL_PERIOD_MS;
    p_slot->ok          = true;
    return true;
}

bool BME280_ReadCachedOrStartDma(I2C_HandleTypeDef *p_hi2c, uint8_t addr7Bit, uint32_t nowMs,
                                 float *p_tempC, float *p_humPct, float *p_pressHpa, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (p_hi2c == NULL || idx < 0) {
        if (p_tempC != NULL)    { *p_tempC = 0.0f; }
        if (p_humPct != NULL)   { *p_humPct = 0.0f; }
        if (p_pressHpa != NULL) { *p_pressHpa = 0.0f; }
        if (p_ok != NULL)       { *p_ok = false; }
        return false;
    }

    Bme280Slot_t *p_slot = &g_bme280_slots[idx];
    if (!p_slot->initialized) {
        if ((nowMs - p_slot->lastRetryMs) >= 1000U &&
            HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY && g_bme280_activeDmaIdx < 0) {
            p_slot->lastRetryMs = nowMs;
            BME280_EnsureInit(p_hi2c, addr7Bit);
        }
    } else if ((nowMs - p_slot->lastPollMs) >= BME280_POLL_PERIOD_MS) {
        if (g_bme280_activeDmaIdx < 0 && HAL_I2C_GetState(p_hi2c) == HAL_I2C_STATE_READY) {
            uint16_t addr8Bit = (uint16_t)(addr7Bit << 1U);
            p_slot->lastPollMs = nowMs;
            g_bme280_activeDmaIdx = idx;
            if (HAL_I2C_Mem_Read_DMA(p_hi2c, addr8Bit, BME280_REG_DATA_START, I2C_MEMADD_SIZE_8BIT,
                                     p_slot->rxBuf, 8U) != HAL_OK) {
                g_bme280_activeDmaIdx = -1;
                p_slot->ok = false;
            }
        }
    }

    if (p_tempC != NULL)    { *p_tempC = p_slot->tempC; }
    if (p_humPct != NULL)   { *p_humPct = p_slot->humPct; }
    if (p_pressHpa != NULL) { *p_pressHpa = p_slot->pressHpa; }
    if (p_ok != NULL)       { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

bool BME280_GetCached(uint8_t addr7Bit, float *p_tempC, float *p_humPct, float *p_pressHpa, bool *p_ok)
{
    int8_t idx = addr_to_idx(addr7Bit);
    if (idx < 0) {
        if (p_tempC != NULL)    { *p_tempC = 0.0f; }
        if (p_humPct != NULL)   { *p_humPct = 0.0f; }
        if (p_pressHpa != NULL) { *p_pressHpa = 0.0f; }
        if (p_ok != NULL)       { *p_ok = false; }
        return false;
    }
    Bme280Slot_t *p_slot = &g_bme280_slots[idx];
    p_slot->active = true;
    if (p_tempC != NULL)    { *p_tempC = p_slot->tempC; }
    if (p_humPct != NULL)   { *p_humPct = p_slot->humPct; }
    if (p_pressHpa != NULL) { *p_pressHpa = p_slot->pressHpa; }
    if (p_ok != NULL)       { *p_ok = p_slot->ok; }
    return p_slot->ok;
}

void BME280_PollAll(I2C_HandleTypeDef *p_hi2c, uint32_t nowMs)
{
    if (p_hi2c == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < BME280_MAX_SENSORS; i++) {
        if (g_bme280_slots[i].active) {
            BME280_ReadCachedOrStartDma(p_hi2c, (uint8_t)(BME280_BASE_ADDR_7BIT + i), nowMs, NULL, NULL, NULL, NULL);
        }
    }
}

void BME280_OnI2cRxCplt(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_bme280_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)BME280_MAX_SENSORS) {
        Bme280Slot_t *p_slot = &g_bme280_slots[idx];
        compensate_raw(p_slot, p_slot->rxBuf);
        p_slot->ok = true;
    }
    g_bme280_activeDmaIdx = -1;
}

void BME280_OnI2cError(I2C_HandleTypeDef *p_hi2c)
{
    (void)p_hi2c;
    int8_t idx = g_bme280_activeDmaIdx;
    if (idx >= 0 && idx < (int8_t)BME280_MAX_SENSORS) {
        g_bme280_slots[idx].ok = false;
    }
    g_bme280_activeDmaIdx = -1;
}
