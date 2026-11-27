// Non-blocking TIM7-ISR 1-Wire driver for addressable DS18B20 temperature sensors.

#include "ds18b20.h"
#include "ch455g.h"
#include "hw_tim7.h"
#include <string.h>

#define OW_CMD_SEARCH_ROM       0xF0U
#define OW_CMD_MATCH_ROM        0x55U
#define OW_CMD_SKIP_ROM         0xCCU
#define OW_CMD_CONVERT_T        0x44U
#define OW_CMD_READ_SCRATCHPAD  0xBEU

typedef enum {
    DS_BUS_IDLE = 0,
    DS_BUS_TX_CONVERT_WAIT,
    DS_BUS_WAIT_CONVERSION,
    DS_BUS_READ_SENSOR_WAIT
} DsBusState_t;

typedef enum {
    OW_OP_NONE = 0,
    OW_OP_RESET_PRE,
    OW_OP_RESET_LOW,
    OW_OP_RESET_SAMPLE,
    OW_OP_RESET_DONE,
    OW_OP_TX_LOW,
    OW_OP_TX_HOLD,
    OW_OP_TX_RECOVER,
    OW_OP_RX_LOW,
    OW_OP_RX_SAMPLE,
    OW_OP_RX_RECOVER
} OwIsrOp_t;

static bool            g_ds_initialized = false;
static GPIO_TypeDef   *g_ds_port = NULL;
static uint16_t        g_ds_pin  = 0U;
static Ds18b20Sensor_t g_ds_sensors[DS18B20_MAX_SENSORS];
static uint8_t         g_ds_sensorCount = 0U;

static DsBusState_t    g_ds_busState = DS_BUS_IDLE;
static uint32_t        g_ds_convStartMs = 0U;
static uint8_t         g_ds_curSensorIdx = 0U;

// Async TIM7 transaction buffers
static volatile bool      g_ds_txBusy = false;
static volatile OwIsrOp_t g_ds_isrOp  = OW_OP_NONE;
static volatile bool      g_ds_presenceOk = false;
static uint8_t            g_ds_txBuf[12];
static uint8_t            g_ds_txLen = 0U;
static uint8_t            g_ds_txPos = 0U;
static uint8_t            g_ds_rxBuf[9];
static uint8_t            g_ds_rxLen = 0U;
static uint8_t            g_ds_rxPos = 0U;
static uint8_t            g_ds_bitIdx = 0U;

static inline void ow_low(GPIO_TypeDef *p_port, uint16_t pin)
{
    p_port->BSRR = ((uint32_t)pin << 16U);
}

static inline void ow_release(GPIO_TypeDef *p_port, uint16_t pin)
{
    p_port->BSRR = pin;
}

static inline uint8_t ow_read_pin(GPIO_TypeDef *p_port, uint16_t pin)
{
    return ((p_port->IDR & pin) != 0U) ? 1U : 0U;
}

static uint8_t dallas_crc8(const uint8_t *p_data, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        uint8_t inbyte = p_data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            uint8_t mix = (crc ^ inbyte) & 0x01U;
            crc >>= 1U;
            if (mix != 0U) {
                crc ^= 0x8CU;
            }
            inbyte >>= 1U;
        }
    }
    return crc;
}

// Synchronous primitives used only during explicit 1-Wire Search ROM bus scan
static bool ow_sync_reset(GPIO_TypeDef *p_port, uint16_t pin)
{
    ow_low(p_port, pin);
    HW_DWT_DelayUs(480U);
    __disable_irq();
    ow_release(p_port, pin);
    HW_DWT_DelayUs(70U);
    uint8_t pres = ow_read_pin(p_port, pin);
    __enable_irq();
    HW_DWT_DelayUs(410U);
    return (pres == 0U);
}

static void ow_sync_write_bit(GPIO_TypeDef *p_port, uint16_t pin, uint8_t bit)
{
    __disable_irq();
    ow_low(p_port, pin);
    if (bit != 0U) {
        HW_DWT_DelayUs(6U);
        ow_release(p_port, pin);
        __enable_irq();
        HW_DWT_DelayUs(64U);
    } else {
        HW_DWT_DelayUs(60U);
        ow_release(p_port, pin);
        __enable_irq();
        HW_DWT_DelayUs(10U);
    }
}

static uint8_t ow_sync_read_bit(GPIO_TypeDef *p_port, uint16_t pin)
{
    __disable_irq();
    ow_low(p_port, pin);
    HW_DWT_DelayUs(3U);
    ow_release(p_port, pin);
    HW_DWT_DelayUs(10U);
    uint8_t bit = ow_read_pin(p_port, pin);
    __enable_irq();
    HW_DWT_DelayUs(52U);
    return bit;
}

static void ow_sync_write_byte(GPIO_TypeDef *p_port, uint16_t pin, uint8_t byteVal)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        ow_sync_write_bit(p_port, pin, (byteVal >> i) & 1U);
    }
}

// Non-blocking TIM7 ISR state machine for Reset + TX bytes + RX bytes
static uint32_t ds18b20_tim7_step(void)
{
    switch (g_ds_isrOp) {
        case OW_OP_RESET_PRE:
            ow_low(g_ds_port, g_ds_pin);
            g_ds_isrOp = OW_OP_RESET_LOW;
            return 480U;

        case OW_OP_RESET_LOW:
            ow_release(g_ds_port, g_ds_pin);
            g_ds_isrOp = OW_OP_RESET_SAMPLE;
            return 70U;

        case OW_OP_RESET_SAMPLE:
            g_ds_presenceOk = (ow_read_pin(g_ds_port, g_ds_pin) == 0U);
            g_ds_isrOp = OW_OP_RESET_DONE;
            return 410U;

        case OW_OP_RESET_DONE:
            if (!g_ds_presenceOk || g_ds_txLen == 0U) {
                g_ds_txBusy = false;
                g_ds_isrOp  = OW_OP_NONE;
                CH455G_PumpQueue();
                return 0U;
            }
            g_ds_txPos  = 0U;
            g_ds_rxPos  = 0U;
            g_ds_bitIdx = 0U;
            g_ds_isrOp  = OW_OP_TX_LOW;
            return 1U;

        case OW_OP_TX_LOW: {
            ow_low(g_ds_port, g_ds_pin);
            uint8_t bit = (g_ds_txBuf[g_ds_txPos] >> g_ds_bitIdx) & 1U;
            g_ds_isrOp = OW_OP_TX_HOLD;
            if (bit != 0U) {
                return 6U;
            }
            return 60U;
        }

        case OW_OP_TX_HOLD: {
            ow_release(g_ds_port, g_ds_pin);
            uint8_t bit = (g_ds_txBuf[g_ds_txPos] >> g_ds_bitIdx) & 1U;
            g_ds_isrOp = OW_OP_TX_RECOVER;
            if (bit != 0U) {
                return 64U;
            }
            return 10U;
        }

        case OW_OP_TX_RECOVER:
            g_ds_bitIdx++;
            if (g_ds_bitIdx >= 8U) {
                g_ds_bitIdx = 0U;
                g_ds_txPos++;
            }
            if (g_ds_txPos < g_ds_txLen) {
                g_ds_isrOp = OW_OP_TX_LOW;
                return 1U;
            }
            if (g_ds_rxLen > 0U) {
                g_ds_rxBuf[0] = 0U;
                g_ds_isrOp = OW_OP_RX_LOW;
                return 1U;
            }
            g_ds_txBusy = false;
            g_ds_isrOp  = OW_OP_NONE;
            CH455G_PumpQueue();
            return 0U;

        case OW_OP_RX_LOW:
            ow_low(g_ds_port, g_ds_pin);
            g_ds_isrOp = OW_OP_RX_SAMPLE;
            return 2U;

        case OW_OP_RX_SAMPLE:
            ow_release(g_ds_port, g_ds_pin);
            HW_DWT_DelayUs(10U);
            if (ow_read_pin(g_ds_port, g_ds_pin) != 0U) {
                g_ds_rxBuf[g_ds_rxPos] |= (uint8_t)(1U << g_ds_bitIdx);
            }
            g_ds_isrOp = OW_OP_RX_RECOVER;
            return 52U;

        case OW_OP_RX_RECOVER:
            g_ds_bitIdx++;
            if (g_ds_bitIdx >= 8U) {
                g_ds_bitIdx = 0U;
                g_ds_rxPos++;
                if (g_ds_rxPos < g_ds_rxLen) {
                    g_ds_rxBuf[g_ds_rxPos] = 0U;
                }
            }
            if (g_ds_rxPos < g_ds_rxLen) {
                g_ds_isrOp = OW_OP_RX_LOW;
                return 1U;
            }
            g_ds_txBusy = false;
            g_ds_isrOp  = OW_OP_NONE;
            CH455G_PumpQueue();
            return 0U;

        default:
            g_ds_txBusy = false;
            g_ds_isrOp  = OW_OP_NONE;
            return 0U;
    }
}

static bool start_async_tx_rx(const uint8_t *p_tx, uint8_t txLen, uint8_t rxLen)
{
    if (g_ds_txBusy || HW_TIM7_IsBusy() || txLen > sizeof(g_ds_txBuf) || rxLen > sizeof(g_ds_rxBuf)) {
        return false;
    }
    memcpy(g_ds_txBuf, p_tx, txLen);
    g_ds_txLen      = txLen;
    g_ds_rxLen      = rxLen;
    g_ds_presenceOk = false;
    g_ds_txBusy     = true;
    g_ds_isrOp      = OW_OP_RESET_PRE;
    HW_TIM7_ScheduleUs(1U, ds18b20_tim7_step);
    return true;
}

static void configure_pin_od(GPIO_TypeDef *p_port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(p_port, &gpio);
    ow_release(p_port, pin);
    HW_DWT_DelayUs(1000U);
}

void DS18B20_ResetAll(void)
{
    g_ds_initialized  = false;
    g_ds_port         = NULL;
    g_ds_pin          = 0U;
    g_ds_sensorCount  = 0U;
    g_ds_busState     = DS_BUS_IDLE;
    g_ds_txBusy       = false;
    g_ds_isrOp        = OW_OP_NONE;
    memset(g_ds_sensors, 0, sizeof(g_ds_sensors));
}

uint8_t DS18B20_ScanBus(GPIO_TypeDef *p_port, uint16_t pin,
                        Ds18b20Sensor_t *p_outSensors, uint8_t maxSensors)
{
    if (p_port == NULL || maxSensors == 0U) {
        return 0U;
    }
    HW_DWT_EnsureInit();
    configure_pin_od(p_port, pin);

    uint8_t found = 0U;
    uint8_t rom[8] = {0};
    uint8_t lastDiscrepancy = 0U;
    bool    lastDeviceFlag = false;

    while (!lastDeviceFlag && found < maxSensors) {
        if (!ow_sync_reset(p_port, pin)) {
            break;
        }
        ow_sync_write_byte(p_port, pin, OW_CMD_SEARCH_ROM);

        uint8_t idBitNumber = 1U;
        uint8_t lastZero = 0U;
        uint8_t romByteNumber = 0U;
        uint8_t romByteMask = 1U;
        bool    searchOk = true;

        while (romByteNumber < 8U) {
            uint8_t idBit    = ow_sync_read_bit(p_port, pin);
            uint8_t cmpIdBit = ow_sync_read_bit(p_port, pin);

            if (idBit == 1U && cmpIdBit == 1U) {
                searchOk = false;
                break;
            }

            uint8_t searchDir;
            if (idBit != cmpIdBit) {
                searchDir = idBit;
            } else {
                if (idBitNumber < lastDiscrepancy) {
                    searchDir = ((rom[romByteNumber] & romByteMask) > 0U) ? 1U : 0U;
                } else {
                    searchDir = (idBitNumber == lastDiscrepancy) ? 1U : 0U;
                }
                if (searchDir == 0U) {
                    lastZero = idBitNumber;
                }
            }

            if (searchDir == 1U) {
                rom[romByteNumber] |= romByteMask;
            } else {
                rom[romByteNumber] &= (uint8_t)~romByteMask;
            }

            ow_sync_write_bit(p_port, pin, searchDir);
            idBitNumber++;
            romByteMask <<= 1U;
            if (romByteMask == 0U) {
                romByteNumber++;
                romByteMask = 1U;
            }
        }

        if (!searchOk || rom[0] == 0x00U || dallas_crc8(rom, 7U) != rom[7]) {
            break;
        }

        memcpy(p_outSensors[found].rom, rom, 8U);
        p_outSensors[found].active = true;
        found++;

        lastDiscrepancy = lastZero;
        if (lastDiscrepancy == 0U) {
            lastDeviceFlag = true;
        }
    }

    return found;
}

bool DS18B20_EnsureInit(GPIO_TypeDef *p_port, uint16_t pin)
{
    if (p_port == NULL) {
        return false;
    }
    if (g_ds_initialized && g_ds_port == p_port && g_ds_pin == pin) {
        return true;
    }

    g_ds_port = p_port;
    g_ds_pin  = pin;
    HW_DWT_EnsureInit();
    HW_TIM7_EnsureInit();

    g_ds_sensorCount = DS18B20_ScanBus(p_port, pin, g_ds_sensors, DS18B20_MAX_SENSORS);
    g_ds_busState    = DS_BUS_IDLE;
    g_ds_initialized = true;
    return true;
}

static int8_t find_or_register_sensor(uint64_t rom64, uint8_t busIndex)
{
    if (rom64 != 0ULL) {
        uint8_t target[8];
        for (uint8_t i = 0U; i < 8U; i++) {
            target[i] = (uint8_t)((rom64 >> (i * 8U)) & 0xFFU);
        }
        for (uint8_t i = 0U; i < g_ds_sensorCount; i++) {
            if (memcmp(g_ds_sensors[i].rom, target, 8U) == 0) {
                return (int8_t)i;
            }
        }
        if (g_ds_sensorCount < DS18B20_MAX_SENSORS) {
            uint8_t idx = g_ds_sensorCount++;
            memcpy(g_ds_sensors[idx].rom, target, 8U);
            g_ds_sensors[idx].active = true;
            g_ds_sensors[idx].ok     = false;
            return (int8_t)idx;
        }
        return -1;
    }

    if (busIndex < g_ds_sensorCount) {
        return (int8_t)busIndex;
    }
    return -1;
}

static void step_bus_machine(uint32_t nowMs)
{
    if (!g_ds_initialized || g_ds_sensorCount == 0U || g_ds_txBusy) {
        return;
    }

    switch (g_ds_busState) {
        case DS_BUS_IDLE: {
            uint8_t cmd[2] = {OW_CMD_SKIP_ROM, OW_CMD_CONVERT_T};
            if (start_async_tx_rx(cmd, 2U, 0U)) {
                g_ds_busState = DS_BUS_TX_CONVERT_WAIT;
            }
            break;
        }

        case DS_BUS_TX_CONVERT_WAIT:
            g_ds_convStartMs = nowMs;
            g_ds_busState    = DS_BUS_WAIT_CONVERSION;
            break;

        case DS_BUS_WAIT_CONVERSION:
            if ((nowMs - g_ds_convStartMs) >= DS18B20_CONV_WAIT_MS) {
                g_ds_curSensorIdx = 0U;
                uint8_t cmd[10];
                cmd[0] = OW_CMD_MATCH_ROM;
                memcpy(&cmd[1], g_ds_sensors[0].rom, 8U);
                cmd[9] = OW_CMD_READ_SCRATCHPAD;
                if (start_async_tx_rx(cmd, 10U, 9U)) {
                    g_ds_busState = DS_BUS_READ_SENSOR_WAIT;
                }
            }
            break;

        case DS_BUS_READ_SENSOR_WAIT: {
            Ds18b20Sensor_t *p_s = &g_ds_sensors[g_ds_curSensorIdx];
            if (g_ds_presenceOk && dallas_crc8(g_ds_rxBuf, 8U) == g_ds_rxBuf[8]) {
                int16_t raw = (int16_t)(((uint16_t)g_ds_rxBuf[1] << 8U) | g_ds_rxBuf[0]);
                p_s->tempC = (float)raw * 0.0625f;
                p_s->ok    = true;
            } else {
                p_s->ok = false;
            }

            g_ds_curSensorIdx++;
            if (g_ds_curSensorIdx < g_ds_sensorCount) {
                uint8_t cmd[10];
                cmd[0] = OW_CMD_MATCH_ROM;
                memcpy(&cmd[1], g_ds_sensors[g_ds_curSensorIdx].rom, 8U);
                cmd[9] = OW_CMD_READ_SCRATCHPAD;
                start_async_tx_rx(cmd, 10U, 9U);
            } else {
                g_ds_busState = DS_BUS_IDLE;
            }
            break;
        }
    }
}

bool DS18B20_PollAndRead(GPIO_TypeDef *p_port, uint16_t pin,
                         uint64_t rom64, uint8_t busIndex,
                         uint32_t nowMs, float *p_tempC, bool *p_ok)
{
    DS18B20_EnsureInit(p_port, pin);
    step_bus_machine(nowMs);

    int8_t idx = find_or_register_sensor(rom64, busIndex);
    if (idx < 0) {
        if (p_tempC != NULL) { *p_tempC = 0.0f; }
        if (p_ok != NULL)    { *p_ok = false; }
        return false;
    }

    if (p_tempC != NULL) { *p_tempC = g_ds_sensors[idx].tempC; }
    if (p_ok != NULL)    { *p_ok = g_ds_sensors[idx].ok; }
    return g_ds_sensors[idx].ok;
}
