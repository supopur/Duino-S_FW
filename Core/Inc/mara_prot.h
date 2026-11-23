// Portable, transport-agnostic MaraProt v1 protocol parser and dispatcher.
// Works identically over USB CDC, TCP/IP, and RS-485 via ProtTransport_t.

#ifndef MARA_PROT_H
#define MARA_PROT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mara_vm.h"
#include <stdbool.h>
#include <stdint.h>

#define PROT_MAGIC_0               0x4DU
#define PROT_MAGIC_1               0x50U
#define PROT_VERSION_1             0x10U
#define PROT_FLAG_RESPONSE         0x01U
#define PROT_FLAG_ERROR            0x02U
#define PROT_FLAG_ENCRYPTED        0x04U
#define PROT_HEADER_BYTES          10U
#define PROT_CRC_BYTES             2U
#define PROT_OVERHEAD_BYTES        (PROT_HEADER_BYTES + PROT_CRC_BYTES)
#define PROT_MAX_PAYLOAD_BYTES     520U
#define PROT_MAX_FRAME_BYTES       (PROT_OVERHEAD_BYTES + PROT_MAX_PAYLOAD_BYTES)
#define PROT_MAX_WATCH_SLOTS       512U
#define PROT_STATION_P2P           0x00U
#define PROT_STATION_BROADCAST     0xFFU
#define PROT_DFU_MAGIC             0xDF00B007U
#define PROT_AUTH_CHALLENGE_BYTES  16U
#define PROT_AUTH_KEY_BYTES        32U
#define PROT_MAX_PASSWORD_BYTES    32U
#define PROT_UNIT_CONFIG_BYTES     50U
#define PROT_MAX_MODBUS_MAP_ENTRIES 64U

typedef enum {
    PROT_MB_TABLE_COIL          = 0,
    PROT_MB_TABLE_DISCRETE      = 1,
    PROT_MB_TABLE_HOLDING_REG   = 2,
    PROT_MB_TABLE_INPUT_REG     = 3
} ProtModbusTable_t;

typedef enum {
    PROT_MB_TARGET_DI       = 0,
    PROT_MB_TARGET_DO       = 1,
    PROT_MB_TARGET_AI       = 2,
    PROT_MB_TARGET_AO       = 3,
    PROT_MB_TARGET_VM_BOOL  = 4,
    PROT_MB_TARGET_VM_INT   = 5,
    PROT_MB_TARGET_VM_FLOAT = 6
} ProtModbusTargetKind_t;

typedef struct {
    uint8_t  table;
    uint8_t  targetKind;
    uint16_t modbusAddr;
    uint16_t targetIdx;
    uint8_t  flags;
    uint8_t  reserved;
} ProtModbusMapEntry_t;

typedef enum {
    PROT_RS485_MODE_MARAPROT      = 0,
    PROT_RS485_MODE_MODBUS_MASTER = 1,
    PROT_RS485_MODE_MODBUS_SLAVE  = 2
} ProtRs485Mode_t;

typedef struct {
    char     unitName[16];
    uint8_t  stationId;
    uint8_t  ipMode;
    uint8_t  staticIp[4];
    uint8_t  subnetMask[4];
    uint8_t  gateway[4];
    uint16_t tcpPort;
    uint8_t  runOutputPin;
    uint8_t  faultOutputPin;
    uint8_t  estopOutputPin;
    uint8_t  bootMode;
    uint8_t  safeOutputsOnStop;
    uint8_t  watchdogEnabled;
    uint8_t  rs485Mode;
    uint8_t  rs485BaudIndex;
} ProtUnitConfig_t;

typedef enum {
    PROT_STATUS_OK = 0,
    PROT_STATUS_ERR_CRC,
    PROT_STATUS_ERR_OPCODE,
    PROT_STATUS_ERR_PARAM,
    PROT_STATUS_ERR_BUSY,
    PROT_STATUS_ERR_STATE,
    PROT_STATUS_ERR_STORAGE,
    PROT_STATUS_ERR_AUTH
} ProtStatus_t;

typedef enum {
    PROT_OP_PING             = 0x01,
    PROT_OP_GET_INFO         = 0x02,
    PROT_OP_AUTH_CHALLENGE   = 0x03,
    PROT_OP_AUTH_VERIFY      = 0x04,
    PROT_OP_VM_STATUS        = 0x10,
    PROT_OP_VM_CONTROL       = 0x11,
    PROT_OP_VM_UPLOAD_BEGIN  = 0x20,
    PROT_OP_VM_UPLOAD_CHUNK  = 0x21,
    PROT_OP_VM_UPLOAD_COMMIT = 0x22,
    PROT_OP_WATCH_SUBSCRIBE  = 0x30,
    PROT_OP_WATCH_POLL       = 0x31,
    PROT_OP_SLOT_READ_BATCH  = 0x32,
    PROT_OP_SLOT_WRITE_BATCH = 0x33,
    PROT_OP_RTC_GET          = 0x40,
    PROT_OP_RTC_SET          = 0x41,
    PROT_OP_ONEWIRE_SCAN     = 0x50,
    PROT_OP_UNIT_CONFIG_GET  = 0x51,
    PROT_OP_UNIT_CONFIG_SET  = 0x52,
    PROT_OP_MODBUS_MAP_GET   = 0x53,
    PROT_OP_MODBUS_MAP_SET   = 0x54,
    PROT_OP_FW_ENTER_ROM_DFU = 0xF3
} ProtOpcode_t;

typedef enum {
    PROT_TRANSPORT_USB_CDC = 0,
    PROT_TRANSPORT_TCP_IP  = 1,
    PROT_TRANSPORT_RS485   = 2
} ProtTransportKind_t;

typedef enum {
    PROT_RX_SOF0 = 0,
    PROT_RX_SOF1,
    PROT_RX_HEADER,
    PROT_RX_PAYLOAD,
    PROT_RX_CRC0,
    PROT_RX_CRC1
} ProtRxState_t;

typedef struct {
    ProtTransportKind_t kind;
    uint16_t            maxMtuBytes;
    uint16_t            modelId;
    uint32_t            fwVersion;
    uint8_t             stationId;
    bool                (*tx_send)(const uint8_t *p_data, uint16_t lenBytes);
    uint32_t            (*get_uptime_ms)(void);
    bool                (*storage_write)(uint32_t offset, const uint8_t *p_data, uint16_t lenBytes);
    bool                (*storage_read)(uint32_t offset, uint8_t *p_buf, uint16_t lenBytes);
    int64_t             (*rtc_get)(void);
    bool                (*rtc_set)(int64_t unixSec);
    uint8_t             (*onewire_scan)(uint8_t dioPin, uint8_t *p_outRoms8, uint8_t maxSensors);
    void                (*enter_dfu)(void);
    uint32_t            (*rng_next)(void);
    const char         *p_password;
    void                (*unit_config_saved)(const ProtUnitConfig_t *p_cfg, const char *p_password);
    uint8_t             (*modbus_map_get)(ProtModbusMapEntry_t *p_outEntries, uint8_t maxEntries);
    bool                (*modbus_map_set)(const ProtModbusMapEntry_t *p_entries, uint8_t count);
} ProtTransport_t;

typedef struct {
    VmInstance_t   *p_vm;
    ProtTransport_t transport;
    ProtRxState_t   rxState;
    uint16_t        rxPos;
    uint16_t        rxExpectedPayload;
    uint8_t         rxBuf[PROT_MAX_FRAME_BYTES];
    uint8_t         txBuf[PROT_MAX_FRAME_BYTES];
    uint32_t        uploadTotalBytes;
    uint32_t        uploadExpectedCrc32;
    uint32_t        uploadNextOffset;
    bool            uploadActive;
    uint16_t        watchCount;
    uint16_t        watchEpoch;
    uint16_t        watchSlots[PROT_MAX_WATCH_SLOTS];
    uint8_t         watchLastRaw[PROT_MAX_WATCH_SLOTS][VM_MAX_SLOT_BYTES];
    bool            authRequired;
    bool            authValid;
    bool            replyEncrypted;
    uint8_t         authChallenge[PROT_AUTH_CHALLENGE_BYTES];
    uint8_t         sessionKey[PROT_AUTH_KEY_BYTES];
    char            password[PROT_MAX_PASSWORD_BYTES + 1U];
    ProtUnitConfig_t unitConfig;
} ProtInstance_t;

// Initialize MaraProt server instance bound to a MaraVM and transport callbacks.
bool PROT_Init(ProtInstance_t *p_prot, VmInstance_t *p_vm, const ProtTransport_t *p_transport);

// Feed a single received byte into the framing state machine.
// Automatically dispatches complete, CRC-verified frames.
void PROT_FeedByte(ProtInstance_t *p_prot, uint8_t byte);

// Feed a buffer of received bytes into the framing state machine.
void PROT_FeedBuffer(ProtInstance_t *p_prot, const uint8_t *p_data, uint16_t lenBytes);

// Compute CRC-16-CCITT (poly 0x1021, init 0xFFFF).
uint16_t PROT_CalcCrc16(const uint8_t *p_data, uint16_t lenBytes);

#ifdef __cplusplus
}
#endif

#endif // MARA_PROT_H
