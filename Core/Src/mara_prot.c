// Portable, transport-agnostic MaraProt v1 protocol parser and command dispatcher.

#include "mara_prot.h"
#include <string.h>

#define PROT_CRC16_POLY          0x1021U
#define PROT_CRC16_INIT          0xFFFFU
#define PROT_MODEL_NAME          "Duino - S"
#define PROT_MODEL_NAME_BYTES    24U
#define PROT_CTRL_STOP           0U
#define PROT_CTRL_START          1U
#define PROT_CTRL_PAUSE          2U
#define PROT_CTRL_ESTOP_RESET    5U
#define PROT_CTRL_CLEAR_FAULT    6U
#define PROT_CRYPT_DIR_REQ       0x01U
#define PROT_CRYPT_DIR_RESP      0x02U

static const uint32_t kSha256K[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U
};

static inline uint32_t rotr32(uint32_t x, uint32_t n) { return (x >> n) | (x << (32U - n)); }

static void sha256_transform(uint32_t state[8], const uint8_t block[64])
{
    uint32_t w[64];
    for (uint32_t i = 0U; i < 16U; i++) {
        w[i] = ((uint32_t)block[i * 4U] << 24U) | ((uint32_t)block[i * 4U + 1U] << 16U)
             | ((uint32_t)block[i * 4U + 2U] << 8U) | (uint32_t)block[i * 4U + 3U];
    }
    for (uint32_t i = 16U; i < 64U; i++) {
        uint32_t s0 = rotr32(w[i - 15U], 7U) ^ rotr32(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
        uint32_t s1 = rotr32(w[i - 2U], 17U) ^ rotr32(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    for (uint32_t i = 0U; i < 64U; i++) {
        uint32_t S1  = rotr32(e, 6U) ^ rotr32(e, 11U) ^ rotr32(e, 25U);
        uint32_t ch  = (e & f) ^ ((~e) & g);
        uint32_t t1  = h + S1 + ch + kSha256K[i] + w[i];
        uint32_t S0  = rotr32(a, 2U) ^ rotr32(a, 13U) ^ rotr32(a, 22U);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2  = S0 + maj;
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

static void sha256_two(const uint8_t *p_a, uint16_t lenA, const uint8_t *p_b, uint16_t lenB, uint8_t out[32])
{
    uint32_t state[8] = {
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U
    };
    uint8_t buf[64];
    uint32_t pos = 0U;
    uint64_t totalBits = ((uint64_t)lenA + (uint64_t)lenB) * 8ULL;

    for (uint16_t i = 0U; i < lenA; i++) {
        buf[pos++] = p_a[i];
        if (pos == 64U) { sha256_transform(state, buf); pos = 0U; }
    }
    for (uint16_t i = 0U; i < lenB; i++) {
        buf[pos++] = p_b[i];
        if (pos == 64U) { sha256_transform(state, buf); pos = 0U; }
    }
    buf[pos++] = 0x80U;
    if (pos > 56U) {
        while (pos < 64U) { buf[pos++] = 0U; }
        sha256_transform(state, buf);
        pos = 0U;
    }
    while (pos < 56U) { buf[pos++] = 0U; }
    for (uint32_t i = 0U; i < 8U; i++) {
        buf[56U + i] = (uint8_t)((totalBits >> ((7U - i) * 8U)) & 0xFFU);
    }
    sha256_transform(state, buf);
    for (uint32_t i = 0U; i < 8U; i++) {
        out[i * 4U + 0U] = (uint8_t)((state[i] >> 24U) & 0xFFU);
        out[i * 4U + 1U] = (uint8_t)((state[i] >> 16U) & 0xFFU);
        out[i * 4U + 2U] = (uint8_t)((state[i] >> 8U) & 0xFFU);
        out[i * 4U + 3U] = (uint8_t)(state[i] & 0xFFU);
    }
}

static void prot_crypt_payload(const uint8_t key[32], uint16_t seqId, uint8_t dir, uint8_t *p_data, uint16_t len)
{
    uint8_t iv[4] = { (uint8_t)(seqId & 0xFFU), (uint8_t)(seqId >> 8U), dir, 0U };
    uint8_t stream[32];
    uint16_t offset = 0U;
    uint8_t blk = 0U;
    while (offset < len) {
        iv[3] = blk++;
        sha256_two(key, 32U, iv, 4U, stream);
        for (uint16_t i = 0U; i < 32U && offset < len; i++, offset++) {
            p_data[offset] ^= stream[i];
        }
    }
}

static void write_u16_le(uint8_t *p_buf, uint16_t val)
{
    p_buf[0] = (uint8_t)(val & 0xFFU);
    p_buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *p_buf, uint32_t val)
{
    p_buf[0] = (uint8_t)(val & 0xFFU);
    p_buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
    p_buf[2] = (uint8_t)((val >> 16U) & 0xFFU);
    p_buf[3] = (uint8_t)((val >> 24U) & 0xFFU);
}

static uint16_t read_u16_le(const uint8_t *p_buf)
{
    return (uint16_t)p_buf[0] | ((uint16_t)p_buf[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *p_buf)
{
    return (uint32_t)p_buf[0]
         | ((uint32_t)p_buf[1] << 8U)
         | ((uint32_t)p_buf[2] << 16U)
         | ((uint32_t)p_buf[3] << 24U);
}

static void send_response(ProtInstance_t *p_prot, uint16_t seqId, uint8_t opcode,
                          ProtStatus_t status, uint16_t payloadLen)
{
    if (p_prot->transport.tx_send == NULL || payloadLen > PROT_MAX_PAYLOAD_BYTES) {
        return;
    }

    uint8_t verFlags = PROT_VERSION_1 | PROT_FLAG_RESPONSE;
    if (status != PROT_STATUS_OK) {
        verFlags |= PROT_FLAG_ERROR;
    }
    if (p_prot->replyEncrypted && p_prot->authValid && payloadLen > 0U) {
        prot_crypt_payload(p_prot->sessionKey, seqId, PROT_CRYPT_DIR_RESP,
                           &p_prot->txBuf[PROT_HEADER_BYTES], payloadLen);
        verFlags |= PROT_FLAG_ENCRYPTED;
    }

    p_prot->txBuf[0] = PROT_MAGIC_0;
    p_prot->txBuf[1] = PROT_MAGIC_1;
    p_prot->txBuf[2] = verFlags;
    p_prot->txBuf[3] = p_prot->transport.stationId;
    write_u16_le(&p_prot->txBuf[4], seqId);
    p_prot->txBuf[6] = opcode;
    p_prot->txBuf[7] = (uint8_t)status;
    write_u16_le(&p_prot->txBuf[8], payloadLen);

    uint16_t crcPos = (uint16_t)(PROT_HEADER_BYTES + payloadLen);
    uint16_t crc    = PROT_CalcCrc16(p_prot->txBuf, crcPos);
    write_u16_le(&p_prot->txBuf[crcPos], crc);

    p_prot->transport.tx_send(p_prot->txBuf, (uint16_t)(crcPos + PROT_CRC_BYTES));
}

static void handle_ping(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    uint32_t echoVal   = (reqLen >= 4U) ? read_u32_le(p_req) : 0U;
    uint32_t uptime_ms = (p_prot->transport.get_uptime_ms != NULL) ? p_prot->transport.get_uptime_ms() : 0U;

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES + 0U], echoVal);
    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES + 4U], uptime_ms);
    send_response(p_prot, seqId, PROT_OP_PING, PROT_STATUS_OK, 8U);
}

static void handle_get_info(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
    memset(p_out, 0, 44U);

    write_u16_le(&p_out[0], 0x0001U);
    write_u16_le(&p_out[2], p_prot->transport.modelId);
    write_u32_le(&p_out[4], p_prot->transport.fwVersion);
    p_out[8] = (uint8_t)p_prot->transport.kind;
    write_u16_le(&p_out[9], p_prot->transport.maxMtuBytes);
    p_out[11] = (uint8_t)VM_GetState(p_prot->p_vm);
    write_u32_le(&p_out[12], p_prot->p_vm->hasValidImage ? p_prot->p_vm->header.payloadCrc32 : 0U);
    write_u32_le(&p_out[16], p_prot->p_vm->hasValidImage ? p_prot->p_vm->header.projectHash : 0U);
    strncpy((char *)&p_out[20], PROT_MODEL_NAME, PROT_MODEL_NAME_BYTES - 1U);

    send_response(p_prot, seqId, PROT_OP_GET_INFO, PROT_STATUS_OK, 44U);
}

static void handle_vm_status(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
    p_out[0] = (uint8_t)p_prot->p_vm->runState;
    p_out[1] = p_prot->p_vm->estopActive;
    write_u16_le(&p_out[2], p_prot->p_vm->estopMask);
    write_u32_le(&p_out[4], p_prot->p_vm->scanCount);
    for (uint32_t i = 0U; i < 5U; i++) {
        write_u32_le(&p_out[8U + (i * 4U)], p_prot->p_vm->threadActual_us[i + 1U]);
    }
    write_u16_le(&p_out[28], (uint16_t)p_prot->p_vm->lastFaultCode);
    write_u16_le(&p_out[30], p_prot->p_vm->lastFaultPc);

    send_response(p_prot, seqId, PROT_OP_VM_STATUS, PROT_STATUS_OK, 32U);
}

static void handle_vm_control(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 1U) {
        send_response(p_prot, seqId, PROT_OP_VM_CONTROL, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t    action = p_req[0];
    VmStatus_t vmSt   = VM_OK;
    if (action == PROT_CTRL_STOP) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_STOPPED);
        if (vmSt == VM_OK) { vmSt = VM_Reset(p_prot->p_vm); }
    } else if (action == PROT_CTRL_START) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_RUNNING);
    } else if (action == PROT_CTRL_PAUSE) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_PAUSED);
    } else if (action == PROT_CTRL_ESTOP_RESET) {
        p_prot->p_vm->estopActive = 0U;
        p_prot->p_vm->estopMask   = 0U;
    } else if (action == PROT_CTRL_CLEAR_FAULT) {
        p_prot->p_vm->lastFaultCode = VM_OK;
        p_prot->p_vm->runState      = VM_STATE_STOPPED;
    } else {
        send_response(p_prot, seqId, PROT_OP_VM_CONTROL, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    ProtStatus_t respSt = (vmSt == VM_OK) ? PROT_STATUS_OK : PROT_STATUS_ERR_STATE;
    p_prot->txBuf[PROT_HEADER_BYTES + 0U] = (uint8_t)p_prot->p_vm->runState;
    p_prot->txBuf[PROT_HEADER_BYTES + 1U] = p_prot->p_vm->estopActive;
    send_response(p_prot, seqId, PROT_OP_VM_CONTROL, respSt, 2U);
}

static void handle_upload_begin(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 8U) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint32_t totalBytes = read_u32_le(&p_req[0]);
    uint32_t crc32      = read_u32_le(&p_req[4]);
    if (totalBytes < VM_HEADER_SIZE_BYTES || totalBytes > VM_MAX_IMAGE_BYTES) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    VM_SetState(p_prot->p_vm, VM_STATE_STOPPED);
    p_prot->p_vm->hasValidImage = false;
    p_prot->uploadTotalBytes      = totalBytes;
    p_prot->uploadExpectedCrc32   = crc32;
    p_prot->uploadNextOffset      = 0U;
    p_prot->uploadActive          = true;

    uint16_t maxChunk = (uint16_t)(p_prot->transport.maxMtuBytes - 4U);
    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES], maxChunk);
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_OK, 2U);
}

static void handle_upload_chunk(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (!p_prot->uploadActive || reqLen < 5U) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_STATE, 0U);
        return;
    }

    uint32_t offset   = read_u32_le(&p_req[0]);
    uint16_t dataLen  = (uint16_t)(reqLen - 4U);
    if ((offset + dataLen) > p_prot->uploadTotalBytes || offset > p_prot->uploadNextOffset) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    // Idempotent retry: if offset already received, ACK current nextOffset
    if (offset == p_prot->uploadNextOffset) {
        memcpy(&p_prot->p_vm->imageBuf[offset], &p_req[4], dataLen);
        if (p_prot->transport.storage_write != NULL) {
            if (!p_prot->transport.storage_write(offset, &p_req[4], dataLen)) {
                send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_STORAGE, 0U);
                return;
            }
        }
        p_prot->uploadNextOffset += dataLen;
    }

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES], p_prot->uploadNextOffset);
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_OK, 4U);
}

static void handle_upload_commit(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (!p_prot->uploadActive || p_prot->uploadNextOffset != p_prot->uploadTotalBytes) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_STATE, 0U);
        return;
    }

    p_prot->uploadActive = false;
    uint32_t fullCrc = VM_CalcCrc32(p_prot->p_vm->imageBuf, p_prot->uploadTotalBytes);
    if (fullCrc != p_prot->uploadExpectedCrc32) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_CRC, 0U);
        return;
    }

    // Validate MFLW header and payload CRC in-place inside imageBuf
    VmStatus_t st = VM_LoadImage(p_prot->p_vm, p_prot->p_vm->imageBuf, p_prot->uploadTotalBytes);
    if (st != VM_OK) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    if (reqLen >= 1U && p_req[0] != 0U) {
        VM_SetState(p_prot->p_vm, VM_STATE_RUNNING);
    }

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES], p_prot->p_vm->header.payloadCrc32);
    p_prot->txBuf[PROT_HEADER_BYTES + 4U] = (uint8_t)p_prot->p_vm->runState;
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_OK, 5U);
}

static void handle_watch_subscribe(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 3U) {
        send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t  mode     = p_req[0];
    uint16_t count    = read_u16_le(&p_req[1]);
    uint16_t startIdx = (mode == 1U) ? p_prot->watchCount : 0U;
    if ((startIdx + count) > PROT_MAX_WATCH_SLOTS || (3U + (count * 2U)) > reqLen) {
        send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    p_prot->watchCount = (uint16_t)(startIdx + count);
    p_prot->watchEpoch = 1U;
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t dstIdx = (uint16_t)(startIdx + i);
        p_prot->watchSlots[dstIdx] = read_u16_le(&p_req[3U + (i * 2U)]);
        memset(p_prot->watchLastRaw[dstIdx], 0, VM_MAX_SLOT_BYTES);
    }

    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES + 0U], p_prot->watchCount);
    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES + 2U], p_prot->watchEpoch);
    send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_OK, 4U);
}

static void handle_watch_poll(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    uint16_t ackEpoch   = (reqLen >= 2U) ? read_u16_le(&p_req[0]) : 0U;
    bool     forceFull  = (ackEpoch == 0U || ackEpoch != p_prot->watchEpoch);
    uint8_t *p_out      = &p_prot->txBuf[PROT_HEADER_BYTES];
    uint16_t pos        = 6U;
    uint16_t emitted    = 0U;
    uint16_t maxPayload = p_prot->transport.maxMtuBytes;
    if (maxPayload > PROT_MAX_PAYLOAD_BYTES) {
        maxPayload = PROT_MAX_PAYLOAD_BYTES;
    }

    uint8_t curRaw[VM_MAX_SLOT_BYTES];
    for (uint16_t i = 0U; i < p_prot->watchCount; i++) {
        uint16_t slotId = p_prot->watchSlots[i];
        uint8_t  valLen = VM_ReadSlotRaw(p_prot->p_vm, slotId, curRaw);
        if (valLen == 0U) {
            continue;
        }
        if (!forceFull && memcmp(curRaw, p_prot->watchLastRaw[i], valLen) == 0) {
            continue;
        }
        if ((pos + 2U + valLen) > maxPayload) {
            break;
        }
        write_u16_le(&p_out[pos], slotId);
        memcpy(&p_out[pos + 2U], curRaw, valLen);
        memcpy(p_prot->watchLastRaw[i], curRaw, valLen);
        pos += (uint16_t)(2U + valLen);
        emitted++;
    }

    p_prot->watchEpoch++;
    if (p_prot->watchEpoch == 0U) {
        p_prot->watchEpoch = 1U;
    }

    write_u16_le(&p_out[0], p_prot->watchEpoch);
    p_out[2] = (uint8_t)p_prot->p_vm->runState;
    p_out[3] = p_prot->p_vm->estopActive;
    write_u16_le(&p_out[4], emitted);
    send_response(p_prot, seqId, PROT_OP_WATCH_POLL, PROT_STATUS_OK, pos);
}

static void handle_slot_read_batch(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 2U) {
        send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint16_t count   = read_u16_le(&p_req[0]);
    if ((2U + (count * 2U)) > reqLen) {
        send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t *p_out   = &p_prot->txBuf[PROT_HEADER_BYTES];
    uint16_t pos     = 2U;
    uint16_t emitted = 0U;
    uint8_t  curRaw[VM_MAX_SLOT_BYTES];

    for (uint16_t i = 0U; i < count; i++) {
        uint16_t slotId = read_u16_le(&p_req[2U + (i * 2U)]);
        uint8_t  valLen = VM_ReadSlotRaw(p_prot->p_vm, slotId, curRaw);
        if (valLen == 0U || (pos + 2U + valLen) > p_prot->transport.maxMtuBytes) {
            break;
        }
        write_u16_le(&p_out[pos], slotId);
        memcpy(&p_out[pos + 2U], curRaw, valLen);
        pos += (uint16_t)(2U + valLen);
        emitted++;
    }

    write_u16_le(&p_out[0], emitted);
    send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_OK, pos);
}

static void handle_slot_write_batch(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 2U) {
        send_response(p_prot, seqId, PROT_OP_SLOT_WRITE_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint16_t count   = read_u16_le(&p_req[0]);
    uint16_t pos     = 2U;
    uint16_t applied = 0U;

    for (uint16_t i = 0U; i < count; i++) {
        if ((pos + 2U) > reqLen) {
            break;
        }
        uint16_t slotId = read_u16_le(&p_req[pos]);
        uint8_t  valLen = VM_GetSlotSize(slotId);
        if (valLen == 0U || (pos + 2U + valLen) > reqLen) {
            break;
        }
        if (VM_WriteSlotRaw(p_prot->p_vm, slotId, &p_req[pos + 2U]) > 0U) {
            applied++;
        }
        pos += (uint16_t)(2U + valLen);
    }

    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES], applied);
    send_response(p_prot, seqId, PROT_OP_SLOT_WRITE_BATCH, PROT_STATUS_OK, 2U);
}

static void handle_auth_challenge(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint32_t seed = (p_prot->transport.get_uptime_ms != NULL) ? p_prot->transport.get_uptime_ms() : 0x12345678U;
    for (uint8_t i = 0U; i < 4U; i++) {
        uint32_t r = (p_prot->transport.rng_next != NULL) ? p_prot->transport.rng_next()
                                                          : (seed = (seed * 1664525U) + 1013904223U);
        write_u32_le(&p_prot->authChallenge[i * 4U], r);
    }
    p_prot->authValid = false;
    p_prot->txBuf[PROT_HEADER_BYTES] = p_prot->authRequired ? 1U : 0U;
    memcpy(&p_prot->txBuf[PROT_HEADER_BYTES + 1U], p_prot->authChallenge, PROT_AUTH_CHALLENGE_BYTES);
    p_prot->replyEncrypted = false;
    send_response(p_prot, seqId, PROT_OP_AUTH_CHALLENGE, PROT_STATUS_OK, 1U + PROT_AUTH_CHALLENGE_BYTES);
}

static void handle_auth_verify(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    p_prot->replyEncrypted = false;
    if (!p_prot->authRequired) {
        p_prot->authValid = true;
        p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
        send_response(p_prot, seqId, PROT_OP_AUTH_VERIFY, PROT_STATUS_OK, 1U);
        return;
    }
    if (reqLen < PROT_AUTH_KEY_BYTES) {
        send_response(p_prot, seqId, PROT_OP_AUTH_VERIFY, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }
    uint8_t sessionKey[PROT_AUTH_KEY_BYTES];
    uint8_t expected[PROT_AUTH_KEY_BYTES];
    uint16_t pwLen = (uint16_t)strlen(p_prot->password);
    sha256_two((const uint8_t *)p_prot->password, pwLen,
               p_prot->authChallenge, PROT_AUTH_CHALLENGE_BYTES, sessionKey);
    sha256_two(sessionKey, PROT_AUTH_KEY_BYTES,
               p_prot->authChallenge, PROT_AUTH_CHALLENGE_BYTES, expected);

    uint8_t diff = 0U;
    for (uint8_t i = 0U; i < PROT_AUTH_KEY_BYTES; i++) {
        diff |= (uint8_t)(expected[i] ^ p_req[i]);
    }
    if (diff != 0U) {
        p_prot->authValid = false;
        send_response(p_prot, seqId, PROT_OP_AUTH_VERIFY, PROT_STATUS_ERR_AUTH, 0U);
        return;
    }
    memcpy(p_prot->sessionKey, sessionKey, PROT_AUTH_KEY_BYTES);
    p_prot->authValid = true;
    p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
    send_response(p_prot, seqId, PROT_OP_AUTH_VERIFY, PROT_STATUS_OK, 1U);
}

static void handle_unit_config_get(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
    memset(p_out, 0, PROT_UNIT_CONFIG_BYTES);
    memcpy(&p_out[0], p_prot->unitConfig.unitName, 16U);
    p_out[16] = p_prot->unitConfig.stationId;
    p_out[17] = p_prot->unitConfig.ipMode;
    memcpy(&p_out[18], p_prot->unitConfig.staticIp, 4U);
    memcpy(&p_out[22], p_prot->unitConfig.subnetMask, 4U);
    memcpy(&p_out[26], p_prot->unitConfig.gateway, 4U);
    write_u16_le(&p_out[30], p_prot->unitConfig.tcpPort);
    p_out[32] = p_prot->unitConfig.runOutputPin;
    p_out[33] = p_prot->unitConfig.faultOutputPin;
    p_out[34] = p_prot->unitConfig.estopOutputPin;
    p_out[35] = p_prot->unitConfig.bootMode;
    p_out[36] = p_prot->unitConfig.safeOutputsOnStop;
    p_out[37] = p_prot->unitConfig.watchdogEnabled;
    p_out[48] = p_prot->unitConfig.rs485Mode;
    p_out[49] = p_prot->unitConfig.rs485BaudIndex;
    send_response(p_prot, seqId, PROT_OP_UNIT_CONFIG_GET, PROT_STATUS_OK, PROT_UNIT_CONFIG_BYTES);
}

static void handle_unit_config_set(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 48U) {
        send_response(p_prot, seqId, PROT_OP_UNIT_CONFIG_SET, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }
    memcpy(p_prot->unitConfig.unitName, &p_req[0], 16U);
    p_prot->unitConfig.unitName[15] = '\0';
    p_prot->unitConfig.stationId = (p_req[16] >= 1U && p_req[16] <= 247U) ? p_req[16] : 1U;
    p_prot->unitConfig.ipMode    = p_req[17] ? 1U : 0U;
    memcpy(p_prot->unitConfig.staticIp, &p_req[18], 4U);
    memcpy(p_prot->unitConfig.subnetMask, &p_req[22], 4U);
    memcpy(p_prot->unitConfig.gateway, &p_req[26], 4U);
    uint16_t port = read_u16_le(&p_req[30]);
    if (port >= 1U) {
        p_prot->unitConfig.tcpPort = port;
    }
    p_prot->unitConfig.runOutputPin      = p_req[32];
    p_prot->unitConfig.faultOutputPin    = p_req[33];
    p_prot->unitConfig.estopOutputPin    = p_req[34];
    p_prot->unitConfig.bootMode          = p_req[35] ? 1U : 0U;
    p_prot->unitConfig.safeOutputsOnStop = p_req[36] ? 1U : 0U;
    p_prot->unitConfig.watchdogEnabled   = p_req[37] ? 1U : 0U;
    if (reqLen >= PROT_UNIT_CONFIG_BYTES) {
        p_prot->unitConfig.rs485Mode      = (p_req[48] <= (uint8_t)PROT_RS485_MODE_MODBUS_SLAVE) ? p_req[48] : (uint8_t)PROT_RS485_MODE_MARAPROT;
        p_prot->unitConfig.rs485BaudIndex = (p_req[49] <= 7U) ? p_req[49] : 4U;
    }
    p_prot->transport.stationId          = p_prot->unitConfig.stationId;

    if (p_req[38] != '\0') {
        char newPw[11];
        memcpy(newPw, &p_req[38], 10U);
        newPw[10] = '\0';
        strncpy(p_prot->password, newPw, PROT_MAX_PASSWORD_BYTES);
        p_prot->password[PROT_MAX_PASSWORD_BYTES] = '\0';
        p_prot->authRequired = (p_prot->password[0] != '\0');
    }

    p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
    send_response(p_prot, seqId, PROT_OP_UNIT_CONFIG_SET, PROT_STATUS_OK, 1U);

    if (p_prot->transport.unit_config_saved != NULL) {
        p_prot->transport.unit_config_saved(&p_prot->unitConfig, p_prot->password);
    }
}

static void dispatch_frame(ProtInstance_t *p_prot)
{
    uint8_t        verFlags   = p_prot->rxBuf[2];
    uint8_t        stationId  = p_prot->rxBuf[3];
    uint16_t       seqId      = read_u16_le(&p_prot->rxBuf[4]);
    uint8_t        opcode     = p_prot->rxBuf[6];
    uint16_t       payloadLen = p_prot->rxExpectedPayload;
    uint8_t       *p_payload  = &p_prot->rxBuf[PROT_HEADER_BYTES];

    if (p_prot->transport.kind == PROT_TRANSPORT_RS485 &&
        stationId != PROT_STATION_P2P &&
        stationId != PROT_STATION_BROADCAST &&
        stationId != p_prot->transport.stationId) {
        return;
    }

    p_prot->replyEncrypted = false;
    uint16_t crcCalc = PROT_CalcCrc16(p_prot->rxBuf, (uint16_t)(PROT_HEADER_BYTES + payloadLen));
    uint16_t crcRx   = read_u16_le(&p_prot->rxBuf[PROT_HEADER_BYTES + payloadLen]);
    if (crcCalc != crcRx) {
        send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_CRC, 0U);
        return;
    }

    if ((verFlags & PROT_FLAG_ENCRYPTED) != 0U) {
        if (!p_prot->authValid) {
            send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_AUTH, 0U);
            return;
        }
        if (payloadLen > 0U) {
            prot_crypt_payload(p_prot->sessionKey, seqId, PROT_CRYPT_DIR_REQ, p_payload, payloadLen);
        }
        p_prot->replyEncrypted = true;
    }

    if (p_prot->authRequired && !p_prot->authValid &&
        opcode != PROT_OP_PING && opcode != PROT_OP_GET_INFO &&
        opcode != PROT_OP_AUTH_CHALLENGE && opcode != PROT_OP_AUTH_VERIFY) {
        send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_AUTH, 0U);
        return;
    }

    switch (opcode) {
        case PROT_OP_PING:             handle_ping(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_GET_INFO:         handle_get_info(p_prot, seqId); break;
        case PROT_OP_AUTH_CHALLENGE:   handle_auth_challenge(p_prot, seqId); break;
        case PROT_OP_AUTH_VERIFY:      handle_auth_verify(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_STATUS:        handle_vm_status(p_prot, seqId); break;
        case PROT_OP_VM_CONTROL:       handle_vm_control(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_BEGIN:  handle_upload_begin(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_CHUNK:  handle_upload_chunk(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_COMMIT: handle_upload_commit(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_WATCH_SUBSCRIBE:  handle_watch_subscribe(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_WATCH_POLL:       handle_watch_poll(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_SLOT_READ_BATCH:  handle_slot_read_batch(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_SLOT_WRITE_BATCH: handle_slot_write_batch(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_RTC_GET: {
            if (p_prot->transport.rtc_get != NULL) {
                uint64_t raw = (uint64_t)p_prot->transport.rtc_get();
                for (uint8_t i = 0U; i < 8U; i++) {
                    p_prot->txBuf[PROT_HEADER_BYTES + i] = (uint8_t)((raw >> (i * 8U)) & 0xFFU);
                }
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, 8U);
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_RTC_SET: {
            if (payloadLen < 8U) {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
            } else if (p_prot->transport.rtc_set != NULL) {
                uint64_t raw = 0U;
                for (uint8_t i = 0U; i < 8U; i++) {
                    raw |= ((uint64_t)p_payload[i]) << (i * 8U);
                }
                bool ok = p_prot->transport.rtc_set((int64_t)raw);
                send_response(p_prot, seqId, opcode, ok ? PROT_STATUS_OK : PROT_STATUS_ERR_PARAM, 0U);
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_ONEWIRE_SCAN: {
            if (p_prot->transport.onewire_scan != NULL) {
                uint8_t dioPin = (payloadLen >= 1U) ? p_payload[0] : 13U;
                uint8_t roms[8U * 8U];
                uint8_t count = p_prot->transport.onewire_scan(dioPin, roms, 8U);
                if (count > 8U) { count = 8U; }
                p_prot->txBuf[PROT_HEADER_BYTES] = count;
                if (count > 0U) {
                    memcpy(&p_prot->txBuf[PROT_HEADER_BYTES + 1U], roms, (size_t)count * 8U);
                }
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, (uint16_t)(1U + (count * 8U)));
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_UNIT_CONFIG_GET:  handle_unit_config_get(p_prot, seqId); break;
        case PROT_OP_UNIT_CONFIG_SET:  handle_unit_config_set(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_MODBUS_MAP_GET: {
            if (p_prot->transport.modbus_map_get != NULL) {
                ProtModbusMapEntry_t entries[PROT_MAX_MODBUS_MAP_ENTRIES];
                uint8_t count = p_prot->transport.modbus_map_get(entries, PROT_MAX_MODBUS_MAP_ENTRIES);
                if (count > PROT_MAX_MODBUS_MAP_ENTRIES) {
                    count = PROT_MAX_MODBUS_MAP_ENTRIES;
                }
                uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
                p_out[0] = count;
                for (uint8_t i = 0U; i < count; i++) {
                    uint16_t off = (uint16_t)(1U + ((uint16_t)i * 8U));
                    p_out[off + 0U] = entries[i].table;
                    p_out[off + 1U] = entries[i].targetKind;
                    write_u16_le(&p_out[off + 2U], entries[i].modbusAddr);
                    write_u16_le(&p_out[off + 4U], entries[i].targetIdx);
                    p_out[off + 6U] = entries[i].flags;
                    p_out[off + 7U] = entries[i].reserved;
                }
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, (uint16_t)(1U + ((uint16_t)count * 8U)));
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_MODBUS_MAP_SET: {
            if (p_prot->transport.modbus_map_set == NULL) {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
                break;
            }
            if (payloadLen < 1U) {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
                break;
            }
            uint8_t count = p_payload[0];
            if (count > PROT_MAX_MODBUS_MAP_ENTRIES || payloadLen < (uint16_t)(1U + ((uint16_t)count * 8U))) {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
                break;
            }
            ProtModbusMapEntry_t entries[PROT_MAX_MODBUS_MAP_ENTRIES];
            for (uint8_t i = 0U; i < count; i++) {
                uint16_t off = (uint16_t)(1U + ((uint16_t)i * 8U));
                entries[i].table      = p_payload[off + 0U];
                entries[i].targetKind = p_payload[off + 1U];
                entries[i].modbusAddr = read_u16_le(&p_payload[off + 2U]);
                entries[i].targetIdx  = read_u16_le(&p_payload[off + 4U]);
                entries[i].flags      = p_payload[off + 6U];
                entries[i].reserved   = p_payload[off + 7U];
            }
            if (p_prot->transport.modbus_map_set(entries, count)) {
                p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, 1U);
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_STORAGE, 0U);
            }
            break;
        }
        case PROT_OP_FW_ENTER_ROM_DFU: {
            if (payloadLen >= 4U && read_u32_le(p_payload) == PROT_DFU_MAGIC && p_prot->transport.enter_dfu != NULL) {
                p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, 1U);
                p_prot->transport.enter_dfu();
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
            }
            break;
        }
        default:
            send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            break;
    }
}

uint16_t PROT_CalcCrc16(const uint8_t *p_data, uint16_t lenBytes)
{
    uint16_t crc = PROT_CRC16_INIT;
    for (uint16_t i = 0U; i < lenBytes; i++) {
        crc ^= (uint16_t)p_data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ PROT_CRC16_POLY);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

bool PROT_Init(ProtInstance_t *p_prot, VmInstance_t *p_vm, const ProtTransport_t *p_transport)
{
    if (p_prot == NULL || p_vm == NULL || p_transport == NULL) {
        return false;
    }
    memset(p_prot, 0, sizeof(*p_prot));
    p_prot->p_vm      = p_vm;
    p_prot->transport = *p_transport;
    if (p_prot->transport.maxMtuBytes == 0U || p_prot->transport.maxMtuBytes > PROT_MAX_PAYLOAD_BYTES) {
        p_prot->transport.maxMtuBytes = PROT_MAX_PAYLOAD_BYTES;
    }
    p_prot->rxState = PROT_RX_SOF0;
    const char *p_pw = (p_transport->p_password != NULL) ? p_transport->p_password : "admin";
    strncpy(p_prot->password, p_pw, PROT_MAX_PASSWORD_BYTES);
    p_prot->password[PROT_MAX_PASSWORD_BYTES] = '\0';
    p_prot->authRequired = (p_prot->password[0] != '\0');
    p_prot->authValid    = !p_prot->authRequired;
    strncpy(p_prot->unitConfig.unitName, "PLC-1", 15U);
    p_prot->unitConfig.stationId         = p_prot->transport.stationId ? p_prot->transport.stationId : 1U;
    p_prot->unitConfig.ipMode            = 0U;
    p_prot->unitConfig.staticIp[0]       = 192U; p_prot->unitConfig.staticIp[1] = 168U;
    p_prot->unitConfig.staticIp[2]       = 1U;   p_prot->unitConfig.staticIp[3] = 50U;
    p_prot->unitConfig.subnetMask[0]     = 255U; p_prot->unitConfig.subnetMask[1] = 255U;
    p_prot->unitConfig.subnetMask[2]     = 255U; p_prot->unitConfig.subnetMask[3] = 0U;
    p_prot->unitConfig.gateway[0]        = 192U; p_prot->unitConfig.gateway[1] = 168U;
    p_prot->unitConfig.gateway[2]        = 1U;   p_prot->unitConfig.gateway[3] = 1U;
    p_prot->unitConfig.tcpPort           = 5020U;
    p_prot->unitConfig.runOutputPin      = 0xFFU;
    p_prot->unitConfig.faultOutputPin    = 0xFFU;
    p_prot->unitConfig.estopOutputPin    = 0xFFU;
    p_prot->unitConfig.bootMode          = 0U;
    p_prot->unitConfig.safeOutputsOnStop = 1U;
    p_prot->unitConfig.watchdogEnabled   = 1U;
    p_prot->unitConfig.rs485Mode         = (uint8_t)PROT_RS485_MODE_MARAPROT;
    p_prot->unitConfig.rs485BaudIndex    = 4U;
    return true;
}

void PROT_FeedByte(ProtInstance_t *p_prot, uint8_t byte)
{
    if (p_prot == NULL) {
        return;
    }

    switch (p_prot->rxState) {
        case PROT_RX_SOF0:
            if (byte == PROT_MAGIC_0) {
                p_prot->rxBuf[0] = byte;
                p_prot->rxState  = PROT_RX_SOF1;
            }
            break;

        case PROT_RX_SOF1:
            if (byte == PROT_MAGIC_1) {
                p_prot->rxBuf[1] = byte;
                p_prot->rxPos    = 2U;
                p_prot->rxState  = PROT_RX_HEADER;
            } else if (byte != PROT_MAGIC_0) {
                p_prot->rxState  = PROT_RX_SOF0;
            }
            break;

        case PROT_RX_HEADER:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            if (p_prot->rxPos >= PROT_HEADER_BYTES) {
                p_prot->rxExpectedPayload = read_u16_le(&p_prot->rxBuf[8]);
                if (p_prot->rxExpectedPayload > PROT_MAX_PAYLOAD_BYTES) {
                    p_prot->rxState = PROT_RX_SOF0;
                } else if (p_prot->rxExpectedPayload == 0U) {
                    p_prot->rxState = PROT_RX_CRC0;
                } else {
                    p_prot->rxState = PROT_RX_PAYLOAD;
                }
            }
            break;

        case PROT_RX_PAYLOAD:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            if (p_prot->rxPos >= (uint16_t)(PROT_HEADER_BYTES + p_prot->rxExpectedPayload)) {
                p_prot->rxState = PROT_RX_CRC0;
            }
            break;

        case PROT_RX_CRC0:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            p_prot->rxState = PROT_RX_CRC1;
            break;

        case PROT_RX_CRC1:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            p_prot->rxState = PROT_RX_SOF0;
            dispatch_frame(p_prot);
            break;

        default:
            p_prot->rxState = PROT_RX_SOF0;
            break;
    }
}

void PROT_FeedBuffer(ProtInstance_t *p_prot, const uint8_t *p_data, uint16_t lenBytes)
{
    if (p_prot == NULL || p_data == NULL) {
        return;
    }
    for (uint16_t i = 0U; i < lenBytes; i++) {
        PROT_FeedByte(p_prot, p_data[i]);
    }
}
