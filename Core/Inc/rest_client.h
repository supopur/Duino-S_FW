// Non-blocking LwIP raw-TCP HTTP/1.1 REST client for RestGet and RestPost VM blocks.

#ifndef REST_CLIENT_H
#define REST_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define REST_MAX_SLOTS      8U
#define REST_MAX_URL_LEN    200U
#define REST_MAX_KEY_LEN    64U
#define REST_RX_BUF_BYTES   512U

typedef struct {
    const char *p_url;
    uint8_t     urlLen;
    const char *p_jsonKey;
    uint8_t     keyLen;
    uint16_t    timeoutMs;
    float       txFloatVal;
} RestReqParams_t;

void REST_ResetAll(void);
void REST_Exec(uint16_t slotKey, bool isPost, const RestReqParams_t *p_req, bool ethReady,
               float *p_outVal, int32_t *p_httpStatus, bool *p_ready);

#ifdef __cplusplus
}
#endif

#endif // REST_CLIENT_H
