// Non-blocking LwIP raw-TCP HTTP/1.1 REST client for RestGet and RestPost VM blocks.

#include "rest_client.h"
#include "stm32f4xx_hal.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    REST_STATE_IDLE = 0,
    REST_STATE_PENDING = 1,
    REST_STATE_DONE = 2
} RestSlotState_t;

typedef struct {
    bool                 inUse;
    uint16_t             slotKey;
    volatile uint8_t     state;
    volatile uint32_t    gen;
    bool                 isPost;
    char                 url[REST_MAX_URL_LEN + 1U];
    char                 jsonKey[REST_MAX_KEY_LEN + 1U];
    char                 host[96];
    char                 path[128];
    uint16_t             port;
    uint16_t             timeoutMs;
    float                txVal;
    uint32_t             startTickMs;
    struct tcp_pcb      *p_pcb;
    char                 rxBuf[REST_RX_BUF_BYTES];
    uint16_t             rxLen;
    float                lastVal;
    int32_t              lastStatus;
} RestSlot_t;

static RestSlot_t g_rest_slots[REST_MAX_SLOTS];

static RestSlot_t *find_or_alloc_slot(uint16_t slotKey)
{
    for (uint8_t i = 0U; i < REST_MAX_SLOTS; i++) {
        if (g_rest_slots[i].inUse && g_rest_slots[i].slotKey == slotKey) {
            return &g_rest_slots[i];
        }
    }
    for (uint8_t i = 0U; i < REST_MAX_SLOTS; i++) {
        if (!g_rest_slots[i].inUse) {
            memset(&g_rest_slots[i], 0, sizeof(RestSlot_t));
            g_rest_slots[i].inUse   = true;
            g_rest_slots[i].slotKey = slotKey;
            return &g_rest_slots[i];
        }
    }
    return NULL;
}

static void parse_http_url(RestSlot_t *p_slot)
{
    const char *p = p_slot->url;
    if (strncmp(p, "http://", 7U) == 0) {
        p += 7U;
    } else if (strncmp(p, "https://", 8U) == 0) {
        p += 8U;
    }

    const char *p_slash = strchr(p, '/');
    size_t hostPortLen = (p_slash != NULL) ? (size_t)(p_slash - p) : strlen(p);
    if (hostPortLen >= sizeof(p_slot->host)) {
        hostPortLen = sizeof(p_slot->host) - 1U;
    }
    memcpy(p_slot->host, p, hostPortLen);
    p_slot->host[hostPortLen] = '\0';

    p_slot->port = 80U;
    char *p_colon = strchr(p_slot->host, ':');
    if (p_colon != NULL) {
        *p_colon = '\0';
        long parsedPort = strtol(p_colon + 1, NULL, 10);
        if (parsedPort >= 1L && parsedPort <= 65535L) {
            p_slot->port = (uint16_t)parsedPort;
        }
    }

    if (p_slash != NULL && p_slash[0] != '\0') {
        strncpy(p_slot->path, p_slash, sizeof(p_slot->path) - 1U);
        p_slot->path[sizeof(p_slot->path) - 1U] = '\0';
    } else {
        strcpy(p_slot->path, "/");
    }
}

static bool parse_json_float(const char *p_body, const char *p_key, float *p_out)
{
    if (p_body == NULL || p_out == NULL) {
        return false;
    }
    const char *p_search = p_body;
    if (p_key != NULL && p_key[0] != '\0') {
        char pattern[REST_MAX_KEY_LEN + 4U];
        snprintf(pattern, sizeof(pattern), "\"%s\"", p_key);
        const char *p_found = strstr(p_body, pattern);
        if (p_found == NULL) {
            return false;
        }
        p_search = strchr(p_found + strlen(pattern), ':');
        if (p_search == NULL) {
            return false;
        }
        p_search++;
    }
    while (*p_search != '\0' && (isspace((unsigned char)*p_search) || *p_search == '"')) {
        p_search++;
    }
    char *p_end = NULL;
    float val = strtof(p_search, &p_end);
    if (p_end == p_search) {
        return false;
    }
    *p_out = val;
    return true;
}

static void parse_http_response(RestSlot_t *p_slot)
{
    p_slot->rxBuf[p_slot->rxLen] = '\0';
    int32_t status = -1;
    if (strncmp(p_slot->rxBuf, "HTTP/", 5U) == 0) {
        const char *p_sp = strchr(p_slot->rxBuf, ' ');
        if (p_sp != NULL) {
            status = (int32_t)strtol(p_sp + 1, NULL, 10);
        }
    }
    p_slot->lastStatus = status;

    if (!p_slot->isPost) {
        const char *p_body = strstr(p_slot->rxBuf, "\r\n\r\n");
        if (p_body != NULL) {
            p_body += 4U;
            float parsed = 0.0f;
            if (parse_json_float(p_body, p_slot->jsonKey, &parsed)) {
                p_slot->lastVal = parsed;
            }
        }
    }
}

static void close_pcb(RestSlot_t *p_slot)
{
    if (p_slot->p_pcb != NULL) {
        tcp_arg(p_slot->p_pcb, NULL);
        tcp_recv(p_slot->p_pcb, NULL);
        tcp_err(p_slot->p_pcb, NULL);
        if (tcp_close(p_slot->p_pcb) != ERR_OK) {
            tcp_abort(p_slot->p_pcb);
        }
        p_slot->p_pcb = NULL;
    }
}

static void rest_tcp_err(void *arg, err_t err)
{
    (void)err;
    RestSlot_t *p_slot = (RestSlot_t *)arg;
    if (p_slot != NULL) {
        p_slot->p_pcb = NULL;
        p_slot->lastStatus = -1;
        p_slot->state = REST_STATE_DONE;
    }
}

static err_t rest_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    RestSlot_t *p_slot = (RestSlot_t *)arg;
    if (p_slot == NULL || err != ERR_OK) {
        if (p != NULL) { pbuf_free(p); }
        return ERR_OK;
    }
    if (p == NULL) {
        parse_http_response(p_slot);
        close_pcb(p_slot);
        p_slot->state = REST_STATE_DONE;
        return ERR_OK;
    }

    uint16_t space = (uint16_t)((REST_RX_BUF_BYTES - 1U) - p_slot->rxLen);
    uint16_t toCopy = (p->tot_len < space) ? p->tot_len : space;
    if (toCopy > 0U) {
        pbuf_copy_partial(p, &p_slot->rxBuf[p_slot->rxLen], toCopy, 0U);
        p_slot->rxLen = (uint16_t)(p_slot->rxLen + toCopy);
    }
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t rest_tcp_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    RestSlot_t *p_slot = (RestSlot_t *)arg;
    if (p_slot == NULL || err != ERR_OK) {
        if (p_slot != NULL) {
            close_pcb(p_slot);
            p_slot->lastStatus = -1;
            p_slot->state = REST_STATE_DONE;
        }
        return ERR_OK;
    }

    char txBuf[512];
    int len = 0;
    if (p_slot->isPost) {
        char body[128];
        if (p_slot->jsonKey[0] != '\0') {
            snprintf(body, sizeof(body), "{\"%s\":%.4f}", p_slot->jsonKey, (double)p_slot->txVal);
        } else {
            snprintf(body, sizeof(body), "%.4f", (double)p_slot->txVal);
        }
        len = snprintf(txBuf, sizeof(txBuf),
                       "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: application/json\r\nContent-Length: %u\r\nConnection: close\r\n\r\n%s",
                       p_slot->path, p_slot->host, (unsigned)strlen(body), body);
    } else {
        len = snprintf(txBuf, sizeof(txBuf),
                       "GET %s HTTP/1.1\r\nHost: %s\r\nAccept: application/json\r\nConnection: close\r\n\r\n",
                       p_slot->path, p_slot->host);
    }

    if (len <= 0 || len >= (int)sizeof(txBuf) ||
        tcp_write(tpcb, txBuf, (u16_t)len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        close_pcb(p_slot);
        p_slot->lastStatus = -1;
        p_slot->state = REST_STATE_DONE;
        return ERR_OK;
    }
    tcp_output(tpcb);
    return ERR_OK;
}

static void start_tcp_connect(RestSlot_t *p_slot, const ip_addr_t *p_ip)
{
    struct tcp_pcb *p_pcb = tcp_new();
    if (p_pcb == NULL) {
        p_slot->lastStatus = -1;
        p_slot->state = REST_STATE_DONE;
        return;
    }
    p_slot->p_pcb = p_pcb;
    p_slot->rxLen = 0U;
    tcp_arg(p_pcb, p_slot);
    tcp_err(p_pcb, rest_tcp_err);
    tcp_recv(p_pcb, rest_tcp_recv);
    if (tcp_connect(p_pcb, p_ip, p_slot->port, rest_tcp_connected) != ERR_OK) {
        close_pcb(p_slot);
        p_slot->lastStatus = -1;
        p_slot->state = REST_STATE_DONE;
    }
}

static void rest_dns_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    (void)name;
    RestSlot_t *p_slot = (RestSlot_t *)callback_arg;
    if (p_slot == NULL || p_slot->state != REST_STATE_PENDING) {
        return;
    }
    if (ipaddr == NULL) {
        p_slot->lastStatus = -1;
        p_slot->state = REST_STATE_DONE;
        return;
    }
    start_tcp_connect(p_slot, ipaddr);
}

static void rest_start_cb(void *arg)
{
    RestSlot_t *p_slot = (RestSlot_t *)arg;
    if (p_slot == NULL || p_slot->state != REST_STATE_PENDING) {
        return;
    }
    close_pcb(p_slot);
    ip_addr_t ip;
    if (ip4addr_aton(p_slot->host, ip_2_ip4(&ip))) {
        IP_SET_TYPE_VAL(ip, IPADDR_TYPE_V4);
        start_tcp_connect(p_slot, &ip);
    } else {
        err_t dnsErr = dns_gethostbyname(p_slot->host, &ip, rest_dns_cb, p_slot);
        if (dnsErr == ERR_OK) {
            start_tcp_connect(p_slot, &ip);
        } else if (dnsErr != ERR_INPROGRESS) {
            p_slot->lastStatus = -1;
            p_slot->state = REST_STATE_DONE;
        }
    }
}

static void rest_abort_cb(void *arg)
{
    RestSlot_t *p_slot = (RestSlot_t *)arg;
    if (p_slot != NULL) {
        close_pcb(p_slot);
    }
}

void REST_ResetAll(void)
{
    for (uint8_t i = 0U; i < REST_MAX_SLOTS; i++) {
        if (g_rest_slots[i].p_pcb != NULL) {
            tcpip_callback(rest_abort_cb, &g_rest_slots[i]);
        }
        g_rest_slots[i].inUse = false;
        g_rest_slots[i].state = REST_STATE_IDLE;
    }
}

void REST_Exec(uint16_t slotKey, bool isPost, const RestReqParams_t *p_req, bool ethReady,
               float *p_outVal, int32_t *p_httpStatus, bool *p_ready)
{
    if (!ethReady || p_req == NULL || p_req->p_url == NULL || p_req->urlLen == 0U) {
        if (p_outVal != NULL)     { *p_outVal = 0.0f; }
        if (p_httpStatus != NULL) { *p_httpStatus = -2; }
        if (p_ready != NULL)      { *p_ready = false; }
        return;
    }

    RestSlot_t *p_slot = find_or_alloc_slot(slotKey);
    if (p_slot == NULL) {
        if (p_outVal != NULL)     { *p_outVal = 0.0f; }
        if (p_httpStatus != NULL) { *p_httpStatus = -1; }
        if (p_ready != NULL)      { *p_ready = false; }
        return;
    }

    uint32_t nowMs = HAL_GetTick();
    if (p_slot->state == REST_STATE_IDLE) {
        uint8_t uLen = (p_req->urlLen > REST_MAX_URL_LEN) ? REST_MAX_URL_LEN : p_req->urlLen;
        uint8_t kLen = (p_req->keyLen > REST_MAX_KEY_LEN) ? REST_MAX_KEY_LEN : p_req->keyLen;
        memcpy(p_slot->url, p_req->p_url, uLen);
        p_slot->url[uLen] = '\0';
        if (p_req->p_jsonKey != NULL && kLen > 0U) {
            memcpy(p_slot->jsonKey, p_req->p_jsonKey, kLen);
        }
        p_slot->jsonKey[kLen] = '\0';
        parse_http_url(p_slot);
        p_slot->isPost      = isPost;
        p_slot->timeoutMs   = (p_req->timeoutMs >= 100U) ? p_req->timeoutMs : 1000U;
        p_slot->txVal       = p_req->txFloatVal;
        p_slot->startTickMs = nowMs;
        p_slot->gen++;
        p_slot->state       = REST_STATE_PENDING;
        tcpip_callback(rest_start_cb, p_slot);

        if (p_outVal != NULL)     { *p_outVal = p_slot->lastVal; }
        if (p_httpStatus != NULL) { *p_httpStatus = p_slot->lastStatus; }
        if (p_ready != NULL)      { *p_ready = false; }
    } else if (p_slot->state == REST_STATE_PENDING) {
        if ((nowMs - p_slot->startTickMs) >= p_slot->timeoutMs) {
            tcpip_callback(rest_abort_cb, p_slot);
            p_slot->lastStatus = -1;
            p_slot->state      = REST_STATE_IDLE;
            if (p_outVal != NULL)     { *p_outVal = p_slot->lastVal; }
            if (p_httpStatus != NULL) { *p_httpStatus = -1; }
            if (p_ready != NULL)      { *p_ready = true; }
        } else {
            if (p_outVal != NULL)     { *p_outVal = p_slot->lastVal; }
            if (p_httpStatus != NULL) { *p_httpStatus = p_slot->lastStatus; }
            if (p_ready != NULL)      { *p_ready = false; }
        }
    } else {
        // REST_STATE_DONE
        p_slot->state = REST_STATE_IDLE;
        if (p_outVal != NULL)     { *p_outVal = p_slot->lastVal; }
        if (p_httpStatus != NULL) { *p_httpStatus = p_slot->lastStatus; }
        if (p_ready != NULL)      { *p_ready = true; }
    }
}
