#ifndef OTA_SENDER_H
#define OTA_SENDER_H
#include "cache_store.h"

enum { SEND_OK, SEND_INVALID_IMAGE, SEND_TIMEOUT, SEND_BAD_REPLY, SEND_REJECTED,
       SEND_NO_BOOT, SEND_NO_APP, SEND_IO };
typedef struct {
    /* exchange 只返回匹配 type/seq 的 16 字节 ACK：1 成功、0 超时、-1 I/O 失败。 */
    int (*exchange)(void *ctx, uint8_t type, uint16_t seq, const uint8_t *data,
                    uint16_t size, uint8_t reply[16], uint32_t timeout_ms);
    uint32_t (*mark)(void *ctx);
    int (*wait_identity)(void *ctx, const char tag[4], uint32_t after, uint32_t timeout_ms);
    void *context;
} sender_link;
typedef struct {
    uint32_t result, stage, transferred, retries, blocks, session, version,
             remote_status, running_version;
} sender_report;

/* 同步、有限重试；image 必须在调用期间保持不变。只更新当前无运动诊断 App。 */
int sender_install(const sender_link *link, const cache_info *info, const uint8_t *image,
                   uint32_t session, sender_report *report);
#endif
