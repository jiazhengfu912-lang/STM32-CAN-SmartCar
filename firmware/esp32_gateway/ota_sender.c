#include "ota_sender.h"
#include "boot_guard.h"
#include "car_protocol.h"
#include <string.h>

/** 按小端顺序读取 32 位整数，避免未对齐访问。 */
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8U | (uint32_t)p[2] << 16U | (uint32_t)p[3] << 24U;
}
/** 将 32 位整数编码为小端字节序。 */
static void put32(uint8_t *p, uint32_t v)
{
    unsigned i;
    for (i = 0; i < 4U; ++i) p[i] = (uint8_t)(v >> (8U * i));
}

/** 发送带序号的 OTA 请求，限次重试并检查应答类型及会话。 */
static int rpc(const sender_link *link, sender_report *report, uint8_t type, uint16_t seq,
               const uint8_t *data, uint16_t size, uint8_t reply[16], int check_session)
{
    unsigned attempt;
    report->stage = type;
    for (attempt = 0; attempt < 3U; ++attempt) {
        int result;
        if (attempt) ++report->retries;
        result = link->exchange(link->context, type, seq, data, size, reply,
                                type == CAR_MSG_OTA_BEGIN ? 3000U : 1000U);
        if (result < 0) return SEND_IO;
        if (!result) continue;
        if (reply[2] != type || reply[3] || (check_session && get32(reply + 4U) != report->session))
            return SEND_BAD_REPLY;
        report->remote_status = reply[0];
        return reply[0] ? SEND_REJECTED : SEND_OK;
    }
    return SEND_TIMEOUT;
}

/** 依次完成进入 Bootloader、分块写入、提交、复位和运行版本确认。 */
int sender_install(const sender_link *link, const cache_info *info, const uint8_t *image,
                   uint32_t session, sender_report *report)
{
    uint8_t header[32], payload[136], reply[16];
    uint16_t sequence = 1;
    uint32_t marker, offset;
    int result = SEND_INVALID_IMAGE;
    char target[4] = {'S','C','0','1'};
    memset(report, 0, sizeof(*report));
    report->session = session;
    report->version = info->version;
    /* 第二层门槛：哪怕调用者漏查缓存，也不能把坏镜像送进 BEGIN。 */
    if (!session || (info->version < 100U) ||
        info->size < 258U || info->size > BOOT_APP_CAPACITY) goto done;
    put32(header, BOOT_MAGIC);
    put32(header + 4U, BOOT_FORMAT);
    put32(header + 8U, BOOT_APP_BASE);
    put32(header + 12U, info->size);
    put32(header + 16U, info->crc);
    put32(header + 20U, info->version);
    put32(header + 24U, car_crc32(header, 24U));
    put32(header + 28U, BOOT_COMMITTED);
    if (boot_validate(header, image, info->size) != BOOT_VALID) goto done;

    marker = link->mark(link->context);
    result = rpc(link, report, CAR_MSG_ENTER_UPDATE, sequence++, (const uint8_t *)"OTA1", 4U, reply, 0);
    if (result != SEND_OK && result != SEND_TIMEOUT) goto done;
    if (!link->wait_identity(link->context, "M2B2", marker, 6000U)) { result = SEND_NO_BOOT; goto done; }
    put32(payload, session);
    put32(payload + 4U, BOOT_APP_BASE);
    put32(payload + 8U, info->size);
    put32(payload + 12U, info->crc);
    put32(payload + 16U, info->version);
    result = rpc(link, report, CAR_MSG_OTA_BEGIN, sequence++, payload, 20U, reply, 1);
    if (result != SEND_OK) goto done;
    if (reply[1] != 1U || get32(reply + 8U) || get32(reply + 12U) != info->version) {
        result = SEND_BAD_REPLY; goto done;
    }
    for (offset = 0; offset < info->size;) {
        uint16_t count = (uint16_t)(info->size - offset > 128U ? 128U : info->size - offset);
        put32(payload, session);
        put32(payload + 4U, offset);
        memcpy(payload + 8U, image + offset, count);
        result = rpc(link, report, CAR_MSG_OTA_DATA, sequence++, payload, (uint16_t)(count + 8U), reply, 1);
        if (result != SEND_OK) goto done;
        if (reply[1] != 1U || get32(reply + 8U) != offset + count || get32(reply + 12U) != info->version) {
            result = SEND_BAD_REPLY; goto done;
        }
        offset += count;
        report->transferred = offset;
        ++report->blocks;
    }
    put32(payload, session);
    result = rpc(link, report, CAR_MSG_OTA_END, sequence++, payload, 4U, reply, 1);
    if (result != SEND_OK) goto done;
    if (reply[1] != 2U || get32(reply + 8U) != info->size || get32(reply + 12U) != info->version) {
        result = SEND_BAD_REPLY; goto done;
    }
    marker = link->mark(link->context);
    result = rpc(link, report, CAR_MSG_OTA_RESET, sequence++, payload, 4U, reply, 1);
    if (result != SEND_OK && result != SEND_TIMEOUT) goto done;
    if (result == SEND_OK && (reply[1] != 2U || get32(reply + 8U) != info->size ||
                             get32(reply + 12U) != info->version)) { result = SEND_BAD_REPLY; goto done; }
    if (!link->wait_identity(link->context, target, marker, 6000U)) { result = SEND_NO_APP; goto done; }
    result = rpc(link, report, CAR_MSG_OTA_QUERY, sequence, NULL, 0U, reply, 0);
    if (result != SEND_OK) goto done;
    if (reply[1] != 4U || get32(reply + 12U) != info->version) { result = SEND_BAD_REPLY; goto done; }
    report->running_version = get32(reply + 12U);
done:
    report->result = (uint32_t)result;
    return result;
}
