#include "ota_receiver.h"
#include <string.h>

/** 读取 OTA 消息中的小端 32 位字段。 */
uint32_t ota_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

/** 写入 OTA 消息中的小端 32 位字段。 */
void ota_put32(uint8_t *p, uint32_t value)
{
    unsigned i;
    for (i = 0; i < 4U; ++i) p[i] = (uint8_t)(value >> (8U * i));
}

/** 清空升级会话并绑定受限 Flash 操作接口。 */
void ota_init(ota_receiver *rx, const ota_storage *storage)
{
    memset(rx, 0, sizeof(*rx));
    rx->storage = storage;
}

/** 升级会话长时间无有效数据时标记超时失败。 */
void ota_poll(ota_receiver *rx, uint32_t now)
{
    if (rx->state == OTA_RECEIVING &&
        (uint32_t)(now - rx->last_activity) >= OTA_IDLE_TIMEOUT_MS)
        rx->state = OTA_FAILED;
}

/** 验证目标分区和会话，先撤销有效记录，再擦除 App；重复请求不重复擦除。 */
static uint8_t begin(ota_receiver *rx, uint16_t seq, const uint8_t *p, uint16_t n)
{
    uint32_t session, length, crc, version;
    if (n != 20U) return OTA_BAD_ARGUMENT;
    session = ota_get32(p);
    length = ota_get32(p + 8U);
    crc = ota_get32(p + 12U);
    version = ota_get32(p + 16U);
    if (session == 0U || ota_get32(p + 4U) != BOOT_APP_BASE ||
        length < 258U || length > BOOT_APP_CAPACITY || version == 0U)
        return OTA_BAD_ARGUMENT;
    if (session == rx->session)
    {
        if (length != rx->length || crc != rx->crc || version != rx->version ||
            seq != rx->begin_seq) return OTA_CONFLICT;
        /* BEGIN 的 ACK 丢失不能再次擦除已写内容。失败会话必须换 session。 */
        return (rx->state == OTA_RECEIVING || rx->state == OTA_COMMITTED) ? OTA_OK : OTA_BAD_STATE;
    }
    if (rx->state == OTA_RECEIVING) return OTA_BUSY;
    rx->session = session;
    rx->length = length;
    rx->crc = crc;
    rx->version = version;
    rx->begin_seq = seq;
    rx->next = 0U;
    rx->last_size = 0U;
    rx->state = OTA_FAILED;
    if (!rx->storage->invalidate()) return OTA_IO_ERROR;
    if (!rx->storage->erase(length)) return OTA_IO_ERROR;
    rx->state = OTA_RECEIVING;
    return OTA_OK;
}

/** 校验会话、偏移和长度后写入数据，允许相同尾块的幂等重传。 */
static uint8_t block(ota_receiver *rx, uint16_t seq, const uint8_t *p, uint16_t n)
{
    uint32_t offset;
    uint16_t count;
    if (n <= 8U || n > 8U + OTA_BLOCK_SIZE) return OTA_BAD_ARGUMENT;
    if (ota_get32(p) != rx->session || rx->session == 0U) return OTA_BAD_SESSION;
    if (rx->state != OTA_RECEIVING) return OTA_BAD_STATE;
    offset = ota_get32(p + 4U);
    count = (uint16_t)(n - 8U);
    if (offset > rx->length || count > rx->length - offset || (offset & 1U) != 0U ||
        ((count & 1U) != 0U && offset + count != rx->length)) return OTA_BAD_OFFSET;
    if (rx->last_size != 0U && offset == rx->last_offset)
    {
        if (seq != rx->last_seq || count != rx->last_size ||
            !rx->storage->equal(offset, p + 8U, count)) return OTA_CONFLICT;
        return OTA_OK;
    }
    if (offset != rx->next) return OTA_BAD_OFFSET;
    if (!rx->storage->write(offset, p + 8U, count))
    {
        rx->state = OTA_FAILED;
        return OTA_IO_ERROR;
    }
    rx->last_offset = offset;
    rx->last_seq = seq;
    rx->last_size = count;
    rx->next += count;
    return OTA_OK;
}

/** 执行结束校验或复位许可判断，拒绝状态不匹配的控制请求。 */
static uint8_t command(ota_receiver *rx, uint8_t type, uint16_t seq,
                       const uint8_t *p, uint16_t n)
{
    if (type == CAR_MSG_OTA_QUERY) return n == 0U ? OTA_OK : OTA_BAD_ARGUMENT;
    if (type == CAR_MSG_OTA_BEGIN) return begin(rx, seq, p, n);
    if (type == CAR_MSG_OTA_DATA) return block(rx, seq, p, n);
    if (type != CAR_MSG_OTA_END && type != CAR_MSG_OTA_RESET) return OTA_BAD_ARGUMENT;
    if (n != 4U) return OTA_BAD_ARGUMENT;
    if (rx->session == 0U || ota_get32(p) != rx->session) return OTA_BAD_SESSION;
    if (type == CAR_MSG_OTA_RESET) return rx->state == OTA_COMMITTED ? OTA_OK : OTA_BAD_STATE;
    if (rx->state == OTA_COMMITTED) return seq == rx->end_seq ? OTA_OK : OTA_CONFLICT;
    if (rx->state != OTA_RECEIVING || rx->next != rx->length) return OTA_BAD_STATE;
    rx->state = OTA_FAILED;
    if (!rx->storage->finish(rx->length, rx->crc, rx->version)) return OTA_INVALID_IMAGE;
    rx->end_seq = seq;
    rx->state = OTA_COMMITTED;
    return OTA_OK;
}

/** 分发 OTA 命令并生成固定 16 字节应答，携带下一期待偏移和版本。 */
uint8_t ota_handle(ota_receiver *rx, uint8_t type, uint16_t seq,
                   const uint8_t *data, uint16_t size, uint32_t now, uint8_t reply[16])
{
    uint8_t result;
    ota_poll(rx, now);
    result = command(rx, type, seq, data, size);
    if (result == OTA_OK && type != CAR_MSG_OTA_QUERY) rx->last_activity = now;
    reply[0] = result;
    reply[1] = rx->state;
    reply[2] = type;
    reply[3] = 0U;
    ota_put32(reply + 4U, rx->session);
    ota_put32(reply + 8U, rx->next);
    ota_put32(reply + 12U, rx->version);
    return result;
}
