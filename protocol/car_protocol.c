#include "car_protocol.h"
#include <string.h>

/** 按小端顺序读取 16 位字段。 */
static uint16_t read16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

/** 按小端顺序读取 32 位字段。 */
static uint32_t read32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

/** 写入小端 16 位字段。 */
static void write16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
}

/** 写入小端 32 位字段。 */
static void write32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U);
    p[3] = (uint8_t)(value >> 24U);
}

/** 计算协议与固件使用的 CRC32，用于检测传输或存储损坏。 */
uint32_t car_crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t i;
    unsigned bit;
    for (i = 0U; i < size; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/** 检查载荷和输出容量后组装帧头、序号、正文与 CRC。 */
size_t car_encode(uint8_t *output, size_t capacity, uint8_t type, uint16_t seq,
                  const uint8_t *payload, uint16_t size)
{
    size_t total = (size_t)size + CAR_FRAME_OVERHEAD;
    if (output == NULL || size > CAR_MAX_PAYLOAD || capacity < total ||
        (size != 0U && payload == NULL))
    {
        return 0U;
    }
    output[0] = 0xA5U;
    output[1] = 0x5AU;
    output[2] = CAR_VERSION;
    output[3] = type;
    write16(output + 4U, seq);
    write16(output + 6U, size);
    if (size != 0U)
    {
        memcpy(output + 8U, payload, size);
    }
    write32(output + 8U + size, car_crc32(output + 2U, 6U + size));
    return total;
}

/** 移除已消费或失效的接收字节，保留后续候选帧。 */
static void discard(car_receiver *rx, uint16_t count)
{
    rx->used = (uint16_t)(rx->used - count);
    memmove(rx->buffer, rx->buffer + count, rx->used);
}

/* 错帧仅丢弃一个字节再搜索，避免吞掉后面已经到达的正确帧。 */
static void drain(car_receiver *rx)
{
    while (rx->used != 0U)
    {
        uint16_t size;
        uint16_t total;
        if (rx->buffer[0] != 0xA5U ||
            (rx->used >= 2U && rx->buffer[1] != 0x5AU))
        {
            discard(rx, 1U);
            continue;
        }
        if (rx->used < 8U)
        {
            return;
        }
        size = read16(rx->buffer + 6U);
        if (rx->buffer[2] != CAR_VERSION || size > CAR_MAX_PAYLOAD)
        {
            ++rx->header_errors;
            discard(rx, 1U);
            continue;
        }
        total = (uint16_t)(size + CAR_FRAME_OVERHEAD);
        if (rx->used < total)
        {
            return;
        }
        if (car_crc32(rx->buffer + 2U, 6U + size) !=
            read32(rx->buffer + 8U + size))
        {
            ++rx->crc_errors;
            discard(rx, 1U);
            continue;
        }
        if (rx->handler != NULL)
        {
            rx->handler(rx->context, rx->buffer[3], read16(rx->buffer + 4U),
                        rx->buffer + 8U, size);
        }
        discard(rx, total);
    }
}

/** 初始化流解析器及完整帧回调。 */
void car_receiver_init(car_receiver *rx, car_frame_handler handler, void *context)
{
    memset(rx, 0, sizeof(*rx));
    rx->handler = handler;
    rx->context = context;
}

/** 处理接收超时，恢复被截断帧阻塞的后续数据。 */
void car_receiver_poll(car_receiver *rx, uint32_t now_ms)
{
    if (rx->used != 0U &&
        (uint32_t)(now_ms - rx->last_byte_ms) >= CAR_RX_TIMEOUT_MS)
    {
        ++rx->timeouts;
        /* 长度损坏可能把好帧挡在后面；过期时先抢救完整好帧。 */
        while (rx->used != 0U)
        {
            discard(rx, 1U);
            drain(rx);
        }
    }
}

/** 追加一个接收字节并触发有界帧解析。 */
void car_receiver_feed(car_receiver *rx, uint8_t byte, uint32_t now_ms)
{
    car_receiver_poll(rx, now_ms);
    /* drain 每次处理后 used 必然小于 CAR_MAX_FRAME。 */
    rx->buffer[rx->used++] = byte;
    rx->last_byte_ms = now_ms;
    drain(rx);
}
