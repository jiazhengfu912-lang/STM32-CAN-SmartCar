#include "boot_guard.h"
#include "car_protocol.h"

/** 按小端顺序读取 32 位整数，避免未对齐访问。 */
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

/** 检查提交标记、头部 CRC、分区边界、向量及整个 App CRC，返回具体无效原因。 */
boot_result boot_validate(const uint8_t metadata[BOOT_META_SIZE],
                          const uint8_t *app, size_t available)
{
    uint32_t length, stack, reset;
    if (get32(metadata + 28U) != BOOT_COMMITTED) return BOOT_NOT_COMMITTED;
    if (car_crc32(metadata, 24U) != get32(metadata + 24U)) return BOOT_BAD_METADATA;
    if (get32(metadata) != BOOT_MAGIC || get32(metadata + 4U) != BOOT_FORMAT ||
        get32(metadata + 8U) != BOOT_APP_BASE) return BOOT_BAD_TARGET;
    length = get32(metadata + 12U);
    if (length < 258U || length > BOOT_APP_CAPACITY || length > available)
        return BOOT_BAD_LENGTH;
    stack = get32(app);
    reset = get32(app + 4U);
    /* 入口必须位于镜像正文，不能指向向量表、Bootloader、元数据或镜像末尾。 */
    if (stack <= BOOT_RAM_BASE || stack > BOOT_RAM_LIMIT || (stack & 7U) != 0U ||
        (reset & 1U) == 0U || (reset & ~1U) < BOOT_APP_BASE + 256U ||
        (reset & ~1U) > BOOT_APP_BASE + length - 2U) return BOOT_BAD_VECTOR;
    if (car_crc32(app, length) != get32(metadata + 16U)) return BOOT_BAD_IMAGE_CRC;
    return BOOT_VALID;
}
