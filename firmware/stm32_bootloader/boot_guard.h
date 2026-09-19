#ifndef BOOT_GUARD_H
#define BOOT_GUARD_H

#include <stddef.h>
#include <stdint.h>
#include "boot_layout.h"

typedef enum {
    BOOT_VALID = 0,
    BOOT_NOT_COMMITTED = 1,
    BOOT_BAD_METADATA = 2,
    BOOT_BAD_TARGET = 3,
    BOOT_BAD_LENGTH = 4,
    BOOT_BAD_VECTOR = 5,
    BOOT_BAD_IMAGE_CRC = 6
} boot_result;

/* 无硬件副作用；先检查长度再读取 App，供设备与主机故障注入测试共用。 */
boot_result boot_validate(const uint8_t metadata[BOOT_META_SIZE],
                          const uint8_t *app, size_t available);

#endif
