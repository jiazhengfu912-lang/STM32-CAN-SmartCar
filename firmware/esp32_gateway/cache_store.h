#ifndef CACHE_STORE_H
#define CACHE_STORE_H
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "boot_layout.h"

#define CACHE_MAX_SIZE (BOOT_META_SIZE + BOOT_APP_CAPACITY)
typedef struct { uint32_t size, crc, version; } cache_info;
typedef struct {
    FILE *file;
    size_t written, expected;
    const char *part_path, *ready_path;
    int failed;
} cache_upload;

/* 包格式：32 字节描述记录 + 完整 App；与 STM32 的提交动作无关。 */
int cache_verify(const char *path, cache_info *info, uint8_t *scratch, size_t capacity);
int cache_start(cache_upload *upload, const char *part, const char *ready, size_t expected);
int cache_write(cache_upload *upload, const uint8_t *data, size_t size);
int cache_finish(cache_upload *upload, cache_info *info, uint8_t *scratch, size_t capacity);
void cache_abort(cache_upload *upload);
#endif
