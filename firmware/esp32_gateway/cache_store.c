#include "cache_store.h"
#include "boot_guard.h"
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

/** 按小端顺序读取 32 位整数，避免未对齐访问。 */
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

/** 重新打开完整缓存，检查元数据、长度、向量和正文 CRC。 */
int cache_verify(const char *path, cache_info *info, uint8_t *scratch, size_t capacity)
{
    uint8_t header[BOOT_META_SIZE];
    uint32_t length;
    int ok = 0;
    FILE *file;
    memset(info, 0, sizeof(*info));
    file = fopen(path, "rb");
    if (!file) return 0;
    if (fread(header, 1U, sizeof(header), file) != sizeof(header)) goto done;
    length = get32(header + 12U);
    if (length < 258U || length > BOOT_APP_CAPACITY || length > capacity ||
        get32(header + 20U) == 0U) goto done;
    if (fread(scratch, 1U, length, file) != length || fgetc(file) != EOF || ferror(file)) goto done;
    if (boot_validate(header, scratch, length) != BOOT_VALID) goto done;
    ok = 1;
done:
    if (fclose(file) != 0) ok = 0;
    if (ok) {
        info->size = length;
        info->crc = get32(header + 16U);
        info->version = get32(header + 20U);
    }
    return ok;
}

/** 创建独立临时缓存文件，保留原来可用的完整缓存。 */
int cache_start(cache_upload *upload, const char *part, const char *ready, size_t expected)
{
    memset(upload, 0, sizeof(*upload));
    upload->failed = 1;
    if (expected < BOOT_META_SIZE + 258U || expected > CACHE_MAX_SIZE) return 0;
    upload->part_path = part;
    upload->ready_path = ready;
    upload->expected = expected;
    upload->file = fopen(part, "wb");
    upload->failed = (upload->file == NULL);
    return !upload->failed;
}

/** 写入受总长度约束的数据块，失败时标记本次上传不可发布。 */
int cache_write(cache_upload *upload, const uint8_t *data, size_t size)
{
    if (!upload->file || upload->failed) return 0;
    if (size > upload->expected - upload->written ||
        fwrite(data, 1U, size, upload->file) != size) {
        upload->failed = 1;
        return 0;
    }
    upload->written += size;
    return 1;
}

/** 关闭失败上传并移除临时文件，不影响已发布缓存。 */
void cache_abort(cache_upload *upload)
{
    if (upload->file) fclose(upload->file);
    upload->file = NULL;
    upload->failed = 1;
    if (upload->part_path) remove(upload->part_path);
}

/** 关闭并校验临时文件后原子发布，再读回确认完整性。 */
int cache_finish(cache_upload *upload, cache_info *info, uint8_t *scratch, size_t capacity)
{
    int ok = upload->file && !upload->failed && upload->written == upload->expected;
    memset(info, 0, sizeof(*info));
    if (upload->file && fclose(upload->file) != 0) ok = 0;
    upload->file = NULL;
    if (ok) ok = cache_verify(upload->part_path, info, scratch, capacity);
    /* 只发布已关闭且读回校验通过的文件；绝不先删除旧 ready。 */
    if (ok) {
#ifdef _WIN32
        ok = MoveFileExA(upload->part_path, upload->ready_path,
                        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        ok = rename(upload->part_path, upload->ready_path) == 0;
#endif
    }
    if (ok) ok = cache_verify(upload->ready_path, info, scratch, capacity);
    if (!ok) {
        cache_abort(upload);
        memset(info, 0, sizeof(*info));
    }
    return ok;
}
