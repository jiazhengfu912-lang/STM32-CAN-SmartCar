#ifndef OTA_RECEIVER_H
#define OTA_RECEIVER_H
#include "car_protocol.h"
#include "boot_layout.h"

#define OTA_BLOCK_SIZE 128U
#define OTA_IDLE_TIMEOUT_MS 10000U
enum { OTA_IDLE, OTA_RECEIVING, OTA_COMMITTED, OTA_FAILED, OTA_APP };
enum { OTA_OK, OTA_BAD_ARGUMENT, OTA_BAD_STATE, OTA_BAD_SESSION, OTA_BAD_OFFSET,
       OTA_CONFLICT, OTA_IO_ERROR, OTA_INVALID_IMAGE, OTA_TIMEOUT, OTA_BUSY };

/* 硬件适配层也必须检查范围；invalidate 成功后才允许 erase。 */
typedef struct {
    int (*invalidate)(void);
    int (*erase)(uint32_t length);
    int (*write)(uint32_t offset, const uint8_t *data, uint16_t size);
    int (*equal)(uint32_t offset, const uint8_t *data, uint16_t size);
    int (*finish)(uint32_t length, uint32_t crc, uint32_t version);
} ota_storage;

typedef struct {
    const ota_storage *storage;
    uint32_t session, length, crc, version, next, last_offset, last_activity;
    uint16_t begin_seq, last_seq, last_size, end_seq;
    uint8_t state;
} ota_receiver;

uint32_t ota_get32(const uint8_t *p);
void ota_put32(uint8_t *p, uint32_t value);
void ota_init(ota_receiver *rx, const ota_storage *storage);
void ota_poll(ota_receiver *rx, uint32_t now);
/* 返回业务状态；reply 为固定 16 字节，外层序号由调用者回显。 */
uint8_t ota_handle(ota_receiver *rx, uint8_t type, uint16_t seq,
                   const uint8_t *data, uint16_t size, uint32_t now, uint8_t reply[16]);
#endif
