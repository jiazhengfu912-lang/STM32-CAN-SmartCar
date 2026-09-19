#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define CAR_VERSION 1U
#define CAR_MAX_PAYLOAD 256U
#define CAR_FRAME_OVERHEAD 12U
#define CAR_MAX_FRAME (CAR_MAX_PAYLOAD + CAR_FRAME_OVERHEAD)
#define CAR_RX_TIMEOUT_MS 100U
#define CAR_MSG_PING 0x01U
#define CAR_MSG_PONG 0x02U
#define CAR_MSG_TELEMETRY 0x10U
#define CAR_MSG_DIAGNOSTIC 0x11U
#define CAR_MSG_ENTER_UPDATE 0x20U
#define CAR_MSG_OTA_BEGIN 0x30U
#define CAR_MSG_OTA_DATA 0x31U
#define CAR_MSG_OTA_END 0x32U
#define CAR_MSG_OTA_QUERY 0x33U
#define CAR_MSG_OTA_RESET 0x34U
#define CAR_MSG_OTA_REPLY 0x70U

/* payload 只在回调期间有效。回调不能递归调用同一个接收器。 */
typedef void (*car_frame_handler)(void *context, uint8_t type, uint16_t seq,
                                  const uint8_t *payload, uint16_t size);

typedef struct
{
    uint8_t buffer[CAR_MAX_FRAME];
    uint16_t used;
    uint32_t last_byte_ms;
    uint32_t crc_errors;
    uint32_t header_errors;
    uint32_t timeouts;
    car_frame_handler handler;
    void *context;
} car_receiver;

uint32_t car_crc32(const uint8_t *data, size_t size);
/* 成功返回帧长，参数非法或输出容量不足返回 0；输入输出不得重叠。 */
size_t car_encode(uint8_t *output, size_t capacity, uint8_t type, uint16_t seq,
                  const uint8_t *payload, uint16_t size);
void car_receiver_init(car_receiver *rx, car_frame_handler handler, void *context);
/* 在主循环中喂入字节并定期 poll；本模块不调用 HAL、不擦写 Flash。 */
void car_receiver_feed(car_receiver *rx, uint8_t byte, uint32_t now_ms);
void car_receiver_poll(car_receiver *rx, uint32_t now_ms);

#endif
