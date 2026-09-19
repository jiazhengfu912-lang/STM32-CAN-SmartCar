#ifndef CONTROL_DATAGRAM_H
#define CONTROL_DATAGRAM_H
#include <stddef.h>
#include <stdint.h>
#define CONTROL_UDP_PORT 3232
#define CONTROL_DATAGRAM_SIZE 52
#ifdef __cplusplus
extern "C" {
#endif
/** 校验固定长度、令牌及速度命令；只解码，不负责续命或执行。 */
int control_datagram_decode(const uint8_t *packet, size_t length, const char *token, uint8_t output[16]);
#ifdef __cplusplus
}
#endif
#endif
