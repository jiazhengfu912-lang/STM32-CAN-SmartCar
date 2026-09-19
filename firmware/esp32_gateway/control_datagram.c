#include "control_datagram.h"
#include <string.h>
/** 校验 UDP 速度包；拒绝使能、停止、清故障及缺失会话，避免绕过 HTTP 确认流程。 */
int control_datagram_decode(const uint8_t *packet, size_t length, const char *token, uint8_t output[16])
{
    unsigned i, mismatch=0;
    const uint8_t *p;
    int16_t left,right;
    if (length!=CONTROL_DATAGRAM_SIZE || strlen(token)!=32 || memcmp(packet,"SCU1",4)) return 0;
    for (i=0;i<32;i++) mismatch|=packet[4+i]^(uint8_t)token[i];
    if (mismatch) return 0;
    p=packet+36;
    if (p[0]!=2 || p[1] || p[2] || p[3]) return 0;
    if (!(p[4]|p[5]|p[6]|p[7]) || !(p[8]|p[9]|p[10]|p[11])) return 0;
    left=(int16_t)((uint16_t)p[12]|(uint16_t)p[13]<<8);
    right=(int16_t)((uint16_t)p[14]|(uint16_t)p[15]<<8);
    if (left < -720 || left > 720 || right < -720 || right > 720) return 0;
    memcpy(output,p,16);
    return 1;
}
