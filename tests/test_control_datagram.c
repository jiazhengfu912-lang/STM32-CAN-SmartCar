#include <assert.h>
#include <string.h>
#include "control_datagram.h"
/** 验证正确速度包及令牌、长度、操作码、会话、序号、保留位和速度边界。 */
int main(void)
{
    const char *token="0123456789abcdef0123456789abcdef";
    uint8_t packet[52]={0}, output[16], good[52];
    memcpy(packet,"SCU1",4);memcpy(packet+4,token,32);
    packet[36]=2;packet[40]=1;packet[44]=1;packet[48]=180;packet[50]=180;
    memcpy(good,packet,52);
    assert(control_datagram_decode(packet,52,token,output));assert(!memcmp(output,packet+36,16));
    assert(!control_datagram_decode(packet,51,token,output));
    assert(!control_datagram_decode(packet,53,token,output));
    assert(!control_datagram_decode(packet,52,"",output));
    packet[4]^=1;assert(!control_datagram_decode(packet,52,token,output));memcpy(packet,good,52);
    packet[0]^=1;assert(!control_datagram_decode(packet,52,token,output));memcpy(packet,good,52);
    for (unsigned op=0;op<4;op++) if(op!=2){packet[36]=op;assert(!control_datagram_decode(packet,52,token,output));}
    memcpy(packet,good,52);packet[37]=1;assert(!control_datagram_decode(packet,52,token,output));
    memcpy(packet,good,52);packet[40]=0;assert(!control_datagram_decode(packet,52,token,output));
    memcpy(packet,good,52);packet[44]=0;assert(!control_datagram_decode(packet,52,token,output));
    memcpy(packet,good,52);packet[48]=0xd1;packet[49]=2;assert(!control_datagram_decode(packet,52,token,output));
    packet[48]=0xd0;assert(control_datagram_decode(packet,52,token,output));
    memcpy(packet,good,52);packet[36]=4;assert(!control_datagram_decode(packet,52,token,output));
    memset(packet+48,0,4);assert(control_datagram_decode(packet,52,token,output));
    packet[36]=5;assert(!control_datagram_decode(packet,52,token,output));
    return 0;
}
