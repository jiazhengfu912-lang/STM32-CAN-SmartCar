#include "angle_speed.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

/** 使用恒速角度序列验证正反转、单圈回绕、时钟回绕和断流重建。 */
int main(void)
{
    angle_speed s={0}; unsigned i; int16_t rpm=0;
    for(i=0;i<20;i++) {
        angle_speed_update(&s,(uint16_t)((8000+i*512)%8192),1,i*10);
        rpm=angle_speed_sample(&s,i*10,123);
    }
    assert(s.valid && rpm==375);
    memset(&s,0,sizeof(s));
    for(i=0;i<20;i++) {
        uint32_t now=0xfffffff0U+i*10U;
        angle_speed_update(&s,(uint16_t)((8000+i*512)%8192),-1,now);
        rpm=angle_speed_sample(&s,now,123);
    }
    assert(s.valid && rpm==-375);
    angle_speed_update(&s,100,1,400);
    assert(!s.valid && angle_speed_sample(&s,400,77)==77);
    for(i=1;i<=4;i++) {angle_speed_update(&s,100,1,400+i*10);rpm=angle_speed_sample(&s,400+i*10,77);}
    assert(rpm==0);
    assert(angle_speed_sample(&s,550,88)==88 && !s.valid);
    puts("PASS: angle speed wrap, reverse, warmup and stale recovery");return 0;
}
