#include "angle_speed.h"

/** 接收8192刻度单圈角度并展开累计位置；断流后丢弃旧窗口，避免跨停机误差。 */
void angle_speed_update(angle_speed *s,uint16_t angle,int direction,uint32_t now)
{
    int32_t delta;
    if (angle>=8192U) return;
    if (!s->initialized || now-s->last_rx>=50U) {
        s->initialized=1; s->count=0; s->index=0; s->valid=0;
    } else {
        delta=(int32_t)angle-s->angle;
        if (delta>4096) delta-=8192;
        if (delta<-4096) delta+=8192;
        s->position+=(uint32_t)(delta*direction);
    }
    s->angle=angle; s->last_rx=now;
}

/** 每控制周期采样，使用约30ms位置差计算电机轴rpm；预热时保留原始速度保护。 */
int16_t angle_speed_sample(angle_speed *s,uint32_t now,int16_t fallback)
{
    uint32_t dt; int32_t delta,value; unsigned oldest;
    if (!s->initialized || now-s->last_rx>=50U) {
        s->count=0; s->valid=0; return fallback;
    }
    s->position_history[s->index]=s->position;
    s->time_history[s->index]=now;
    s->index=(uint8_t)((s->index+1U)%4U);
    if (s->count<4U) ++s->count;
    if (s->count<4U) return fallback;
    oldest=s->index; dt=now-s->time_history[oldest];
    if (!dt || dt>=100U) { s->count=0; s->valid=0; return fallback; }
    delta=(int32_t)(s->position-s->position_history[oldest]);
    value=(int32_t)((int64_t)delta*60000/(8192*(int64_t)dt));
    if (value>32767) value=32767;
    if (value<-32767) value=-32767;
    s->rpm=(int16_t)value; s->valid=1; return s->rpm;
}
