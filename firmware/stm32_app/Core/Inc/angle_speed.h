#ifndef ANGLE_SPEED_H
#define ANGLE_SPEED_H
#include <stdint.h>
typedef struct {
    uint32_t position,last_rx,position_history[4],time_history[4],discontinuities;
    uint16_t angle;
    uint8_t initialized,count,index,valid;
    int16_t rpm;
} angle_speed;
void angle_speed_update(angle_speed *s,uint16_t angle,int direction,uint32_t now);
int16_t angle_speed_sample(angle_speed *s,uint32_t now,int16_t fallback);
#endif
