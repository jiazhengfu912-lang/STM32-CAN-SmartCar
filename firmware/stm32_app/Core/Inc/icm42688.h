#ifndef SMARTCAR_ICM42688_H
#define SMARTCAR_ICM42688_H
#include "imu.h"
typedef struct { uint32_t errors,timeouts,restarts; uint8_t ready,identity,address; } icm42688_status;
extern icm42688_status icm_status;
void icm42688_init(void);
void icm42688_poll(imu_state *imu,uint32_t now,int stationary);
#endif
