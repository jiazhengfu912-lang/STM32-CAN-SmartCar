#ifndef SMARTCAR_MPU6050_H
#define SMARTCAR_MPU6050_H
#include "imu.h"
typedef struct { uint32_t errors,timeouts,restarts; uint8_t ready,identity; } mpu6050_status;
extern mpu6050_status mpu_status;
void mpu6050_init(void);
void mpu6050_poll(imu_state *imu,uint32_t now,int stationary);
#endif
