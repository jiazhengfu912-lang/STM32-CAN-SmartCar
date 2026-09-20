#ifndef SMARTCAR_IMU_H
#define SMARTCAR_IMU_H
#include <stdint.h>
enum { IMU_UNCALIBRATED, IMU_CALIBRATING, IMU_CALIBRATED, IMU_CAL_FAILED };
typedef struct {
    int16_t raw[7]; /* ax,ay,az,temp,gx,gy,gz；传感器坐标原始值。 */
    int8_t axes[3]; /* 车体坐标对应的有符号传感器轴，1/2/3 为 X/Y/Z。 */
    uint8_t seen, calibration, attitude_ready;
    uint32_t sample_ms, sequence, interval_ms, max_interval_ms, gaps, calibration_ms, count;
    float accel[3], gyro[3], bias[3], sum[3], reference[3];
    float roll, pitch, yaw, temperature;
} imu_state;
void imu_init(imu_state *s);
int imu_axes(imu_state *s, const int8_t axes[3]);
int imu_calibrate(imu_state *s, uint32_t now);
void imu_sample(imu_state *s, const uint8_t raw[14], uint32_t now, int stationary);
int imu_fresh(const imu_state *s, uint32_t now);
float imu_wrap(float angle);
#endif
