#ifndef SMARTCAR_NAVIGATION_H
#define SMARTCAR_NAVIGATION_H
#include "imu.h"
#include "drive.h"
#include "angle_speed.h"
enum { NAV_IDLE, NAV_STRAIGHT, NAV_TURN };
enum { NAV_NONE, NAV_RUNNING, NAV_DONE, NAV_CANCELLED, NAV_SENSOR_LOST, NAV_TIMEOUT, NAV_DRIVE_LOST };
enum { NAV_CALIBRATE=1, NAV_ORIGIN, NAV_START_STRAIGHT, NAV_START_TURN, NAV_GEOMETRY, NAV_TUNING, NAV_AXES, NAV_DIAGNOSTIC };
typedef struct {
    uint8_t configured, valid, discontinuous, mode, result, diagnostic, axes_confirmed;
    uint32_t previous[2], continuity[2], last_ms, imu_gaps, started_ms, settled_ms, origin_id;
    uint8_t settled;
    float diameter[2], track, reduction, weight, x, y, heading, last_yaw;
    float kp,kd,max_rate,slew,correction,target,base;
} navigation;
void navigation_init(navigation *s);
void navigation_cancel(navigation *s, uint8_t reason);
uint8_t navigation_drive_command(navigation *s,drive_state *d,uint8_t op,uint32_t session,
                                 uint32_t sequence,int16_t left,int16_t right,uint32_t now);
uint8_t navigation_command(navigation *s, imu_state *imu, drive_state *drive,
                          angle_speed wheels[2], const uint8_t payload[32], uint32_t now);
void navigation_tick(navigation *s, const imu_state *imu, drive_state *drive,
                     const angle_speed wheels[2], uint32_t now);
int navigation_stationary(const drive_state *drive, uint32_t now);
#endif
