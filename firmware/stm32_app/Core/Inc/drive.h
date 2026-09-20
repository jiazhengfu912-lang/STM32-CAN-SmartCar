#ifndef SMARTCAR_DRIVE_H
#define SMARTCAR_DRIVE_H
#include <stdint.h>
#define DRIVE_VERSION 127U
#define DRIVE_LIMIT_RPM 720 /* 电机轴转速；36:1 减速后为轮速 20 rpm。 */
#define DRIVE_COMMAND_MS 300U
#define DRIVE_FEEDBACK_MS 100U
enum { DRIVE_DISARMED, DRIVE_ARMED, DRIVE_FAULT, DRIVE_UPDATING };
enum { FAULT_NONE, FAULT_COMMAND, FAULT_FEEDBACK, FAULT_CAN, FAULT_UART, FAULT_IMU };
enum { CMD_DISARM, CMD_ARM, CMD_SPEED, CMD_CLEAR };
enum { CONTROL_OK, CONTROL_ARGUMENT, CONTROL_STATE, CONTROL_FEEDBACK, CONTROL_SEQUENCE };
typedef struct {
    uint8_t state, fault, seen[2], starting[2];
    int16_t rpm[2], measured_current[2], target[2], output[2];
    uint32_t feedback_ms[2], command_ms, session, sequence, stopped_since;
    uint8_t stopped_valid;
    uint8_t stalled_valid[2];
    uint32_t stalled_since[2];
    float integral[2];
} drive_state;
void drive_init(drive_state *s);
void drive_feedback(drive_state *s, unsigned wheel, int16_t rpm, int16_t current, uint32_t now);
uint8_t drive_command(drive_state *s, uint8_t op, uint32_t session, uint32_t sequence,
                      int16_t left, int16_t right, uint32_t now);
void drive_tick(drive_state *s, uint32_t now);
void drive_fault(drive_state *s, uint8_t reason);
int drive_can_update(const drive_state *s, uint32_t now);
#endif
