#include "drive.h"
#include <string.h>

/** 判断两轮反馈是否均在有效期内；无反馈不能解锁运动或确认停稳。 */
static int fresh(const drive_state *s, uint32_t now)
{
    return s->seen[0] && s->seen[1] && now-s->feedback_ms[0] < DRIVE_FEEDBACK_MS &&
           now-s->feedback_ms[1] < DRIVE_FEEDBACK_MS;
}

/** 判断电机轴转速是否接近零，避免仅因目标为零就认定车辆已经停下。 */
static int stationary(const drive_state *s)
{
    return s->rpm[0] > -90 && s->rpm[0] < 90 && s->rpm[1] > -90 && s->rpm[1] < 90;
}

/** 清除目标、电流和积分；禁止驱动表示零电流滑停，不是机械抱闸。 */
static void zero(drive_state *s)
{
    unsigned i;
    for (i=0;i<2;i++) { s->target[i]=0; s->output[i]=0; s->integral[i]=0; s->starting[i]=0; s->stalled_valid[i]=0; }
}

/** 初始化为禁止驱动状态；上电及升级后的启动均不得自动使能。 */
void drive_init(drive_state *s) { memset(s,0,sizeof(*s)); }

/** 更新单个车轮的方向归一化反馈；wheel 为左轮 0、右轮 1。 */
void drive_feedback(drive_state *s, unsigned wheel, int16_t rpm, int16_t current, uint32_t now)
{
    if (wheel>1) return;
    s->rpm[wheel]=rpm; s->measured_current[wheel]=current;
    s->feedback_ms[wheel]=now; s->seen[wheel]=1;
}

/** 锁存故障并立即清零输出；后续速度指令不能自动解除故障。 */
void drive_fault(drive_state *s, uint8_t reason)
{
    s->state=DRIVE_FAULT; s->fault=reason; s->session=0; zero(s);
}

/** 执行明确的解锁/速度/停机/清故障请求；会话和递增序号阻止旧包续命。 */
uint8_t drive_command(drive_state *s, uint8_t op, uint32_t session, uint32_t sequence,
                      int16_t left, int16_t right, uint32_t now)
{
    if (op==CMD_DISARM) {
        zero(s); s->session=0;
        if (s->state!=DRIVE_FAULT && s->state!=DRIVE_UPDATING) s->state=DRIVE_DISARMED;
        return CONTROL_OK;
    }
    if (op>CMD_CLEAR || left < -DRIVE_LIMIT_RPM || left > DRIVE_LIMIT_RPM ||
        right < -DRIVE_LIMIT_RPM || right > DRIVE_LIMIT_RPM) return CONTROL_ARGUMENT;
    if (op==CMD_CLEAR) {
        if (s->state!=DRIVE_FAULT || left || right) return CONTROL_STATE;
        if (!fresh(s,now) || !stationary(s)) return CONTROL_FEEDBACK;
        drive_init(s); return CONTROL_OK;
    }
    if (op==CMD_ARM) {
        if (s->state!=DRIVE_DISARMED || !session || left || right) return CONTROL_STATE;
        if (!fresh(s,now) || !stationary(s)) return CONTROL_FEEDBACK;
        s->state=DRIVE_ARMED; s->session=session; s->sequence=sequence;
        s->command_ms=now; zero(s); return CONTROL_OK;
    }
    if (s->state!=DRIVE_ARMED || s->session!=session) return CONTROL_STATE;
    /* 先检查已有租期，迟到的命令不得抢在周期检查前延长已过期控制权。 */
    if (now-s->command_ms >= DRIVE_COMMAND_MS) {
        drive_fault(s,FAULT_COMMAND); return CONTROL_STATE;
    }
    if (!fresh(s,now)) { drive_fault(s,FAULT_FEEDBACK); return CONTROL_FEEDBACK; }
    if (!sequence || sequence <= s->sequence) return CONTROL_SEQUENCE;
    /* 新起步或换向重新标记；同方向心跳不重复触发起步处理。 */
    if (left && (!s->target[0] || (int32_t)left*s->target[0]<0)) s->starting[0]=1;
    if (right && (!s->target[1] || (int32_t)right*s->target[1]<0)) s->starting[1]=1;
    s->sequence=sequence; s->command_ms=now; s->target[0]=left; s->target[1]=right;
    return CONTROL_OK;
}

/** 每 10 ms 调用：检查通信期限，并计算带积分限幅的两轮速度 PI 电流。 */
void drive_tick(drive_state *s, uint32_t now)
{
    unsigned i;
    if (fresh(s,now) && stationary(s)) {
        if (!s->stopped_valid) { s->stopped_since=now; s->stopped_valid=1; }
    } else s->stopped_valid=0;
    if (s->state==DRIVE_ARMED) {
        if (now-s->command_ms >= DRIVE_COMMAND_MS) drive_fault(s,FAULT_COMMAND);
        else if (!fresh(s,now)) drive_fault(s,FAULT_FEEDBACK);
    }
    if (s->state!=DRIVE_ARMED) { zero(s); return; }
    for (i=0;i<2;i++) {
        float error=(float)s->target[i]-s->rpm[i], value;
        /* v121试验：仅右轮反转提高比例纠偏；两轮积分仍独立，积分系数不变。 */
        float kp=(i==1 && s->target[i]<0) ? 0.45f : 0.3f;
        /* 短暂起转后又停住时重新保护下一次起转；持续100ms低于轮速0.5rpm才重置。 */
        if (s->target[i] && !s->starting[i] && s->rpm[i]>-18 && s->rpm[i]<18) {
            if (!s->stalled_valid[i]) { s->stalled_since[i]=now; s->stalled_valid[i]=1; }
            else if (now-s->stalled_since[i]>=100U) { s->starting[i]=1; s->stalled_valid[i]=0; }
        } else s->stalled_valid[i]=0;
        /* 同方向达到轮速1rpm才释放启动积分；0.5rpm提前释放实测会再次停转。 */
        if (s->starting[i] && (int32_t)s->rpm[i]*s->target[i]>0 &&
            (s->rpm[i]>=36 || s->rpm[i]<=-36)) {
            /* 右轮低速反转需要更高起转支撑；仅放宽释放上限，不强制注入最低电流。 */
            float support=(i==1 && s->target[i]<0 && s->target[i]>=-360) ? 180.0f : 150.0f;
            float limit=(s->target[i]>0 ? support : -support)-kp*error;
            if (s->target[i]>0 && s->integral[i]>limit) s->integral[i]=limit;
            if (s->target[i]<0 && s->integral[i]<limit) s->integral[i]=limit;
            s->starting[i]=0;
        }
        /* 仅5–10rpm尚未起转阶段加快三倍积分，每10ms最多增加6；起转即恢复原PI。 */
        if (s->starting[i] && (s->target[i]>=180 || s->target[i]<=-180) &&
            s->target[i]>=-360 && s->target[i]<=360 && s->rpm[i]>-36 && s->rpm[i]<36) {
            float increment=0.030f*error;
            if (increment>6.0f) increment=6.0f;
            if (increment<-6.0f) increment=-6.0f;
            s->integral[i]+=increment;
            /* 启动限幅比通用电流限幅更保守，避免未起转时持续积累过大电流。 */
            if (s->integral[i]+kp*error>450.0f) s->integral[i]=450.0f-kp*error;
            if (s->integral[i]+kp*error<-450.0f) s->integral[i]=-450.0f-kp*error;
        } else {
            /* 正负误差对称积分，避免起步超调后过量泄放导致低速停转。 */
            s->integral[i]+=0.010f*error;
        }
        if (s->integral[i]>1200) s->integral[i]=1200;
        if (s->integral[i]<-1200) s->integral[i]=-1200;
        /* 保留原有电流限幅，比例项用于修正瞬时速度误差。 */
        value=kp*error+s->integral[i];
        if (value>2000) value=2000;
        if (value<-2000) value=-2000;
        s->output[i]=(int16_t)value;
    }
}

/** 仅禁止驱动且有连续 500 ms 新鲜停稳反馈时允许进入更新；故障时不放行。 */
int drive_can_update(const drive_state *s, uint32_t now)
{
    return s->state==DRIVE_DISARMED && fresh(s,now) && stationary(s) &&
           s->stopped_valid && now-s->stopped_since >= 500U;
}
