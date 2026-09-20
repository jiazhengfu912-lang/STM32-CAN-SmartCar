#include "navigation.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f

/** 限制外环量，避免目标合成超过既有速度边界。 */
static float clamp(float x,float limit) { return x>limit ? limit:(x<-limit ? -limit:x); }
/** 读取小端32位字段，不依赖结构体布局。 */
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
/** 上电不配置几何参数，外环增益为零，必须先完成观察和标定。 */
void navigation_init(navigation *s)
{
    memset(s,0,sizeof(*s)); s->reduction=36; s->weight=0.98f; s->max_rate=0.6f; s->slew=1.0f;
}
/** 取消动作但保留终态，取消本身不能续租用户控制权。 */
void navigation_cancel(navigation *s, uint8_t reason)
{
    if (s->mode) s->result=reason;
    s->mode=NAV_IDLE; s->correction=0; s->settled=0;
}
/** 普通速度命令接管并取消导航；导航心跳仅续租，不重置动作参考。 */
uint8_t navigation_drive_command(navigation *s,drive_state *d,uint8_t op,uint32_t session,
                                 uint32_t sequence,int16_t left,int16_t right,uint32_t now)
{
    uint8_t result;
    if (op==4) {
        if (!s->mode || left || right) return CONTROL_STATE;
        return drive_command(d,CMD_SPEED,session,sequence,d->target[0],d->target[1],now);
    }
    result=drive_command(d,op,session,sequence,left,right,now);
    if (!result) navigation_cancel(s,NAV_CANCELLED);
    return result;
}
/** 配置和校准要求零目标、零输出；有轮速反馈时还要确认实际接近静止。 */
int navigation_stationary(const drive_state *d, uint32_t now)
{
    unsigned i;
    if (d->state!=DRIVE_DISARMED) return 0;
    for (i=0;i<2;i++) {
        if (d->target[i] || d->output[i]) return 0;
        if (d->seen[i] && (now-d->feedback_ms[i]>=100U || d->rpm[i]<=-18 || d->rpm[i]>=18)) return 0;
    }
    return 1;
}
/** 判断定位依赖是否新鲜；大倾角超出平面小车模型时禁止导航。 */
static int sensors(const imu_state *imu,const angle_speed w[2],uint32_t now)
{
    return imu_fresh(imu,now) && imu->calibration==IMU_CALIBRATED && imu->attitude_ready &&
        fabsf(imu->roll)<0.35f && fabsf(imu->pitch)<0.35f &&
        w[0].initialized && w[1].initialized && now-w[0].last_rx<50U && now-w[1].last_rx<50U;
}
/** 执行版本化导航指令；运动只接受当前会话的新序号，重复包不重建参考。 */
uint8_t navigation_command(navigation *s, imu_state *imu, drive_state *d,
                          angle_speed w[2], const uint8_t p[32], uint32_t now)
{
    float a[5]; unsigned i; uint8_t result,op=p[0];
    if (p[1]!=1 || p[2] || p[3] || op<1 || op>NAV_DIAGNOSTIC) return CONTROL_ARGUMENT;
    for (i=0;i<5;i++) { uint32_t bits=u32(p+12+4*i); memcpy(&a[i],&bits,4); if (!isfinite(a[i])) return CONTROL_ARGUMENT; }
    if (op==NAV_START_STRAIGHT || op==NAV_START_TURN) {
        if (!s->configured || !s->axes_confirmed || !s->valid || s->discontinuous || !sensors(imu,w,now) || s->mode) return CONTROL_STATE;
        if (op==NAV_START_STRAIGHT && (fabsf(a[0])<1 || fabsf(a[0])>10)) return CONTROL_ARGUMENT;
        if (op==NAV_START_TURN && ((a[0]!=90 && a[0]!=-90) || s->kp<=0)) return CONTROL_ARGUMENT;
        result=drive_command(d,CMD_SPEED,u32(p+4),u32(p+8),0,0,now);
        if (result) return result;
        s->mode=op==NAV_START_STRAIGHT ? NAV_STRAIGHT:NAV_TURN; s->result=NAV_RUNNING;
        s->target=imu_wrap(s->heading+(s->mode==NAV_TURN ? a[0]*PI_F/180:0));
        s->base=s->mode==NAV_STRAIGHT ? a[0]:0; s->started_ms=now; s->settled=0; s->correction=0;
        return CONTROL_OK;
    }
    if (!navigation_stationary(d,now)) return CONTROL_STATE;
    if (op==NAV_CALIBRATE) { s->valid=0; s->discontinuous=1; return imu_calibrate(imu,now) ? CONTROL_OK:CONTROL_STATE; }
    if (op==NAV_ORIGIN) {
        if (!s->configured || !s->axes_confirmed || !sensors(imu,w,now) || !drive_can_update(d,now)) return CONTROL_STATE;
        s->x=s->y=s->heading=0; s->last_yaw=imu->yaw; s->last_ms=now; s->imu_gaps=imu->gaps;
        for (i=0;i<2;i++) { s->previous[i]=w[i].position; s->continuity[i]=w[i].discontinuities; }
        s->valid=1; s->discontinuous=0; ++s->origin_id; return CONTROL_OK;
    }
    if (op==NAV_GEOMETRY) {
        if (a[0]<0.02f || a[0]>0.5f || a[1]<0.02f || a[1]>0.5f || a[2]<0.05f || a[2]>1.0f || a[3]!=36.0f) return CONTROL_ARGUMENT;
        s->diameter[0]=a[0]; s->diameter[1]=a[1]; s->track=a[2]; s->reduction=a[3]; s->configured=1; s->valid=0; s->discontinuous=1;
    } else if (op==NAV_TUNING) {
        if (a[0]<0 || a[0]>5 || a[1]<0 || a[1]>2 || a[2]<=0 || a[2]>1 || a[3]<=0 || a[3]>2 || a[4]<0 || a[4]>1) return CONTROL_ARGUMENT;
        s->kp=a[0]; s->kd=a[1]; s->max_rate=a[2]; s->slew=a[3]; s->weight=a[4];
    } else if (op==NAV_AXES) {
        int8_t axes[3];
        for (i=0;i<3;i++) { if (a[i]<-3 || a[i]>3 || a[i]!=(int)a[i]) return CONTROL_ARGUMENT; axes[i]=(int8_t)a[i]; }
        if (!imu_axes(imu,axes)) return CONTROL_ARGUMENT;
        s->axes_confirmed=1; s->valid=0; s->discontinuous=1;
    } else if (op==NAV_DIAGNOSTIC) {
        if (a[0]!=0 && a[0]!=1) return CONTROL_ARGUMENT;
        s->diagnostic=(uint8_t)a[0];
    }
    return CONTROL_OK;
}
/** 更新增量里程计与外环；内部计算不调用命令入口，不延长用户心跳期限。 */
void navigation_tick(navigation *s, const imu_state *imu, drive_state *d,const angle_speed w[2],uint32_t now)
{
    unsigned i; float ds[2],dyaw,dt=(now-s->last_ms)*0.001f,rate,error,left,right,scale;
    int fresh=sensors(imu,w,now);
    if (s->valid && (!fresh || imu->gaps!=s->imu_gaps || w[0].discontinuities!=s->continuity[0] || w[1].discontinuities!=s->continuity[1])) {
        s->valid=0; s->discontinuous=1;
    }
    if (s->valid) {
        for (i=0;i<2;i++) {
            int32_t delta=(int32_t)(w[i].position-s->previous[i]); s->previous[i]=w[i].position;
            ds[i]=(float)delta*(PI_F*s->diameter[i])/(8192.0f*s->reduction);
        }
        dyaw=s->weight*imu_wrap(imu->yaw-s->last_yaw)+(1-s->weight)*(ds[1]-ds[0])/s->track;
        s->x+=(ds[0]+ds[1])*0.5f*cosf(s->heading+dyaw*0.5f);
        s->y+=(ds[0]+ds[1])*0.5f*sinf(s->heading+dyaw*0.5f);
        s->heading=imu_wrap(s->heading+dyaw); s->last_yaw=imu->yaw;
    }
    s->last_ms=now;
    if (!s->mode) return;
    if (d->state!=DRIVE_ARMED) { navigation_cancel(s,NAV_DRIVE_LOST); return; }
    if (!fresh || !s->valid) { navigation_cancel(s,NAV_SENSOR_LOST); drive_fault(d,FAULT_IMU); return; }
    if (now-s->started_ms>=15000U) { navigation_cancel(s,NAV_TIMEOUT); (void)drive_command(d,CMD_DISARM,0,0,0,0,now); return; }
    error=imu_wrap(s->target-s->heading);
    if (s->mode==NAV_TURN) {
        if (fabsf(error)<=0.034907f && fabsf(imu->gyro[2])<=0.05236f) {
            if (!s->settled) { s->settled=1; s->settled_ms=now; }
            if (now-s->settled_ms>=300U) { navigation_cancel(s,NAV_DONE); (void)drive_command(d,CMD_DISARM,0,0,0,0,now); return; }
        } else s->settled=0;
    }
    rate=clamp(s->kp*error-s->kd*imu->gyro[2],s->max_rate);
    s->correction+=clamp(rate-s->correction,s->slew*(dt>0.02f ? 0.02f:dt));
    left=s->base-s->correction*s->track*30/(PI_F*s->diameter[0]);
    right=s->base+s->correction*s->track*30/(PI_F*s->diameter[1]);
    scale=fmaxf(fabsf(left),fabsf(right)); if (scale>10) { left*=10/scale; right*=10/scale; }
    for (i=0;i<2;i++) {
        int16_t target=(int16_t)((i ? right:left)*36.0f);
        if (target && (!d->target[i] || (int32_t)target*d->target[i]<0)) d->starting[i]=1;
        d->target[i]=target;
    }
}
