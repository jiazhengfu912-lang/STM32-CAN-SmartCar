#include "imu.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f

/** 将角度约束到正负 pi，避免边界附近选择长路径。 */
float imu_wrap(float angle)
{
    angle=fmodf(angle+PI_F,2.0f*PI_F);
    if (angle<0) angle+=2.0f*PI_F;
    return angle-PI_F;
}

/** 上电只采用默认同轴映射，不继承上次校准或位置。 */
void imu_init(imu_state *s)
{
    memset(s,0,sizeof(*s)); s->axes[0]=1; s->axes[1]=2; s->axes[2]=3;
}

/** 仅接受右手正交轴排列；更改安装方向后强制重新校准。 */
int imu_axes(imu_state *s, const int8_t axes[3])
{
    unsigned i,j; int a[3],sign=1;
    for (i=0;i<3;i++) {
        a[i]=axes[i]<0 ? -axes[i]:axes[i];
        if (a[i]<1 || a[i]>3) return 0;
        if (axes[i]<0) sign=-sign;
        for (j=0;j<i;j++) { if (a[j]==a[i]) return 0; if (a[j]>a[i]) sign=-sign; }
    }
    if (sign!=1) return 0;
    imu_init(s); memcpy(s->axes,axes,3); return 1;
}

/** 数据年龄达到50ms即失效，时间差采用无符号运算兼容时基回绕。 */
int imu_fresh(const imu_state *s, uint32_t now) { return s->seen && now-s->sample_ms<50U; }

/** 启动一次五秒静止校准；不在运动过程中更新零偏。 */
int imu_calibrate(imu_state *s, uint32_t now)
{
    if (!imu_fresh(s,now)) return 0;
    s->calibration=IMU_CALIBRATING; s->calibration_ms=now; s->count=0;
    memset(s->sum,0,sizeof(s->sum)); memcpy(s->reference,s->accel,sizeof(s->reference)); return 1;
}

/** 解码ICM-42688-P新采样并转换为车体坐标；单位为g、rad/s、摄氏度及rad。 */
void imu_sample(imu_state *s, const uint8_t raw[14], uint32_t now, int stationary)
{
    unsigned i; uint32_t dt=now-s->sample_ms; float norm,roll,pitch,alpha,t;
    if (s->seen && dt==0) return;
    if (s->seen) {
        s->interval_ms=dt; if (dt>s->max_interval_ms) s->max_interval_ms=dt;
        if (dt>=50U) { ++s->gaps; s->attitude_ready=0; if (s->calibration==IMU_CALIBRATING) s->calibration=IMU_CAL_FAILED; }
    }
    /* ICM线序为temp,ax,ay,az,gx,gy,gz；遥测继续使用ax,ay,az,temp,gx,gy,gz。 */
    for (i=0;i<7;i++) {
        unsigned n=i<3 ? i+1:(i==3 ? 0:i);
        s->raw[i]=(int16_t)((uint16_t)raw[2*n]<<8|raw[2*n+1]);
    }
    for (i=0;i<3;i++) {
        int axis=s->axes[i],n=(axis<0 ? -axis:axis)-1; float sign=axis<0 ? -1.0f:1.0f;
        s->accel[i]=sign*s->raw[n]/8192.0f;
        s->gyro[i]=sign*s->raw[n+4]*(500.0f/32768.0f)*(PI_F/180.0f);
    }
    s->temperature=s->raw[3]/132.48f+25.0f;
    norm=sqrtf(s->accel[0]*s->accel[0]+s->accel[1]*s->accel[1]+s->accel[2]*s->accel[2]);
    if (s->calibration==IMU_CALIBRATING) {
        int still=stationary && norm>0.9f && norm<1.1f;
        for (i=0;i<3;i++) if (fabsf(s->gyro[i])>0.05236f || fabsf(s->accel[i]-s->reference[i])>0.04f) still=0;
        if (!still) s->calibration=IMU_CAL_FAILED;
        else {
            for (i=0;i<3;i++) s->sum[i]+=s->gyro[i];
            ++s->count;
            if (now-s->calibration_ms>=5000U && s->count>=450U) {
                for (i=0;i<3;i++) s->bias[i]=s->sum[i]/s->count;
                s->calibration=IMU_CALIBRATED; s->yaw=0; s->attitude_ready=0;
            }
        }
    }
    for (i=0;i<3;i++) s->gyro[i]-=s->bias[i];
    roll=atan2f(s->accel[1],s->accel[2]);
    pitch=atan2f(-s->accel[0],sqrtf(s->accel[1]*s->accel[1]+s->accel[2]*s->accel[2]));
    if (!s->attitude_ready && norm>0.8f && norm<1.2f) { s->roll=roll; s->pitch=pitch; s->attitude_ready=1; }
    else if (s->attitude_ready && dt<50U) {
        t=dt*0.001f; alpha=0.5f/(0.5f+t);
        s->roll=imu_wrap(s->roll+s->gyro[0]*t); s->pitch=imu_wrap(s->pitch+s->gyro[1]*t);
        if (norm>0.8f && norm<1.2f) {
            s->roll=imu_wrap(s->roll+(1-alpha)*imu_wrap(roll-s->roll));
            s->pitch=imu_wrap(s->pitch+(1-alpha)*imu_wrap(pitch-s->pitch));
        }
    }
    if (s->seen && dt<50U && s->calibration==IMU_CALIBRATED) s->yaw=imu_wrap(s->yaw+s->gyro[2]*(dt*0.001f));
    s->seen=1; s->sample_ms=now; ++s->sequence;
}
