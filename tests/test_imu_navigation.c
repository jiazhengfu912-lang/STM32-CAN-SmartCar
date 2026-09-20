#include "navigation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t raw[14],packet[32];
/** 写入大端传感器测试数据，不模拟HAL行为。 */
static void be(unsigned i,int16_t x) { raw[i*2]=(uint8_t)((uint16_t)x>>8); raw[i*2+1]=(uint8_t)x; }
/** 写入协议中的小端32位字段。 */
static void le(uint8_t *p,uint32_t n) { unsigned i; for(i=0;i<4;i++) p[i]=(uint8_t)(n>>(8*i)); }
/** 构造一次新导航指令。 */
static void request(unsigned op,uint32_t seq,float a,float b,float c,float d,float e)
{
    float f[5]={a,b,c,d,e}; unsigned i;
    memset(packet,0,sizeof(packet));packet[0]=(uint8_t)op;packet[1]=1;le(packet+4,123);le(packet+8,seq);
    for(i=0;i<5;i++) { uint32_t bits;memcpy(&bits,&f[i],4);le(packet+12+i*4,bits); }
}
/** 提供新鲜静止测量，使断流与控制故障可由测试逐项注入。 */
static void fresh(imu_state *im,drive_state *d,angle_speed w[2],uint32_t now)
{
    unsigned i;im->seen=1;im->sample_ms=now;im->calibration=IMU_CALIBRATED;im->attitude_ready=1;
    for(i=0;i<2;i++) { w[i].initialized=1;w[i].last_rx=now;drive_feedback(d,i,0,0,now); }
}
/** 验证真实解析、静止校准、运动拒绝、时间回绕和重力门控。 */
static void test_imu(void)
{
    imu_state im; unsigned i; int8_t bad[3]={1,1,3},left[3]={1,2,-3},ok[3]={2,-1,3};
    imu_init(&im);assert(!imu_axes(&im,bad));assert(!imu_axes(&im,left));assert(imu_axes(&im,ok));
    be(0,-8192);be(1,0);be(2,0);be(3,-340);be(4,0);be(5,0);be(6,0);
    imu_sample(&im,raw,0xfffffff0U,1);assert(im.raw[0]==-8192 && im.accel[1]==1);
    imu_sample(&im,raw,4,1);assert(im.interval_ms==20);assert(fabsf(im.temperature-35.53f)<0.01f);
    imu_sample(&im,raw,4,1);assert(im.sequence==2);
    imu_init(&im);memset(raw,0,sizeof(raw));be(2,8192);be(6,65);
    imu_sample(&im,raw,10,1);assert(imu_calibrate(&im,10));
    for(i=20;i<=5010;i+=10) imu_sample(&im,raw,i,1);
    assert(im.calibration==IMU_CALIBRATED && fabsf(im.gyro[2])<0.0001f);
    assert(imu_calibrate(&im,5010));be(4,1000);imu_sample(&im,raw,5020,1);assert(im.calibration==IMU_CAL_FAILED);
    be(4,0);assert(imu_calibrate(&im,5020));imu_sample(&im,raw,5030,0);assert(im.calibration==IMU_CAL_FAILED);
    assert(imu_calibrate(&im,5030));imu_sample(&im,raw,5090,1);assert(im.calibration==IMU_CAL_FAILED && im.gaps==1);
    assert(imu_fresh(&im,5139) && !imu_fresh(&im,5140));
    assert(fabsf(imu_wrap(3.2f)+3.0831853f)<0.00001f);
    im.pitch=0;im.roll=0;be(0,20000);be(2,20000);imu_sample(&im,raw,5100,1);assert(fabsf(im.pitch)<0.001f);
}
/** 验证定位、参数门控、闭环限幅、重复命令及断流锁存。 */
static void test_navigation(void)
{
    navigation n; imu_state im;drive_state d;angle_speed w[2]={{0},{0}};uint32_t now=1000;unsigned i;
    navigation_init(&n);imu_init(&im);drive_init(&d);fresh(&im,&d,w,now);d.stopped_valid=1;d.stopped_since=0;
    request(NAV_ORIGIN,1,0,0,0,0,0);assert(navigation_command(&n,&im,&d,w,packet,now)==CONTROL_STATE);
    request(NAV_GEOMETRY,2,.1f,.1f,.3f,36,0);assert(!navigation_command(&n,&im,&d,w,packet,now));
    request(NAV_AXES,3,1,2,3,0,0);assert(!navigation_command(&n,&im,&d,w,packet,now));fresh(&im,&d,w,now);
    request(NAV_ORIGIN,4,0,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,now));assert(n.valid);
    /* 一圈车轮走pi*0.1米，使用累计编码器而非转速。 */
    w[0].position=w[1].position=8192U*36U;fresh(&im,&d,w,now+10);navigation_tick(&n,&im,&d,w,now+10);
    assert(fabsf(n.x-.3141593f)<0.00001f && fabsf(n.y)<0.00001f);
    ++w[0].discontinuities;navigation_tick(&n,&im,&d,w,now+20);assert(!n.valid && n.discontinuous);
    fresh(&im,&d,w,now+30);navigation_tick(&n,&im,&d,w,now+30);assert(!n.valid);
    request(NAV_ORIGIN,5,0,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,now+30));
    request(NAV_TUNING,6,1,.1f,.6f,1,.98f);assert(!navigation_command(&n,&im,&d,w,packet,now+30));
    assert(!drive_command(&d,CMD_ARM,123,7,0,0,now+30));
    request(NAV_START_TURN,8,90,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,now+30));
    assert(navigation_command(&n,&im,&d,w,packet,now+30)==CONTROL_STATE);
    assert(navigation_drive_command(&n,&d,4,123,8,0,0,now+31)==CONTROL_SEQUENCE);
    assert(!navigation_drive_command(&n,&d,4,123,9,0,0,now+32));
    assert(n.started_ms==now+30 && n.mode==NAV_TURN);
    fresh(&im,&d,w,now+40);navigation_tick(&n,&im,&d,w,now+40);assert(d.target[0]<0 && d.target[1]>0);
    assert(abs(d.target[0])<=360 && abs(d.target[1])<=360);
    assert(d.command_ms==now+32); /* 外环运算不得续命。 */
    fresh(&im,&d,w,now+340);navigation_tick(&n,&im,&d,w,now+340);drive_tick(&d,now+340);assert(d.fault==FAULT_COMMAND);
    navigation_tick(&n,&im,&d,w,now+340);assert(n.mode==NAV_IDLE);
    drive_init(&d);fresh(&im,&d,w,2000);d.stopped_valid=1;d.stopped_since=0;
    request(NAV_ORIGIN,9,0,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,2000));
    assert(!drive_command(&d,CMD_ARM,123,10,0,0,2000));request(NAV_START_STRAIGHT,11,5,0,0,0,0);
    assert(!navigation_command(&n,&im,&d,w,packet,2000));navigation_tick(&n,&im,&d,w,2050);
    assert(d.state==DRIVE_FAULT && d.fault==FAULT_IMU && !n.mode && !n.valid);
    drive_init(&d);fresh(&im,&d,w,3000);d.stopped_valid=1;d.stopped_since=0;
    request(NAV_ORIGIN,12,0,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,3000));
    assert(!drive_command(&d,CMD_ARM,123,13,0,0,3000));request(NAV_START_TURN,14,-90,0,0,0,0);
    assert(!navigation_command(&n,&im,&d,w,packet,3000));
    n.heading=n.target;
    for(i=0;i<=31;i++) { fresh(&im,&d,w,3000+i*10);navigation_tick(&n,&im,&d,w,3000+i*10); }
    assert(n.result==NAV_DONE && d.state==DRIVE_DISARMED && !d.output[0]);
    /* 纯编码器圆弧：右轮走0.15米、轮距0.3米，转角0.5rad。 */
    fresh(&im,&d,w,4000);d.stopped_valid=1;d.stopped_since=0;
    request(NAV_ORIGIN,15,0,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,4000));n.weight=0;
    w[1].position+=140809;fresh(&im,&d,w,4010);navigation_tick(&n,&im,&d,w,4010);
    assert(fabsf(n.heading-.5f)<.0001f && n.x>0 && n.y>0);
    /* 累计位置uint32回绕仍以有符号增量更新。 */
    n.previous[0]=n.previous[1]=0xfffffff0U;w[0].position=w[1].position=16;
    { float x=n.x;fresh(&im,&d,w,4020);navigation_tick(&n,&im,&d,w,4020);assert(n.x>x && n.x-x<.001f); }
    assert(!drive_command(&d,CMD_ARM,123,20,0,0,4020));request(NAV_START_STRAIGHT,21,5,0,0,0,0);
    assert(!navigation_command(&n,&im,&d,w,packet,4020));
    assert(!navigation_drive_command(&n,&d,CMD_SPEED,123,22,180,180,4021));assert(!n.mode && n.result==NAV_CANCELLED);
    request(NAV_START_TURN,23,90,0,0,0,0);assert(!navigation_command(&n,&im,&d,w,packet,4021));
    assert(!navigation_drive_command(&n,&d,CMD_DISARM,0,0,0,0,4021));assert(!n.mode && !d.target[0] && !d.output[1]);
    assert(navigation_drive_command(&n,&d,4,123,24,0,0,4022)==CONTROL_STATE);
    assert(!drive_command(&d,CMD_ARM,123,25,0,0,4022));request(NAV_START_TURN,26,-90,0,0,0,0);
    assert(!navigation_command(&n,&im,&d,w,packet,4022));fresh(&im,&d,w,19022);navigation_tick(&n,&im,&d,w,19022);
    assert(n.result==NAV_TIMEOUT && d.state==DRIVE_DISARMED);
}
/** 执行生产算法的离线边界测试，不代表实物精度。 */
int main(void) { test_imu();test_navigation();puts("PASS: IMU calibration, odometry, navigation and safety (HOST ONLY)");return 0; }
