#include "drive.h"
#include <assert.h>
#include <stdio.h>

/** 同时注入两轮静止反馈，构造可安全解锁的测试前提。 */
static void feedback(drive_state *s,uint32_t t)
{ drive_feedback(s,0,0,0,t); drive_feedback(s,1,0,0,t); }

/** 使用实际运动控制模块测试超时、重放、停稳门槛、故障恢复和时钟回绕。 */
int main(void)
{
    drive_state s; unsigned i;
    drive_init(&s); assert(s.state==DRIVE_DISARMED);
    assert(drive_command(&s,CMD_ARM,12,1,0,0,0)==CONTROL_FEEDBACK);
    assert(!drive_can_update(&s,1000));
    feedback(&s,10);
    assert(drive_command(&s,CMD_ARM,12,1,1,0,10)==CONTROL_STATE);
    assert(drive_command(&s,CMD_ARM,12,1,0,0,10)==CONTROL_OK);
    assert(drive_command(&s,CMD_SPEED,99,2,100,100,20)==CONTROL_STATE);
    assert(drive_command(&s,CMD_SPEED,12,2,721,0,20)==CONTROL_ARGUMENT);
    assert(drive_command(&s,CMD_SPEED,12,2,360,-360,20)==CONTROL_OK);
    drive_tick(&s,20); assert(s.output[0]>0 && s.output[1]<0);
    assert(drive_command(&s,CMD_SPEED,12,2,0,0,21)==CONTROL_SEQUENCE);
    assert(drive_command(&s,CMD_SPEED,12,1,0,0,22)==CONTROL_SEQUENCE);
    assert(s.command_ms==20 && s.target[0]==360);
    feedback(&s,320);
    assert(drive_command(&s,CMD_SPEED,12,3,360,360,320)==CONTROL_STATE);
    assert(s.fault==FAULT_COMMAND && !s.output[0] && !s.target[0]);
    assert(drive_command(&s,CMD_DISARM,0,0,0,0,321)==CONTROL_OK && s.state==DRIVE_FAULT);
    assert(drive_command(&s,CMD_ARM,23,1,0,0,322)==CONTROL_STATE);
    assert(drive_command(&s,CMD_CLEAR,0,0,0,0,322)==CONTROL_OK);
    assert(s.state==DRIVE_DISARMED && !s.seen[0]);
    feedback(&s,400); drive_command(&s,CMD_ARM,23,1,0,0,400);
    drive_tick(&s,500); assert(s.fault==FAULT_FEEDBACK && !s.output[1]);
    drive_init(&s);
    for(i=0;i<=500;i+=10) { feedback(&s,i); drive_tick(&s,i); }
    assert(drive_can_update(&s,500)); assert(!drive_can_update(&s,600));
    drive_feedback(&s,1,200,0,510); drive_tick(&s,510); assert(!drive_can_update(&s,510));
    drive_init(&s); feedback(&s,0xFFFFFFE0U);
    assert(drive_command(&s,CMD_ARM,45,1,0,0,0xFFFFFFE0U)==0);
    feedback(&s,0x20); drive_tick(&s,0x20); assert(s.state==DRIVE_ARMED);
    feedback(&s,0x110); drive_tick(&s,0x110); assert(s.fault==FAULT_COMMAND);
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,7,1,0,0,0);
    for(i=1;i<1000;i++) {
        feedback(&s,i*10); drive_command(&s,CMD_SPEED,7,i+1,720,-720,i*10);drive_tick(&s,i*10);
        assert(s.output[0]<=2000 && s.output[1]>=-2000);
    }
    drive_command(&s,CMD_DISARM,0,0,0,0,10000);assert(!s.output[0] && !s.integral[0]);
    /* 重放三倍超速：与相同积分、无速度误差的状态比较，不要求一拍内反向。 */
    {
        drive_state reference;
        drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,8,1,0,0,0);
        drive_command(&s,CMD_SPEED,8,2,180,-180,10);
        /* 此项检查已进入匀速阶段的超速纠偏；起转限幅在后面的独立用例验证。 */
        s.starting[0]=0; s.starting[1]=0;
        s.integral[0]=250; s.integral[1]=-250;
        reference=s;
        drive_feedback(&reference,0,180,0,10);drive_feedback(&reference,1,-180,0,10);
        drive_tick(&reference,10);
        drive_feedback(&s,0,540,0,10);drive_feedback(&s,1,-540,0,10);
        drive_tick(&s,10);
        assert(s.state==DRIVE_ARMED && s.fault==FAULT_NONE);
        assert(s.integral[0]<reference.integral[0] && s.integral[1]>reference.integral[1]);
        assert(s.output[0]<reference.output[0] && s.output[1]>reference.output[1]);
        /* 没有历史积分时，正反向超速都应产生与运动方向相反的纠偏输出。 */
        s.integral[0]=0; s.integral[1]=0;
        drive_tick(&s,20);
        assert(s.output[0]<0 && s.output[1]>0);
    }
    /* 目标附近等幅正负误差应对称积分，不能因超速泄放形成持续偏慢。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,9,1,0,0,0);
    drive_command(&s,CMD_SPEED,9,2,360,360,10);
    s.integral[0]=100; s.integral[1]=100;
    drive_feedback(&s,0,330,0,10);drive_feedback(&s,1,390,0,10);
    drive_tick(&s,10);
    assert(s.integral[0]>100 && s.integral[1]<100);
    assert(s.integral[0]+s.integral[1]>199.99f && s.integral[0]+s.integral[1]<200.01f);
    /* 检测到同方向起转后释放过大积分；续发心跳不能重复触发。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,10,1,0,0,0);
    drive_command(&s,CMD_SPEED,10,2,360,-360,10);
    s.integral[0]=350; s.integral[1]=-350;
    drive_feedback(&s,0,40,0,10);drive_feedback(&s,1,-40,0,10);
    drive_tick(&s,10);
    assert(!s.starting[0] && !s.starting[1]);
    assert(s.integral[0]<160 && s.integral[1]>-160);
    assert(s.output[0]<=154 && s.output[1]>=-184);
    drive_command(&s,CMD_SPEED,10,3,360,-360,20);
    assert(!s.starting[0] && !s.starting[1]);
    /* 低速大于50%的等幅速度误差也应对称积分，不能只加速泄放维持转动的电流。 */
    s.target[0]=180; s.target[1]=180; s.integral[0]=150; s.integral[1]=150;
    drive_feedback(&s,0,60,0,20);drive_feedback(&s,1,300,0,20);
    drive_tick(&s,20);
    assert(s.integral[0]>150 && s.integral[1]<150);
    assert(s.integral[0]+s.integral[1]>299.99f && s.integral[0]+s.integral[1]<300.01f);
    /* 短暂起动后又停住：连续低速100ms后再次起转仍须削减积分，覆盖正反两轮。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,11,1,0,0,0);
    drive_command(&s,CMD_SPEED,11,2,180,-180,10);
    s.integral[0]=350; s.integral[1]=-350;
    drive_feedback(&s,0,40,0,10);drive_feedback(&s,1,-40,0,10);drive_tick(&s,10);
    assert(!s.starting[0] && !s.starting[1]);
    for(i=20;i<=120;i+=10) {
        feedback(&s,i);drive_tick(&s,i);
    }
    s.integral[0]=350; s.integral[1]=-350;
    drive_feedback(&s,0,40,0,130);drive_feedback(&s,1,-40,0,130);drive_tick(&s,130);
    assert(s.output[0]<=154 && s.output[1]>=-184);
    /* 短于100ms的瞬时低速不能反复切断稳态积分。 */
    feedback(&s,140);drive_tick(&s,140);
    s.integral[0]=300; s.integral[1]=-300;
    drive_feedback(&s,0,180,0,150);drive_feedback(&s,1,-180,0,150);drive_tick(&s,150);
    assert(s.output[0]==300 && s.output[1]==-300);
    drive_command(&s,CMD_DISARM,0,0,0,0,160);
    assert(!s.output[0] && !s.output[1] && !s.integral[0] && !s.integral[1]);
    /* 5rpm启动需更快克服静摩擦，同时正反向对称且堵转启动输出不超过450。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,12,1,0,0,0);
    for(i=10;i<=300;i+=10) {
        feedback(&s,i);drive_command(&s,CMD_SPEED,12,i+1,180,-180,i);drive_tick(&s,i);
    }
    assert(s.output[0]>=200 && s.output[1]<=-200);
    for(i=310;i<=1500;i+=10) {
        feedback(&s,i);drive_command(&s,CMD_SPEED,12,i+1,180,-180,i);drive_tick(&s,i);
        assert(s.output[0]<=450 && s.output[1]>=-450);
    }
    drive_feedback(&s,0,40,0,1510);drive_feedback(&s,1,-40,0,1510);drive_tick(&s,1510);
    assert(s.output[0]<=154 && s.output[1]>=-184);
    /* 起转后回到原稳态积分，不能将启动增益带入速度波动纠偏。 */
    s.integral[0]=100;s.integral[1]=-100;
    drive_feedback(&s,0,170,0,1520);drive_feedback(&s,1,-170,0,1520);drive_tick(&s,1520);
    assert(s.integral[0]>100.09f && s.integral[0]<100.11f);
    assert(s.integral[1]<-100.09f && s.integral[1]>-100.11f);
    /* 右轮反转增强比例纠偏，但不能改变同轮积分规律或左轮参数。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,13,1,0,0,0);
    drive_command(&s,CMD_SPEED,13,2,-432,-432,10);
    s.starting[0]=0;s.starting[1]=0;
    s.integral[0]=-180;s.integral[1]=-180;
    drive_feedback(&s,0,-396,0,10);drive_feedback(&s,1,-396,0,10);drive_tick(&s,10);
    assert(s.output[1]<s.output[0]);
    assert(s.integral[0]==s.integral[1]);
    /* 换回正转，两轮对相同误差的比例输出恢复一致。 */
    drive_command(&s,CMD_SPEED,13,3,432,432,20);
    s.starting[0]=0;s.starting[1]=0;s.integral[0]=180;s.integral[1]=180;
    drive_feedback(&s,0,396,0,20);drive_feedback(&s,1,396,0,20);drive_tick(&s,20);
    assert(s.output[0]==s.output[1]);
    /* 保留1rpm起转门槛：0.5rpm仍需启动支撑，反向反馈不能误判已经起转。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,14,1,0,0,0);
    drive_command(&s,CMD_SPEED,14,2,180,-180,10);
    s.integral[0]=300;s.integral[1]=-300;
    drive_feedback(&s,0,17,0,10);drive_feedback(&s,1,18,0,10);drive_tick(&s,10);
    assert(s.starting[0] && s.starting[1]);
    drive_feedback(&s,0,18,0,20);drive_feedback(&s,1,-18,0,20);drive_tick(&s,20);
    assert(s.starting[0] && s.starting[1]);
    drive_feedback(&s,0,36,0,30);drive_feedback(&s,1,-36,0,30);drive_tick(&s,30);
    assert(!s.starting[0] && !s.starting[1]);
    assert(s.output[0]<=152 && s.output[1]>=-182);
    /* 右轮低速反转允许180支撑，但已有较小积分不强制抬高为电流底限。 */
    drive_init(&s);feedback(&s,0);drive_command(&s,CMD_ARM,15,1,0,0,0);
    drive_command(&s,CMD_SPEED,15,2,-180,-180,10);
    s.integral[0]=-350;s.integral[1]=-350;
    drive_feedback(&s,0,-40,0,10);drive_feedback(&s,1,-40,0,10);drive_tick(&s,10);
    assert(s.output[0]>=-152 && s.output[1]>=-182 && s.output[1]<-179);
    s.starting[1]=1;s.integral[1]=0;drive_tick(&s,20);assert(s.output[1]>-100);
    puts("PASS: real drive core safety boundaries and 999 PI cycles");return 0;
}
