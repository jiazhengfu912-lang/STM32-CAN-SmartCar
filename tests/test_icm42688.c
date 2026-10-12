#include "main.h"
#include "icm42688.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
extern I2C_HandleTypeDef hi2c2;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);
Reg i2c_regs,gpio;
static uint32_t tick,mask,calls,reset_at,power_at;
static uint8_t mem[128],present,identity,corrupt,reset_done,pending,writing,reg,size;
static uint8_t *buffer;
static FILE *csv;
static const char *scenario;

/** 提供受测试控制的毫秒时基。 */
uint32_t HAL_GetTick(void) { return tick; }
/** 生产驱动使用相同的BSP时基。 */
uint32_t BSP_System_Millis(void) { return tick; }
/** 保存并恢复中断屏蔽状态。 */
uint32_t __get_PRIMASK(void) { return mask; }
/** 模拟屏蔽中断。 */
void __disable_irq(void) { mask=1; }
/** 恢复调用者原状态。 */
void __set_PRIMASK(uint32_t x) { mask=x; }
/** 总线复位只改变HAL状态，不替换生产BSP。 */
int HAL_I2C_Init(I2C_HandleTypeDef *h) { h->State=1; pending=0; return 0; }
/** 捕获真实BSP启动参数，断言没有忙等或错误地址移位。 */
void record_start(I2C_HandleTypeDef *h)
{
    assert(h->Devaddress==0xd0 || h->Devaddress==0xd2);
    assert(mask==1 && !(i2c_regs.SR2&1) && h->XferOptions==0xffff0000U);
    assert(h->Instance->CR1&I2C_CR1_START);
    if (power_at) assert(tick-power_at>=50U);
    if (reset_at) assert(tick-reset_at>=2U);
    ++calls;buffer=h->pBuffPtr;reg=(uint8_t)h->Memaddress;size=(uint8_t)h->XferSize;
    writing=h->State==HAL_I2C_STATE_BUSY_TX;pending=1;
}
/** 完成一笔模拟芯片事务，写入值来自手册定义的寄存器模型。 */
static void complete(void)
{
    if (!pending) return;
    pending=0;
    if ((hi2c2.Devaddress>>1)!=present) { HAL_I2C_ErrorCallback(&hi2c2); return; }
    if (writing) {
        mem[reg]=buffer[0];
        if (reg==0x11) { memset(mem,0,sizeof(mem));mem[0x2d]=reset_done ? 0x10:0;reset_at=tick; }
        if (reg==0x4e) power_at=tick;
        hi2c2.State=1;HAL_I2C_MemTxCpltCallback(&hi2c2);
    } else {
        if (reg==0x75) buffer[0]=identity;
        else memcpy(buffer,mem+reg,size);
        if (corrupt && reg==0x4f) buffer[0]^=1;
        if (reg==0x2d || reg==0x1d) mem[0x2d]=0;
        hi2c2.State=1;HAL_I2C_MemRxCpltCallback(&hi2c2);
    }
}
/** 每个虚拟毫秒保存一行原样状态，包含失败与缺测，不使用实物标签。 */
static void poll(imu_state *im,int stationary)
{
    icm42688_poll(im,tick,stationary);
    assert(mask==0);
    fprintf(csv,"%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.6f\n",
        scenario,tick,icm_status.address,reg,icm_status.ready,icm_status.errors,
        icm_status.timeouts,icm_status.restarts,im->sequence,imu_fresh(im,tick),im->calibration,im->gaps,
        im->accel[2],im->gyro[2],im->temperature);
}
/** 创建互相隔离的芯片/时基/故障条件。 */
static void setup(imu_state *im,const char *name,uint8_t address,uint32_t now)
{
    scenario=name;tick=now;present=address;identity=0x47;reset_done=1;corrupt=0;
    calls=mask=reset_at=power_at=0;reg=size=writing=0;
    memset(mem,0,sizeof(mem));memset(&icm_status,0,sizeof(icm_status));
    memset(&i2c_regs,0,sizeof(i2c_regs));gpio.IDR=3072;
    imu_init(im);icm42688_init();
}
/** 将实际异步状态机运行到配置完成；限时防止测试自身无限循环。 */
static void boot(imu_state *im)
{
    unsigned n;for(n=0;n<400 && !icm_status.ready;n++,tick++) { poll(im,1);complete(); }
    assert(icm_status.ready && icm_status.identity==0x47 && im->sequence==0);
    assert(mem[0x4c]==0x32 && mem[0x4f]==0x48 && mem[0x50]==0x48);
    assert(mem[0x51]==0x16 && mem[0x52]==0x66 && mem[0x53]==0x0d);
    assert(mem[0x62]==0 && mem[0x63]==0x30 && mem[0x14]==7);
    assert(mem[0x64]==0 && mem[0x65]==8 && mem[0x4e]==15);
    assert(tick-power_at>=50U);
}
/** 按ICM的温度在前线序写大端测试原始值。 */
static void word(unsigned i,int16_t value)
{
    mem[0x1d+2*i]=(uint8_t)((uint16_t)value>>8);mem[0x1e + 2*i]=(uint8_t)value;
}
/** 只注入一个DRDY事件，让真实驱动完成一次解析。 */
static void sample(imu_state *im)
{
    unsigned n;uint32_t seq=im->sequence;
    mem[0x2d]=8;
    for(n=0;n<30 && im->sequence==seq;n++,tick++) { poll(im,1);complete(); }
    assert(im->sequence==seq+1);
}
/** 验证身份、地址、线序、量程、温度、重复就绪门控和五秒校准。 */
static void test_samples(void)
{
    imu_state im;unsigned n;uint32_t seq;
    setup(&im,"sample-and-calibration",0x68,0);boot(&im);
    word(0,3312);word(1,-8192);word(2,4096);word(3,8192);
    word(4,16384);word(5,-16384);word(6,65);sample(&im);
    assert(im.raw[0]==-8192 && im.raw[3]==3312 && im.raw[5]==-16384);
    assert(im.accel[0]==-1 && im.accel[1]==.5f && im.accel[2]==1);
    assert(fabsf(im.gyro[0]-4.363323f)<.00001f && fabsf(im.gyro[1]+4.363323f)<.00001f);
    assert(fabsf(im.temperature-50)<.00001f);
    seq=im.sequence;for(n=0;n<60;n++,tick++) { poll(&im,1);complete(); }
    assert(im.sequence==seq && !imu_fresh(&im,tick));
    word(1,0);word(2,0);word(4,0);word(5,0);sample(&im);
    assert(imu_calibrate(&im,tick));
    for(n=0;n<510;n++) { sample(&im); }
    assert(im.calibration==IMU_CALIBRATED && fabsf(im.gyro[2])<.0001f);
    assert(imu_calibrate(&im,tick));poll(&im,0);assert(im.calibration==IMU_CAL_FAILED);
    assert(imu_calibrate(&im,tick));im.bias[2]=.2f;
    HAL_I2C_ErrorCallback(&hi2c2);poll(&im,1);
    assert(!icm_status.ready && !im.seen && !im.attitude_ready);
    assert(im.calibration==IMU_UNCALIBRATED && im.bias[2]==0);
    setup(&im,"address-69",0x69,0);boot(&im);assert(icm_status.address==0x69 && icm_status.errors==1);
    setup(&im,"wrap",0x68,0xffffff80U);boot(&im);
    word(0,-3312);word(3,8192);sample(&im);assert(fabsf(im.temperature)<.00001f);
}
/** 验证NACK、两地址识别失败、配置错误、复位未完成与非法样本均有界恢复。 */
static void test_failures(void)
{
    imu_state im;unsigned n;uint32_t seq;
    setup(&im,"wrong-identity",0x68,0);identity=0x68;
    for(n=0;n<300;n++,tick++) { poll(&im,1);complete(); }
    assert(!icm_status.ready && icm_status.restarts==1 && im.sequence==0 && power_at==0);
    setup(&im,"config-mismatch",0x68,0);corrupt=1;
    for(n=0;n<300;n++,tick++) { poll(&im,1);complete(); }
    assert(!icm_status.ready && icm_status.restarts==1 && power_at==0);
    setup(&im,"reset-not-done",0x68,0);reset_done=0;
    for(n=0;n<300;n++,tick++) { poll(&im,1);complete(); }
    assert(!icm_status.ready && icm_status.restarts==1 && power_at==0);
    setup(&im,"busy-and-timeout",0x68,0);i2c_regs.SR2=1;tick=100;poll(&im,1);
    assert(calls==0 && !mask);tick++;poll(&im,1);assert(icm_status.address==0x69);
    i2c_regs.SR2=0;tick++;poll(&im,1);assert(calls==1);
    tick+=4;poll(&im,1);assert(icm_status.timeouts==0);
    tick++;poll(&im,1);assert(icm_status.timeouts==1 && icm_status.restarts==1);
    seq=calls;tick+=999;poll(&im,1);assert(calls==seq);
    tick++;poll(&im,1);assert(calls==seq+1 && icm_status.address==0x68);
    setup(&im,"invalid-sample",0x68,0);boot(&im);word(3,-32768);mem[0x2d]=8;
    for(n=0;n<40;n++,tick++) { poll(&im,1);complete(); }
    assert(im.sequence==0 && !im.seen && !icm_status.ready && icm_status.restarts==1);
    setup(&im,"calibration-data-gap",0x68,0);boot(&im);word(3,8192);sample(&im);
    assert(imu_calibrate(&im,tick));
    for(n=0;n<50;n++,tick++) { poll(&im,1);complete(); }
    assert(im.calibration==IMU_CAL_FAILED && !imu_fresh(&im,tick));
    setup(&im,"sda-low",0x68,0);gpio.IDR=1024;tick=100;poll(&im,1);
    assert(calls==0 && !mask);
}
/** 所有CSV均为HAL替身注入，不能作为真实传感器或小车验收证据。 */
int main(void)
{
    csv=fopen("out/icm_test/driver-trace.csv","w");assert(csv);
    fprintf(csv,"scenario,time_ms,address,register,ready,errors,timeouts,restarts,sequence,fresh,calibration,gaps,az_g,gz_rad_s,temperature_c\n");
    test_samples();test_failures();fclose(csv);
    puts("PASS: ICM-42688-P real driver/BSP identity, bank, readback, startup timing, DRDY, conversion, calibration, NACK, BUSY, timeout, retry and wrap (HOST ONLY)");
    return 0;
}
