"""用HAL替身运行真实MPU驱动，注入BUSY、超时、错误身份与重复数据。"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'out/mpu_test'
OUT.mkdir(exist_ok=True)
header = r'''
#include <stdint.h>
typedef struct {uint32_t SR2,IDR,CR1;} Reg;
extern Reg i2c,gpio;
#define I2C2 (&i2c)
#define GPIOB (&gpio)
typedef struct {int ClockSpeed,DutyCycle,AddressingMode,DualAddressMode,GeneralCallMode,NoStretchMode;} Init;
typedef struct {Reg *Instance;Init Init;int State,Lock,Mode,ErrorCode;uint8_t *pBuffPtr;uint32_t XferCount,XferSize,XferOptions,Devaddress,Memaddress,MemaddSize,EventCount;} I2C_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_UNLOCKED 0
#define HAL_I2C_STATE_BUSY_TX 2
#define HAL_I2C_STATE_BUSY_RX 3
#define HAL_I2C_MODE_MEM 1
#define HAL_I2C_ERROR_NONE 0
#define I2C_CR1_POS 2
#define I2C_CR1_PE 4
#define I2C_CR1_ACK 8
#define I2C_CR1_START 16
#define I2C_IT_EVT 1
#define I2C_IT_BUF 2
#define I2C_IT_ERR 4
void record_start(I2C_HandleTypeDef *h);
#define __HAL_I2C_ENABLE_IT(h,flags) record_start(h)
#define HAL_I2C_STATE_READY 1
#define I2C_DUTYCYCLE_2 0
#define I2C_ADDRESSINGMODE_7BIT 0
#define I2C_DUALADDRESS_DISABLE 0
#define I2C_GENERALCALL_DISABLE 0
#define I2C_NOSTRETCH_DISABLE 0
#define I2C_MEMADD_SIZE_8BIT 1
#define I2C2_EV_IRQn 0
#define I2C2_ER_IRQn 1
#define I2C_SR2_BUSY 1
#define GPIO_PIN_10 1024
#define GPIO_PIN_11 2048
#define HAL_NVIC_DisableIRQ(x) ((void)(x))
#define HAL_NVIC_EnableIRQ(x) ((void)(x))
#define HAL_NVIC_ClearPendingIRQ(x) ((void)(x))
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_FORCE_RESET() ((void)0)
#define __HAL_RCC_I2C2_RELEASE_RESET() ((void)0)
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
int HAL_I2C_Init(I2C_HandleTypeDef *);
int HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t);
int HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t);
'''
test = r'''
#include "main.h"
#include "mpu6050.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
extern I2C_HandleTypeDef hi2c2;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);
Reg i2c,gpio;static uint32_t tick,mask,calls;static uint8_t *buffer;static uint16_t reg;
/** 提供可精确推进的时基。 */
uint32_t HAL_GetTick(void){return tick;}
/** 保留驱动的中断屏蔽约定。 */
uint32_t __get_PRIMASK(void){return mask;}
/** 模拟屏蔽，不注入异步回调。 */
void __disable_irq(void){mask=1;}
/** 恢复原中断状态。 */
void __set_PRIMASK(uint32_t x){mask=x;}
/** 初始化替身只设READY，不执行硬件动作。 */
int HAL_I2C_Init(I2C_HandleTypeDef *h){h->State=1;return 0;}
/** 捕获实际中断启动参数，不替代生产驱动状态机。 */
void record_start(I2C_HandleTypeDef *h)
{assert(h->Devaddress==0xd0);assert(!(i2c.SR2&1));assert(mask==1);assert(h->XferOptions==0xffff0000U);assert(h->Instance->CR1&I2C_CR1_START);calls++;buffer=h->pBuffPtr;reg=(uint16_t)h->Memaddress;}
/** 验证忙总线不进入HAL等待、超时恢复及DATA_RDY门控。 */
int main(void){
 imu_state im;unsigned n;imu_init(&im);gpio.IDR=3072;mpu6050_init();
 tick=100;i2c.SR2=1;mpu6050_poll(&im,tick,1);assert(calls==0 && mask==0);
 tick=101;mpu6050_poll(&im,tick,1);assert(mpu_status.errors==1 && !im.seen);
 i2c.SR2=0;tick=1101;mpu6050_poll(&im,tick,1);assert(calls==1);
 tick=1106;mpu6050_poll(&im,tick,1);assert(mpu_status.timeouts==1 && !mpu_status.ready);
 tick=2106;mpu6050_poll(&im,tick,1);assert(reg==0x6b);
 for(n=0;n<6;n++){hi2c2.State=1;HAL_I2C_MemTxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);if(n<2){tick+=100;mpu6050_poll(&im,tick,1);}}
 assert(reg==0x75);buffer[0]=0x68;hi2c2.State=1;HAL_I2C_MemRxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);
 assert(mpu_status.ready && reg==0x3a);
 memset(buffer,0,15);buffer[5]=0x20;buffer[0]=1;hi2c2.State=1;HAL_I2C_MemRxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);
 assert(im.sequence==1 && im.raw[2]==8192);
 tick+=10;mpu6050_poll(&im,tick,1);buffer[0]=0;hi2c2.State=1;HAL_I2C_MemRxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);assert(im.sequence==1);
 HAL_I2C_ErrorCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);assert(!mpu_status.ready && !im.seen && im.calibration==IMU_UNCALIBRATED);
 tick+=1000;mpu6050_poll(&im,tick,1);
 for(n=0;n<6;n++){hi2c2.State=1;HAL_I2C_MemTxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);if(n<2){tick+=100;mpu6050_poll(&im,tick,1);}}
 assert(reg==0x75);buffer[0]=0;hi2c2.State=1;HAL_I2C_MemRxCpltCallback(&hi2c2);tick++;mpu6050_poll(&im,tick,1);tick++;mpu6050_poll(&im,tick,1);assert(!mpu_status.ready);
 puts("PASS: real MPU driver BUSY/5ms timeout/retry/identity/DATA_RDY (HAL stub only)");return 0;
}
'''
(OUT/'main.h').write_text(header, encoding='utf-8')
(OUT/'test.c').write_text(test, encoding='utf-8')
app = ROOT/'firmware/stm32_app'
subprocess.run(['C:/msys64/ucrt64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-I'+str(OUT), '-I'+str(app/'Core/Inc'), str(app/'Core/Src/mpu6050.c'),
                str(app/'Core/Src/imu.c'), str(OUT/'test.c'), '-lm', '-o', str(OUT/'test.exe')], check=True)
subprocess.run([str(OUT/'test.exe')], check=True)
