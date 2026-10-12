"""以HAL寄存器替身执行真实ICM驱动和BSP；全部数据为明确的离线注入。"""
from pathlib import Path
import subprocess
import os
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'out/icm_test'
OUT.mkdir(exist_ok=True)
header = r'''

#include <stdint.h>
typedef struct {uint32_t SR2,IDR,CR1;} Reg;
extern Reg i2c_regs,gpio;
#define I2C2 (&i2c_regs)
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

typedef struct {int Pin,Mode,Pull,Speed;} GPIO_InitTypeDef;
#define GPIO_MODE_AF_OD 1
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 1
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define HAL_GPIO_Init(port,p) ((void)(port),(void)(p))
#define HAL_NVIC_SetPriority(a,b,c) ((void)(a),(void)(b),(void)(c))

#define HAL_I2C_EV_IRQHandler(h) ((void)(h))
#define HAL_I2C_ER_IRQHandler(h) ((void)(h))
'''
test=(ROOT/'tests/test_icm42688.c').read_text(encoding='utf-8')
(OUT/'main.h').write_text(header,encoding='utf-8')
(OUT/'test.c').write_text(test,encoding='utf-8')
app=ROOT/'firmware/stm32_app'
subprocess.run([os.environ.get('HOST_GCC','C:/msys64/ucrt64/bin/gcc.exe'),'-std=c99','-Wall','-Wextra','-Werror',
 '-I'+str(OUT),'-I'+str(app/'Core/Inc'),'-I'+str(app/'BSP'),str(app/'Core/Src/icm42688.c'),
 str(app/'BSP/BSP_i2c.c'),str(app/'Core/Src/imu.c'),str(OUT/'test.c'),'-lm','-o',str(OUT/'test.exe')],check=True)
subprocess.run([str(OUT/'test.exe')],check=True,cwd=ROOT)
