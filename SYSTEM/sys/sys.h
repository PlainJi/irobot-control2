#ifndef __SYS_H
#define __SYS_H

#include <stm32f10x.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "delay.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "usart.h"
#include "usart2.h"
#include "usart3.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "show.h"					
#include "exti.h"
#include "DataScope_DP.h"
#include "stmflash.h"  
#include "control.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)

#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1
#define RTIR   2

#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00

extern u16 led_freq;
extern int Voltage;
extern int Temperature;

extern int Encoder_Left,Encoder_Right;
extern int Moto1,Moto2;

extern s32 DesireL, DesireR;
extern s32 SpeedL, SpeedR;
extern float Velocity_Kp,Velocity_Ki,Velocity_Kd;
extern u8 speed_limit;

extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu;
extern u8 Flag_Stop,report_flag, bluetooth_report, save_flag, send_param_flag;

void Stm32_Clock_Init(u8 PLL);
void Sys_Soft_Reset(void);
void Sys_Standby(void); 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);
void JTAG_Set(u8 mode);

void WFI_SET(void);
void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(u32 addr);

#endif
