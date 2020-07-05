#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
#define ZHONGZHI 3

extern int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

int EXTI15_10_IRQHandler(void);
int velocity(int encoder_left, int encoder_right);
void Set_Pwm(void);
u8 Turn_Off(void);
void Get_Angle(void);
void pid_velocity_weizhi(void);
void pid_velocity_zengliang(void);
void ReportEncoderBattery(void);

#endif
