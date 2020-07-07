#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
#define ENCODER_LINES   (500*30*4)  //500线，减速比30，4倍频
#define CONTROL_FREQ    (50)        //PID控制频率
#define WHEEL_PERIMETER (0.21)      //轮直径0.21m
#define SPEED_LIMIT     (0.3)       //限速0.3m/s
#define SPEED_LIMIT_BY_ENCODER  (int)(ENCODER_LINES * SPEED_LIMIT / WHEEL_PERIMETER / CONTROL_FREQ)
#define DEFAULT_KP      (1.453)
#define DEFAULT_KI      (0.58)
#define DEFAULT_KD      (0)
#define DEFAULT_VEL     (500)       //默认蓝牙遥控时的行驶速度

extern u8 report_flag;

extern s32 SetSpeedL, SetSpeedR;
extern s32 DesireL, DesireR;
extern int Encoder_Left,Encoder_Right;
extern int Moto1,Moto2;
extern float Velocity_Kp,Velocity_Ki,Velocity_Kd;
extern s32 pulse_cnt;

int EXTI15_10_IRQHandler(void);
int velocity(int encoder_left, int encoder_right);
void Set_Pwm(void);
u8 Turn_Off(void);
void Get_Angle(void);
void pid_velocity_weizhi(void);
void pid_velocity_zengliang(void);
void ReportEncoderBattery(void);

#endif
