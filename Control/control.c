#include <math.h>

#include "sys.h"

u8 report_flag = 0;
u8 control_cnt = 0;

s32 SetSpeedL = 50, SetSpeedR = 50;                   //通过蓝牙设置速度
s32 DesireL = 0, DesireR = 0;                         //上位机通过UART2下发的期望速度
int Encoder_Left_Once = 0, Encoder_Right_Once = 0;    //每5ms的编码器计数
s32 Encoder_Left = 0, Encoder_Right = 0;              //控制周期内的编码器计数
s32 Moto1, Moto2;                                     //电机PWM输出参数
float Velocity_Kp = 1.453, Velocity_Ki = 0.58, Velocity_Kd = 0;

/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI9_5_IRQHandler(void) {
  if (PBin(5) == 0) {
    EXTI->PR = 1 << 5;  //清除LINE5上的中断标志位

    control_cnt++;
    Encoder_Left_Once += -Read_Encoder(2);
    Encoder_Right_Once += Read_Encoder(4);

    if (control_cnt == 200 / CONTROL_FREQ) {
      Encoder_Left = Encoder_Left_Once;
      Encoder_Right = Encoder_Right_Once;
      Encoder_Left_Once = 0;
      Encoder_Right_Once = 0;
      control_cnt = 0;

      //pid_velocity_weizhi();
      pid_velocity_zengliang();
      if (!Turn_Off()) {
        Set_Pwm();
      }

      report_flag = 1;
    }

    Led_Flash(200 / led_freq);
  }
  return 0;
}

void pid_velocity_weizhi(void) {
  static int IntegralL = 0, IntegralR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  int desire_left = DesireL;
  int desire_right = DesireR;
  if (abs(desire_left) <= SPEED_LIMIT_BY_ENCODER && abs(desire_right) <= SPEED_LIMIT_BY_ENCODER) {
    led_freq = 1;
  }
  if (desire_left > SPEED_LIMIT_BY_ENCODER) {
    desire_left = SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  } else if (desire_left < -SPEED_LIMIT_BY_ENCODER) {
    desire_left = -SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  }
  if (desire_right > SPEED_LIMIT_BY_ENCODER) {
    desire_right = SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  } else if (desire_right < -SPEED_LIMIT_BY_ENCODER) {
    desire_right = -SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  }

  ErrorL = ((int)desire_left - Encoder_Left);
  ErrorR = ((int)desire_right - Encoder_Right);
  if (Flag_Stop) {
    Moto1 = 0, Moto2 = 0;
    IntegralL = 0, IntegralR = 0;
  } else {
    IntegralL += ErrorL;
    IntegralR += ErrorR;
    Moto1 = Velocity_Kp * ErrorL + Velocity_Ki * IntegralL +
            Velocity_Kd * (ErrorL - LastErrorL);
    Moto2 = Velocity_Kp * ErrorR + Velocity_Ki * IntegralR +
            Velocity_Kd * (ErrorR - LastErrorR);
  }

  LastErrorL = ErrorL;
  LastErrorR = ErrorR;
  if (IntegralL > 360000) IntegralL = 360000;
  if (IntegralL < -360000) IntegralL = -360000;
  if (IntegralR > 360000) IntegralR = 360000;
  if (IntegralR < -360000) IntegralR = -360000;
}

void pid_velocity_zengliang(void) {
  static int PreErrorL = 0, PreErrorR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  int desire_left = DesireL;
  int desire_right = DesireR;
  if (abs(desire_left) <= SPEED_LIMIT_BY_ENCODER && abs(desire_right) <= SPEED_LIMIT_BY_ENCODER) {
    led_freq = 1;
  }
  if (desire_left > SPEED_LIMIT_BY_ENCODER) {
    desire_left = SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  } else if (desire_left < -SPEED_LIMIT_BY_ENCODER) {
    desire_left = -SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  }
  if (desire_right > SPEED_LIMIT_BY_ENCODER) {
    desire_right = SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  } else if (desire_right < -SPEED_LIMIT_BY_ENCODER) {
    desire_right = -SPEED_LIMIT_BY_ENCODER;
    led_freq = 10;
  }

  ErrorL = ((int)desire_left - Encoder_Left);
  ErrorR = ((int)desire_right - Encoder_Right);
  if (Flag_Stop) {
    Moto1 = 0, Moto2 = 0;
  } else {
    Moto1 += Velocity_Kp * (ErrorL - LastErrorL) + Velocity_Ki * ErrorL +
             Velocity_Kd * (ErrorL - 2 * LastErrorL + PreErrorL);
    Moto2 += Velocity_Kp * (ErrorR - LastErrorR) + Velocity_Ki * ErrorR +
             Velocity_Kd * (ErrorR - 2 * LastErrorR + PreErrorR);
  }

  PreErrorL = LastErrorL;
  PreErrorR = LastErrorR;
  LastErrorL = ErrorL;
  LastErrorR = ErrorR;
}

void Set_Pwm(void) {
  int siqu = 100;
  int Amplitude = 6900;  // PWM满幅是7200 限制在6900
  if (Moto1 < -Amplitude) Moto1 = -Amplitude;
  if (Moto1 > Amplitude) Moto1 = Amplitude;
  if (Moto2 < -Amplitude) Moto2 = -Amplitude;
  if (Moto2 > Amplitude) Moto2 = Amplitude;

  if (Moto1 < 0)
    AIN1 = 1, AIN2 = 0;
  else
    AIN1 = 0, AIN2 = 1;
  PWMA = abs(Moto1) + siqu;

  if (Moto2 < 0)
    BIN1 = 1, BIN2 = 0;
  else
    BIN1 = 0, BIN2 = 1;
  PWMB = abs(Moto2) + siqu;
}

u8 Turn_Off(void) {
  u8 temp;
  //电池电压低于11.1V关闭电机
  if (Voltage < 1110 || Flag_Stop) {
    temp = 1;
    AIN1 = 0;
    AIN2 = 0;
    BIN1 = 0;
    BIN2 = 0;
  } else
    temp = 0;
  return temp;
}

void ReportEncoderBattery(void) {
  static u8 cnt = 0;
  static long voltage_temp = 0;

  USART2_ReportEncoder(Encoder_Left, Encoder_Right);
  app_1_encL_report = Encoder_Left;
  app_2_encR_report = Encoder_Right;
  app_4_angle_report = Yaw;

  cnt++;
  voltage_temp += Get_battery_volt();
  if (cnt == 200) {
    Voltage = voltage_temp / cnt;
    USART2_ReportBattery(Voltage);
    cnt = 0;
    voltage_temp = 0;
  }
}
