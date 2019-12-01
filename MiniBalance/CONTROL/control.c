#include <math.h>
#include "control.h"
#include "filter.h"

u8 control_cnt = 0;
u8 control_fre = 20;
int Encoder_Left_Once = 0;
int Encoder_Right_Once = 0;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI9_5_IRQHandler(void) {
  if (PBin(5) == 0) {
    EXTI->PR = 1 << 5;  //清除LINE5上的中断标志位

    control_cnt++;
    Encoder_Left_Once += Read_Encoder(2);
    Encoder_Right_Once += -Read_Encoder(4);
  
    if (control_cnt == 200/control_fre) {
      Encoder_Left = Encoder_Left_Once;
      Encoder_Right = Encoder_Right_Once;
      Encoder_Left_Once = 0;
      Encoder_Right_Once = 0;
      control_cnt = 0;

      pid_velocity_weizhi();
      //pid_velocity_zengliang();
      if (!Turn_Off()) {
        Set_Pwm();
      }

      report_flag = 1;
    }
  
    Led_Flash(200/led_freq);
  }
  return 0;
}

void pid_velocity_weizhi(void) {
  static int IntegralL = 0, IntegralR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  int desire_left  = DesireL;
	int desire_right = DesireR;
  if (abs(desire_left)<=speed_limit && abs(desire_right)<=speed_limit) {
    led_freq = 1;
  }
 	if (desire_left >  speed_limit) {
    desire_left  =  speed_limit;
    led_freq=10;
  } else if (desire_left < -speed_limit) {
    desire_left  = -speed_limit;
    led_freq=10;
  }
	if (desire_right>  speed_limit) {
    desire_right =  speed_limit;
    led_freq=10;
  } else if (desire_right< -speed_limit) {
    desire_right = -speed_limit;
    led_freq=10;
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
  int desire_left  = DesireL;
	int desire_right = DesireR;
	if (abs(desire_left)<=speed_limit && abs(desire_right)<=speed_limit) {
    led_freq = 1;
  }
 	if (desire_left >  speed_limit) {
    desire_left  =  speed_limit;
    led_freq=10;
  } else if (desire_left < -speed_limit) {
    desire_left  = -speed_limit;
    led_freq=10;
  }
	if (desire_right > speed_limit) {
    desire_right =  speed_limit;
    led_freq=10;
  } else if (desire_right < -speed_limit) {
    desire_right = -speed_limit;
    led_freq=10;
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

/*
int velocity(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity = 130;
  //=============遥控前进后退部分=======================//
  if (1 == Flag_Qian)
    Movement = -Target_Velocity;  //===前进标志位置1
  else if (1 == Flag_Hou)
    Movement = Target_Velocity;  //===后退标志位置1
  else
    Movement = 0;
  if (Bi_zhang == 1 && Distance < 500 && Flag_Left != 1 &&
      Flag_Right != 1)  //避障标志位置1且非遥控转弯的时候，进入避障模式
    Movement = Target_Velocity;
  //=============速度PI控制器=======================//
  Encoder_Least =
      (Encoder_Left + Encoder_Right) -
      0;  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.7;                  //===一阶低通滤波器
  Encoder += Encoder_Least * 0.3;  //===一阶低通滤波器
  Encoder_Integral += Encoder;     //===积分出位移 积分时间：10ms
  Encoder_Integral =
      Encoder_Integral - Movement;  //===接收遥控器数据，控制前进后退
  if (Encoder_Integral > 15000) Encoder_Integral = 15000;    //===积分限幅
  if (Encoder_Integral < -15000) Encoder_Integral = -15000;  //===积分限幅
  Velocity =
      Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;  //===速度控制
  if (Turn_Off(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
    Encoder_Integral = 0;  //===电机关闭后清除积分
  return Velocity;
}
*/

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(void) {
  int siqu = 300;
  int Amplitude = 6900;  //===PWM满幅是7200 限制在6900
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

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
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

/**************************************************************************
函数功能：获取角度 三种算法经过我们的调校，都非常理想
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
运行时间：500us
**************************************************************************/
void Get_Angle(u8 way) {
  float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
  // 读取MPU6050内置温度传感器数据，近似表示主板温度
  //Temperature = Read_Temperature();
  if (way == 1) {
    // DMP的读取在数据采集中断提醒的时候，严格遵循时序要求
    Read_DMP();                 //===读取加速度、角速度、倾角
    Angle_Balance = Pitch;      //===更新平衡倾角
    Gyro_Balance = gyro[1];     //===更新平衡角速度
    Gyro_Turn = gyro[2];        //===更新转向角速度
    Acceleration_Z = accel[2];  //===更新Z轴加速度计
  } else {
    Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) +
             I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);  //读取Y轴陀螺仪
    Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) +
             I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);  //读取Z轴陀螺仪
    Accel_X =
        (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) +
        I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L);  //读取X轴加速度计
    Accel_Z =
        (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) +
        I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);  //读取Z轴加速度计
    if (Gyro_Y > 32768)
      Gyro_Y -= 65536;  //数据类型转换  也可通过short强制类型转换
    if (Gyro_Z > 32768) Gyro_Z -= 65536;           //数据类型转换
    if (Accel_X > 32768) Accel_X -= 65536;         //数据类型转换
    if (Accel_Z > 32768) Accel_Z -= 65536;         //数据类型转换
    Gyro_Balance = -Gyro_Y;                        //更新平衡角速度
    Accel_Y = atan2(Accel_X, Accel_Z) * 180 / PI;  //计算倾角
    Gyro_Y = Gyro_Y / 16.4;                        //陀螺仪量程转换
    if (Way_Angle == 2)
      Kalman_Filter(Accel_Y, -Gyro_Y);  //卡尔曼滤波
    else if (Way_Angle == 3)
      Yijielvbo(Accel_Y, -Gyro_Y);  //互补滤波
    Angle_Balance = angle;          //更新平衡倾角
    Gyro_Turn = Gyro_Z;             //更新转向角速度
    Acceleration_Z = Accel_Z;       //===更新Z轴加速度计
  }
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
