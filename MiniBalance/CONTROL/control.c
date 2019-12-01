#include <math.h>
#include "control.h"
#include "filter.h"

u8 control_cnt = 0;
u8 control_fre = 20;
int Encoder_Left_Once = 0;
int Encoder_Right_Once = 0;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��
**************************************************************************/
int EXTI9_5_IRQHandler(void) {
  if (PBin(5) == 0) {
    EXTI->PR = 1 << 5;  //���LINE5�ϵ��жϱ�־λ

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
  //=============ң��ǰ�����˲���=======================//
  if (1 == Flag_Qian)
    Movement = -Target_Velocity;  //===ǰ����־λ��1
  else if (1 == Flag_Hou)
    Movement = Target_Velocity;  //===���˱�־λ��1
  else
    Movement = 0;
  if (Bi_zhang == 1 && Distance < 500 && Flag_Left != 1 &&
      Flag_Right != 1)  //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
    Movement = Target_Velocity;
  //=============�ٶ�PI������=======================//
  Encoder_Least =
      (Encoder_Left + Encoder_Right) -
      0;  //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
  Encoder *= 0.7;                  //===һ�׵�ͨ�˲���
  Encoder += Encoder_Least * 0.3;  //===һ�׵�ͨ�˲���
  Encoder_Integral += Encoder;     //===���ֳ�λ�� ����ʱ�䣺10ms
  Encoder_Integral =
      Encoder_Integral - Movement;  //===����ң�������ݣ�����ǰ������
  if (Encoder_Integral > 15000) Encoder_Integral = 15000;    //===�����޷�
  if (Encoder_Integral < -15000) Encoder_Integral = -15000;  //===�����޷�
  Velocity =
      Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;  //===�ٶȿ���
  if (Turn_Off(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
    Encoder_Integral = 0;  //===����رպ��������
  return Velocity;
}
*/

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(void) {
  int siqu = 300;
  int Amplitude = 6900;  //===PWM������7200 ������6900
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
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(void) {
  u8 temp;
  //��ص�ѹ����11.1V�رյ��
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
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ�����
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
����ʱ�䣺500us
**************************************************************************/
void Get_Angle(u8 way) {
  float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
  // ��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶�
  //Temperature = Read_Temperature();
  if (way == 1) {
    // DMP�Ķ�ȡ�����ݲɼ��ж����ѵ�ʱ���ϸ���ѭʱ��Ҫ��
    Read_DMP();                 //===��ȡ���ٶȡ����ٶȡ����
    Angle_Balance = Pitch;      //===����ƽ�����
    Gyro_Balance = gyro[1];     //===����ƽ����ٶ�
    Gyro_Turn = gyro[2];        //===����ת����ٶ�
    Acceleration_Z = accel[2];  //===����Z����ٶȼ�
  } else {
    Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) +
             I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);  //��ȡY��������
    Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) +
             I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);  //��ȡZ��������
    Accel_X =
        (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) +
        I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L);  //��ȡX����ٶȼ�
    Accel_Z =
        (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) +
        I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);  //��ȡZ����ٶȼ�
    if (Gyro_Y > 32768)
      Gyro_Y -= 65536;  //��������ת��  Ҳ��ͨ��shortǿ������ת��
    if (Gyro_Z > 32768) Gyro_Z -= 65536;           //��������ת��
    if (Accel_X > 32768) Accel_X -= 65536;         //��������ת��
    if (Accel_Z > 32768) Accel_Z -= 65536;         //��������ת��
    Gyro_Balance = -Gyro_Y;                        //����ƽ����ٶ�
    Accel_Y = atan2(Accel_X, Accel_Z) * 180 / PI;  //�������
    Gyro_Y = Gyro_Y / 16.4;                        //����������ת��
    if (Way_Angle == 2)
      Kalman_Filter(Accel_Y, -Gyro_Y);  //�������˲�
    else if (Way_Angle == 3)
      Yijielvbo(Accel_Y, -Gyro_Y);  //�����˲�
    Angle_Balance = angle;          //����ƽ�����
    Gyro_Turn = Gyro_Z;             //����ת����ٶ�
    Acceleration_Z = Accel_Z;       //===����Z����ٶȼ�
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
