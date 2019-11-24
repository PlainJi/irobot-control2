#include "stm32f10x.h"
#include "sys.h"

u16 led_freq=1;
s32 DesireL = 0, DesireR = 0;
s32 SpeedL = 50, SpeedR = 50;
float Velocity_Kp = 6.540, Velocity_Ki = 7.270, Velocity_Kd = 0;
int Voltage;                      //��ص�ѹ������صı���
u8 Flag_Stop=1, report_flag=0, bluetooth_report=0, scope_report=0, save_flag=0, send_param_flag=0;  //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int app_1_encL_report = 0, app_2_encR_report = 0, app_3_vol_report = 0, app_4_angle_report = 0;
u8 speed_limit = 130;   // ����0.3m/s������һȦ1040�����壬��ֱ��0.238m������10Hz��0.3/0.238*1040/10=131

u8 Way_Angle = 1;  //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu = 1;  //����ң����صı���
int Encoder_Left, Encoder_Right;  //���ұ��������������
int Moto1, Moto2;                 //���PWM���� Ӧ��Motor�� ��Moto�¾�
int Temperature;                  //��ʾ�¶�
float Angle_Balance=0, Gyro_Balance=0, Gyro_Turn=0;  //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;  //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance=0;        //���������
float Acceleration_Z;  // Z����ٶȼ�
u16 PID_Parameter[10], Flash_Parameter[10];  // Flash�������

int main(void) {
  delay_init();                //=====��ʱ������ʼ��
  uart_init(256000);           //=====���ڳ�ʼ��Ϊ
  uart1_send("uart1 ok\r\n");
  JTAG_Set(JTAG_SWD_DISABLE);  //=====�ر�JTAG�ӿ�
  JTAG_Set(SWD_ENABLE);  //=====��SWD�ӿ� �������������SWD�ӿڵ���
  LED_Init();            //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
  KEY_Init();            //=====������ʼ��
  MY_NVIC_PriorityGroupConfig(2);  //=====�жϷ���
  //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ�
  MiniBalance_PWM_Init(7199, 0);
  uart2_init(230400);             //=====����2��ʼ��
  uart3_init(9600);               //=====����3��ʼ��
  Encoder_Init_TIM2();            //=====�������ӿ�
  Encoder_Init_TIM4();            //=====��ʼ��������2
  Adc_Init();                     //=====adc��ʼ��
  IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��
  DMP_Init();                     //=====��ʼ��DMP
  MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
  Flash_Read();

  while (1) {
    if (report_flag) {
      report_flag = 0;
      ReportEncoderBattery();

      if (bluetooth_report) {
        APP_Show();
      }
      if (scope_report) {
        DataScope();
      }
    }
    if (save_flag) {
        Flash_Write();
        save_flag = 0;
    }
    if (send_param_flag) {
      printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_Kp * 1000), (int)(Velocity_Ki * 1000), (int)(Velocity_Kd * 1000), SpeedL*100, SpeedR*100, Voltage, bluetooth_report, 0, 0);
      send_param_flag = 0;
    }
  }
}
