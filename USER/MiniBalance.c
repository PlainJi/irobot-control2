#include "stm32f10x.h"
#include "sys.h"

u16 led_freq = 1;  //����LED����˸Ƶ��
s32 Voltage;       //��ص�ѹ������صı���
s32 Temperature;   // MPU6050�¶ȣ����������¶ȣ�

s32 Encoder_Left, Encoder_Right;  //���ұ��������������
s32 Moto1, Moto2;                 //���PWM����

s32 DesireL = 0, DesireR = 0;
s32 SpeedL = 50, SpeedR = 50;
float Velocity_Kp = 6.540, Velocity_Ki = 7.270, Velocity_Kd = 0;
u8 speed_limit = 130;  // ����0.3m/s������һȦ1040�����壬��ֱ��0.238m������10Hz��0.3/0.238*1040/10=131

u8 Flag_Stop = 1, report_flag = 0, bluetooth_report = 0, scope_report = 0,
   save_flag = 0, send_param_flag = 0;
u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu = 1;  //����ң����صı���

int main(void) {
  delay_init();
  uart_init(256000);
  uart1_send("uart1 ok\r\n");
  JTAG_Set(JTAG_SWD_DISABLE);
  JTAG_Set(SWD_ENABLE);
  LED_Init();
  KEY_Init();
  MY_NVIC_PriorityGroupConfig(2);
  MiniBalance_PWM_Init(7199, 0);
  uart2_init(230400);
  uart3_init(9600);
  Encoder_Init_TIM2();
  Encoder_Init_TIM4();
  Adc_Init();
  IIC_Init();
  MPU6050_initialize();
  DMP_Init();
  MiniBalance_EXTI_Init();
  Flash_Read();

  while (1) {
    if (report_flag) {
      report_flag = 0;

      ReportEncoderBattery();
      Get_Angle();
      USART2_ReportIMU();

      if (bluetooth_report) {
        APP_Show();
      }
      if (scope_report) {
        DataScope();
      }
    }

    if (save_flag) {
      save_flag = 0;
      Flash_Write();
    }

    if (send_param_flag) {
      send_param_flag = 0;
      printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_Kp * 1000),
             (int)(Velocity_Ki * 1000), (int)(Velocity_Kd * 1000), SpeedL * 100,
             SpeedR * 100, Voltage, bluetooth_report, 0, 0);
    }
  }
}
