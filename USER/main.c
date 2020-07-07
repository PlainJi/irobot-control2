#include "stm32f10x.h"
#include "sys.h"

u8 scope_report = 0;      //上报数目到上位机软件
u8 bluetooth_report = 0;  //上报数据到蓝牙客户端

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
        ReportToAPP();
      }
      if (scope_report) {
        ReportToDataScope();
      }
    }

    if (save_flag) {
      save_flag = 0;
      Flash_Write();
    }

    if (send_param_flag) {
      send_param_flag = 0;
      printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_Kp * 1000),
             (int)(Velocity_Ki * 1000), (int)(Velocity_Kd * 1000), SetSpeedL,
             SetSpeedR, Voltage, bluetooth_report, 0, 0);
    }
  }
}
