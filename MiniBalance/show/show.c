#include "show.h"

void APP_Show(void) {
  //static u8 flag;
  //flag = !flag;
  app_3_vol_report = (Voltage - 1110) * 2 / 3;
  if (app_3_vol_report < 0) app_3_vol_report = 0;
  if (app_3_vol_report > 100) app_3_vol_report = 100;

  //if (flag == 0)
  printf("{A%d:%d:%d:%d}$", (u8)app_1_encL_report, (u8)app_2_encR_report, app_3_vol_report, app_4_angle_report);	// APPÊ×Ò³
  //else
  //  printf("{B%d:%d:%d:%d}$", (int)Angle_Balance, Distance, Encoder_Left, Encoder_Right);							// APP²¨ÐÎ
}

//?????1.2ms
void DataScope(void) {
  char SendCount = 0, i = 0;
  // DataScope_Get_Channel_Data(DesireL, 1);
  // DataScope_Get_Channel_Data(Encoder_Left, 2);
  // DataScope_Get_Channel_Data(Moto1, 3);
  // DataScope_Get_Channel_Data(DesireR, 4);
  // DataScope_Get_Channel_Data(Encoder_Right, 5);
  // DataScope_Get_Channel_Data(Moto2, 6);
  // DataScope_Get_Channel_Data(0, 7);
  // DataScope_Get_Channel_Data(0, 8);
  // DataScope_Get_Channel_Data(0, 9);
  // DataScope_Get_Channel_Data(0, 10);

  DataScope_Get_Channel_Data(gyro_output[0], 1);
  DataScope_Get_Channel_Data(gyro_output[1], 2);
  DataScope_Get_Channel_Data(gyro_output[2], 3);
  DataScope_Get_Channel_Data(accel_output[0], 4);
  DataScope_Get_Channel_Data(accel_output[1], 5);
  DataScope_Get_Channel_Data(accel_output[2], 6);
  DataScope_Get_Channel_Data(Roll, 7);
  DataScope_Get_Channel_Data(Pitch, 8);
  DataScope_Get_Channel_Data(Yaw, 9);
  DataScope_Get_Channel_Data(q3, 10);

  SendCount = DataScope_Data_Generate(9);
   for (i = 0; i < SendCount; i++) {
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = DataScope_OutPut_Buffer[i];
  }
}
