#include "show.h"

#include <string.h>

int app_1_encL_report = 0, app_2_encR_report = 0, app_3_vol_report = 0,
    app_4_angle_report = 0;

u8 DataScope_OutPut_Buffer[42] = {0};  //串口发送缓冲区

//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
void DataScope_Get_Channel_Data(float Data, unsigned char Channel) {
  if ((Channel > 10) || (Channel == 0))
    return;
  else {
    memcpy(DataScope_OutPut_Buffer + 1 + (Channel - 1) * 4, &Data, 4);
  }
}

//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败
unsigned char DataScope_Data_Generate(unsigned char Channel_Number) {
  char offset = 1 + Channel_Number * 4;
  if ((Channel_Number > 10) || (Channel_Number == 0)) {
    return 0;
  } else {
    DataScope_OutPut_Buffer[0] = '$';  //帧头
    DataScope_OutPut_Buffer[offset] = offset;
  }
  return (offset + 1);
}


void ReportToAPP(void) {
  app_3_vol_report = (Voltage - 1110) * 2 / 3;
  if (app_3_vol_report < 0) app_3_vol_report = 0;
  if (app_3_vol_report > 100) app_3_vol_report = 100;

  printf("{A%d:%d:%d:%d}$", (u8)app_1_encL_report, (u8)app_2_encR_report,
         app_3_vol_report, app_4_angle_report);
}

void ReportToDataScope(void) {
  char SendCount = 0, i = 0;
  DataScope_Get_Channel_Data(DesireL, 1);
  DataScope_Get_Channel_Data(Encoder_Left, 2);
  DataScope_Get_Channel_Data(Moto1, 3);
  DataScope_Get_Channel_Data(DesireR, 4);
  DataScope_Get_Channel_Data(Encoder_Right, 5);
  DataScope_Get_Channel_Data(Moto2, 6);
  DataScope_Get_Channel_Data(SPEED_LIMIT_BY_ENCODER, 7);
  DataScope_Get_Channel_Data(pulse_cnt, 8);
  DataScope_Get_Channel_Data(0, 9);
  DataScope_Get_Channel_Data(0, 10);

  // DataScope_Get_Channel_Data(gyro_output[0], 1);
  // DataScope_Get_Channel_Data(gyro_output[1], 2);
  // DataScope_Get_Channel_Data(gyro_output[2], 3);
  // DataScope_Get_Channel_Data(accel_output[0], 4);
  // DataScope_Get_Channel_Data(accel_output[1], 5);
  // DataScope_Get_Channel_Data(accel_output[2], 6);
  // DataScope_Get_Channel_Data(Roll, 7);
  // DataScope_Get_Channel_Data(Pitch, 8);
  // DataScope_Get_Channel_Data(Yaw, 9);
  // DataScope_Get_Channel_Data(q3, 10);

  SendCount = DataScope_Data_Generate(9);
  for (i = 0; i < SendCount; i++) {
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = DataScope_OutPut_Buffer[i];
  }
}
