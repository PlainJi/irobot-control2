#include <string.h>

#include "DataScope_DP.h"

unsigned char DataScope_OutPut_Buffer[42] = {0};  //串口发送缓冲区

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
