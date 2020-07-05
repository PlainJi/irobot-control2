#include "usart3.h"

void uart3_init(u32 bound) {
  // GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  // USART3_TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // USART3_RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART3, ENABLE);
}

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE {
  int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x) { x = x; }
//重定义fputc函数
int fputc(int ch, FILE *f) {
  while ((USART3->SR & 0X40) == 0)
    ;
  USART3->DR = (u8)ch;
  return ch;
}
#endif

void USART3_IRQHandler(void) {
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    static u8 uart3_buf[57], p_w = 0, flag = 1;
    u8 temp = 0;
    u8 uart_receive = USART_ReceiveData(USART3);

    if (flag && uart_receive >= 'A' && uart_receive <= 'Z') {
      switch (uart_receive) {
        case 'A':
          DesireL = SpeedL;
          DesireR = SpeedR;
          Flag_Stop = 0;
          break;
        case 'B':
          DesireL = SpeedL;
          DesireR = 0;
          Flag_Stop = 0;
          break;
        case 'C':
          DesireL = SpeedL;
          DesireR = -SpeedR;
          Flag_Stop = 0;
          break;
        case 'D':
          DesireL = 0;
          DesireR = 0;
          Flag_Stop = 1;
          break;
        case 'E':
          DesireL = 0;
          DesireR = 0;
          Flag_Stop = 1;
          break;
        case 'F':
          DesireL = 0;
          DesireR = 0;
          Flag_Stop = 1;
          break;
        case 'G':
          DesireL = -SpeedL;
          DesireR = SpeedR;
          Flag_Stop = 0;
          break;
        case 'H':
          DesireL = 0;
          DesireR = SpeedR;
          Flag_Stop = 0;
          break;
        case 'X':
          SpeedL += 10;
          SpeedR += 10;
          Flag_Stop = 1;
          break;
        case 'Y':
          SpeedL -= 10;
          SpeedR -= 10;
          Flag_Stop = 1;
          break;
        case 'Z':
          DesireL = 0;
          DesireR = 0;
          Flag_Stop = 1;
          break;
        default:
          break;
      }
    } else {
      if (p_w == sizeof(uart3_buf)) p_w = 0;
      if (uart_receive == '{') {
        flag = 0;
        p_w = 0;
      }
      uart3_buf[p_w++] = uart_receive;

      if (uart_receive == '}') {
        flag = 1;
        uart3_buf[p_w++] = '\0';
        if (uart3_buf[1] == 'Q' && uart3_buf[3] == 'W') {
          // 掉电保存
          save_flag = 1;
        } else if (uart3_buf[1] == 'Q' && uart3_buf[3] == 'P') {
          // 获取设备参数
          send_param_flag = 1;
        } else if (uart3_buf[1] == '#') {
          // 更新所有参数
          sscanf((const char *)uart3_buf, "{#%u:%u:%u:%u:%u:%u:%u:%u:%u}",
                 (u32 *)&Flash_Parameter[0], (u32 *)&Flash_Parameter[1],
                 (u32 *)&Flash_Parameter[2], (u32 *)&Flash_Parameter[3],
                 (u32 *)&Flash_Parameter[4], (u32 *)&Flash_Parameter[5],
                 (u32 *)&Flash_Parameter[6], (u32 *)&Flash_Parameter[7],
                 (u32 *)&Flash_Parameter[8]);
          Velocity_Kp = Flash_Parameter[0] / 1000.0;
          Velocity_Ki = Flash_Parameter[1] / 1000.0;
          Velocity_Kd = Flash_Parameter[2] / 1000.0;
          SpeedL = Flash_Parameter[3] / 100.0;
          SpeedR = Flash_Parameter[4] / 100.0;
          // Voltage doesn't need to be set.
          bluetooth_report = Flash_Parameter[6];
        } else if (uart3_buf[1] >= '0' && uart3_buf[1] <= '8') {
          // 更新特定参数
          sscanf((const char *)uart3_buf, "{%c:%u}", &temp,
                 (u32 *)&Flash_Parameter[uart3_buf[1] - '0']);
          switch (temp) {
            case '0':
              Velocity_Kp = Flash_Parameter[0] / 1000.0;
              break;
            case '1':
              Velocity_Ki = Flash_Parameter[1] / 1000.0;
              break;
            case '2':
              Velocity_Kd = Flash_Parameter[2] / 1000.0;
              break;
            case '3':
              SpeedL = Flash_Parameter[3] / 100.0;
              break;
            case '4':
              SpeedR = Flash_Parameter[4] / 100.0;
              break;
            case '6':
              bluetooth_report = Flash_Parameter[6];
              break;
            default:
              break;
          }
        }
      }
    }
  }
  // if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET) {
  //   USART_ReceiveData(USART3);
  //   USART_ClearFlag(USART3,USART_FLAG_ORE);
  // }
  // USART_ClearFlag(USART3,USART_IT_RXNE);
}
