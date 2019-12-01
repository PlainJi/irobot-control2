#include "usart2.h"

char uart2_recv_buf[17] = "$+12345,+12345\n";
char uart2_send_buf[42] = "#+12345,+12345\n";

void uart2_init(u32 bound) {
  // GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  //USART_DeInit(USART2);
  // GPIO配置 PA2_TX,PA3_RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);
  // RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);

  // Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE);
}

// #+00001,+00001\n
void USART2_ReportEncoder(s16 l, s16 r) {
  u8 cnt = 0;
  u8 send_cnt = 15;

  sprintf(uart2_send_buf, "E%+06d,%+06d\n", l, r);
  while (cnt < send_cnt) {
    USART2->DR = uart2_send_buf[cnt++];
    while ((USART2->SR & 0x40) == 0);
  }
}

// #+01234\n
void USART2_ReportBattery(int voltage) {
  u8 cnt = 0;
  u8 send_cnt = 8;

  sprintf(uart2_send_buf, "B%+06d\n", voltage);
  while (cnt < send_cnt) {
    USART2->DR = uart2_send_buf[cnt++];
    while ((USART2->SR & 0x40) == 0);
  }
}

void USART2_ReportIMU(void) {
  u8 cnt = 0;
  u8 send_cnt = 42;

  uart2_send_buf[0] = 'I';
  memcpy(uart2_send_buf + 1, gyro_output, sizeof(float)*3);
  memcpy(uart2_send_buf + 1 + sizeof(float)*3, accel_output, sizeof(float)*3);
  memcpy(uart2_send_buf + 1 + sizeof(float)*(3*2+0), &q0, sizeof(float));
  memcpy(uart2_send_buf + 1 + sizeof(float)*(3*2+1), &q1, sizeof(float));
  memcpy(uart2_send_buf + 1 + sizeof(float)*(3*2+2), &q2, sizeof(float));
  memcpy(uart2_send_buf + 1 + sizeof(float)*(3*2+3), &q3, sizeof(float));
  uart2_send_buf[41] = '\n';
  while (cnt < send_cnt) {
    USART2->DR = uart2_send_buf[cnt++];
    while ((USART2->SR & 0x40) == 0);
  }
}

// $+12345,+12345\n
void USART2_IRQHandler(void) {
  static u8 p_w = 0;
  char uart_receive = 0;
  static int temp_desire_l = 0;
  static int temp_desire_r = 0;

  if (USART2->SR & 0x20) {
    uart_receive = USART2->DR;
    if (uart_receive == '$') p_w = 0;
    if (p_w == sizeof(uart2_recv_buf)) p_w = 0;
    uart2_recv_buf[p_w++] = uart_receive;

    if (uart_receive == '\n' && uart2_recv_buf[0] == '$') {
      temp_desire_l = (uart2_recv_buf[2]-'0')*10000 + (uart2_recv_buf[3]-'0')*1000 +
        (uart2_recv_buf[4]-'0')*100 + (uart2_recv_buf[5]-'0')*10 + (uart2_recv_buf[6]-'0');
      if (uart2_recv_buf[1] == '-') {
        temp_desire_l = -temp_desire_l;
      }

      temp_desire_r = (uart2_recv_buf[9]-'0')*10000 + (uart2_recv_buf[10]-'0')*1000 +
        (uart2_recv_buf[11]-'0')*100 + (uart2_recv_buf[12]-'0')*10 + (uart2_recv_buf[13]-'0');
      if (uart2_recv_buf[8] == '-') {
        temp_desire_r = -temp_desire_r;
      }

      if (0 == temp_desire_l || 0 == temp_desire_r) {
          Flag_Stop = 1;
          DesireL = temp_desire_l;
          DesireR = temp_desire_r;
      } else {
          DesireL = temp_desire_l;
          DesireR = temp_desire_r;
          Flag_Stop = 0;
      }
    }
  }
  // if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) == SET) {
  //   USART_ReceiveData(USART2);
  //   USART_ClearFlag(USART2,USART_FLAG_ORE);
  // }
  // USART_ClearFlag(USART2,USART_IT_RXNE);
}
