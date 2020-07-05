#include "usart.h"

#include "sys.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"  //ucos ʹ��
#endif
//////////////////////////////////////////////////////////////////

void uart1_send(u8 *buf) {
  u8 *temp = buf;
  while (*temp) {
    USART1->DR = *temp;
    while ((USART1->SR & 0x40) == 0)
      ;
    temp++;
  }
}

void uart_init(u32 bound) {
  // GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,
                         ENABLE);  //ʹ��USART1��GPIOAʱ��

  // USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  // PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //��ʼ��GPIOA.9

  // USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             // PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);                 //��ʼ��GPIOA.10
                                          // USART ��ʼ������

  USART_InitStructure.USART_BaudRate = bound;  //���ڲ�����
  USART_InitStructure.USART_WordLength =
      USART_WordLength_8b;  //�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;     //����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;  //��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure);        //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  //�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                       //ʹ�ܴ���1
}
