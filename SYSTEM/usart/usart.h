#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

void uart_init(u32 bound);
void uart1_send(u8 *buf);

#endif
