#ifndef __USRAT2_H
#define __USRAT2_H
#include "sys.h"	  	

void uart2_init(u32 bound);
void USART2_IRQHandler(void);
void USART2_ReportEncoder(s16 l, s16 r);
void USART2_ReportBattery(int voltage);
void USART2_ReportIMU(void);

#endif
