#ifndef __USRAT3_H
#define __USRAT3_H

#include "sys.h"

extern u8 Flag_Stop;
extern u8 save_flag, send_param_flag;

void uart3_init(u32 bound);
void USART3_IRQHandler(void);

#endif
