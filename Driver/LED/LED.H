#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED PAout(12)  // LED �˿ڶ���

extern u16 led_freq;

void LED_Init(void);   //��ʼ��
void Led_Flash(u16 time);

#endif