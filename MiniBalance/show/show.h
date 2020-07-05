#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"

extern int app_1_encL_report, app_2_encR_report, app_3_vol_report, app_4_angle_report;

void oled_show(void);
void APP_Show(void);
void DataScope(void);

#endif
