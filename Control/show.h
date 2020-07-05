#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"

extern u8 DataScope_OutPut_Buffer[42];
extern int app_1_encL_report, app_2_encR_report, app_3_vol_report, app_4_angle_report;

void ReportToAPP(void);
void ReportToDataScope(void);

#endif
