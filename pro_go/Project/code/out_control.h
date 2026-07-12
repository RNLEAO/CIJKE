

#ifndef __OUT_CONTROL_H_
#define __OUT_CONTROL_H_
#include "headfile.h"

//函数名称：充电电压
int wait_for_voltage(float target_voltage);
/**************************************************************************
函数名称：pwm输出
**************************************************************************/
void out_pwm();

/**************************************************************************
函数名称：数据采集
**************************************************************************/
void acquire_sensor_data(void);
void angle_project(int threshold);
float Calculate_Weight_Mid(uint16 M);
float Calculate_Vertical();
#endif


