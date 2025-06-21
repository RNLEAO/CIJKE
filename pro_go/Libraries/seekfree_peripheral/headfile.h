#ifndef __HEADFILE_H_
#define __HEADFILE_H_




#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"
#include <math.h>
//------STC32G SDK等
#include "STC32Gxx.h"
#include "board.h"
#include "common.h"

//------逐飞科技单片机外设驱动头文件
#include "zf_uart.h"
#include "zf_gpio.h"
#include "zf_iic.h"
#include "zf_adc.h"
#include "zf_spi.h"
#include "zf_tim.h"
#include "zf_pwm.h"
#include "zf_nvic.h"
#include "zf_exti.h"
#include "zf_delay.h"
#include "zf_eeprom.h"
#include "zf_fifo.h"

//------逐飞科技产品驱动头文件
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_18TFT.h"

#include "SEEKFREE_TSL1401.h"
#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_AT24C02.h"
#include "SEEKFREE_BLUETOOTH_CH9141.h"
#include "SEEKFREE_WIRELESS_CH573.h"
#include "SEEKFREE_CONFIG.h"
#include "SEEKFREE_IMU660RA.h"
#include "SEEKFREE_VIRSCO.h"



//------逐飞科技产品组件层文件
#include "seekfree_assistant.h"

//------组件层文件
#include "control.h"
#include "Motor.h"
#include "PID.h"
#include "MENU_DATA.h"
#include "Menu.h"
#include "MATH.h"
#include "vofa.h"
#include "element.h"
#include "out_control.H"


	//蜂鸣器
#define BEEP P67
#define ON 1
#define OFF 0

typedef 	unsigned char	u8;
typedef 	unsigned int	u16;
typedef 	unsigned long	u32;



//动态权重后的差比和算法
#define H_SENSOR_BASE 330  // 横向电感基准值
#define V_SENSOR_BASE 340  // 垂直电感基准值


#endif