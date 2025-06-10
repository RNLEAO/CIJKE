
#ifndef __ELEMENT_H
#define __ELEMENT_H
#include "common.h"


void Circle_detect(void);
void dir_outpt(void);//根据元素决策方向环的选择方案
void cricle_cl(void);
void cross_judge(void);
void cross_proc(void);

extern float distance_per_pulse;
extern float dia;  //车轮直径


extern uint8 cir_flag;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)
extern uint8 encoder_sign;			//编码器积分开关


extern uint8 yuansu_flag; 
extern uint16 temp_flag;
extern uint16 temp_flag_speed;
extern uint8 speed_strategy_active;


extern uint16 cir_angle_flag ;   
extern int cross_flag;


extern float run_mode;

char set_control_mode(char mode);
void check_hall_sensor(void);

#endif
