
#ifndef __ELEMENT_H
#define __ELEMENT_H
#include "common.h"
#include "headfile.h"


extern uint8 cir_flag;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)
extern uint8 encoder_sign;			//编码器积分开关


extern uint8 yuansu_flag; 
extern int16 circle_exit_count;

extern uint16 temp_flag_speed;

extern float in_circle_LR;
extern float in_circle_MID;
extern float in_circle_LRMID;
extern float ring_error;

extern uint8 circle_enter_case;

extern uint16 cir_angle_flag ;   
extern float temp_flag ;   

extern float run_mode;


extern float ring_inc_element12;
extern float ring_inc_element56;
extern float ring_inc_element67;
extern float ring_angle_23;
extern float ring_angle_34;
extern float ring_angle_45;
extern float temp_flag_tar;





void check_hall_sensor(void);
void Circle_detect(void);
void Circle_cl(void);


// ======================== 直道判断 ========================
extern float straight_err_threshold;       // 误差判断阈值（越小越严格）
extern float straight_integral_threshold;  // 积分量判断阈值（越大越稳妥）

extern char right_angle_flag;
extern int right_angle_count;
extern int16 circle_exit_count;


void right_angle_judge();
void right_angle_cl();





#endif
