
#ifndef __ELEMENT_H
#define __ELEMENT_H
#include "common.h"


//速度状态标志位
extern uint8 speed_ctrl_flag;     



extern uint8 cir_flag;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)
extern uint8 encoder_sign;			//编码器积分开关


extern uint8 yuansu_flag; 
extern uint16 temp_flag;
extern uint16 temp_flag_speed;
extern uint8 speed_strategy_active;


extern uint16 cir_angle_flag ;   


extern float run_mode;

void check_hall_sensor(void);
void Circle_detect(void);
void cricle_cl(void);


// ======================== 直道判断 ========================
extern float straight_err_threshold;       // 误差判断阈值（越小越严格）
extern float straight_integral_threshold;  // 积分量判断阈值（越大越稳妥）

void straight_judge(float straight_err_threshold, float straight_integral_threshold);
extern char straight_flag;




#endif
