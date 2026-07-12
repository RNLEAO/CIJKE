#ifndef _MOTOR_H
#define _MOTOR_H
#include "headfile.h"

extern float r_encoder;
extern float l_encoder;
extern float r_encoder_last;
extern float l_encoder_last;

//电机速度
extern float r_speed_now ;
extern float l_speed_now ;
extern float speed_now;

extern float forwardfeed_L(float in);
extern float forwardfeed_R(float inc_in);			
extern float forwardfeed_turn(float inc_in);
float limit_function(float value, float min_val, float max_val);//限幅

extern char distance_protect;
// 累计电机增量
extern float motor_inc;
// PWM 值
extern float r_pwm ;
extern float l_pwm ;


#endif 

