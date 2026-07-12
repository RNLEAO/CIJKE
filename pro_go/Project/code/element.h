#ifndef __ELEMENT_H_
#define __ELEMENT_H_

#include "common.h"

typedef enum
{
    ELEMENT4_TRACK = 0,
    ELEMENT4_RIGHT_ANGLE_CONFIRM,
    ELEMENT4_RIGHT_ANGLE_TURN,
    ELEMENT4_RIGHT_ANGLE_RECOVER,
    ELEMENT4_RING_CONFIRM,
    ELEMENT4_RING_ENTER,
    ELEMENT4_RING_HOLD,
    ELEMENT4_RING_EXIT,
    ELEMENT4_RING_RECOVER
} Element4State;

extern Element4State element4_state;
extern int8 element4_direction;

extern uint8 cir_flag;
extern uint8 encoder_sign;
extern uint8 yuansu_flag;
extern int16 circle_exit_count;
extern uint16 temp_flag_speed;
extern uint16 cir_angle_flag;
extern uint8 circle_enter_case;
extern float temp_flag;
extern float run_mode;

extern float in_circle_LR;
extern float in_circle_MID;
extern float in_circle_LRMID;
extern float ring_error;
extern float ring_exit_error;
extern float ring_inc_element12;
extern float ring_inc_element56;
extern float ring_inc_element67;
extern float ring_angle_23;
extern float ring_angle_34;
extern float ring_angle_45;
extern float temp_flag_tar;

extern char right_angle_flag;
extern int right_angle_count;
extern float straight_err_threshold;
extern float straight_integral_threshold;

extern float right_angle_outer_low;
extern float right_angle_inner_high;
extern float right_angle_inner_low;
extern float right_angle_inner_diff;
extern float right_angle_min_angle;
extern float right_angle_target_angle;
extern float right_angle_reverse_speed;
extern float right_angle_forward_speed;

extern float ring_outer_high;
extern float ring_inner_low;
extern float ring_inner_active;
extern float ring_side_diff;

void element4_init(void);
float element4_process(float track_error);
uint8 element4_get_speed_override(float *left_target, float *right_target);
const char *element4_state_name(void);
uint8 element4_negative_pressure_request(void);

void check_hall_sensor(void);
void Circle_detect(void);
void Circle_cl(void);
void right_angle_judge(void);
void right_angle_cl(void);

#endif
