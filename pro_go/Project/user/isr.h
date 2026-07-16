/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __ISR_H_
#define __ISR_H_



extern float error;

extern float mot_inc_last;
extern float mot_inc;

extern float Roll_x;
extern float err_t;

	// 定义变量存储当前的PWM占空比
extern float current_l_pwm_duty ;
extern float current_r_pwm_duty ;
extern float l_pwm_duty ;
extern float r_pwm_duty ;
extern float current_l_pwm_inc;
extern float current_r_pwm_inc;

extern float current_l_pwm_inc_last;
extern float current_r_pwm_inc_last;
unsigned char motion_line_wait_is_active(void);
unsigned char motion_direction_guard_mask(void);
void reset_motion_pid_state(void);
void reset_track_test_steering_state(void);
void reset_track_test_exit_diagnostic(void);

#define TRACK_T12_EXIT_TRIGGER_SENSOR 0x01U
#define TRACK_T12_EXIT_TRIGGER_ANGLE  0x02U
#define TRACK_T12_EXIT_TRIGGER_TIME   0x04U

#define TRACK_T12_START_RELEASE_BALANCED 0x01U
#define TRACK_T12_START_RELEASE_FALLBACK 0x02U

extern volatile unsigned char xdata g_track_t12_start_release_reason;
extern volatile unsigned int xdata g_track_t12_start_release_sample_count;
extern volatile unsigned long xdata g_track_t12_start_release_left_total;
extern volatile unsigned long xdata g_track_t12_start_release_right_total;
extern volatile unsigned char xdata g_track_t12_exit_trigger_mask;
extern volatile unsigned int xdata g_track_t12_exit_angle_x10;
extern volatile unsigned int xdata g_track_t12_exit_half_ticks;
extern volatile unsigned char xdata g_track_t12_exit_norm_l;
extern volatile unsigned char xdata g_track_t12_exit_norm_lm;
extern volatile unsigned char xdata g_track_t12_exit_norm_rm;
extern volatile unsigned char xdata g_track_t12_exit_norm_r;
extern volatile signed int xdata g_track_t12_exit_error_x1000;
extern volatile unsigned int xdata g_track_t12_exit_sum;
extern volatile unsigned char xdata g_track_t12_post_valid;
extern volatile unsigned char xdata g_track_t12_post_delay_ticks;
extern volatile unsigned int xdata g_track_t12_post_angle_x10;
extern volatile unsigned char xdata g_track_t12_post_norm_l;
extern volatile unsigned char xdata g_track_t12_post_norm_lm;
extern volatile unsigned char xdata g_track_t12_post_norm_rm;
extern volatile unsigned char xdata g_track_t12_post_norm_r;
extern volatile signed int xdata g_track_t12_post_error_x1000;
extern volatile unsigned int xdata g_track_t12_post_sum;


extern float mid_dynamic_weight;

//积分开关
extern float mot_inc_element;
extern float gyro_roll_sign_rign;

//直道角度积分开关
extern char gyro_roll_sign_angle;
//直道角度积分数值
extern float gyro_right_angle;

//圆环退出
extern float ring_out_element;
extern char ring_out_sign;

//充电编码器积分开关
extern char encoder_speedup_sign;
extern float encoder_speedup_element;
//加速编码器积分开关
extern char encoder_charge_sign;
extern float encoder_charge_element;
//十字编码器积分开关
extern char encoder_cross_sign;
extern float encoder_cross_element;
//十字编码器积分开关
extern char time_speedup_sign;
extern float time_speedup_element;
//长直道积分开关
extern char encoder_straight_sign;
extern float encoder_straight_element;

extern float dir_loop_limit; //方向环限幅
extern float dir_enlarge;  		//方向环pid放大系数
extern float speed_damping; // 速度抑制系数
extern float track_turn_ratio;
extern float track_line_speed_scale;

extern float err_H;
extern float err_X;
extern float err_HM;
extern float err_D;
extern float err_M;

extern float A_CBH;
extern float B_CBH;
extern float C_CBH;


extern float encoder_charge_element_vbat_tar;
extern float adc_vbat_tar;
extern float charge_pwm_open_val;


extern float error; 



extern float LM_x;  
extern float RM_x;  
extern float LM_y;  
extern float RM_y;  

//电池adc采集
extern float vbat_in;
extern float adc_vbat;

extern float gyro_roll_sign_cross;
extern float gyro_roll_cross;

//速度
extern float speed[5];


#endif
