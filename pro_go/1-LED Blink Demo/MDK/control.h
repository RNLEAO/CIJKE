#ifndef _CONTROL_H
#define _CONTROL_H
#include "headfile.h"
//编码器接口定义
#define MOTOR1_ENCODER CTIM0_P34			//Z相
#define MOTOR2_ENCODER CTIM3_P04
#define MOTOR1_DIR P35								//dir方向
#define MOTOR2_DIR P53


//电机接口定义
#define MOTOR1 PWMA_CH2P_P62 //左  8701
#define MOTOR1_d P64

#define MOTOR2 PWMA_CH1P_P60 //右
#define MOTOR2_d P66






#define Screen_switch P37

#define M_PI 3.14159265358979323846  // Define M_PI if not defined
#define  ang_limit 200


uint8 key_scan(int mode);//按键扫描
float low_pass_filter(float current_value, float last_value, float alpha);//低通滤波

void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step);//浮点数调整
float Get_roll(void);//取roll


extern int sample_count;  //计数器，用于收集一定数量的数据来计算零漂。
extern int state ;  		   // 状态标志，指示是否已经计算完偏移量

// 零偏值

extern float gyro_data[1];  // 陀螺仪数据

extern float gyro_roll,gyro_pitch;
extern float roll_accel,pitch_accel;


extern float roll,pitch;
extern float Angle_x ;
/************角度环参数**************/
extern float current_angle;//当前角度
extern float target_angle;//目标角度
extern float angle_error;//误差
extern float turn_control_output;
extern float base_speed; // 原地转向，基础速度设为 0 

// 函数：赛道保护
void track_protection(void); 
extern uint8 current_key;
extern uint8 last_key_state ;
extern uint8 pwm_state ;      // 当前输出状态
extern uint8 Pwmout;

extern char pwm_state_charge;      // 当前输出状态

uint8 key_scan_with_pwm(void);  // 更名后的函数

/*--------电感算法--------*/
extern float L_raw,R_raw;//后面四个还没用到
extern float L,R,LM,RM,MID ;//后面四个还没用到
extern uint16  max_AD ,min_AD ;

extern uint16  i ,j ,k1 ,temp ;

/*--------测试算法--------*/
void motor_rotate_and_reverse(int pwm_channel_forward, int pwm_channel_reverse, int rotate_time, int reverse_time, int pwm_duty_max);





float filter(float value);
void key_scan_cycle_pwm_state(void); // 返回类型改为 void
float limit_range(float input, float limit);


/**************************************************************************
函数功能：外设及pid初始化
入口参数：无
返回  值：wu
**************************************************************************/
void init(void);
/**************************************************************************
函数功能：--- 辅助函数：设置左右轮目标速度 ---
入口参数：无
返回  值：wu
**************************************************************************/
void set_target_speeds(float left_target, float right_target);

//权重
float Calculate_Weight_Mid(uint16 M);
//蜂鸣器
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state);
//读取
uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution);
//限幅
float limit_float(float value, float min_limit, float max_limit);
//归一化
float normalize_float(float value, float min, float max);
//速度
float calculate_dynamic_target_speed_quadratic(float current_mid_value);

//typedef enum {
//	RGB_COLOR_OFF = 0,        // 无: P26=0, P74=0, P07=0
//	RGB_COLOR_WHITE = 1,      // 白: P26=0, P74=0, P07=1
//	RGB_COLOR_CYAN = 2,       // 青: P26=0, P74=1, P07=0
//	RGB_COLOR_YELLOW_GREEN = 3, // 黄绿: P26=1, P74=0, P07=0
//	RGB_COLOR_MAGENTA = 4,    // 紫: P26=0, P74=1, P07=1
//	RGB_COLOR_GREEN = 5,      // 绿: P26=1, P74=1, P07=0
//	RGB_COLOR_RED = 6,        // 红: P26=1, P74=0, P07=1
//	RGB_COLOR_BLUE = 7        // 蓝: P26=1, P74=1, P07=1
//} RgbColorCode_t;

//void set_rgb_pins(int p26_val, int p74_val, int p07_val);

//void control_rgb_led_conditional(float check_value, float abs_threshold, RgbColorCode_t feedback_color, int enable_state);

//void control_rgb_led( RgbColorCode_t feedback_color);

void update_encoder_speedup_value(float* p_encoder_speedup_element, 
                                  int current_encoder_speedup_sign);

void update_gyro_angle_accumulator(float* p_angle_accumulator,
                                   int sign_flag);

void change_speed_Target(int speed);
void change_speed_Target_base(int speed);
////卡尔曼
//void kalanma_data();
//void init_kalman();
//extern float k_roll;
//extern float k_pitch;




#endif 













