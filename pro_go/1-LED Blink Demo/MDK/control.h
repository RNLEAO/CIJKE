#ifndef _CONTROL_H
#define _CONTROL_H
#include "headfile.h"
// encoder interface
#define MOTOR1_ENCODER CTIM0_P34
#define MOTOR2_ENCODER CTIM3_P04
#define MOTOR1_DIR P35
#define MOTOR2_DIR P53


//閻㈠灚婧€閹恒儱褰涚€规矮绠?
#define MOTOR1 PWMA_CH2P_P62 //瀹? 8701
#define MOTOR1_d P64

#define MOTOR2 PWMA_CH1P_P60
#define MOTOR2_d P66






#define Screen_switch P37

#define M_PI 3.14159265358979323846  // Define M_PI if not defined
#define  ang_limit 200


uint8 key_scan(int mode);//閹稿鏁幍顐ｅ伎
uint8 fetch_ui_key_event(void);
extern int8 key_mode;
float low_pass_filter(float current_value, float last_value, float alpha);//娴ｅ酣鈧碍鎶ゅ▔?
void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step);
float Get_roll(void);


extern int sample_count;
extern int state;

// 闂嗚泛浜搁崐?
extern float gyro_data[1];  // 闂勨偓閾昏桨鍗庨弫鐗堝祦

extern float gyro_roll,gyro_pitch;
extern float roll_accel,pitch_accel;


extern float roll,pitch;
extern float Angle_x ;
/************鐟欐帒瀹抽悳顖氬棘閺?*************/
extern float current_angle;//瑜版挸澧犵憴鎺戝
extern float target_angle;//閻╊喗鐖ｇ憴鎺戝
extern float angle_error;//鐠囶垰妯?
extern float turn_control_output;
extern float base_speed; // 閸樼喎婀存潪顒€鎮滈敍灞界唨绾偓闁喎瀹崇拋鍙ヨ礋 0 

void track_protection(void);
extern uint8 current_key;
extern uint8 last_key_state ;
extern uint8 pwm_state;
extern uint8 Pwmout;

extern char pwm_state_charge;      // 瑜版挸澧犳潏鎾冲毉閻樿埖鈧?
uint8 key_scan_with_pwm(void);  // 閺囨潙鎮曢崥搴ｆ畱閸戣姤鏆?

#define KEY_EVENT_NONE        0
#define KEY_EVENT_PAGE_PREV   1
#define KEY_EVENT_ADJ_INC     2
#define KEY_EVENT_ITEM_NEXT   3
#define KEY_EVENT_PAGE_NEXT   4
#define KEY_EVENT_ADJ_DEC     5
#define KEY_EVENT_RUN_TOGGLE  6
#define KEY_EVENT_SAVE_ALL    7
#define KEY_EVENT_ENTER_CLEAN 8

/*--------閻㈠灚鍔呯粻妤佺《--------*/
extern float L_raw,R_raw;//閸氬酣娼伴崶娑楅嚋鏉╂ɑ鐥呴悽銊ュ煂
extern float LM_raw,RM_raw;//閸氬酣娼伴崶娑楅嚋鏉╂ɑ鐥呴悽銊ュ煂

extern float L,R,LM,RM,MID ;//閸氬酣娼伴崶娑楅嚋鏉╂ɑ鐥呴悽銊ュ煂
extern uint16  max_AD ,min_AD ;

extern uint16  i ,j ,k1 ,temp ;

/*--------濞村鐦粻妤佺《--------*/
void motor_rotate_and_reverse(int pwm_channel_forward, int pwm_channel_reverse, int rotate_time, int reverse_time, int pwm_duty_max);





float filter(float value);
void key_scan_cycle_pwm_state(void); // 鏉╂柨娲栫猾璇茬€烽弨閫涜礋 void
float limit_range(float input, float limit);


/**************************************************************************
閸戣姤鏆熼崝鐔诲厴閿涙艾顦荤拋鎯у挤pid閸掓繂顫愰崠?閸忋儱褰涢崣鍌涙殶閿涙碍妫?
鏉╂柨娲? 閸婄》绱皐u
**************************************************************************/
void init(void);
/**************************************************************************
閸戣姤鏆熼崝鐔诲厴閿?-- 鏉堝懎濮崙鑺ユ殶閿涙俺顔曠純顔间箯閸欏疇鐤嗛惄顔界垼闁喎瀹?---
閸忋儱褰涢崣鍌涙殶閿涙碍妫?
鏉╂柨娲? 閸婄》绱皐u
**************************************************************************/
void set_target_speeds(float left_target, float right_target);

//閺夊啴鍣?
float Calculate_Weight_Mid(uint16 M);
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state);
//鐠囪褰?
uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution);
//闂勬劕绠?
float limit_float(float value, float min_limit, float max_limit);
float normalize_float(float value, float min, float max);
//闁喎瀹?
float calculate_dynamic_target_speed_quadratic(float current_mid_value);

//typedef enum {
//	RGB_COLOR_OFF = 0,        // 閺? P26=0, P74=0, P07=0
//	RGB_COLOR_WHITE = 1,      // 閻? P26=0, P74=0, P07=1
//	RGB_COLOR_CYAN = 2,       // 闂? P26=0, P74=1, P07=0
//	RGB_COLOR_YELLOW_GREEN = 3, // 姒涘嫮璞? P26=1, P74=0, P07=0
//	RGB_COLOR_MAGENTA = 4,    // 缁? P26=0, P74=1, P07=1
//	RGB_COLOR_GREEN = 5,      // 缂? P26=1, P74=1, P07=0
//	RGB_COLOR_RED = 6,        // 缁? P26=1, P74=0, P07=1
//	RGB_COLOR_BLUE = 7        // 閽? P26=1, P74=1, P07=1
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
////閸椻€崇毜閺?//void kalanma_data();
//void init_kalman();
//extern float k_roll;
//extern float k_pitch;




#endif 












