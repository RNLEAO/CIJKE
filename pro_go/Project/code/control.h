#ifndef _CONTROL_H
#define _CONTROL_H
#include "headfile.h"
// encoder interface
#define MOTOR1_ENCODER CTIM0_P34
#define MOTOR2_ENCODER CTIM3_P04
#define MOTOR1_DIR P35
#define MOTOR2_DIR P53


// Dual-PWM H-bridge outputs.
#define MOTOR1 PWMA_CH2P_P62
#define MOTOR1_d P64

#define MOTOR2 PWMA_CH1P_P60
#define MOTOR2_d P66






#define Screen_switch P37

#define M_PI 3.14159265358979323846  // Define M_PI if not defined
#define  ang_limit 200


uint8 key_scan(int mode);//閹稿鏁幍顐ｅ伎
uint8 fetch_ui_key_event(void);
float low_pass_filter(float current_value, float last_value, float alpha); // First-order low-pass filter.
void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step);
float Get_roll(void);


extern int sample_count;
extern int state;

// IMU data used by the steering controller.
extern float gyro_data[1];

extern float gyro_roll,gyro_pitch;
extern float roll_accel,pitch_accel;


extern float roll,pitch;
extern float Angle_x ;
/************ Angle control state *************/
extern float current_angle;//瑜版挸澧犵憴鎺戝
extern float target_angle;//閻╊喗鐖ｇ憴鎺戝
extern float angle_error;
extern float turn_control_output;
extern float base_speed;

void track_protection(void);
extern uint8 current_key;
extern uint8 last_key_state ;
extern uint8 pwm_state;
extern uint8 Pwmout;

extern char pwm_state_charge;
uint8 key_scan_with_pwm(void);

#define KEY_EVENT_NONE        0
#define KEY_EVENT_PAGE_PREV   1
#define KEY_EVENT_ADJ_INC     2
#define KEY_EVENT_ITEM_NEXT   3
#define KEY_EVENT_PAGE_NEXT   4
#define KEY_EVENT_ADJ_DEC     5
#define KEY_EVENT_RUN_TOGGLE  6
#define KEY_EVENT_SAVE_ALL    7
#define KEY_EVENT_ENTER_CLEAN 8

/* Four-channel inductance values: outer-left, inner-left, inner-right, outer-right. */
extern float L_raw,R_raw;
extern float LM_raw,RM_raw;

extern float L,R,LM,RM,MID; // MID is retained only as a zero-valued legacy symbol.
extern uint16  max_AD ,min_AD ;

extern uint16  i ,j ,k1 ,temp ;

/* Motor and control helpers. */
void motor_rotate_and_reverse(int pwm_channel_forward, int pwm_channel_reverse, int rotate_time, int reverse_time, int pwm_duty_max);





float filter(float value);
void key_scan_cycle_pwm_state(void);
float limit_range(float input, float limit);


/**************************************************************************
Initialize ADC, encoder, PWM, wireless UART, and control GPIO resources.
**************************************************************************/
void init(void);
/**************************************************************************
Set independent left and right differential-drive target speeds.
**************************************************************************/
void set_target_speeds(float left_target, float right_target);

// Legacy middle-channel weight helper. Four-channel mode keeps MID at zero.
float Calculate_Weight_Mid(uint16 M);
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state);
// ADC averaging helper.
uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution);
// Floating-point clamp helper.
float limit_float(float value, float min_limit, float max_limit);
float normalize_float(float value, float min, float max);
// Dynamic target-speed helper.
float calculate_dynamic_target_speed_quadratic(float current_mid_value);

//typedef enum {
//	RGB_COLOR_OFF = 0,
//	RGB_COLOR_WHITE = 1,
//	RGB_COLOR_CYAN = 2,
//	RGB_COLOR_YELLOW_GREEN = 3,
//	RGB_COLOR_MAGENTA = 4,
//	RGB_COLOR_GREEN = 5,
//	RGB_COLOR_RED = 6,
//	RGB_COLOR_BLUE = 7
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
// Optional Kalman-filter interfaces remain disabled.
// void kalanma_data();
//void init_kalman();
//extern float k_roll;
//extern float k_pitch;




#endif 












