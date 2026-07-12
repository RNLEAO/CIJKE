#ifndef _MENU_H
#define _MENU_H

typedef enum
{
    MENU_PAGE_HOME = 0,
    MENU_PAGE_OVERVIEW,
    MENU_PAGE_LEFT_MOTOR,
    MENU_PAGE_RIGHT_MOTOR,
    MENU_PAGE_TURN_PID,
    MENU_PAGE_ERROR_WEIGHTS,
    MENU_PAGE_GYRO,
    MENU_PAGE_GYRO_DATA,
    MENU_PAGE_STRAIGHT,
    MENU_PAGE_RIGHT_ANGLE,
    MENU_PAGE_RING_DEBUG,
    MENU_PAGE_RING_ADVANCED,
    MENU_PAGE_SPEED,
    MENU_PAGE_CHARGE,
    MENU_PAGE_INDUCTANCE4_CALIBRATION,
    MENU_PAGE_INDUCTANCE4_DATA,
    MENU_PAGE_NEGATIVE_PRESSURE_AUTO,
    MENU_PAGE_NEGATIVE_PRESSURE_OUTPUT,
    MENU_PAGE_COUNT
} MenuPage;

void display_r();
void display_l();
void display_t(uint8 key_press);
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty, uint8 key_press, uint8 dir_id);
void display(void);
void display_gyro(uint8 key_press);
void display_submenu_ee(uint8 key_press);
void display_submenu_check(uint8 key_press);
void display_g(uint8 key_press);
void display_straight_param(uint8 key_press);
void display_right_angle_param(uint8 key_press);
void display_circle_debug_menu(uint8 key_press);
void display_circle_advanced_menu(uint8 key_press);
void display_speed_menu(uint8 key_press);
void display_submenu_charge_debug(uint8 key_press);
extern float x_t_int;
extern float x_t_float;







typedef struct {
    // Turn PID
    float kp, ki, kd, kp1;

    // Error weights
    float err_H, err_X, err_HM, err_D, err_M;

    // Left Motor PID
    float L_kp, L_ki, L_kd;

    // Right Motor PID
    float R_kp, R_ki, R_kd;

    // Gyro PID
    float G_kp, G_ki, G_kd, G_kp1;
	  float in_circle_LR;   // 新增
    float in_circle_MID;  // 新增
	  float in_circle_LRMID;  // 新增
		float ring_error;  // 新增
	
	
    float ring_inc_element12;
    float ring_inc_element56;
    float ring_inc_element67;

    float ring_angle_23;
    float ring_angle_34;
    float ring_angle_45;

    float temp_flag_tar; 
		float speed[5];
		
		float adc_vbat_tar;
		float encoder_charge_element_vbat_tar;
		float charge_pwm_open_val;


} SystemParams;



void save_all_params_to_flash(void);
bit load_all_params_from_flash(void);






#endif 



