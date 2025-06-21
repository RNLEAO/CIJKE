#ifndef _MENU_H
#define _MENU_H

void display_r();
void display_l();
void display_t();
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty, uint8 key_press, uint8 dir_id);
void display(void);
void display_gyro(uint8 key_press);
void display_submenu_ee();
void display_submenu_check();
void display_g(void);
void display_straight_param(void);
extern float x_t_int;
extern float x_t_float;



//typedef struct {
//    float q;       
//    float r;       
//    float x;       
//    float p;       
//    float k;       
//} KalmanFilter;
//extern KalmanFilter kf_L, kf_LM, kf_RM, kf_R, kf_MID;
//void kalman_init(KalmanFilter* filter, float q, float r, float initial_value);
//void kalman_filters_init(void);
//float kalman_update(KalmanFilter* filter, float measurement);





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
} SystemParams;



void save_all_params_to_flash(void);
bit load_all_params_from_flash(void);






#endif 





