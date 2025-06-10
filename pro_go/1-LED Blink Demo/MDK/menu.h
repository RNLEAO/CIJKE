#ifndef _MENU_H
#define _MENU_H

void display_r();
void display_l();
void display_t();
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty,  char* direction,uint8 key_press);
void display(void);
void display_gyro(uint8 key_press);
void display_submenu_ee();
void display_submenu_check();


extern float x_t_int;
extern float x_t_float;


#endif 

