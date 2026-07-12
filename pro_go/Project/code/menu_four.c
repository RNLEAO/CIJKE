#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "isr.h"
#include "inductance4.h"

uint8 key_mode_local =0;
/* ASCII-cleaned legacy section. */
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty, uint8 key_press, uint8 dir_id)
{
    static unsigned char key_mode_local = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode_local++;
        if (key_mode_local >= 3) key_mode_local = 0;
    }

    switch (key_mode_local)
    {
        case 0:
            adjust_parameter_by_key_float(key_press, &sptr->kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_press, &sptr->ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_press, &sptr->kd, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    if (dir_id == 0)
        lcd_showstr(0, 0, "left");
    else
        lcd_showstr(0, 0, "right");

    lcd_showfloat(0, 1, sptr->kp, 3, 6);
    lcd_showstr(80, 1, " Kp ");
    lcd_showstr(120, 1, (key_mode_local == 0) ? "<" : " ");

    lcd_showfloat(0, 2, sptr->ki, 3, 6);
    lcd_showstr(80, 2, " Ki ");
    lcd_showstr(120, 2, (key_mode_local == 1) ? "<" : " ");

    lcd_showfloat(0, 3, sptr->kd, 3, 6);
    lcd_showstr(80, 3, " Kd ");
    lcd_showstr(120, 3, (key_mode_local == 2) ? "<" : " ");

    lcd_showfloat(0, 4, sptr->kp_out, 3, 2);
    lcd_showstr(80, 4, " Pout ");

    lcd_showfloat(0, 5, sptr->ki_out, 3, 2);
    lcd_showstr(80, 5, " Iout ");

    lcd_showuint16(0, 6, pwm_duty);
    lcd_showstr(80, 6, " PWM ");

    lcd_showfloat(0, 7, speed_now, 3, 3);
    lcd_showstr(80, 7, " V_now ");

    lcd_showuint16(0, 8, sptr->Target);
    lcd_showstr(80, 8, " Tar ");
}

void display_g(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 6) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kd, (float)x_t_int * x_t_float);
            break;
        case 3:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kp1, (float)x_t_int * x_t_float * 0.01f);
            break;
        case 4:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kd2, (float)x_t_int * x_t_float);
            break;
        case 5:
            adjust_parameter_by_key_float(key_value_test, &err_t, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "gyro");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, Gyro_PID.kp, 3, 3);
    lcd_showstr(89, 1, " Kp ");
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 2, Gyro_PID.ki, 4, 3);
    lcd_showstr(89, 2, " Ki ");
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 3, Gyro_PID.kd, 3, 3);
    lcd_showstr(89, 3, " Kd ");
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showfloat(0, 4, Gyro_PID.kp1, 3, 4);
    lcd_showstr(89, 4, " Kp1 ");
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showfloat(0, 5, Gyro_PID.kd2, 3, 4);
    lcd_showstr(89, 5, " Kd2 ");
    lcd_showstr(120, 5, key_mode == 4 ? "<" : " ");

    lcd_showfloat(0, 6, err_t, 3, 3);
    lcd_showstr(89, 6, " et ");
    lcd_showstr(120, 6, key_mode == 5 ? "<" : " ");

    lcd_showfloat(0, 7, Gyro_PID.out, 5, 2);
    lcd_showstr(89, 7, " out ");

    lcd_showfloat(0, 8, Gyro_PID.err, 5, 2);
    lcd_showstr(89, 8, " err ");
}

void display_t(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kd, (float)x_t_int * x_t_float);
            break;
        case 3:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kp1, (float)x_t_int * x_t_float * 0.01f);
            break;
        case 4:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kd2, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "turn");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, Turn_PID.kp, 3, 3);
    lcd_showstr(89, 1, " Kp ");
    lcd_showstr(120, 1, (key_mode == 0) ? "<" : " ");

    lcd_showfloat(0, 2, Turn_PID.ki, 4, 3);
    lcd_showstr(89, 2, " Ki ");
    lcd_showstr(120, 2, (key_mode == 1) ? "<" : " ");

    lcd_showfloat(0, 3, Turn_PID.kd, 3, 3);
    lcd_showstr(89, 3, " Kd ");
    lcd_showstr(120, 3, (key_mode == 2) ? "<" : " ");

    lcd_showfloat(0, 4, Turn_PID.kp1, 3, 4);
    lcd_showstr(89, 4, " Kp1 ");
    lcd_showstr(120, 4, (key_mode == 3) ? "<" : " ");

    lcd_showfloat(0, 5, Turn_PID.kd2, 3, 3);
    lcd_showstr(89, 5, " Kd2 ");
    lcd_showstr(120, 5, (key_mode == 4) ? "<" : " ");

    lcd_showfloat(0, 6, Turn_PID.out, 5, 2);
    lcd_showstr(89, 6, " out ");

    lcd_showfloat(0, 7, Turn_PID.err, 3, 5);
    lcd_showstr(89, 7, " err ");

    lcd_showfloat(0, 8,
        inductance_error_a * (L - R) +
        inductance_error_b * (LM - RM), 3, 4);

    lcd_showfloat(60, 8,
        inductance_error_a * (L + R) +
        inductance_error_c * fabs(LM - RM), 3, 4);
}







//void display_straight_param(void)
//{
//    static unsigned char key_mode = 0;
//    unsigned char key_value_test;

//    key_value_test = key_scan(1);

// ASCII-cleaned legacy comment.
//    if (key_value_test == 3) {
//        lcd_clear(WHITE);
//        key_mode++;
// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
//    switch (key_mode) {
//        case 0:
//            adjust_parameter_by_key_float(key_value_test, &straight_err_threshold, (float)x_t_int * x_t_float);
//            break;
//        case 1:
//            adjust_parameter_by_key_float(key_value_test, &straight_integral_threshold, (float)x_t_int * x_t_float);
//            break;
//        default:
//            break;
//    }

// ASCII-cleaned legacy comment.
//    lcd_showstr(0, 0, "straight");
//    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

// ASCII-cleaned legacy comment.
//    lcd_showstr(60, 1, " Err_Th ");
//    if (key_mode == 0) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

// ASCII-cleaned legacy comment.
//    lcd_showstr(60, 2, " Int_Th ");
//    if (key_mode == 1) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");
//		
//	 lcd_showfloat(0, 3, gyro_right_angle, 2, 3);
//   lcd_showstr(60, 3, " rtang ");

//		

// ASCII-cleaned legacy comment.
//    lcd_showstr(89, 4, " Err ");

// ASCII-cleaned legacy comment.
//    lcd_showstr(89, 5, " Int ");

// ASCII-cleaned legacy comment.
//    if (right_angle_flag) lcd_showstr(50, 7, "1"); else lcd_showstr(50, 7, "0");
//}


// ASCII-cleaned legacy comment.
// ASCII-cleaned legacy comment.
// ASCII-cleaned legacy comment.
// ==========================================================
//void display_right_angle_param(void)
//{
// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
//    lcd_showfloat(0, 2, right_angle_flag, 1, 0);
// ASCII-cleaned legacy comment.
//    lcd_showfloat(0, 3, gyro_roll_sign_angle, 1, 0);
// ASCII-cleaned legacy comment.
// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
//    lcd_showfloat(0, 4, gyro_right_angle, 3, 1);
//    lcd_showstr(45, 4, "/");
//    lcd_showfloat(55, 4, RIGHT_ANGLE_TARGET_ANGLE, 3, 0);
// ASCII-cleaned legacy comment.
// ASCII-cleaned legacy comment.
//    lcd_showstr(45, 5, "/");
//    lcd_showfloat(55, 5, RIGHT_ANGLE_EXIT_COUNT, 3, 0);
// ASCII-cleaned legacy comment.

#define FLASH_SYSTEM_PARAMS_ADDR   0x000  // ASCII-cleaned legacy comment.
#define FLASH_MAGIC_ADDR           0x100  // ASCII-cleaned legacy comment.
#define FLASH_MAGIC_NUMBER         0x55AA55AAUL
// Global variables for menu state
int current_menu_level = 1; // 1 = Main Menu, 2 = Submenu
int main_menu_selection = 0; // Index of the highlighted item in the main menu (0 to 5 - 1)
int active_submenu = -1; // Which submenu is active (-1 if none)
uint8 key_value_test = 0; // Stores the result of key_scan

#define MAIN_MENU_ITEMS 6   // ASCII-cleaned legacy comment.
#define CURSOR_X_POS    100   // ASCII-cleaned legacy comment.
#define CURSOR_STR      "< " // ASCII-cleaned legacy comment.
#define NO_CURSOR_STR   "  " // ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
extern int main_menu_selection; // ASCII-cleaned legacy comment.
extern const char *main_menu_options[MAIN_MENU_ITEMS]; // ASCII-cleaned legacy comment.


//const char *main_menu_options[MAIN_MENU_ITEMS] = {
//    "R_PID",
//    "L_PID",
//    "TURN_PID",
//    "check",
//    "MOVE",
//		"ERROR"
//};


void save_all_params_to_flash(void)
{
    SystemParams xdata buffer;
    unsigned long xdata magic = FLASH_MAGIC_NUMBER;

    // Turn PID
    buffer.kp  = Turn_PID.kp;
    buffer.ki  = Turn_PID.ki;
    buffer.kd  = Turn_PID.kd;
    buffer.kp1 = Turn_PID.kp1;

    // Error params
    buffer.err_H  = err_H;
    buffer.err_X  = err_X;
    buffer.err_HM = err_HM;
    buffer.err_D  = err_D;
    buffer.err_M  = err_M;

    // Left motor PID
    buffer.L_kp = L_pid.kp;
    buffer.L_ki = L_pid.ki;
    buffer.L_kd = L_pid.kd;

    // Right motor PID
    buffer.R_kp = R_pid.kp;
    buffer.R_ki = R_pid.ki;
    buffer.R_kd = R_pid.kd;

    // Gyro PID
    buffer.G_kp  = Gyro_PID.kp;
    buffer.G_ki  = Gyro_PID.ki;
    buffer.G_kd  = Gyro_PID.kd;
    buffer.G_kp1 = Gyro_PID.kp1;

    // ASCII-cleaned legacy comment.
    buffer.in_circle_LR = in_circle_LR;
    buffer.in_circle_MID = in_circle_MID;
    buffer.in_circle_LRMID = in_circle_LRMID;
		buffer.ring_error = ring_error;
		
		
		
    // ASCII-cleaned legacy comment.
    buffer.ring_inc_element12 = ring_inc_element12;
    buffer.ring_inc_element56 = ring_inc_element56;
    buffer.ring_inc_element67 = ring_inc_element67;

    buffer.ring_angle_23 = ring_angle_23;
    buffer.ring_angle_34 = ring_angle_34;
    buffer.ring_angle_45 = ring_angle_45;

    buffer.temp_flag_tar = temp_flag_tar;;
		
		
				// ASCII-cleaned legacy comment.
		buffer.speed[1] = speed[1];
		buffer.speed[2] = speed[2];
		buffer.speed[3] = speed[3];
		buffer.speed[4] = speed[4];



    // ASCII-cleaned legacy comment.
    buffer.adc_vbat_tar = adc_vbat_tar;
    buffer.encoder_charge_element_vbat_tar = encoder_charge_element_vbat_tar;
    buffer.charge_pwm_open_val = charge_pwm_open_val;



    iap_erase_page(0); // ASCII-cleaned legacy comment.
    iap_write_bytes(FLASH_SYSTEM_PARAMS_ADDR, (unsigned char*)&buffer, sizeof(buffer));
    iap_write_bytes(FLASH_MAGIC_ADDR, (unsigned char*)&magic, sizeof(magic));
}





bit load_all_params_from_flash(void)
{
    SystemParams xdata buffer;
    unsigned long xdata magic;
    bit has_valid_data = 0;

    iap_read_bytes(FLASH_MAGIC_ADDR, (unsigned char*)&magic, sizeof(magic));

    if (magic == FLASH_MAGIC_NUMBER) {
        iap_read_bytes(FLASH_SYSTEM_PARAMS_ADDR, (unsigned char*)&buffer, sizeof(buffer));

        // Turn PID
        Turn_PID.kp  = buffer.kp;
        Turn_PID.ki  = buffer.ki;
        Turn_PID.kd  = buffer.kd;
        Turn_PID.kp1 = buffer.kp1;

        // Error weights
        err_H  = buffer.err_H;
        err_X  = buffer.err_X;
        err_HM = buffer.err_HM;
        err_D  = buffer.err_D;
        err_M  = buffer.err_M;

        // Left motor PID
        L_pid.kp = buffer.L_kp;
        L_pid.ki = buffer.L_ki;
        L_pid.kd = buffer.L_kd;

        // Right motor PID
        R_pid.kp = buffer.R_kp;
        R_pid.ki = buffer.R_ki;
        R_pid.kd = buffer.R_kd;

        // Gyro PID
        Gyro_PID.kp  = buffer.G_kp;
        Gyro_PID.ki  = buffer.G_ki;
        Gyro_PID.kd  = buffer.G_kd;
        Gyro_PID.kp1 = buffer.G_kp1;

        // ASCII-cleaned legacy comment.
        in_circle_MID = buffer.in_circle_MID;
				in_circle_LRMID= buffer.in_circle_LRMID;
				ring_error= buffer.ring_error;
				
				
        ring_inc_element12 = buffer.ring_inc_element12;
        ring_inc_element56 = buffer.ring_inc_element56;
        ring_inc_element67 = buffer.ring_inc_element67;

        ring_angle_23 = buffer.ring_angle_23;
        ring_angle_34 = buffer.ring_angle_34;
        ring_angle_45 = buffer.ring_angle_45;


								// ASCII-cleaned legacy comment.
				speed[1] = buffer.speed[1];
				speed[2] = buffer.speed[2];
				speed[3] = buffer.speed[3];
				speed[4] = buffer.speed[4];


        temp_flag_tar = buffer.temp_flag_tar;
				
				
				 // ASCII-cleaned legacy comment.
        adc_vbat_tar = buffer.adc_vbat_tar;
        encoder_charge_element_vbat_tar = buffer.encoder_charge_element_vbat_tar;
        charge_pwm_open_val = buffer.charge_pwm_open_val;

				
        has_valid_data = 1;
    } else {
        
    }

    return has_valid_data;
}



/* ASCII-cleaned legacy section. */


void display_submenu_check(uint8 key_press) {
    key_value_test = key_press;

    lcd_showstr(0, 0, "L:   ");
    lcd_showuint16(31, 0, L);
    lcd_showfloat(80, 0, adc_vbat, 3, 1);

    lcd_showstr(0, 1, "R:   ");
    lcd_showuint16(31, 1, R);

    lcd_showstr(0, 2, "LM:  ");
    lcd_showuint16(31, 2, LM);

    lcd_showstr(0, 3, "RM:  ");
    lcd_showuint16(31, 3, RM);

    lcd_showstr(0, 4, "State:");
    lcd_showstr(42, 4, element4_state_name());

    lcd_showstr(0, 5, "Sum: ");
    lcd_showuint16(31, 5, L + R + LM + RM);

    lcd_showstr(0, 6, "Err: ");
    lcd_showfloat(36, 6, Turn_PID.err, 5, 2);

    lcd_showstr(0, 7, "lv");
    lcd_showfloat(14, 7, l_speed_now, 4, 1);
    lcd_showstr(65, 7, "lT");
    lcd_showfloat(80, 7, L_pid.Target_base, 4, 1);

    lcd_showstr(0, 8, "rv");
    lcd_showfloat(14, 8, r_speed_now, 4, 1);
    lcd_showstr(65, 8, "rT");
    lcd_showfloat(80, 8, R_pid.Target_base, 4, 1);

    lcd_showstr(49, 9, "inc");
    lcd_showfloat(0, 9, mot_inc, 3, 1);

    adjust_parameter_by_key_float(key_value_test, &speed[0], (float)x_t_int * x_t_float);
    R_pid.Target_base = speed[0];
    L_pid.Target_base = R_pid.Target_base;
}


void display_circle_debug_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
        lcd_clear(WHITE);
    }

    if (key_mode == 0)
        adjust_parameter_by_key_float(key_value, &ring_outer_high, 1);
    else if (key_mode == 1)
        adjust_parameter_by_key_float(key_value, &ring_side_diff, 1);
    else if (key_mode == 2)
        adjust_parameter_by_key_float(key_value, &ring_inner_low, 1);
    else if (key_mode == 3)
        adjust_parameter_by_key_float(key_value, &ring_inner_active, 1);
    else if (key_mode == 4)
        adjust_parameter_by_key_float(key_value, &ring_error, (float)x_t_int * x_t_float * 0.05f);

    lcd_showstr(0, 0, "Circle");

    lcd_showstr(0, 1, "OuterHi:");
    lcd_showfloat(80, 1, ring_outer_high, 3, 1);
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "SideDif:");
    lcd_showfloat(80, 2, ring_side_diff, 3, 1);
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "InnerLo:");
    lcd_showfloat(80, 3, ring_inner_low, 3, 1);
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "InnerOn:");
    lcd_showfloat(80, 4, ring_inner_active, 3, 1);
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "RingErr:");
    lcd_showfloat(80, 5, ring_error, 2, 3);
    lcd_showstr(120, 5, key_mode == 4 ? "<" : " ");

    lcd_showstr(0, 6, "Flag:");
    lcd_showuint8(80, 6, cir_flag);

    lcd_showstr(0, 7, "GSign:");
    lcd_showuint8(80, 7, gyro_roll_sign_rign);
}


void display_circle_advanced_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 7) key_mode = 0;
        lcd_clear(WHITE);
    }

    if (key_mode == 0)
        adjust_parameter_by_key_float(key_value, &ring_inc_element12, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 1)
        adjust_parameter_by_key_float(key_value, &ring_inc_element56, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 2)
        adjust_parameter_by_key_float(key_value, &ring_inc_element67, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 3)
        adjust_parameter_by_key_float(key_value, &ring_angle_23, (float)x_t_int * x_t_float);
    else if (key_mode == 4)
        adjust_parameter_by_key_float(key_value, &ring_angle_34, (float)x_t_int * x_t_float);
    else if (key_mode == 5)
        adjust_parameter_by_key_float(key_value, &ring_angle_45, (float)x_t_int * x_t_float);
    else if (key_mode == 6)
        adjust_parameter_by_key_float(key_value, &temp_flag_tar, (float)x_t_int * x_t_float);

    lcd_showstr(0, 0, "Circle Adv");

    lcd_showstr(0, 1, "inc12:");
    lcd_showfloat(65, 1, ring_inc_element12, 1, 2);
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "inc56:");
    lcd_showfloat(65, 2, ring_inc_element56, 1, 2);
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "inc67:");
    lcd_showfloat(65, 3, ring_inc_element67, 1, 2);
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "ang23:");
    lcd_showfloat(65, 4, ring_angle_23, 3, 1);
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "ang34:");
    lcd_showfloat(65, 5, ring_angle_34, 3, 1);
    lcd_showstr(120, 5, key_mode == 4 ? "<" : " ");

    lcd_showstr(0, 6, "ang45:");
    lcd_showfloat(65, 6, ring_angle_45, 3, 1);
    lcd_showstr(120, 6, key_mode == 5 ? "<" : " ");

    lcd_showstr(0, 7, "TempTar:");
    lcd_showfloat(65, 7, temp_flag_tar, 3, 0);
    lcd_showstr(120, 7, key_mode == 6 ? "<" : " ");
}


void display_speed_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
        lcd_clear(WHITE);
    }

    adjust_parameter_by_key_float(key_value, &speed[key_mode], (float)x_t_int * x_t_float);

    lcd_showstr(0, 0, "Speed Menu");

    lcd_showstr(0, 1, "Spd0:");
    lcd_showfloat(60, 1, speed[0], 3, 0);
    lcd_showstr(110, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "Spd1:");
    lcd_showfloat(60, 2, speed[1], 3, 0);
    lcd_showstr(110, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "Spd2:");
    lcd_showfloat(60, 3, speed[2], 3, 0);
    lcd_showstr(110, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "Spd3:");
    lcd_showfloat(60, 4, speed[3], 3, 0);
    lcd_showstr(110, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "Spd4:");
    lcd_showfloat(60, 5, speed[4], 3, 0);
    lcd_showstr(110, 5, key_mode == 4 ? "<" : " ");

    lcd_showstr(0, 7, "xT:");
    lcd_showfloat(40, 7, x_t_int * x_t_float, 3, 2);
}


void display_submenu_ee(uint8 key_press) {
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_value_test, &x_t_int, 2); break;
        case 1: adjust_parameter_by_key_float(key_value_test, &x_t_float, 0.1f); break;
        case 2: adjust_parameter_by_key_float(key_value_test, &err_t, (float)x_t_int * x_t_float * 0.00001f); break;
        default: break;
    }

    lcd_showfloat(0, 0, x_t_int, 2, 4);
    lcd_showstr(80, 0, " x_t ");
    lcd_showstr(120, 0, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 1, x_t_float, 2, 4);
    lcd_showstr(80, 1, " x_f ");
    lcd_showstr(120, 1, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 2, err_t, 2, 4);
    lcd_showstr(80, 2, " e_t ");
    lcd_showstr(120, 2, key_mode == 2 ? "<" : " ");

    lcd_showfloat(20, 9, x_t_int * x_t_float, 2, 3);
}


void display_gyro(uint8 key_press) {
    static unsigned char key_mode = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 7) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_press, &dir_loop_limit, (float)x_t_int * x_t_float); break;
        case 1: adjust_parameter_by_key_float(key_press, &dir_enlarge, (float)x_t_int * x_t_float); break;
        case 2: adjust_parameter_by_key_float(key_press, &err_H, (float)x_t_int * x_t_float * 0.1f); break;
        case 3: adjust_parameter_by_key_float(key_press, &err_X, (float)x_t_int * x_t_float * 0.1f); break;
        case 4: adjust_parameter_by_key_float(key_press, &err_HM, (float)x_t_int * x_t_float * 0.1f); break;
        case 5: adjust_parameter_by_key_float(key_press, &err_D, (float)x_t_int * x_t_float * 0.1f); break;
        case 6: adjust_parameter_by_key_float(key_press, &err_M, (float)x_t_int * x_t_float * 0.1f); break;
        default: break;
    }

    lcd_showfloat(0, 0, dir_loop_limit, 5, 2);
    lcd_showstr(70, 0, " mit");
    lcd_showstr(120, 0, key_mode == 0 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 1, dir_enlarge, 4, 2);
    lcd_showstr(70, 1, " lar");
    lcd_showstr(120, 1, key_mode == 1 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 2, err_H, 2, 4);
    lcd_showstr(70, 2, " errH");
    lcd_showstr(120, 2, key_mode == 2 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 3, err_X, 2, 4);
    lcd_showstr(70, 3, " errX");
    lcd_showstr(120, 3, key_mode == 3 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 4, err_HM, 2, 4);
    lcd_showstr(70, 4, " errC");
    lcd_showstr(120, 4, key_mode == 4 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 5, err_D, 2, 4);
    lcd_showstr(70, 5, " errD");
    lcd_showstr(120, 5, key_mode == 5 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 6, err_M, 2, 4);
    lcd_showstr(70, 6, " errM");
    lcd_showstr(120, 6, key_mode == 6 ? CURSOR_STR : NO_CURSOR_STR);
}


void display_submenu_charge_debug(uint8 key_press) {
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_value_test, &adc_vbat_tar, 0.2f); break;
        case 1: adjust_parameter_by_key_float(key_value_test, &encoder_charge_element_vbat_tar, (float)x_t_int * x_t_float * 0.2f); break;
        case 2: adjust_parameter_by_key_float(key_value_test, &charge_pwm_open_val, (float)x_t_int); break;
        default: break;
    }

    lcd_showfloat(0, 0, adc_vbat_tar, 2, 2);
    lcd_showstr(60, 0, " Vtar ");
    lcd_showstr(120, 0, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 1, encoder_charge_element_vbat_tar, 2, 2);
    lcd_showstr(60, 1, " ENC_tar ");
    lcd_showstr(120, 1, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 2, charge_pwm_open_val, 4, 0);
    lcd_showstr(60, 2, " PWM ");
    lcd_showstr(120, 2, key_mode == 2 ? "<" : " ");

    lcd_showfloat(0, 4, encoder_charge_sign, 2, 1);
    lcd_showstr(52, 4, " ENC_SIGN ");

    lcd_showfloat(0, 5, pwm_state_charge, 2, 1);
    lcd_showstr(52, 5, " PWM_STATE ");

    lcd_showfloat(0, 6, encoder_charge_element, 2, 4);
    lcd_showstr(52, 6, " ENC_ELE ");
}


void display_straight_param(uint8 key_press)
{
    static unsigned char key_mode = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 2) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_press, &straight_err_threshold, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_press, &straight_integral_threshold, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "Straight");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, straight_err_threshold, 2, 3);
    lcd_showstr(60, 1, " Err_Th ");
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 2, straight_integral_threshold, 2, 3);
    lcd_showstr(60, 2, " Int_Th ");
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 3, gyro_right_angle, 2, 3);
    lcd_showstr(60, 3, " rtang ");

    lcd_showfloat(0, 4, Turn_PID.err, 3, 3);
    lcd_showstr(89, 4, " Err ");

    lcd_showfloat(0, 5, encoder_straight_element, 3, 3);
    lcd_showstr(89, 5, " Int ");

    lcd_showstr(0, 7, "Flag:");
    lcd_showfloat(50, 7, right_angle_flag, 1, 0);
}

void display_right_angle_param(uint8 key_press)
{
    static uint8 selected = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        selected++;
        if (selected >= 5U) selected = 0;
        lcd_clear(WHITE);
    }

    if (selected == 0U)
        adjust_parameter_by_key_float(key_press, &right_angle_outer_low, 1.0f);
    else if (selected == 1U)
        adjust_parameter_by_key_float(key_press, &right_angle_inner_high, 1.0f);
    else if (selected == 2U)
        adjust_parameter_by_key_float(key_press, &right_angle_inner_low, 1.0f);
    else if (selected == 3U)
        adjust_parameter_by_key_float(key_press, &right_angle_inner_diff, 1.0f);
    else if (selected == 4U)
        adjust_parameter_by_key_float(key_press, &right_angle_target_angle, 1.0f);

    lcd_showstr(0, 0, "RA PARAM");

    lcd_showstr(0, 1, selected == 0U ? ">OutLo" : " OutLo");
    lcd_showfloat(75, 1, right_angle_outer_low, 3, 1);

    lcd_showstr(0, 2, selected == 1U ? ">InHi" : " InHi");
    lcd_showfloat(75, 2, right_angle_inner_high, 3, 1);

    lcd_showstr(0, 3, selected == 2U ? ">InLo" : " InLo");
    lcd_showfloat(75, 3, right_angle_inner_low, 3, 1);

    lcd_showstr(0, 4, selected == 3U ? ">Diff" : " Diff");
    lcd_showfloat(75, 4, right_angle_inner_diff, 3, 1);

    lcd_showstr(0, 5, selected == 4U ? ">Angle" : " Angle");
    lcd_showfloat(75, 5, right_angle_target_angle, 3, 1);

    lcd_showstr(0, 6, "NowAng:");
    lcd_showfloat(75, 6, gyro_right_angle, 3, 1);

    lcd_showstr(0, 7, "State:");
    lcd_showstr(45, 7, element4_state_name());
}

float x_t_int=1;
float x_t_float=0.2;
/* ASCII-cleaned legacy section. */

//void display(void)
//{
//    static unsigned char key_mode = 0;
//	  static unsigned char key_mode_in = -1;

//    unsigned char key_value_test;

//    key_value_test = key_scan(1);

//	        if (current_menu_level == 1) {
//            // --- Handle Main Menu ---
//            if (key_value_test == 3) { // P76 (Down Navigation)
//										main_menu_selection++;
//                if (main_menu_selection >= MAIN_MENU_ITEMS) {
//                    main_menu_selection = 0; // Wrap around
//                }
//                  key_scan(1); // IMPORTANT: Reset key state after processing navigation
//                  display_main_menu(); // Update display immediately
//            } else if (key_value_test == 4) { // P46 (Confirm/Enter)
//                current_menu_level = 2;
//                active_submenu = main_menu_selection;
//                key_scan(1);// IMPORTANT: Reset key state after processing action
//                lcd_clear(WHITE); // Clear screen for submenu
//            } 
//								display_main_menu();
//				} 
//	
//			
//				
//				
//			else if (current_menu_level == 2) {
//            // --- Handle Submenu ---
//            if (key_value_test == 1) { // P77 (Back)
//							
//                current_menu_level = 1;
//                active_submenu = -1;
//                key_value_test =key_scan(1); // IMPORTANT: Reset key state after processing action
//                lcd_clear(WHITE); // Clear screen for main menu
//                display_main_menu(); // Display main menu immediately
//            } else {
//							
//							
//                switch (active_submenu) {
//                    case 0: display_motor(&R_pid,r_speed_now,current_r_pwm_duty, key_value); break;
//                    case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty, key_value); break;
//                    case 2: display_t(); break;
//                    case 3: display_submenu_check(); break;
//                    case 4: display_submenu_ee(); break;
//										case 5: display_gyro(key_value_test);break;
//									
//                    default:
//										lcd_showstr(0, 0, "Error: Invalid Submenu");
//										break;
//                }
//            }
//        }
//	
//	

//}

/**************************************************************************
 ?
**************************************************************************/
//KalmanFilter kf_L, kf_LM, kf_RM, kf_R, kf_MID;

// ASCII-cleaned legacy comment.
//{
//    filter->q = q;
//    filter->r = r;
//    filter->x = initial_value;
// ASCII-cleaned legacy comment.
//}
//void kalman_filters_init(void)
//{
//    kalman_init(&kf_L,   0.01f, 3.0f, 0);
//    kalman_init(&kf_LM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_RM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_R,   0.01f, 3.0f, 0);
//    kalman_init(&kf_MID, 0.01f, 3.0f, 0);
//}

// ASCII-cleaned legacy comment.
//float kalman_update(KalmanFilter* filter, float measurement)
//{
// ASCII-cleaned legacy comment.
//    filter->p = filter->p + filter->q;

// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.

//    return filter->x;
//}


// ASCII-cleaned legacy comment.
//		LM  = kalman_update(&kf_LM,  LM);
//		RM  = kalman_update(&kf_RM,  RM);
//		R   = kalman_update(&kf_R,   R);
//		MID = kalman_update(&kf_MID, MID);













