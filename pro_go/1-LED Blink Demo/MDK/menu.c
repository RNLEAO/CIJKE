#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "isr.h"

uint8 key_mode_local =0;
/****************电机****************//****************电机****************//****************电机****************/
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty, uint8 key_press, uint8 dir_id)
{
    static unsigned char key_mode_local = 0;

    if (key_press == 3) {
        lcd_clear(WHITE); 
        key_mode_local++;
        if (key_mode_local >= 3) key_mode_local = 0; 
    }

    // 参数调节
    switch (key_mode_local) {
        case 0:
            adjust_parameter_by_key_float(key_press, &sptr->kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_press, &sptr->ki,(float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_press, &sptr->kd, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    // 第一行标题与方向判断
		if (dir_id == 0)
				lcd_showstr(0, 0, "left");
		else
				lcd_showstr(0, 0, "right");


    // 显示 PID 参数
    lcd_showfloat(0, 1, sptr->kp, 3, 6);
    lcd_showstr(80, 1, " Kp ");
    lcd_showstr(120, 1, (key_mode_local == 0) ? "<" : " ");

    lcd_showfloat(0, 2, sptr->ki, 3, 6);
    lcd_showstr(80, 2, " Ki ");
    lcd_showstr(120, 2, (key_mode_local == 1) ? "<" : " ");

    lcd_showfloat(0, 3, sptr->kd, 3, 6);
    lcd_showstr(80, 3, " Kd ");
    lcd_showstr(120, 3, (key_mode_local == 2) ? "<" : " ");

    // 显示 PID 输出项和状态信息
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



void display_g(void)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test;

    key_value_test = key_scan(1);

    if (key_value_test == 3) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 6) key_mode = 0; // kp, ki, kd, kp1, kd2, err_t
    }

    switch (key_mode) {
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

    // 标题行
    lcd_showstr(0, 0, "gyro");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    // 参数显示从第1行开始
    lcd_showfloat(0, 1, Gyro_PID.kp, 3, 3);
    lcd_showstr(89, 1, " Kp ");
    if (key_mode == 0) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

    lcd_showfloat(0, 2, Gyro_PID.ki, 4, 3);
    lcd_showstr(89, 2, " Ki ");
    if (key_mode == 1) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");

    lcd_showfloat(0, 3, Gyro_PID.kd, 3, 3);
    lcd_showstr(89, 3, " Kd ");
    if (key_mode == 2) lcd_showstr(120, 3, "<"); else lcd_showstr(120, 3, " ");

    lcd_showfloat(0, 4, Gyro_PID.kp1, 3, 4);
    lcd_showstr(89, 4, " Kp1 ");
    if (key_mode == 3) lcd_showstr(120, 4, "<"); else lcd_showstr(120, 4, " ");

    lcd_showfloat(0, 5, Gyro_PID.kd2, 3, 4);
    lcd_showstr(89, 5, " Kd2 ");
    if (key_mode == 4) lcd_showstr(120, 5, "<"); else lcd_showstr(120, 5, " ");

    lcd_showfloat(0, 6, err_t, 3, 3);
    lcd_showstr(89, 6, " et ");
    if (key_mode == 5) lcd_showstr(120, 6, "<"); else lcd_showstr(120, 6, " ");

    lcd_showfloat(0, 7, Gyro_PID.out, 5, 2); 
    lcd_showstr(89, 7, " out ");

    lcd_showfloat(0, 8, Gyro_PID.err, 5, 2); 
    lcd_showstr(89, 8, " err ");
}



/*********************方向环*********************//*********************方向环*********************//*********************方向环*********************/	

void display_t(void)
{
	
	    static unsigned char key_mode = 0;
    unsigned char key_value_test;

    key_value_test = key_scan(1);

    if (key_value_test == 3) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 5) key_mode = 0; // 原来是6项，现在只剩5项
    }

    switch (key_mode) {
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

    // 标题行
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
        err_H * (L - R) +
        err_X * (LM - RM), 3, 4);

    lcd_showfloat(60, 8,
        err_HM * (L + R) +
        err_D * fabs(LM - RM) +
        err_M * MID, 3, 4);



}






void display_straight_param(void)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test;

    key_value_test = key_scan(1);

    // 切换编辑项（误差阈值 / 积分阈值）
    if (key_value_test == 3) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 2) key_mode = 0; // 只有两个参数
    }

    // 参数调节
    switch (key_mode) {
        case 0:
            adjust_parameter_by_key_float(key_value_test, &straight_err_threshold, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_value_test, &straight_integral_threshold, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    // 显示标题
    lcd_showstr(0, 0, "straight");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    // 显示误差阈值
    lcd_showfloat(0, 1, straight_err_threshold, 2, 3);
    lcd_showstr(60, 1, " Err_Th ");
    if (key_mode == 0) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

    // 显示积分阈值
    lcd_showfloat(0, 2, straight_integral_threshold, 2, 3);
    lcd_showstr(60, 2, " Int_Th ");
    if (key_mode == 1) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");

    // 显示当前误差值（Turn_PID.err）
    lcd_showfloat(0, 4, Turn_PID.err, 3, 3);
    lcd_showstr(89, 4, " Err ");

    // 显示当前积分值
    lcd_showfloat(0, 5, encoder_straight_element, 3, 3);
    lcd_showstr(89, 5, " Int ");

    // 显示状态标志位
    lcd_showstr(0, 7, "Flag:");
    if (straight_flag) lcd_showstr(50, 7, "1"); else lcd_showstr(50, 7, "0");
}





#define FLASH_SYSTEM_PARAMS_ADDR   0x000  // 合并后的数据地址
#define FLASH_MAGIC_ADDR           0x100  // 魔数地址（可以放页 1）
#define FLASH_MAGIC_NUMBER         0x55AA55AAUL
void save_all_params_to_flash(void)
{
    SystemParams buffer;
    unsigned long magic = FLASH_MAGIC_NUMBER;

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

    iap_erase_page(0); // 参数页
    iap_erase_page(1); // 魔数页

    iap_write_bytes(FLASH_SYSTEM_PARAMS_ADDR, (unsigned char*)&buffer, sizeof(buffer));
    iap_write_bytes(FLASH_MAGIC_ADDR, (unsigned char*)&magic, sizeof(magic));
}


bit load_all_params_from_flash(void)
{
    SystemParams buffer;
    unsigned long magic;
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

        has_valid_data = 1;
    } else {
        // 默认值设置
        Turn_PID.kp  = 164.0f;
        Turn_PID.ki  = 78.1f;
        Turn_PID.kd  = 76.0f;
        Turn_PID.kp1 = 0.42f;

        err_H  = 1.0f;
        err_X  = 3.0f;
        err_HM = 1.0f;
        err_D  = 1.0f;
        err_M  = 0.28f;

        L_pid.kp = 0.9f;  L_pid.ki = 0.65f;  L_pid.kd = 0.0f;
        R_pid.kp = 0.9f;  R_pid.ki = 0.65f;  R_pid.kd = 0.0f;

        Gyro_PID.kp  = 2.2f;
        Gyro_PID.ki  = 0.58f;
        Gyro_PID.kd  = 2.0f;
        Gyro_PID.kp1 = 0.0f;
    }

    return has_valid_data;
}



// Global variables for menu state
int current_menu_level = 1; // 1 = Main Menu, 2 = Submenu
int main_menu_selection = 0; // Index of the highlighted item in the main menu (0 to 5 - 1)
int active_submenu = -1; // Which submenu is active (-1 if none)
uint8 key_value_test = 0; // Stores the result of key_scan

#define MAIN_MENU_ITEMS 6   // ���˵�������
#define CURSOR_X_POS    100   // ָʾ��ͷ "<<<" ��ʾ�� X ����
#define CURSOR_STR      "< " // ָʾ��ͷ�ַ��� (���Ե������Ⱥ���ʽ)
#define NO_CURSOR_STR   "  " // ���������ͷ�Ŀո��ַ��� (ȷ������ƥ�� CURSOR_STR)

// --- ȷ����Щȫ�ֱ�������������Ĵ����ж����� ---
extern int main_menu_selection; // ��ǰѡ�еĲ˵������� (0 �� MAIN_MENU_ITEMS - 1)
extern const char *main_menu_options[MAIN_MENU_ITEMS]; // �洢�˵����ı�������



const char *main_menu_options[MAIN_MENU_ITEMS] = {
    "R_PID",
    "L_PID",
    "TURN_PID",
    "check",
    "MOVE",
		"ERROR"
};



/****************主屏幕****************//****************主屏幕****************//****************主屏幕****************/


void display_submenu_check() {
						
					// 第 0 行：左电感 L
					lcd_showstr(0, 0, "L:   ");
					lcd_showuint16(36, 0, L);

					// 第 1 行：右电感 R
					lcd_showstr(0, 1, "R:   ");
					lcd_showuint16(36, 1, R);

					// 第 2 行：左中电感 LM
					lcd_showstr(0, 2, "LM:  ");
					lcd_showuint16(36, 2, LM);

					// 第 3 行：右中电感 RM
					lcd_showstr(0, 3, "RM:  ");
					lcd_showuint16(36, 3, RM);

					// 第 4 行：中线电感 MID
					lcd_showstr(0, 4, "MID: ");
					lcd_showuint16(36, 4, MID);

					// 第 5 行：电感总和
					lcd_showstr(0, 5, "Sum: ");
					lcd_showuint16(36, 5, L + R + LM + RM + MID);

					// 第 6 行：当前路径误差
					lcd_showstr(0, 6, "Err: ");
					lcd_showfloat(36, 6, Turn_PID.err, 5, 2);

					// 第 7 行：轮速和目标
					lcd_showstr(0, 7, "lv");
					lcd_showfloat(14, 7, l_speed_now, 4, 1);
					lcd_showstr(65, 7, "lT");
					lcd_showfloat(80, 7, L_pid.Target_base, 4, 0);
					
					lcd_showstr(0, 8, "rv");
					lcd_showfloat(14, 8, r_speed_now, 4, 1);
					lcd_showstr(65, 8, "rT");
					lcd_showfloat(80, 8, R_pid.Target_base, 4, 0);

					lcd_showstr(49, 9, "inc");
					lcd_showfloat(0, 9, mot_inc, 3, 1);


					// 可调参数
					key_value_test = key_scan(1);
					adjust_parameter_by_key_float(key_value_test, &R_pid.Target_base, (float)x_t_int * x_t_float);
					L_pid.Target_base = R_pid.Target_base;


					

}



/****************步进值****************//****************步进值****************//****************步进值****************/
void display_submenu_ee() {
    static unsigned char key_mode = 0;
    unsigned char key_value_test;

    key_value_test = key_scan(1);

    if (key_value_test == 3) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0; 
					lcd_clear(WHITE); // Clear screen for submenu
    }

    switch (key_mode) {

        case 0:
            adjust_parameter_by_key_float(key_value_test, &x_t_int, 2);
            break;
				 case 1:
            adjust_parameter_by_key_float(key_value_test, &x_t_float, 0.1);
            break;
				 case 2:
            adjust_parameter_by_key_float(key_value_test, &err_t, (float)x_t_int * x_t_float*0.00001);
            break;
        default:			
            break;
    }


		
    lcd_showfloat(0, 0, x_t_int, 2, 4);
    lcd_showstr(80, 0, " x_t ");
    if (key_mode == 0) lcd_showstr(120, 0, "<"); else lcd_showstr(120, 0, " ");
		
		lcd_showfloat(0, 1, x_t_float, 2, 4);
    lcd_showstr(80, 1, " x_f ");
    if (key_mode == 1) lcd_showstr(120,1, "<"); else lcd_showstr(120, 1, " ");
		
		lcd_showfloat(0, 2, err_t, 2, 4);
    lcd_showstr(80, 2, " e_t ");
    if (key_mode == 2) lcd_showstr(120,2, "<"); else lcd_showstr(120, 2, " ");
		
		
    
    lcd_showfloat(20, 9, x_t_int * x_t_float, 2,3);
}
/****************电感误差系数****************//****************电感误差系数****************//****************电感误差系数****************/

void display_gyro(uint8 key_press) {
		
	
	
		static unsigned char key_mode = 0;

	
	    if (key_press == 3) {
        lcd_clear(WHITE); 
        key_mode++;
        if (key_mode >= 7) key_mode = 0; // Adjust cycling to the number of parameters
    }
			
	
			switch (key_mode) {
					case 0:
							adjust_parameter_by_key_float(key_press, &dir_loop_limit, (float)x_t_int * x_t_float);
							break;
					case 1:
							adjust_parameter_by_key_float(key_press, &dir_enlarge, (float)x_t_int * x_t_float);
							break;
					case 2:
						   adjust_parameter_by_key_float(key_press, &err_H, (float)x_t_int * x_t_float*0.1);
						   break;
					case 3:
								adjust_parameter_by_key_float(key_press, &err_X, (float)x_t_int * x_t_float*0.1);
								break;
					case 4:
								adjust_parameter_by_key_float(key_press, &err_HM, (float)x_t_int * x_t_float*0.1);
					case 5:
								adjust_parameter_by_key_float(key_press, &err_D, (float)x_t_int * x_t_float*0.1);
					case 6:
								adjust_parameter_by_key_float(key_press, &err_M, (float)x_t_int * x_t_float*0.1);
					
					default:
							break;
		}
		
		
		lcd_showfloat(0, 0, dir_loop_limit, 5, 2);
		lcd_showstr(70, 0, " mit");
		if (key_mode == 0) lcd_showstr(120, 0, CURSOR_STR); else lcd_showstr(120, 0, NO_CURSOR_STR);

		lcd_showfloat(0, 1, dir_enlarge, 4, 2);
		lcd_showstr(70, 1, " lar");
		if (key_mode == 1) lcd_showstr(120, 1, CURSOR_STR); else lcd_showstr(120, 1, NO_CURSOR_STR);

		lcd_showfloat(0, 2, err_H, 2, 4); // 假设 err_H 显示格式为X.XX
		lcd_showstr(70, 2, " errH");
		if (key_mode == 2) lcd_showstr(120, 2, CURSOR_STR); else lcd_showstr(120, 2, NO_CURSOR_STR);

		lcd_showfloat(0, 3, err_X, 2, 4); // 假设 err_X 显示格式为X.XX
		lcd_showstr(70, 3, " errX");
		if (key_mode == 3) lcd_showstr(120, 3, CURSOR_STR); else lcd_showstr(120, 3, NO_CURSOR_STR);

		lcd_showfloat(0, 4, err_HM, 2, 4); // 假设 err_HM 显示格式为X.XX
		lcd_showstr(70, 4, " errC");
		if (key_mode == 4) lcd_showstr(120, 4, CURSOR_STR); else lcd_showstr(120, 4, NO_CURSOR_STR);

		lcd_showfloat(0, 5, err_D, 2, 4); // 假设 err_HM 显示格式为X.XX
		lcd_showstr(70, 5, " errD");
		if (key_mode == 5) lcd_showstr(120, 5, CURSOR_STR); else lcd_showstr(120, 5, NO_CURSOR_STR);

		lcd_showfloat(0, 6, err_M, 2, 4); // 假设 err_HM 显示格式为X.XX
		lcd_showstr(70, 6, " errM");
		if (key_mode == 6) lcd_showstr(120, 6, CURSOR_STR); else lcd_showstr(120, 6, NO_CURSOR_STR);
		

}




// --- Main Menu Display Function (Unchanged) ---
//The main menu displays content
void display_main_menu() {
    int i = 0;
    for ( i = 0; i < MAIN_MENU_ITEMS; i++) {
        lcd_showstr(0, i, main_menu_options[i]);
				//
        if (i == main_menu_selection) {
            lcd_showstr(CURSOR_X_POS, i, CURSOR_STR);
        } else {
            lcd_showstr(CURSOR_X_POS, i, NO_CURSOR_STR);
        }
    }
}



float x_t_int=1;
float x_t_float=0.2;
/****************菜单切换****************//****************菜单切换****************//****************菜单切换****************/

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
 一维卡尔曼滤波器 
**************************************************************************/
//KalmanFilter kf_L, kf_LM, kf_RM, kf_R, kf_MID;

////初始化
//void kalman_init(KalmanFilter* filter, float q, float r, float initial_value)
//{
//    filter->q = q;
//    filter->r = r;
//    filter->x = initial_value;
//    filter->p = 1.0f; // 初始协方差
//    filter->k = 0;
//}
//void kalman_filters_init(void)
//{
//    kalman_init(&kf_L,   0.01f, 3.0f, 0);
//    kalman_init(&kf_LM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_RM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_R,   0.01f, 3.0f, 0);
//    kalman_init(&kf_MID, 0.01f, 3.0f, 0);
//}

////更新函数
//float kalman_update(KalmanFilter* filter, float measurement)
//{
//    // 预测更新
//    filter->p = filter->p + filter->q;

//    // 计算卡尔曼增益
//    filter->k = filter->p / (filter->p + filter->r);

//    // 更新估计值
//    filter->x = filter->x + filter->k * (measurement - filter->x);

//    // 更新误差协方差
//    filter->p = (1 - filter->k) * filter->p;

//    return filter->x;
//}


//		// 卡尔曼滤波处理
//		L   = kalman_update(&kf_L,   L);
//		LM  = kalman_update(&kf_LM,  LM);
//		RM  = kalman_update(&kf_RM,  RM);
//		R   = kalman_update(&kf_R,   R);
//		MID = kalman_update(&kf_MID, MID);














