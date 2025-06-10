#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "isr.h"

uint8 key_mode_local =0;
/****************电机****************//****************电机****************//****************电机****************/

void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty,  char* direction,uint8 key_press)
{
    static unsigned char key_mode_local; 
//    unsigned char key_value_test=0;
//		key_value_test = key_scan(1);

    if (key_press == 3) {
        lcd_clear(WHITE); 
        key_mode_local++;
        if (key_mode_local >= 3) key_mode_local = 0; 
    }

    switch (key_mode_local) {
        case 0:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &sptr->kp, 0.005);
            break;
        case 1:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &sptr->ki, 0.001);
            break;
        case 2:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &sptr->kd, 0.005);
            break;

        default:
            break;
    }

    
    lcd_showfloat(0, 0, sptr->kp, 3, 6);
    lcd_showstr(80, 0, " Kp ");
    if (key_mode_local == 0) lcd_showstr(120, 0, "<"); else lcd_showstr(120, 0, " ");

    lcd_showfloat(0, 1, sptr->ki, 4, 6);
    lcd_showstr(80, 1, " Ki ");
    if (key_mode_local == 1) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

    lcd_showfloat(0, 2, sptr->kd, 3, 6);
    lcd_showstr(80, 2, " Kd ");
    if (key_mode_local == 2) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");

    lcd_showfloat(3, 3, sptr->kp_out, 3, 2);
    lcd_showstr(60, 3, " pout ");

    lcd_showfloat(3, 4, sptr->ki_out, 3, 2);
    lcd_showstr(60, 4, " iout ");

    lcd_showfloat(3, 5, sptr->kd_out, 3, 2);
    lcd_showstr(60, 5, " dout ");

    lcd_showuint16(0, 6, pwm_duty);
    lcd_showstr(60, 6, " pwm ");

    lcd_showfloat(0, 7, speed_now, 3, 3);
    lcd_showstr(60, 7, " V_now ");

    lcd_showuint16(0, 8, sptr->Target);
    lcd_showstr(60, 8, " tar ");

    lcd_showstr(0, 9, direction);
  

}




void display_t(void)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test;

    key_value_test = key_scan(1);

    if (key_value_test == 3) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 5) key_mode = 0; // 调整循环到参数数量 (kp, ki, kd, kp1, kd2, err_t, x_t_int 共7个)
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
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kp1, (float)x_t_int * x_t_float*0.01);
            break;
        case 4:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kd2, (float)x_t_int * x_t_float); // 添加对 kd2 的调整
            break;
        default:
            break;
    }

    lcd_showfloat(0, 0, Turn_PID.kp, 5, 3);
    lcd_showstr(80, 0, " Kp ");
    if (key_mode == 0) lcd_showstr(120, 0, "<"); else lcd_showstr(120, 0, " ");

    lcd_showfloat(0, 1, Turn_PID.ki, 4, 6);
    lcd_showstr(80, 1, " Ki ");
    if (key_mode == 1) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

    lcd_showfloat(0, 2, Turn_PID.kd, 3, 6);
    lcd_showstr(80, 2, " Kd ");
    if (key_mode == 2) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");

    lcd_showfloat(0, 3, Turn_PID.kp1, 3, 6);
    lcd_showstr(80, 3, " Kp1 ");
    if (key_mode == 3) lcd_showstr(120, 3, "<"); else lcd_showstr(120, 3, " ");

    lcd_showfloat(0, 4, Turn_PID.kd2, 3, 6); // 显示 kd2
    lcd_showstr(80, 4, " Kd2 ");
    if (key_mode == 4) lcd_showstr(120, 4, "<"); else lcd_showstr(120, 4, " ");

    lcd_showfloat(0, 5, err_t, 2, 4);
    lcd_showstr(80, 5, " e_t ");
    if (key_mode == 5) lcd_showstr(120, 5, "<"); else lcd_showstr(120, 5, " ");

		
		
    
    lcd_showstr(0, 9, "turn");
    lcd_showfloat(20, 9, x_t_int * x_t_float, 2,3);
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


void display_r(uint8 key_press) {
		
	
	
		static unsigned char key_mode = 0;

	
	    if (key_press == 3) {
        lcd_clear(WHITE); 
        key_mode++;
        if (key_mode >= 5) key_mode = 0; // Adjust cycling to the number of parameters
    }
			
	
	    switch (key_mode) {
        case 0:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &Turn_PID.kp, 5);
            break;
        case 1:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &Turn_PID.ki, 0.001);
            break;
        case 2:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &Turn_PID.kd, 0.005);
            break;
        case 3:
            key_press = key_scan(1);
            adjust_parameter_by_key_float(key_press, &Turn_PID.kp1, 0.005);
            break;
        default:
            break;
    }
			
		
		lcd_showfloat(0, 0, Turn_PID.kp, 5, 3);
    lcd_showstr(80, 0, " Kp ");
    if (key_mode == 0) lcd_showstr(120, 0, CURSOR_STR); else lcd_showstr(120, 0, NO_CURSOR_STR);

    lcd_showfloat(0, 1, Turn_PID.ki, 4, 6);
    lcd_showstr(80, 1, " Ki ");
    if (key_mode == 1) lcd_showstr(120, 1, CURSOR_STR); else lcd_showstr(120, 1, NO_CURSOR_STR);

    lcd_showfloat(0, 2, Turn_PID.kd, 3, 6);
    lcd_showstr(80, 2, " Kd ");
    if (key_mode == 2) lcd_showstr(120, 2, CURSOR_STR); else lcd_showstr(120, 2, NO_CURSOR_STR);

    lcd_showfloat(0, 3, Turn_PID.kp1, 3, 6);
    lcd_showstr(80, 3, " Kp1 ");
    if (key_mode == 3) lcd_showstr(120, 3, CURSOR_STR); else lcd_showstr(120, 2, NO_CURSOR_STR);
	


    

}
/****************主屏幕****************//****************主屏幕****************//****************主屏幕****************/


void display_submenu_check() {
	
	
//					lcd_showuint16(0,0,L);
//					lcd_showuint16(60,0,R);
//					lcd_showuint16(0,1,LM);
//					lcd_showuint16(60,1,RM);
//					lcd_showuint16(0,2,MID);
//					lcd_showuint16(60,2,L+R+LM+RM+MID);

//					lcd_showstr(80, 3, "error");					
//					lcd_showfloat(0,3,Turn_PID.err,2,4);

//					key_value_test=key_scan(1);
//					adjust_parameter_by_key_float(key_value_test,&R_pid.Target,(float)x_t_int * x_t_float);
//				//串级记得换
//					L_pid.Target=R_pid.Target;
//					lcd_showfloat(0,4,l_speed_now,4,2);		
//					lcd_showfloat(60,4,r_speed_now,4,2);
//					lcd_showfloat(0,5,angular_acceleration*err_t,4,2);
//					
//					lcd_showint32(0,6,(int32)current_l_pwm_duty,6);		
//					lcd_showint32(60,6,(int32)current_r_pwm_duty,6);		
//					
//					lcd_showint32(0 ,7, (int32)L_pid.Target,4);		
//					lcd_showint32(50,7,(int32)R_pid.Target,4);		
//					
//					lcd_showint32(40,8,(int32)Turn_PID.out,6);		
//					lcd_showstr(0, 9, "check");
//					lcd_showfloat(0,8,mot_inc,2,3);

					lcd_showuint16(0,0,L);
					lcd_showuint16(60,0,R);
					lcd_showuint16(0,1,LM);
					lcd_showuint16(60,1,RM);
					lcd_showuint16(0,2,MID);
					lcd_showuint16(60,2,L+R+LM+RM+MID);

					lcd_showstr(80, 3, "error");					
					lcd_showfloat(0,3,Turn_PID.err,2,4);

					key_value_test=key_scan(1);
					
					#if 0
					adjust_parameter_by_key_float(key_value_test,&R_pid.Target,(float)x_t_int * x_t_float);
					L_pid.Target=R_pid.Target;
					lcd_showint32(0 ,7, (int32)L_pid.Target,4);		
					lcd_showint32(50,7,(int32)R_pid.Target,4);		
					#endif
					
					#if 1
					adjust_parameter_by_key_float(key_value_test,&R_pid.Target_base,(float)x_t_int * x_t_float);
					L_pid.Target_base=R_pid.Target_base;					
					lcd_showfloat(0,7,L_pid.Target_base,4,0);		
					lcd_showfloat(70,7,R_pid.Target_base,4,0);
					
					#endif
					
					
					
					lcd_showfloat(0,4,l_speed_now,4,2);		
					lcd_showfloat(60,4,r_speed_now,4,2);
					
					lcd_showint32(0,6,(int32)current_l_pwm_duty,6);		
					lcd_showint32(60,6,(int32)current_r_pwm_duty,6);		
					

					

					lcd_showfloat(50,9,cir_angle_flag,2,2);
					lcd_showfloat(0,9,cir_flag,2,2);
					lcd_showfloat(0,8,mot_inc_element,2,3);

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
/****************角速度****************//****************角速度****************//****************角速度****************/

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
								adjust_parameter_by_key_float(key_press, &err_d, (float)x_t_int * x_t_float*0.1);
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

		lcd_showfloat(0, 5, err_d, 2, 4); // 假设 err_HM 显示格式为X.XX
		lcd_showstr(70, 5, " errD");
		if (key_mode == 5) lcd_showstr(120, 5, CURSOR_STR); else lcd_showstr(120, 5, NO_CURSOR_STR);

		lcd_showfloat(0, 6, err_M, 2, 4); // 假设 err_HM 显示格式为X.XX
		lcd_showstr(70, 6, " errM");
		if (key_mode == 6) lcd_showstr(120, 6, CURSOR_STR); else lcd_showstr(120, 6, NO_CURSOR_STR);
		
//		lcd_showfloat(0, 0, Gyro_PID.kp, 5, 3);
//    lcd_showstr(80, 0, " Kp ");
//    if (key_mode == 0) lcd_showstr(120, 0, CURSOR_STR); else lcd_showstr(120, 0, NO_CURSOR_STR);

//    lcd_showfloat(0, 1, Gyro_PID.ki, 4, 6);
//    lcd_showstr(80, 1, " Ki ");
//    if (key_mode == 1) lcd_showstr(120, 1, CURSOR_STR); else lcd_showstr(120, 1, NO_CURSOR_STR);

//    lcd_showfloat(0, 2, Gyro_PID.kd, 3, 6);
//    lcd_showstr(80, 2, " Kd ");
//    if (key_mode == 2) lcd_showstr(120, 2, CURSOR_STR); else lcd_showstr(120, 2, NO_CURSOR_STR);

//    lcd_showfloat(0, 3, Gyro_PID.kp1, 3, 6);
//    lcd_showstr(80, 3, " Kp1 ");
//    if (key_mode == 3) lcd_showstr(120, 3, CURSOR_STR); else lcd_showstr(120, 2, NO_CURSOR_STR);
//	
//		
//		
//		
//		
//		
//		
//		
//		
//		lcd_showfloat(0, 4, Gyro_PID.err, 3, 3);  
//    lcd_showstr(80,4,"err");
//		
//		lcd_showfloat(0, 5, Gyro_PID.out, 4, 3);  
//		lcd_showstr(80,5,"out");
//		
//		
//		lcd_showfloat(0, 6, LEFT_WEIGHT, 4, 3);  
//		lcd_showstr(80,6,"LEFT");
//		if (key_mode == 4) lcd_showstr(120, 6, CURSOR_STR); else lcd_showstr(120, 6, NO_CURSOR_STR);
//		
//		lcd_showfloat(0, 7, RIGHT_WEIGHT, 4, 3);  
//		lcd_showstr(80,7,"RIGHT");
//		if (key_mode == 5) lcd_showstr(120, 7, CURSOR_STR); else lcd_showstr(120, 7, NO_CURSOR_STR);


//				lcd_showuint8(0,0,cir_flag);
//			  lcd_showstr(70,0,"flag");
//				lcd_showfloat(0,1,mot_inc_element,2,4);
//				lcd_showstr(70,1,"inc");

//				lcd_showfloat(0,2,gyro_roll,4,2);
//				lcd_showstr(70,2,"gyro");

//				lcd_showfloat(0,3,Turn_PID.err,4,2);
//			  lcd_showstr(70,3,"err");
//				
//				lcd_showuint8(0,4,temp_flag);
//			  lcd_showstr(70,4,"temp");

				


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

void display(void)
{
    static unsigned char key_mode = 0;
	  static unsigned char key_mode_in = -1;

    unsigned char key_value_test;

    key_value_test = key_scan(1);

	        if (current_menu_level == 1) {
            // --- Handle Main Menu ---
            if (key_value_test == 3) { // P76 (Down Navigation)
										main_menu_selection++;
                if (main_menu_selection >= MAIN_MENU_ITEMS) {
                    main_menu_selection = 0; // Wrap around
                }
                  key_scan(1); // IMPORTANT: Reset key state after processing navigation
                  display_main_menu(); // Update display immediately
            } else if (key_value_test == 4) { // P46 (Confirm/Enter)
                current_menu_level = 2;
                active_submenu = main_menu_selection;
                key_scan(1);// IMPORTANT: Reset key state after processing action
                lcd_clear(WHITE); // Clear screen for submenu
            } 
								display_main_menu();
				} 
	
			
				
				
			else if (current_menu_level == 2) {
            // --- Handle Submenu ---
            if (key_value_test == 1) { // P77 (Back)
							
                current_menu_level = 1;
                active_submenu = -1;
                key_value_test =key_scan(1); // IMPORTANT: Reset key state after processing action
                lcd_clear(WHITE); // Clear screen for main menu
                display_main_menu(); // Display main menu immediately
            } else {
							
							
                switch (active_submenu) {
                    case 0: display_motor(&R_pid,r_speed_now,current_r_pwm_duty,"right", key_value_test); break;
                    case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty,"leftt", key_value_test); break;
                    case 2: display_t(); break;
                    case 3: display_submenu_check(); break;
                    case 4: display_submenu_ee(); break;
										case 5: display_gyro(key_value_test);break;
									
                    default:
										lcd_showstr(0, 0, "Error: Invalid Submenu");
										break;
                }
            }
        }
	
	

}