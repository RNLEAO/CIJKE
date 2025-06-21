

#include "headfile.h"


		////////////////////////////////////////***************��ʱ����***************////////////////////////////////////////

uint8 key_value;
int8 key_mode=-1;                 
uint8 menu_sign=9;
uint8 display_mode=0;

//电池adc采集
float vbat_in=0;
float adc_vbat=0;




		////////////////////////////////////////***************修车指南***************////////////////////////////////////////
		//甩尾看方向环

void main()
{

		L_pid.kp=10.9;
		L_pid.ki=0.61;
		L_pid.kd=0;
		 
		R_pid.kp=10.9;
		R_pid.ki=0.61;
		R_pid.kd=0;
		
		//	error = (left_mag - right_mag) / (left_mag + right_mag);
		Turn_PID.kp=174;
		Turn_PID.ki=132;
		Turn_PID.kd=56;
		Turn_PID.kp1=0.54;
	

	
	
		Gyro_PID.kp=2.2;
		Gyro_PID.ki=0.58;
		Gyro_PID.kd=2;
		Gyro_PID.kp1=0;
	

		board_init();
		adc_init(ADC_P11, ADC_SYSclk_DIV_2);	  
		delay_init();	

		
	////////////////////////////////////////***************������***************////////////////////////////////////////
		imu660ra_init();
		lcd_init();  		
		delay_init();
		
		//flash
//		iap_init();                     
//		load_all_params_from_flash();
		
		#if 1
		pit_timer_ms(TIM_1,5);				
		pit_timer_ms(TIM_4, 10);			
		#endif
		
		init();
		
		change_speed_Target_base(260);
		

		P52=0;





	while(1)
	{


			#if 1
	
			if(pwm_state==0)  lcd_showstr(82, 9, " stop");
			if(pwm_state==1)  lcd_showstr(82, 9, " gogo");
			if(pwm_state==2)  lcd_showstr(82, 9, " clean");

			key_value = key_scan(1);
			current_key = key_scan(1); 

				// 向下翻页 (key_value == 4)
				if (key_value == 4) {
						lcd_clear(WHITE); // 清屏操作，可以根据需求放置
						// 如果当前是最后一项，则回到第一项 (0)
						if (key_mode >= (menu_sign - 1)) { 
								key_mode = 0;
						} else { // 否则，切换到下一项
								key_mode++;
						}
				}
				// 向上翻页 (key_value == 1)
				else if (key_value == 1) { // 注意这里是 else if，确保了 key_value 不会同时满足 4 和 1
						lcd_clear(WHITE); // 清屏操作，可以根据需求放置
						// 如果当前是第一项 (0)，则回到最后一项 (menu_sign - 1)
						if (key_mode <= 0) {
								key_mode = (menu_sign - 1);
						} else { // 否则，切换到前一项
								key_mode--;
						}
				}
			

					
//				if (key_value == 4) {
//          save_all_params_to_flash();
//					lcd_showstr(0, 9, "SAVE");
//				}

				
		
			switch (key_mode) {
			case 0: display_submenu_check(); break;
				
			case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty, key_value,0); break;
				
			case 2: display_motor(&R_pid,r_speed_now,current_r_pwm_duty, key_value,1); break;
				
			case 3: display_t(); break;
				
			case 4: display_submenu_ee(); break;
				
			case 5: display_gyro(key_value);break;
			
			case 6: display_g();break;	
			
			case 7: display_straight_param();break;		
				
			default:



				lcd_show_font(60, 20, 32, 32, Tang_32x32, BLACK, WHITE);
				lcd_show_font(60, 60, 32, 32, Hua_32x32, BLACK, WHITE);
				lcd_show_font(90, 50, 32, 32, Wei_32x32, BLACK, WHITE);
				lcd_show_font(90, 90, 32, 32, Mian_32x32, BLACK, WHITE);

			
			
			
				break;
	    }
		
		

			#endif
		
		
		
			#if 0


				vofa_send_data[0] = L;
				vofa_send_data[1] = R;
				vofa_send_data[2] = LM;
				vofa_send_data[3] = RM;
				vofa_send_data[4] = MID;
				vofa_send_data[5] = error;
			
				vodka_JustFloat_send(vofa_send_data, 6);
			 #endif
		 
			
			
		 

			
			
			

		}


}

