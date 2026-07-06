

#include "headfile.h"


		////////////////////////////////////////***************闂佽法鍠愰弸濠氬箯闁垮顦ч梺璺ㄥ枑閺嬪骞忛悜鑺ユ櫢闁哄倶鍊栫€?**************////////////////////////////////////////

uint8 key_value;
int8 key_mode=-1;                 
uint8 menu_sign=13;
uint8 display_mode=0;



extern int zhijiao_flag;


		////////////////////////////////////////***************婵烇絽娴傞崰娑樷攦閸涙潙绠伴柛銉戝啰顢?**************////////////////////////////////////////
		//闂佺儵鎳囬弲娑㈡偄椤掑嫭鍎戦悗锝庡亝閻撴瑩鏌涘顓炵仼妤?
void main()
{		

	
		L_pid.kp=5.9;
		L_pid.ki=0.6;
		L_pid.kd=3;
		 
		R_pid.kp=5.9;
		R_pid.ki=0.6;
		R_pid.kd=3;
	
	
		Turn_PID.kp=108;
		Turn_PID.ki=12.2;
		Turn_PID.kd=447;
		Turn_PID.kp1=0.26;
	
		board_init();
		adc_init(ADC_P11, ADC_SYSclk_DIV_2);	  
		delay_init();	
		
		
//			while(1) {
//				vbat_in=adc_once(ADC_P11, ADC_12BIT);
//				adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;

//				if (adc_vbat > adc_vbat_tar) {
//						pwm_state_charge=1;
//						pwm_state=1;

//						break;    //闂佹眹鍨藉褏鏁崶銊︾秶闁绘劦鍓涢崹濂告煥濞戞鐒烽柣鎴畵瀹曟瑩鎼归崷顓熸儯閻庡灚婢橀幊搴ㄥ箚婵犲洦鍋?//				}
//			}
		
	////////////////////////////////////////***************闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁?**************////////////////////////////////////////
		imu660ra_init();
		lcd_init();  		
		delay_init();
		

		
		#if 1
		pit_timer_ms(TIM_1,5);				
		pit_timer_ms(TIM_4, 10);			
		#endif
		
		init();
		
		change_speed_Target_base(speed[0]);
		//flash
		iap_init();                     
		load_all_params_from_flash();
		

		P52=0;


	while(1)
	{
			#if 1
	
			if(pwm_state==0)  lcd_showstr(82, 9, " stop");
			if(pwm_state==1)  lcd_showstr(82, 9, " gogo");
			if(pwm_state==2)  lcd_showstr(82, 9, " clean");

			key_value = fetch_ui_key_event();
			current_key = key_value; 

				// 闂佸憡纰嶉崹宕囩箔閸涱垳绀勫┑鐘冲搸閳?(key_value == 4)
				if (key_value == KEY_EVENT_PAGE_NEXT) {
						lcd_clear(WHITE); // 濠电偞鎸搁幊搴ㄦ儓閸℃稑绠肩€广儱瀚粙濠囨煥濞戞瀚扮憸鏉垮级缁傛帡濡烽敂鎯ь棏闂佺顕栭崳锝呫€掔捄鐩捬囧磼濞戞瑦婢掔紓?						// 婵犵鈧啿鈧綊鎮樻径搴涗汗闁规儳鍟块·鍛存煛閸曢潧鐏℃繛鎾崇埣瀹曘儲鎯斿┑鍫紘婵＄偑鍊楀▍銏㈡濠靛绀嗘繛鍡楃箲缁€鈧梺鍛婂笚婢瑰棝顢栭崶銊р枖闁逞屽墯閵?(0)
						if (key_mode >= (menu_sign - 1)) { 
								key_mode = 0;
						} else { // 闂佸憡鐔粻鎴﹀垂椤栫偞鏅悘鐐舵閻庡ジ鏌熼獮鍨仼闁糕晛鏈粙澶屸偓锝傛櫇椤忓崬顪?								key_mode++;
						}
				}
				// 闂佸憡纰嶉崹宕囩箔閸屾粎绀勫┑鐘冲搸閳?(key_value == 1)
				else if (key_value == KEY_EVENT_PAGE_PREV) { // 濠电偛顦崝宥夊礈閻楀牊浜ゆ繛鍡樻尭濞咃繝鏌?else if闂佹寧绋戦惉濂稿灳濡崵鈹嶆繝闈涙噽閸?key_value 婵炴垶鎸哥粔宕囨娴兼潙瑙﹂悘鐐靛亾椤ρ勭節婵炴劑鍊曢崰?4 闂?1
						lcd_clear(WHITE); // 濠电偞鎸搁幊搴ㄦ儓閸℃稑绠肩€广儱瀚粙濠囨煥濞戞瀚扮憸鏉垮级缁傛帡濡烽敂鎯ь棏闂佺顕栭崳锝呫€掔捄鐩捬囧磼濞戞瑦婢掔紓?						// 婵犵鈧啿鈧綊鎮樻径搴涗汗闁规儳鍟块·鍛存煛閸曢潧鐏ｆい鎴濇处缁嬪鍩€椤掍降浜?(0)闂佹寧绋戦懟顖炲垂椤栫偛鐐婇柣鎰絻閻撳倿鏌￠崼姘壕闂佸憡鑹剧花鑲╃博鐎涙ǜ浜?(menu_sign - 1)
						if (key_mode <= 0) {
								key_mode = (menu_sign - 1);
						} else { // 闂佸憡鐔粻鎴﹀垂椤栫偞鏅悘鐐舵閻庡ジ鏌熼獮鍨仼闁糕晛鐭傚畷婊冾吋閸曨収浼囨俊?								key_mode--;
						}
				}
			

					
				if (key_value == KEY_EVENT_SAVE_ALL) {
          save_all_params_to_flash();
					lcd_showstr(0, 9, "SAVE ");
				}

				if (key_value == KEY_EVENT_RUN_TOGGLE) {
					lcd_showstr(0, 9, "MODE ");
				}

				if (key_value == KEY_EVENT_ENTER_CLEAN) {
					lcd_showstr(0, 9, "CLEAN");
				}

				
				vbat_in=adc_once(ADC_P11, ADC_12BIT);
				adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;


				switch (key_mode) {
        case 0:  display_submenu_check(key_value); break;
        case 1:  display_motor(&L_pid, l_speed_now, current_l_pwm_duty, key_value, 0); break;
        case 2:  display_motor(&R_pid, r_speed_now, current_r_pwm_duty, key_value, 1); break;
        case 3:  display_t(key_value); break;
        case 4:  display_submenu_ee(key_value); break;
        case 5:  display_gyro(key_value); break;
        case 6:  display_g(key_value); break;
        case 7:  display_straight_param(key_value); break;
        case 8:  display_right_angle_param(key_value); break;
        case 9:  display_circle_debug_menu(key_value); break;
        case 10: display_circle_advanced_menu(key_value); break;
        case 11: display_speed_menu(key_value); break;
        case 12: display_submenu_charge_debug(key_value); break;
        default:
            lcd_show_font(16, 56, 32, 32, Ci_32x32, BLACK, WHITE);
            lcd_show_font(48, 56, 32, 32, Jian_32x32, BLACK, WHITE);
            lcd_show_font(80, 56, 32, 32, Ke_32x32, BLACK, WHITE);
            break;
    }

		
		

			#endif

			#if 0
	
			if(pwm_state==0)  lcd_showstr(82, 9, " stop");
			if(pwm_state==1)  lcd_showstr(82, 9, " gogo");
			if(pwm_state==2)  lcd_showstr(82, 9, " clean");

			key_value = key_scan(1);
			current_key = key_scan(1); 

				// 闂佸憡纰嶉崹宕囩箔閸涱垳绀勫┑鐘冲搸閳?(key_value == 4)
				if (key_value == 4) {
						lcd_clear(WHITE); // 濠电偞鎸搁幊搴ㄦ儓閸℃稑绠肩€广儱瀚粙濠囨煥濞戞瀚扮憸鏉垮级缁傛帡濡烽敂鎯ь棏闂佺顕栭崳锝呫€掔捄鐩捬囧磼濞戞瑦婢掔紓?						// 婵犵鈧啿鈧綊鎮樻径搴涗汗闁规儳鍟块·鍛存煛閸曢潧鐏℃繛鎾崇埣瀹曘儲鎯斿┑鍫紘婵＄偑鍊楀▍銏㈡濠靛绀嗘繛鍡楃箲缁€鈧梺鍛婂笚婢瑰棝顢栭崶銊р枖闁逞屽墯閵?(0)
						if (key_mode >= (menu_sign - 1)) { 
								key_mode = 0;
						} else { // 闂佸憡鐔粻鎴﹀垂椤栫偞鏅悘鐐舵閻庡ジ鏌熼獮鍨仼闁糕晛鏈粙澶屸偓锝傛櫇椤忓崬顪?								key_mode++;
						}
				}
				// 闂佸憡纰嶉崹宕囩箔閸屾粎绀勫┑鐘冲搸閳?(key_value == 1)
				else if (key_value == KEY_EVENT_PAGE_PREV) { // 濠电偛顦崝宥夊礈閻楀牊浜ゆ繛鍡樻尭濞咃繝鏌?else if闂佹寧绋戦惉濂稿灳濡崵鈹嶆繝闈涙噽閸?key_value 婵炴垶鎸哥粔宕囨娴兼潙瑙﹂悘鐐靛亾椤ρ勭節婵炴劑鍊曢崰?4 闂?1
						lcd_clear(WHITE); // 濠电偞鎸搁幊搴ㄦ儓閸℃稑绠肩€广儱瀚粙濠囨煥濞戞瀚扮憸鏉垮级缁傛帡濡烽敂鎯ь棏闂佺顕栭崳锝呫€掔捄鐩捬囧磼濞戞瑦婢掔紓?						// 婵犵鈧啿鈧綊鎮樻径搴涗汗闁规儳鍟块·鍛存煛閸曢潧鐏ｆい鎴濇处缁嬪鍩€椤掍降浜?(0)闂佹寧绋戦懟顖炲垂椤栫偛鐐婇柣鎰絻閻撳倿鏌￠崼姘壕闂佸憡鑹剧花鑲╃博鐎涙ǜ浜?(menu_sign - 1)
						if (key_mode <= 0) {
								key_mode = (menu_sign - 1);
						} else { // 闂佸憡鐔粻鎴﹀垂椤栫偞鏅悘鐐舵閻庡ジ鏌熼獮鍨仼闁糕晛鐭傚畷婊冾吋閸曨収浼囨俊?								key_mode--;
						}
				}
			

					
				if (key_value == 4) {
          save_all_params_to_flash();
					lcd_showstr(0, 9, "SAVE ");
				}

				
				vbat_in=adc_once(ADC_P11, ADC_12BIT);
				adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;


			
		
			switch (key_mode) {
			case 0: display_submenu_check(key_value); break;
				
			case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty, key_value,0); break;
				
			case 2: display_motor(&R_pid,r_speed_now,current_r_pwm_duty, key_value,1); break;
				
			case 3: display_t(key_value); break;
				
			case 4: display_submenu_ee(key_value); break;
				
			case 5: display_gyro(key_value);break;
			
			case 6: display_g(key_value);break;	
			
			case 7: display_straight_param(key_value);break;
				
			case 8: display_right_angle_param(key_value);break;		
			
			case 9: display_circle_debug_menu(key_value);break;		
				
			case 10: display_circle_advanced_menu(key_value);break;	
			
			case 11: display_speed_menu(key_value);break;	
			
			case 12: display_submenu_charge_debug(key_value);break;	
				
			default:

				lcd_show_font(60, 20, 32, 32, Tang_32x32, BLACK, WHITE);
				lcd_show_font(60, 60, 32, 32, Hua_32x32, BLACK, WHITE);
				lcd_show_font(90, 50, 32, 32, Wei_32x32, BLACK, WHITE);
				lcd_show_font(90, 90, 32, 32, Mian_32x32, BLACK, WHITE);

				break;
	    }
		
		

			#endif
		
		
		
			#if 0


				vofa_send_data[0] = zhijiao_flag;
				vofa_send_data[1] = R;
				vofa_send_data[2] = LM;
				vofa_send_data[3] = RM;
				vofa_send_data[4] = MID;
				vofa_send_data[5] = error;
			
				vodka_JustFloat_send(vofa_send_data, 6);
			 #endif
		 
			
			
		 

			
			
			

		}


}
