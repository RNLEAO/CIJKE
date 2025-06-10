

#include "headfile.h"




#define BEEP P67
#define ON 1
#define OFF 0

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

	
//		L_pid.kp=4.9;
//		L_pid.ki=0.25;
//		L_pid.kd=0;
//		 
//		R_pid.kp=4.9;
//		R_pid.ki=0.25;
//		R_pid.kd=0;

		L_pid.kp=10.9;
		L_pid.ki=0.65;
		L_pid.kd=0;
		 
		R_pid.kp=10.9;
		R_pid.ki=0.65;
		R_pid.kd=0;
		
		//	error = (left_mag - right_mag) / (left_mag + right_mag);
		Turn_PID.kp=144;
		Turn_PID.ki=66.1;
		Turn_PID.kd=56;
		Turn_PID.kp1=0.54;
	
		
		ang_pid.kp=28;
		ang_pid.ki=0;
		ang_pid.kd=20;
		ang_pid.kp1=0.54;
	

		board_init();
		adc_init(ADC_P11, ADC_SYSclk_DIV_2);	  
		delay_init();	
		
//		while(1) {
//			vbat_in=adc_once(ADC_P11, ADC_12BIT);
//			adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;

//			if (adc_vbat > 12.8f) {
//					pwm_state_charge=1;
//					pwm_state=1;

//					break; // 电压达标，跳出等待循环
//			}
//    }
		
	////////////////////////////////////////***************������***************////////////////////////////////////////
		imu660ra_init();
		lcd_init();  		
		delay_init();
		#if 1
		pit_timer_ms(TIM_1,5);				
		#endif
		
		adc_init(ADC_P00, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P01, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P10, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P05, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P06, ADC_SYSclk_DIV_2);	  
		delay_ms(10);

		ctimer_count_init(MOTOR1_ENCODER);
		ctimer_count_init(MOTOR2_ENCODER);
		delay_ms(10);

		pwm_init(PWMA_CH2P_P62, 17000, 0);
		pwm_init(PWMA_CH1P_P60, 17000, 0);
		pwm_init(PWMA_CH4P_P66, 17000, 0);
		pwm_init(PWMA_CH3P_P64, 17000, 0);
		delay_ms(10);
		
		wireless_uart_init();
		
			
		gpio_mode(P2_6,GPO_PP);
		gpio_mode(P7_4,GPO_PP);
		gpio_mode(P0_7,GPO_PP);
		gpio_mode(P5_2,GPO_PP);		



		L_pid.Target_base = 251; // 设置左电机的目标速度
		R_pid.Target_base = 251;
		
		
	  L_pid.Target = 230; // 设置左电机的目标速度
    R_pid.Target = 230;

		P52=0;

	while(1)
	{


			#if 1
	
		if(pwm_state==0)  lcd_showstr(82, 9, " stop");
		if(pwm_state==1) {
		
		 lcd_showstr(82, 9, " gogo");
		
		}
		if(pwm_state==2)  lcd_showstr(82, 9, " clean");


		key_value=key_scan(1);
		current_key=key_scan(1);
		if(key_value==4){
			lcd_clear(WHITE);
				if(key_mode++ >=(menu_sign - 1))key_mode=0;

		} else if(key_value==4) {
			if(key_mode-- <= 0) key_mode = (menu_sign - 1);
		}
		
		
		
			switch (key_mode) {
			case 0: display_submenu_check(); break;
				
			case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty,"leftt", key_value); break;
				
			case 2: display_motor(&R_pid,r_speed_now,current_r_pwm_duty,"right", key_value); break;
				
			case 3: display_t(); break;
				
			case 4: display_submenu_ee(); break;
				
			case 5: display_gyro(key_value);break;
				
			case 6: 
		  lcd_showfloat(0,6,ang_pid.out,4,1);
			lcd_showfloat(70,7,current_angle,4,1);
			lcd_showfloat(0,7,encoder_charge_element,4,1);
			lcd_showfloat(0,8,cir_flag,2,2);
			lcd_showfloat(70,8,encoder_charge_sign,2,2);

			lcd_showfloat(0,9,ring_out_element,4,1);
			break;	
			
			case 7: 
		  lcd_showfloat(0,0,gyro_roll_cross,4,1);
			break;	
			
			
			default:
			lcd_showstr(0, 0, "shit");
			lcd_showfloat(0,1,cross_flag,4,1);
			
			lcd_showfloat(55,1,temp_flag_speed,4,1);
			lcd_showstr(95, 1, "temp");

			lcd_showfloat(0,2,Turn_PID.err,4,1);
			lcd_showfloat(0,3,gyro_roll_cross,4,1);
			
			
  		lcd_showfloat(0,4,run_mode,4,1);
			lcd_showstr(70, 4, "mode");

			lcd_showfloat(0,5,P36,4,1);

			
  		lcd_showfloat(0,6,encoder_speedup_element,4,1);
			lcd_showstr(70, 6, "element");
			
			lcd_showfloat(0,7,L_pid.Target_base,4,0);
			lcd_showfloat(45,7,R_pid.Target_base,4,0);
			lcd_showstr(95, 7, "base");
			

			lcd_showfloat(0,8,L_pid.Target,4,0);
			lcd_showfloat(45,8,R_pid.Target,4,0);
			lcd_showstr(95, 8, "Tar");
			lcd_showfloat(0,9,mot_inc,4,1);

			break;
	    }
		
		

			#endif
		
		
		
			#if 0
				L_pid.kp=R_pid.kp;
				L_pid.ki=R_pid.ki;
				L_pid.kd=R_pid.kd;
				L_pid.Target=R_pid.Target;

				vofa_send_data[0] = l_speed_now;
				vofa_send_data[1] = r_speed_now;
				vofa_send_data[2] =current_l_pwm_duty;
				vofa_send_data[3] =current_r_pwm_duty;
				vofa_send_data[4] =L_pid.Target;

				 vodka_JustFloat_send(vofa_send_data, 5);
			 #endif
		 
		 

			
			
			

		}


}

