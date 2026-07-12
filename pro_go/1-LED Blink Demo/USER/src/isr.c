
#include "headfile.h"



void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鐑樼暦閻犱浇顫夌€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏?
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2闂佽法鍠庤ぐ銊ф媼鐟欏嫬顏?
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲箲瀹勬壆妲戦弶鍫㈠亾鐎氬綊鏌ㄩ悢鍛婄伄闁圭柉娓圭拹鐔兼煥閻斿憡鐏柟椋幟?BUF

	}
}


//UART3闂佽法鍠庤ぐ銊ф媼鐟欏嫬顏?
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲箲瀹勬壆妲戦弶鍫㈠亾鐎氬綊鏌ㄩ悢鍛婄伄闁圭柉娓圭拹鐔兼煥閻斿憡鐏柟椋幟?BUF

	}
}


//UART4闂佽法鍠庤ぐ銊ф媼鐟欏嫬顏?
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG){
			
        UART4_CLEAR_TX_FLAG;
				busy[4] = 0;
	  }
	
    if(UART4_GET_RX_FLAG){
			
        UART4_CLEAR_RX_FLAG;
				 
            RxLine++;
            DataBuff[RxLine-1]=S4BUF;
            if(S4BUF==0x21)
            {
                USART_PID_Adjust();
                memset(DataBuff,0,sizeof(DataBuff));
                RxLine=0;
            }
			
		
		//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲箲瀹勬壆妲戦弶鍫㈠亾鐎氬綊鏌ㄩ悢鍛婄伄闁圭柉娓圭拹鐔兼煥閻斿憡鐏柟椋幟?BUF;
		if(wireless_module_uart_handler != NULL){
			// 闂佽法鍠撻悡顐︽偖鐎涙ê顏堕梺璺ㄥ枑閺嬪骞忛摎鍌濈闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊骞愰崶顒佹櫢闁哄倶鍊栫€?
			// 闂佽法鍠庢慨顓犵驳鐟欏嫬顏跺┑顔碱儔閺佹捇寮妶鍡楊伓闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氱懓螣閿熺姵鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢铏瑰摵闁规彃顑嗙€氬綊鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺鏉垮閸欏懘鏌ㄩ悢鍛婄伄闁归鍏橀弫鎾诲棘閵堝棗顏堕梺璺ㄥ枑閺嬪骞?
			wireless_module_uart_handler(S4BUF);
			
			
		}
	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氱瓉ED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊宕＄粙娆锯偓鎺楁煥閻斿憡鐏柟?
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊宕＄粙娆锯偓鎺楁煥閻斿憡鐏柟?
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊宕＄粙娆锯偓鎺楁煥閻斿憡鐏柟?
}

void TM0_Isr() interrupt 1
{

}



float error=0;




float Roll_x=0;
float err_t=0.000036035f;
#define PWM_DUTY_MIN -8000 
float current_l_pwm_inc = 0;
float current_r_pwm_inc = 0;

float current_l_pwm_inc_last = 0;
float current_r_pwm_inc_last = 0;

float current_l_pwm_duty_turn = 0;
float current_r_pwm_duty_turn = 0;
float current_l_pwm_duty = 0;
float current_r_pwm_duty = 0;

uint16 a,b;
float mot_inc=0;
float mot_inc_element=0;
float gyro_roll_cross=0;
float gyro_right_angle=0;

float gyro_roll_sign_rign=0;
float gyro_roll_sign_cross=0;
char gyro_roll_sign_angle=0;

char encoder_charge_sign=0;
float encoder_charge_element=0;
char encoder_cross_sign=0;
float encoder_cross_element=0;
char encoder_straight_sign = 0;
float encoder_straight_element = 0;

float ring_out_element=0;
char ring_out_sign=0;

float dir_loop_limit=450;
float dir_enlarge=1;
float speed_damping=0;
float speed_damping_enlarge=0.38f;

float err_H=1.5;
float err_X=1;
float err_HM=1;
float err_D=1;
float err_M=1;

float vbat_in=0;
float adc_vbat=0;
float adc_vbat_tar=12.8f;
float encoder_charge_element_vbat_tar=0.25f;
float charge_pwm_open_val=300;


int zhijiao_flag;

float speed[5]={258,258,258,0,0};//0闂佺儵鏅滈幐铏妤ｅ啯鏅?闂佺儵鏅濋ˉ鎰帮綖?



void TM1_Isr() interrupt 3
{
	
			static int timer_call_count = 0; 

//			angle_project(100);
		
		/*********************闂佽桨鑳舵晶妤€鐣垫笟鈧弻宀勫炊椤掍浇鍋?********************/
			
			acquire_sensor_data();

		/*********************闂傚倸瀚ч崑鎾绘煣閺勫繑銆冮柛妤€娴风划鏃堫敍濮橆剛鈧?********************/

            update_gyro_angle_accumulator(&gyro_roll,gyro_roll_sign_rign);
            update_gyro_angle_accumulator(&gyro_roll_cross,gyro_roll_sign_cross);
            update_gyro_angle_accumulator(&gyro_right_angle,gyro_roll_sign_angle);
		/*********************闁荤姳璀﹂崹鎶藉煝閼测晝鐭撴い鏍ㄨ壘閻?********************/

			//Encoder integration
			if (timer_call_count < 30) {
			timer_call_count++;
				l_speed_now=0;
				r_speed_now=0;
				mot_inc = 0.0f;
			} 
			update_encoder_speedup_value(&mot_inc_element,encoder_sign);
			
		
	  /*********************pid闁荤姳绶ょ槐鏇㈡偩?********************/

		if(MID <= 45) dir_enlarge = 1.0011;  // 闁诲繐绻愮换妤咁敋閵堝拋鍟呮い鏇炴缁€澶愭煕閳哄嫭顏犳い銏犳噺閺呭爼鎮欓崹顐ｅΡ
		else dir_enlarge = 0.99; // 闁荤姴娴傞崹鏉课熸繝鍐ㄧ窞鐟滄繄妲愬┑瀣珘闁逞屽墯瀵板嫯顦叉い銈呭暣閹?
		// 闂?闂佸憡鐟禍婊冿耿椤忓牆瀚夋い蹇撴川缁犲鏌涢敃鈧Λ娑㈢嵁閸℃稒鏅柛褍娼焤_flag == 0闂佹寧绋戦ˇ鏉款渻閸岀偞鏅悘鐐插⒔婢瑰矂鏌?MID 闂佽　鍋撻柣鐔告緲缂嶄線鏌涢埡鍕仩妞ゃ垹鎳橀弻鍛存偄瀹勯偊鍞?
//		if (cir_flag == 0) {
//				if (MID <= 45)
//						change_speed_Target_base(speed[1]);
//				else
//						change_speed_Target_base(speed[0]);
//		}

			
		if(MID <= 40) speed_damping_enlarge=0.3;
		else speed_damping_enlarge=0.1;
		
		
		speed_damping = fabs(gyro_data[0]) * speed_damping_enlarge;
		speed_damping = limit_function(speed_damping, 0, 150);

		

		if(Turn_PID.err > 0) {
				L_pid.Target = L_pid.Target_base - dir_enlarge * Turn_PID.out;
				R_pid.Target = R_pid.Target_base + dir_enlarge * Turn_PID.out - speed_damping;
		} else {
				L_pid.Target = L_pid.Target_base - dir_enlarge * Turn_PID.out - speed_damping;
				R_pid.Target = R_pid.Target_base + dir_enlarge * Turn_PID.out;
		}
		
		
        current_l_pwm_inc = current_l_pwm_inc + IncPID(l_speed_now, L_pid.Target, &L_pid);
		current_r_pwm_inc = current_r_pwm_inc + IncPID(r_speed_now, R_pid.Target, &R_pid);

        current_l_pwm_inc = current_l_pwm_inc_last * 0.2f + current_l_pwm_inc * 0.8f;
		current_r_pwm_inc = current_r_pwm_inc_last * 0.2f + current_r_pwm_inc * 0.8f;

        current_l_pwm_inc_last = current_l_pwm_inc;
		current_r_pwm_inc_last = current_r_pwm_inc;

        current_l_pwm_inc = limit_function(current_l_pwm_inc, -2000, 2000);
		current_r_pwm_inc = limit_function(current_r_pwm_inc, -2000, 2000);


	 
	  /*********************闂佸搫鐗冮崑鎾剁磽娴ｅ摜澧ｇ紒棰濆弮瀹曟瑧鑺遍崬姝柣鐘辩筏缁辨洟鎮?********************/



			
			
		//婵炴垶鎹佺亸娆愵殽閸ヮ剚鐒婚柣鏂垮槻椤斿﹪鏌ｅ搴＄仴缂侀鍙冨畷?
		#if 1
			
		current_l_pwm_duty=current_l_pwm_inc;  //current_l_pwm_inc
		current_r_pwm_duty=current_r_pwm_inc;  //current_r_pwm_inc
		
		#endif
			
		

        #if 0
            if (pwm_state_charge == 1) {
                current_l_pwm_duty=charge_pwm_open_val;
                current_r_pwm_duty=charge_pwm_open_val;

                encoder_charge_sign = 1;
                update_encoder_speedup_value(&encoder_charge_element,encoder_charge_sign);
                if(encoder_charge_element >= encoder_charge_element_vbat_tar){
                    pwm_state_charge = 0;
                    encoder_charge_sign = 0;
                    encoder_charge_element = 0.0f;
                }
            }
            else if(pwm_state_charge==0){
                current_l_pwm_duty=current_l_pwm_inc;
                current_r_pwm_duty=current_r_pwm_inc;

                current_l_pwm_inc=limit_function(current_l_pwm_inc,-2000,2000);
                current_r_pwm_inc=limit_function(current_r_pwm_inc,-2000,2000);
            }
        #endif
		
		
        current_l_pwm_duty=limit_function(current_l_pwm_duty,-5000,5000);
		current_r_pwm_duty=limit_function(current_r_pwm_duty,-5000,5000);


		
	  /*********************婵烇絽娲︾换鍐╂叏閵忥紕鈹嶆繝闈涙搐琚?********************/

				//protect
		key_scan_cycle_pwm_state();

		negative_pressure_auto_update_request(
			(cir_flag >= 2 && cir_flag <= 5) ? 1 : 0,
			NEG_PRESS_TRIGGER_CIRCLE);

		// Fourth-stage layer 1: run the auto-state machine globally,
		// but keep the real negative-pressure output locked off.
		negative_pressure_auto_tick();

		if(pwm_state==2){
		mot_inc=0;
		current_l_pwm_inc=0;
		current_r_pwm_inc=0;
		}	
		
	 /*********************闁哄鐗婇幐鎼佸吹?********************/
		out_pwm();
	
}




 





// ======================== 闂佸憡銇炵粈渚€鎮?========================

float left_value, right_value;   /* 闂佸憡纰嶉崹鍧楀闯閸濆嫀鐔兼晸閻樿櫕鐦?*/
int16 ad_diff;                 /* 濠碘槅鍨界换婵嬪汲閸楃儐鍟呮い鏂跨毞閸?*/
float ad_sum;                    /* 濠碘槅鍨界换婵嬪汲闁秴妞?*/
float deviation;                 /* 闂佸搫鐗冮崑鎾剁磽娴ｅ摜澧曠紓宥咃攻缁嬪鍩€椤掑嫬纾婚煫鍥ㄦ尰閳绘洟鏌ㄥ☉妯煎ⅱ婵炲懌鍔戝畷姘跺幢濡吋啸闂?*/
float A_CBH=1;
float B_CBH=1;
float C_CBH=1;


void TM4_Isr() interrupt 20
{
	
	
			
            L   = read_adc_average(ADC_P06,5,ADC_12BIT);
			LM	=	read_adc_average(ADC_P00,5,ADC_12BIT);
			RM	=	read_adc_average(ADC_P01,5,ADC_12BIT);//900
			R		=	read_adc_average(ADC_P05,5,ADC_12BIT);
			MID	=	read_adc_average(ADC_P10,5,ADC_12BIT);//1200

            L   = limit_float(L,0,4000);
			LM 	=	limit_float(LM,0,4000);
			RM 	=	limit_float(RM,0,4000);
			R  	=	limit_float(R,0,4000);
			MID	=	limit_float(MID,0,4000);
	
			//闂佸憡顭囬崰搴綖閹版澘鏋侀柣妤€鐗嗙粊?
			R_raw=R;
			L_raw=L;
			RM_raw=RM;
			LM_raw=LM;
            L   = normalize_float(L,0,4000);
			LM	= normalize_float(LM,0,4000);
			RM	= normalize_float(RM,0,4000);
			R 	= normalize_float(R,0,4000);
			MID =	normalize_float(MID,0,4000);

			
		/****************error***************/
	

			//濠?
			#if 1
			
			error = (
									err_H  * (L - R) + 
									err_X  * (2*LM - 2*RM)
							 ) / (
									err_HM * (L + R) + 
									err_D  * fabs(LM - RM)
								
							 );
			
			#endif 
			
			
			//閻庣懓澹婇崰妤呮儊椤曗偓瀹曨亞浠﹂懞銉㈡灆2
			#if 0
				A_CBH=Calculate_Weight_Mid(MID);
				B_CBH=1-A_CBH;
				error = (
            A_CBH * (1.5 * (L - R)) +
            B_CBH * (2.0 * (LM - RM))
         ) /
         (
            A_CBH * (L + R) +
            B_CBH * fabs(LM - RM)
         );
			#endif 
			
			
		/****************闂侀潻绠戝Λ娑㈢嵁?**************/
			if(cir_flag==2){
				ring_out_element++;
				if(ring_out_element >300){
					if(fabs(gyro_roll) < 240.0f)
					{
						cir_flag=0;

						gyro_roll_sign_rign=0;
						encoder_sign=0;
						mot_inc_element=0;
						gyro_roll=0;
						circle_enter_case = 0;
						change_speed_Target_base(speed[0]);
					}
				}
			}
			else{
			ring_out_element=0;
			}
			
			Circle_detect();
			Circle_cl();
			
		/*********************闂佸搫鍊婚幊鎾诲箖濠婂牊鍋?********************/
		Turn_PID.err=error;
		Turn_PID.out=Turn_PID.kp*Turn_PID.err+
								 Turn_PID.ki*Turn_PID.err*fabs(Turn_PID.err)*2+
								 Turn_PID.kd*(Turn_PID.err-Turn_PID.err_last)+
								 Turn_PID.kp1*gyro_data[0];
		Turn_PID.last=Turn_PID.out;
		Turn_PID.out=limit_function(Turn_PID.out,-dir_loop_limit,dir_loop_limit);

		Turn_PID.err_last=Turn_PID.err;


		TIM4_CLEAR_FLAG; //濠电偞鎸搁幊妯衡枍鎼淬垻鈻旀い鎾跺枑閻掍粙鏌″鍛悙缂?
}







