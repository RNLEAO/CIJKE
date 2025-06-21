
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
        //�����Զ�����
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

//UART2�ж�
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
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
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
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG){
			
        UART4_CLEAR_TX_FLAG;
				busy[4] = 0;
	  }
	
    if(UART4_GET_RX_FLAG){
			
        UART4_CLEAR_RX_FLAG;
				 
			//ÿ���յ�һ�����ݣ����ݳ��ȼ�1��������
			RxLine++;                            
			//��ÿ�ν��յ������ݱ��浽��������
			DataBuff[RxLine-1]=S4BUF;                 
			
			if(S4BUF==0x21) // //���ս�����־λ
			{
			USART_PID_Adjust();//���ݽ����Ͳ�����ֵ����
			memset(DataBuff,0,sizeof(DataBuff));  //��ջ�������??
			RxLine=0;  //��ս��ճ���??
			}
			
		
		//�������ݼĴ���Ϊ��S4BUF;
		if(wireless_module_uart_handler != NULL){
			// �ú���Ϊ����ָ��
			// �ٳ�ʼ������ģ���ʱ�����øú������?
			wireless_module_uart_handler(S4BUF);
			
			
		}
	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//����LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ��?
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ��?
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ��?
}

void TM0_Isr() interrupt 1
{

}





float error=0;




float Roll_x=0;
//临时变量
float err_t=0.000036035;
#define PWM_DUTY_MIN -8000 
//速度环
float current_l_pwm_inc = 0;
float current_r_pwm_inc = 0;

float current_l_pwm_inc_last = 0;
float current_r_pwm_inc_last = 0;

float l_pwm_inc_raw= 0;
float r_pwm_inc_raw= 0;
//方向环
float current_l_pwm_duty_turn = 0;
float current_r_pwm_duty_turn = 0;
//角度环
float current_l_pwm_duty_ang_turn = 0;
float current_r_pwm_duty_ang_turn = 0;
//最终输出pwm
float current_l_pwm_duty = 0;
float current_r_pwm_duty = 0;

uint16 a,b;
float adcdata_aver[5]={0};


float mot_inc=0;
float mot_inc_element=0;
float gyro_roll_cross=0;


//圆环角度积分
float gyro_roll_sign_rign=0;
float gyro_roll_sign_cross=0;
//充电编码器积分开关
char encoder_charge_sign=0;
float encoder_charge_element=0;
//十字编码器积分开关
char encoder_cross_sign=0;
float encoder_cross_element=0;
//加速编码器积分开关
char encoder_speedup_sign=0;
float encoder_speedup_element=0;
//直道 积分开关
//直道 积分变量
char encoder_straight_sign = 0;             
float encoder_straight_element = 0;         



//圆环退出
float ring_out_element=0;
char ring_out_sign=0;

//出环时间开关
char time_speedup_sign=0;
float time_speedup_element=0;


/*****************堵转检测*****************//*****************堵转检测*****************//*****************堵转检测*****************/
/* 在全局变量区添加 */
#define STALL_SPEED_THRESHOLD    5      /* 速度阈值（编码器计数值/周期） */
#define STALL_PWM_THRESHOLD      200     /* PWM占空比阈值 */
#define STALL_DURATION_MS        2100     /* 持续判定时间（毫秒） */
#define STALL_CHECK_CYCLES       (STALL_DURATION_MS / 10) /* 10ms中断周期 */

/*****************向量和公式左右差异补偿*****************//*****************向量和公式左右差异补偿*****************//*****************向量和公式左右差异补偿*****************/

float dir_loop_limit=190; 
float dir_enlarge=1;  
float speed_damping=0; // 速度抑制系数



float err_H=1;
float err_X=1;
float err_HM=1;
float err_D=1;
float err_M=1;


/* 结构体声明 */
typedef struct {
    unsigned int counter;
    unsigned char detected;
} StallDetection;


/* 全局变量声明 */
StallDetection left_stall = {0, 0};
StallDetection right_stall = {0, 0};



void TM1_Isr() interrupt 3
{
	
			static int timer_call_count = 0; 
			static float l_speed_abs, r_speed_abs;
			static float l_pwm_abs, r_pwm_abs;
			static unsigned int stall_event_counter = 0;  /* 堵转事件计数器 */
			static bit time_speedup_brake_started = 0;  // 是否开始刹车流程
			static unsigned int run_mode2_counter = 0;
			static bit run_mode2_timing_started = 0;

		
		/*********************数据采集*********************//*********************数据采集*********************//*********************数据采集*********************/	
			acquire_sensor_data();

		/*********************陀螺仪积分*********************//*********************陀螺仪积分*********************//*********************陀螺仪积分*********************/	

			update_gyro_angle_accumulator(&gyro_roll,gyro_roll_sign_rign);
			update_gyro_angle_accumulator(&gyro_roll_cross,gyro_roll_sign_cross);
	
		/*********************路程积分*********************//*********************路程积分*********************//*********************路程积分*********************/	

			//Encoder integration
			if (timer_call_count < 30) {
			timer_call_count++;
				l_speed_now=0;
				r_speed_now=0;
				mot_inc = 0.0f;
			} 
				else if(pwm_state==1){
				mot_inc+= (fabs(l_speed_now) + fabs(r_speed_now)) * 0.5f * 0.00003895f;					
					
			}
				//圆环积分			
			update_encoder_speedup_value(&mot_inc_element,encoder_sign);	
			
				//十字积分
			update_encoder_speedup_value(&encoder_cross_element,encoder_cross_sign);	

			//充电积分
			if(encoder_charge_sign==1){	
			
			encoder_charge_element+=(fabs(l_speed_now)+fabs(r_speed_now))*0.5f*0.00003895;}
			
			//加速积分
			update_encoder_speedup_value(&encoder_speedup_element,encoder_speedup_sign);	


			//长直道积分
			update_encoder_speedup_value(&encoder_straight_element,encoder_straight_sign);

				
			
			
			#if 0
			
		 // ======================== 角度环 PID 调用逻辑 ========================
			target_angle=0;			
			current_angle+=gyro_data[0] * 0.005;
			ang_pid.err=target_angle-current_angle;
			
			ang_pid.out=ang_pid.kp*ang_pid.err+
									 ang_pid.ki*ang_pid.err*fabs(ang_pid.err)*2+
									 ang_pid.kd*(ang_pid.err-ang_pid.err_last)+
									 ang_pid.kp1*gyro_data[0];
			ang_pid.last=ang_pid.out;
			//限幅
			ang_pid.err_last=ang_pid.err;


			current_l_pwm_duty_ang_turn=ang_pid.out;
			current_r_pwm_duty_ang_turn=ang_pid.out;

			

		 current_l_pwm_duty_ang_turn=limit_function(current_l_pwm_duty_ang_turn,-350,350);
		 current_r_pwm_duty_ang_turn=limit_function(current_r_pwm_duty_ang_turn,-350,350);

		 #endif





		  #if 0
			
			
			
			// ======================== 角速度环 PID 调用逻辑 ========================
//			Gyro_PID.Target = 0;                               // 希望角速度为 0，保持直线
//			Gyro_PID.now = gyro_data[0];                      // 读取实际角速度（单位：°/s）

//			Gyro_PID.out = GyroPositionPID(Gyro_PID.now, Gyro_PID.Target, &Gyro_PID); // 位置式PID
//			Gyro_PID.out = limit_function(Gyro_PID.out, -800, 800);                    // 限幅
		
			
			//Turn_PID.out
			
			Gyro_PID.Target = -1*Turn_PID.out;
//			Gyro_PID.Target = 0;
			Gyro_PID.now = gyro_data[0];
			Gyro_PID.out = Gyro_PID.out + IncPID(Gyro_PID.now, Gyro_PID.Target, &Gyro_PID); // 注意是“+=”
			Gyro_PID.out = limit_function(Gyro_PID.out, -800, 800);                    // 限幅
			#endif
		
			
			
			straight_judge(straight_err_threshold,straight_integral_threshold);
			
	// ======================== 速度环 PID 调用逻辑 ========================



		#if 1
		

		if (fabs(Turn_PID.err) < 1.4f) {
				dir_enlarge = 0.9f; // 小误差，基础增益
		}  else {
				dir_enlarge = 1.38f; // 误差大，最大增益
		}

		
		// 2. 根据陀螺仪角速度进行速度抑制，防止甩尾
		speed_damping = fabs(gyro_data[0]) * 0.1f; 
		speed_damping = limit_function(speed_damping, 0, 220);
		
		
		if(Turn_PID.err > 0) { // 右转
				 // 右轮是内轮，减速更多；左轮是外轮
				L_pid.Target = L_pid.Target_base - dir_enlarge * Turn_PID.out;
				R_pid.Target = R_pid.Target_base + dir_enlarge * Turn_PID.out - speed_damping;
		} else { // 左转
				// 左轮是内轮，减速更多；右轮是外轮
				L_pid.Target = L_pid.Target_base - dir_enlarge * Turn_PID.out - speed_damping;
				R_pid.Target = R_pid.Target_base + dir_enlarge * Turn_PID.out;
		}
		
		
		// 保持原有增量叠加结构
		current_l_pwm_inc = current_l_pwm_inc + IncPID(l_speed_now, L_pid.Target, &L_pid);
		current_r_pwm_inc = current_r_pwm_inc + IncPID(r_speed_now, R_pid.Target, &R_pid);

		// 对 PWM 增量值进行一阶滤波（后处理）
		current_l_pwm_inc = current_l_pwm_inc_last * 0.2f + current_l_pwm_inc * 0.8f;
		current_r_pwm_inc = current_r_pwm_inc_last * 0.2f + current_r_pwm_inc * 0.8f;

		// 存储本次滤波后的值用于下一次滤波
		current_l_pwm_inc_last = current_l_pwm_inc;
		current_r_pwm_inc_last = current_r_pwm_inc;

		// 限幅
		current_l_pwm_inc = limit_function(current_l_pwm_inc, -4000, 4000);
		current_r_pwm_inc = limit_function(current_r_pwm_inc, -4000, 4000);

		
		#endif
		
		
		

	 
	 
	 /*********************最终输出pwm计算*********************//*********************最终输出pwm计算*********************//*********************最终输出pwm计算*********************/	



		
		//充电
		#if 0
			if (pwm_state_charge == 1) {
				
					current_l_pwm_duty=current_l_pwm_inc+current_l_pwm_duty_ang_turn;
					current_r_pwm_duty=current_r_pwm_inc-current_r_pwm_duty_ang_turn;
					encoder_charge_sign = 1;
					if(encoder_charge_element >= 0.5){
					pwm_state_charge = 0;     // 切换状态为转向中
          encoder_charge_sign = 2;				
					}
			}
			else if(pwm_state_charge==0){

				current_l_pwm_duty=current_l_pwm_inc;
				current_r_pwm_duty=current_r_pwm_inc;
					
				//限幅
				 current_l_pwm_inc=limit_function(current_l_pwm_inc,-1600,1600);
				 current_r_pwm_inc=limit_function(current_r_pwm_inc,-1600,1600);
			}


		#endif
		
		
			
			
			
			
			// ======================== 串级速度环输出 ========================
		#if 1
			

		current_l_pwm_duty=current_l_pwm_inc;
		current_r_pwm_duty=current_r_pwm_inc;
			
		//限幅
		 current_l_pwm_inc=limit_function(current_l_pwm_inc,-4000,4000);
		 current_r_pwm_inc=limit_function(current_r_pwm_inc,-4000,4000);

		#endif
			
			
	  // ======================== 单方向环输出 ========================
		#if 0
			
		current_l_pwm_duty=-Turn_PID.out;
		current_r_pwm_duty=+Turn_PID.out;
			
		#endif
		
		
		
		#if 0
		
		
		
		current_l_pwm_duty= + Gyro_PID.out;
		current_r_pwm_duty= - Gyro_PID.out;
			
		#endif




		
		
		//最终输出限幅
		current_l_pwm_duty=limit_function(current_l_pwm_duty,-4000,4000);
		current_r_pwm_duty=limit_function(current_r_pwm_duty,-4000,4000);




		
	  /*********************保护保护*********************//*********************保护保护*********************//*********************保护保护*********************/	
		//protect
		key_scan_cycle_pwm_state();

		if(pwm_state==2){
		mot_inc=0;
		current_l_pwm_inc=0;
		current_r_pwm_inc=0;
		}
		
		#if 0
		if((L+R+MID<=0)&&(LM+RM<=0)) pwm_state=0;
		#endif
		
		
	 /*********************输出*********************//*********************输出*********************//*********************输出*********************/	
		#if 1
		out_pwm();
		#endif
	

	
}




 





// ======================== 十字 ========================

#define SQRT1_2 0.7071f  /* √1/2 ≈ 0.7071 */

float LM_x;  
float RM_x;  
float LM_y;  
float RM_y;  







void TM4_Isr() interrupt 20
{
	
	
	
			
			//电感
			L  	=	read_adc_average(ADC_P06,5,ADC_12BIT);//1000
			LM	=	read_adc_average(ADC_P00,5,ADC_12BIT);
			RM	=	read_adc_average(ADC_P01,5,ADC_12BIT);//900
			R		=	read_adc_average(ADC_P05,5,ADC_12BIT);
			MID	=	read_adc_average(ADC_P10,5,ADC_12BIT);//1200

			//限幅
			L  	=	limit_float(L,0,4000);
			LM 	=	limit_float(LM,0,4000);
			RM 	=	limit_float(RM,0,4000);
			R  	=	limit_float(R,0,4000);
			MID	=	limit_float(MID,0,4000);
			//归一化
			L 	= normalize_float(L,0,4000);
			LM	= normalize_float(LM,0,4000);
			RM	= normalize_float(RM,0,4000);
			R 	= normalize_float(R,0,4000);
			MID =	normalize_float(MID,0,4000);

			

			R_raw=R;
			L_raw=L;
			
			
			
		/****************error***************//****************error***************//****************error***************/
	



			error = (
									err_H  * (L - R) + 
									err_X  * (2.5*LM - 2.5*RM)
							 ) / (
									err_HM * (L + R) + 
									err_D  * fabs(LM - RM) + 
									err_M  * MID
							 );



		/*********************方向环*********************//*********************方向环*********************//*********************方向环*********************/	
		

		Turn_PID.err=error;
		//1 串级
		Turn_PID.out=Turn_PID.kp*Turn_PID.err+
								 Turn_PID.ki*Turn_PID.err*fabs(Turn_PID.err)*2+
								 Turn_PID.kd*(Turn_PID.err-Turn_PID.err_last)+
								 Turn_PID.kp1*gyro_data[0];
		Turn_PID.last=Turn_PID.out;
		//限幅
		Turn_PID.out=limit_function(Turn_PID.out,-dir_loop_limit,dir_loop_limit);
		
		Turn_PID.err_last=Turn_PID.err;

		buzzer_control_with_enable(Turn_PID.out,dir_loop_limit,pwm_state);










	
		  TIM4_CLEAR_FLAG; //清除中断标志

}







