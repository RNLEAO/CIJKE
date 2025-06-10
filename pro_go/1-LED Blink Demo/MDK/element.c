#include "headfile.h"
uint16 cir_angle_flag = 0;    // 标志位，用于判断和记录入环时的左右打角方向 (0: 初始, 1: 右侧入环, 2: 左侧入环, 3: 右侧出环, 4: 左侧出环)
uint16 flag_incir = 0;        // 标志位，表示小车是否已经进入圆环内 (0: 否, 1: 是)
uint8 cir_flag = 0;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)

uint8 encoder_sign=0;//编码器积分开关
uint8 yuansu_flag=1;     //各元素标志位
uint16 temp_flag=0;
uint16 temp_flag_speed=0;
uint8 speed_strategy_active =0;


int temp_t;


void Circle_detect(void)
{
    static float mot_inc_last_at_entry = 0.0f;
    static uint8 in_circle_integration = 0;

		//检测到圆环
		//入环条件
    if ((yuansu_flag==1)&&(MID >= 95)||(((L+R)>150)&&MID>80) && cir_flag == 0 )
    {	
				//清除
		

				gyro_roll_sign_rign=0;//积分开关
				gyro_roll=0;
				mot_inc_element=0;
			
			
				cir_flag = 1;//全局标记圆环	
				encoder_sign=1;//编码器
			
				yuansu_flag=2;
			


    }
		//入环
    else if ((cir_flag == 1)) 
    {
				cir_flag=2;
				gyro_roll_sign_rign=1;//陀螺仪开始积分
			
				if (R_raw > L_raw && cir_angle_flag != 2)
        {
            cir_angle_flag = 1;//左入环					
				
        }
        else if (L_raw >R_raw && cir_angle_flag != 1)
        {
            cir_angle_flag = 2;//右入环

        }


		}
			//圆环内
			else if( (cir_flag==2) && (L>80&&R>80&&LM>80&&RM>80&&MID>80) && cir_angle_flag && (mot_inc_element>=0.4) )
			{
            cir_flag=3;
            encoder_sign = 0;                        // 编码器可能用于不同目的或关闭
						change_speed_Target_base(280);
						//！！！！！
			

			}
		
			//准备出环
			else if((cir_flag==3)&& fabs(gyro_roll) > 240.0f)
			{
				cir_flag=4;

								
			}
				
			//出环
			else if(cir_flag==4&&( fabs(gyro_roll) > 280.0f))
			{
				cir_flag=5;
				encoder_sign=1;
			}
			//出环成功
			else if((cir_flag==5)&&( fabs(gyro_roll) > 250.0f)&&(mot_inc_element>0.3))
			{
				cir_flag=6;
			}
			
			else if((cir_flag==6)&&(mot_inc_element>1.2))
			{
				
				if(temp_flag<100)
				{
						change_speed_Target_base(240);
					//！！！！！
					
				}
				else if (temp_flag>= 100 && temp_flag<415){
//						change_speed_Target_base(220);  // 切换状态时设置目标速度
					  //！！！！！
//					
						L_pid.Target_base=calculate_dynamic_target_speed_quadratic(MID);
						R_pid.Target_base=calculate_dynamic_target_speed_quadratic(MID);
					
				}
				else if(temp_flag>=415)
				{
					
					temp_flag=0;
					cir_flag=0;
				
					yuansu_flag=1; 
					
					gyro_roll_sign_rign=0;
					encoder_sign=0;
					mot_inc_element=0;
					gyro_roll=0;
					time_speedup_sign=1;
					change_speed_Target_base(220);
					//！！！！！
				}
				
			}
			


}









//........圆环处理...........//
void cricle_cl(void)
{

		//环内转圈，非写死
	if(cir_flag==3)
	{
		
			if (cir_angle_flag == 1) {
					Turn_PID.err=-0.98;
			} else if(cir_angle_flag == 2) {
					Turn_PID.err=0.98;
			}
		
	}
	//出环
	if((cir_flag==6)||(cir_flag==5))
	{
		
		Turn_PID.err=error;
	
	}
	
}


int cross_flag=0;
void cross_judge(void)
{
	switch(cross_flag)
	{
		case 0:
			if(LM>70&&RM>70)
			{
				P52=1;
				cross_flag=1;
			}
			break;
		
		case 1:
			if(LM<70&&RM<70)
			{
				P52=0;
				cross_flag=2;
				gyro_roll_sign_cross=1;
			}
			break;
		case 2://在十字里
			if(LM>70&&RM>70)
			{
				P52=1;
				cross_flag=3;
				gyro_roll_sign_cross=0;
			}
			break;
		case 3:
			if(LM<70&&RM<70)
			{
				P52=0;
				cross_flag=0;
			}
			break;
		
	}
}

void cross_proc(void)
{
	static char cross_handle_state = 0;
	if(cross_flag==2)
	{
		
//		if(gyro_roll_cross<=240) Turn_PID.err=2;
		if(gyro_roll_cross<-140&&gyro_roll_cross>-175) 
		{
			error=1;
		}
		

			
//		switch(cross_handle_state)
//		{
//			case 0:
//				encoder_cross_sign=1;
//				if(encoder_cross_element>0.5)
//				{
//					cross_handle_state=1;
//					encoder_cross_sign=0;
//				}
//				break;
//			
//			case 1:
//				break;
//			
//			case 2:
//				break;
//			
//			case 3:
//				break;
//		}
		

	}
	
}





		/*********************加速*********************//*********************加速*********************//*********************加速*********************/	
void speed_up(){
	











}
	

/*********************pid切换*********************//*********************pid切换*********************//*********************pid切换*********************/	

float run_mode=1;

char set_control_mode(char mode){
		if(mode==1){
		L_pid.kp=10.9;
		L_pid.ki=0.65;
		L_pid.kd=0;
		 
		R_pid.kp=10.9;
		R_pid.ki=0.65;
		R_pid.kd=0;
		
		//	error = (left_mag - right_mag) / (left_mag + right_mag);
		Turn_PID.kp=144;
		Turn_PID.ki=76.1;
		Turn_PID.kd=56;
		Turn_PID.kp1=0.54;
	

	  return 1;
		}
		
		else if(mode==2){
		L_pid.kp=4.9;
		L_pid.ki=0.2;
		L_pid.kd=0;
		 
		R_pid.kp=4.9;
		R_pid.ki=0.2;
		R_pid.kd=0;

		Turn_PID.kp=144;
		Turn_PID.ki=66.1;
		Turn_PID.kd=66;
		Turn_PID.kp1=0.34;
			
		dir_loop_limit=46; 
		dir_enlarge=15.5;  
		
			
		
			return 2;
		}
		
		return 0;


}

/*********************起跑检测*********************//*********************起跑检测*********************//*********************起跑检测*********************/	


volatile unsigned char hall_triggered = 0;     // 是否正在锁定，0 表示可以检测
volatile unsigned int hall_timer_count = 0;    // 5ms计时器计数，用于延时1s
volatile unsigned int hall_count = 0;          // 霍尔开关触发次数



void check_hall_sensor(void)
{
    if (hall_triggered == 0)
    {
        // 假设 P36 == 0 表示霍尔开关闭合（被磁铁触发）
        if (P36 == 0)
        {
            hall_count++;               // 记录一次触发
            hall_triggered = 1;         // 开始1秒冷却
            hall_timer_count = 0;       // 重置计数器
					
        }
    }
    else
    {
        hall_timer_count++;             // 每次中断+1，5ms一次

        if (hall_timer_count >= 200)    // 1秒 = 200 * 5ms
        {
            hall_triggered = 0;         // 解锁，允许再次检测
            hall_timer_count = 0;       // 重置
        }
				
    }


		
}




