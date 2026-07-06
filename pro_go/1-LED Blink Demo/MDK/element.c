#include "headfile.h"
uint16 cir_angle_flag = 0;    // 标志位，用于判断和记录入环时的左右打角方向 (0: 初始, 1: 右侧入环, 2: 左侧入环, 3: 右侧出环, 4: 左侧出环)
uint8 cir_flag = 0;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)

uint8 encoder_sign=0;//编码器积分开关
uint8 yuansu_flag=1;     //各元素标志位,1为直道

uint16 temp_flag_speed=0;

float in_circle_LR =60;
float in_circle_MID=70;
float in_circle_LRMID=70;
float ring_error=1.11;
uint8 circle_enter_case = 0; // 0: 未进环, 1: LR+LRMID, 2: MID单独满足
float temp_flag=0;



float ring_inc_element12=0.21;
float ring_inc_element56=0.3;
float ring_inc_element67=0.35;



float ring_angle_23=15;
float ring_angle_34=120;
float ring_angle_45=280;

float temp_flag_tar=30;


//void Circle_detect(void)
//{

//			//float in_circle_LR =60;
//			//float in_circle_MID=70;
//			if ((L + R > in_circle_LR && MID > in_circle_LRMID) && cir_flag == 0)
//			{

//					cir_flag=1;
//					gyro_roll_sign_rign = 0; // 积分开关
//					encoder_sign = 1;        // 编码器
//					P52=1;
//							
//					if (LM_raw > RM_raw && cir_angle_flag != 2)
//					{
//							cir_angle_flag = 1;//左入环					
//					}
//					else if (LM_raw < RM_raw && cir_angle_flag != 1)
//					{
//							cir_angle_flag = 2;//右入环
//					}
//				
//					
//			}

//			//入环
//			//前瞻的投影打到切点上
//			//ring_inc_element12
//			else if (  (cir_flag == 1) && ( mot_inc_element>=ring_inc_element12 ) ) 
//			{
//					P52=0;
//					cir_flag=2;
//					gyro_roll_sign_rign=1;//陀螺仪开始积分
//					change_speed_Target_base(speed[2]);	
//					
//			}
//			//圆环内
//			//float ring_angle_23=15;
//			else if( (cir_flag==2) && cir_angle_flag &&( fabs(gyro_roll) > ring_angle_23))
//			{
//            cir_flag=3;
//            encoder_sign = 0;   
//						
//			}
//		
//			//准备出环
//			//float ring_angle_34=120;
//			else if((cir_flag==3)&& fabs(gyro_roll) > ring_angle_34)
//			{
//				cir_flag=4;
//			}
//			
//			//出环
//			//float ring_angle_45=270;
//			else if(cir_flag==4&&( fabs(gyro_roll) > ring_angle_45))
//			{
//			  cir_flag=5;
//				encoder_sign=1;

//			}
//			
//						
//			//出环成功
//			//float ring_inc_element56=0.3;
//			//float temp_flag_tar=40;
//			else if((cir_flag==5)&&(mot_inc_element>ring_inc_element56))
//			{
//				cir_flag=6;
//				P52=1;
//			}
//			
//			 else if((cir_flag==6)&&(mot_inc_element>ring_inc_element56+0.1))
//				{
//						temp_flag++;
//						
//						if (temp_flag > temp_flag_tar) // 计数足够，彻底归零退出
//						{
//								cir_flag = 0;

//								gyro_roll_sign_rign = 0;
//								encoder_sign = 0;
//								mot_inc_element = 0;
//								gyro_roll = 0;
//								circle_enter_case = 0;
//								temp_flag = 0;
//								P52=0;
//								change_speed_Target_base(speed[0]);
//						}
//				}
//			
//}




////........圆环处理...........//
//void Circle_cl(void)
//{

//		//环内转圈，写死
//	if(  (cir_flag==2)||(cir_flag==3) )
//	{
//		
//			if (cir_angle_flag == 1) {
//					error=ring_error;
//					
//			} else if(cir_angle_flag == 2) {
//					error=-ring_error;
//			}
//		
//	}

//	//出环
//	else if(  (cir_flag==4)||  (cir_flag==5)  ||  (cir_flag==6)  )
//	{
//		
//		  Turn_PID.err=error;
//	
//	}
//	
//}




void Circle_detect(void)
{

			//float in_circle_LR =60;
			//float in_circle_MID=70;
			if ((L + R > in_circle_LR && MID > in_circle_LRMID) && cir_flag == 0)
			{

					cir_flag=1;
					gyro_roll_sign_rign = 0; // 积分开关
					encoder_sign = 1;        // 编码器
					P52=1;
							
					if (LM_raw > RM_raw && cir_angle_flag != 2)
					{
							cir_angle_flag = 1;//左入环					
					}
					else if (LM_raw < RM_raw && cir_angle_flag != 1)
					{
							cir_angle_flag = 2;//右入环
					}
				
					
			}

			//入环
			//前瞻的投影打到切点上
			//ring_inc_element12
			else if (  (cir_flag == 1) && ( mot_inc_element>=ring_inc_element12 ) ) 
			{
					P52=0;
					cir_flag=2;
					gyro_roll_sign_rign=1;//陀螺仪开始积分
					change_speed_Target_base(speed[2]);	
					
			}
			//圆环内
			//float ring_angle_23=15;
			else if( (cir_flag==2) && cir_angle_flag &&( fabs(gyro_roll) > ring_angle_23))
			{
            cir_flag=3;
            encoder_sign = 0;   
						
			}
		
			//准备出环
			//float ring_angle_34=120;
			else if((cir_flag==3)&& fabs(gyro_roll) > ring_angle_34)
			{
				cir_flag=4;
			}
			
			//出环
			//float ring_angle_45=280;
			else if(cir_flag==4&&( fabs(gyro_roll) > ring_angle_45))
			{
			  cir_flag=5;
				encoder_sign=1;

			}
			
						
			//出环成功
			//float ring_inc_element56=0.3;
			//float temp_flag_tar=40;
			else if((cir_flag==5)&&(mot_inc_element>ring_inc_element56))
			{
				cir_flag=6;
				P52=1;
			}
			
			 else if((cir_flag==6)&&(mot_inc_element>ring_inc_element56))
				{
						temp_flag++;

						if (temp_flag > temp_flag_tar) // 计数足够，彻底归零退出
						{
								cir_flag = 0;

								gyro_roll_sign_rign = 0;
								encoder_sign = 0;
								mot_inc_element = 0;
								gyro_roll = 0;
								circle_enter_case = 0;
								temp_flag = 0;
								P52=0;
								change_speed_Target_base(speed[0]);
						}
				}
			
}




//........圆环处理...........//
void Circle_cl(void)
{

		//环内转圈，写死
	if(  (cir_flag==2)||(cir_flag==3) )
	{
		
			if (cir_angle_flag == 1) {
					error=ring_error;
					
			} else if(cir_angle_flag == 2) {
					error=-ring_error;
			}
		
	}
	else if(cir_flag==5){
	
			if (cir_angle_flag == 1) {
					error=0.55;
					
			} else if(cir_angle_flag == 2) {
					error=-0.55;
			}
	
	}
	
	
	//出环
	else if(  (cir_flag==4)  ||  (cir_flag==6)  )
	{
		
		  Turn_PID.err=error;
	
	}
	
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



// ======================== 直角判断 ========================

char right_angle_flag=0;
int right_angle_count=0;

void right_angle_judge()
{
	if( MID<45 && (L<=12&&R<=12) && (((LM-RM)>20&&RM<7)||((RM-LM)>20&&LM<7)) && right_angle_flag==0 )
	{
		right_angle_flag=1;
	}
	
	if( right_angle_flag==1 && (fabs(gyro_right_angle)>75) )
	{
		right_angle_flag=2;
	}

}




void right_angle_cl()
{
	if(right_angle_flag==1)
	{
		gyro_roll_sign_angle=1;//开启积分开关
		change_speed_Target_base(speed[1]);
		
	}
	
	else if(right_angle_flag==2)
	{
		right_angle_count++;
		if(right_angle_count>40)
		{
			right_angle_count=0;
			gyro_roll_sign_angle=0;
			right_angle_flag=0;
			change_speed_Target_base(speed[0]);
		}
		

	}
}
// ======================== 直道判断 ========================


float straight_err_threshold = 0.28f;       // 误差判断阈值（越小越严格）
float straight_integral_threshold = 0.15f;  // 积分量判断阈值（越大越稳妥）







