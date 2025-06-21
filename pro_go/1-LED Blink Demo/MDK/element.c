#include "headfile.h"
uint16 cir_angle_flag = 0;    // 标志位，用于判断和记录入环时的左右打角方向 (0: 初始, 1: 右侧入环, 2: 左侧入环, 3: 右侧出环, 4: 左侧出环)
uint16 flag_incir = 0;        // 标志位，表示小车是否已经进入圆环内 (0: 否, 1: 是)
uint8 cir_flag = 0;          // 状态标志位，用于控制圆环检测和循迹的不同阶段 (0: 初始, 1: 准备入环, 2: 准备打角, 3: 打角结束, 4: 准备出环, 5: 准备离开圆环)

uint8 encoder_sign=0;//编码器积分开关
uint8 yuansu_flag=1;     //各元素标志位,1为直道
uint16 temp_flag=0;
uint16 temp_flag_speed=0;
uint8 speed_strategy_active =0;
uint8 speed_ctrl_flag = 0;     // 控速状态设置为“直道加速”


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



// ======================== 直道判断 ========================
float straight_err_threshold = 0.28f;       // 误差判断阈值（越小越严格）
float straight_integral_threshold = 0.15f;  // 积分量判断阈值（越大越稳妥）
char straight_flag=0;
void straight_judge(float straight_err_threshold, float straight_integral_threshold)
{
    // 未进入直道，且处于元素状态，才允许判断是否进入直道
    if (straight_flag == 0 && yuansu_flag == 1)
    {
        if (fabs(Turn_PID.err) < straight_err_threshold)
        {
            encoder_straight_sign = 1;
        }
        else
        {
            encoder_straight_sign = 0;
            encoder_straight_element = 0; // 偏离清零
        }

        if (encoder_straight_element > straight_integral_threshold)
        {
            straight_flag = 1;
            speed_ctrl_flag = 8;
        }
    }

    // ? 不再满足误差条件 或 元素状态退出 —— 则退出直道状态
    if (straight_flag == 1)
    {
        if (fabs(Turn_PID.err) > straight_err_threshold || yuansu_flag != 1)
        {
            straight_flag = 0;
            speed_ctrl_flag = 1;
            encoder_straight_element = 0;
            encoder_straight_sign = 0;
        }
    }
}


