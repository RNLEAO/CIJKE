
#include "headfile.h"


/**************************************************************************
函数名称：充电电压
if (wait_for_voltage(12.8f))
{
    pwm_state_charge = 1;
    pwm_state = 1;
}
**************************************************************************/
int wait_for_voltage(float target_voltage)
{
    uint16 vbat_in;
    float adc_vbat;

    while (1)
    {
        vbat_in = adc_once(ADC_P11, ADC_12BIT);
        adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;

        if (adc_vbat > target_voltage)
        {
            return 1;  /* 达标返回1 */
        }
    }

}


/**************************************************************************
函数名称：pwm输出
**************************************************************************/

void out_pwm(){
	
	
	
		if(pwm_state==0||pwm_state==2)
		{
			LEFT_MOTOR_DIR = 0;
			RIGHT_MOTOR_DIR = 0;
			pwm_duty(LEFT_MOTOR_PWM, 0);
			pwm_duty(RIGHT_MOTOR_PWM, 0);
			current_l_pwm_duty = 0;
			current_r_pwm_duty = 0;
			current_l_pwm_inc=0;
			current_r_pwm_inc=0;
		}

		
		else if(pwm_state==1)
		{
			/* --- 左电机控制 --- */
			if(current_l_pwm_duty > 0) // 正占空比，期望电机正转
			{
				LEFT_MOTOR_DIR = 1;
				pwm_duty(LEFT_MOTOR_PWM, (uint32)current_l_pwm_duty);
			}
			else if(current_l_pwm_duty < 0) // 负占空比，期望电机反转
			{
				LEFT_MOTOR_DIR = 0;
				pwm_duty(LEFT_MOTOR_PWM, (uint32)(-current_l_pwm_duty));
			}
			else // 占空比为0，电机停止
			{
				LEFT_MOTOR_DIR = 0;
				pwm_duty(LEFT_MOTOR_PWM, 0);
			}

			/******************************************************
			 * 右电机控制 (使用原始逻辑)
			 ******************************************************/
			if(current_r_pwm_duty > 0) // 正占空比 -> 右轮正转
			{
				RIGHT_MOTOR_DIR = 1;
				pwm_duty(RIGHT_MOTOR_PWM, (uint32)current_r_pwm_duty);
			}
			else if(current_r_pwm_duty < 0) // 负占空比 -> 右轮反转
			{
				RIGHT_MOTOR_DIR = 0;
				pwm_duty(RIGHT_MOTOR_PWM, (uint32)(-current_r_pwm_duty));
			}
			else // 占空比为0 -> 右轮停止
			{
				RIGHT_MOTOR_DIR = 0;
				pwm_duty(RIGHT_MOTOR_PWM, 0);
			}
		}

}


/**************************************************************************
函数名称：数据采集
**************************************************************************/

void acquire_sensor_data(void)
{
    imu660ra_get_gyro();
    gyro_data[0] = imu660ra_gyro_transition(imu660ra_gyro_x);

    l_encoder = (float)ctimer_count_read(MOTOR1_ENCODER);  
    r_encoder = (float)ctimer_count_read(MOTOR2_ENCODER);  

    ctimer_count_clean(MOTOR1_ENCODER);  
    ctimer_count_clean(MOTOR2_ENCODER); 

    l_speed_now = l_speed_now * 0.2f + l_encoder * 0.8f;
    r_speed_now = r_speed_now * 0.2f + r_encoder * 0.8f;

    // Motor direction adjustment
    if (MOTOR1_DIR == 0) l_speed_now = -l_speed_now;
    if (MOTOR2_DIR == 1) r_speed_now = -r_speed_now;

}

//旋转保护
void angle_project(int threshold)
{
	static int time=0;
	
	if(pwm_state!=1) 
	{
		time=0;
	}
	else
	{
		time++;
		if(time%threshold==0)//0.5s
		{
			if(gyro_roll>300||gyro_roll<-300)
			{
				pwm_state=2;
			}		
			gyro_roll=0;//0.5s清零一次
			time=0;
		}
	}
}


/**
 * @brief 根据输入值 M 计算横电感的权重系数 k
 * 
 * 当中电感值 M 越大，表示对横电感更信任。
 * 
 * @param M 横向电感值
 * @return double 权重系数 k，范围为 [0.2, 1.0]
 */
float Calculate_Weight_Mid(uint16 M)
{
    float k;

    if (M > 45)
    {
        k = 1.0;
    }
    else if (M >= 25)
    {
        // 从 0.2 线性过渡到 1.0，对应 M 从 25 到 45
        k = 0.2 + (M - 25) * (0.8 / 20.0);
    }
    else
    {
        k = 0.2;
    }

    return k;
}
/**
 * @brief 计算垂直方向的归一化差值
 * 
 * 该函数通过计算归一化处理偏差数组 ADC_FA 中索引为 3 和 1 的元素差值的绝对值，
 * 并将其归一化到 0.0 到 1.0 的范围，最终返回一个介于 0.1 到 1.0 之间的 double 类型值。
 * 
 * @return double 计算得到的垂直方向归一化差值，范围在 0.1 到 1.0 之间
 */
float Calculate_Vertical()
{
    
    float k = 0;
    k = fabs(LM - RM)/100.0;
    // 确保 k 的值在 0.1 到 1.0 之间，若超出范围则取边界值
		if(LM>=0 && RM>=0){
				if (k > 1.0)
				return 1.0;
				else if (k < 0.1)
				return 0.1;
		    else
				return k;
		}
		else{
		return 1;
		}
	



}




