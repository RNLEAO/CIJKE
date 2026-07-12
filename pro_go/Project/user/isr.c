
#include "headfile.h"
#include "inductance4.h"



void DMA_UART1_IRQHandler(void) interrupt 4
{
    static vuint8 download_count = 0;
    uint8 rx_data;

    if (DMA_UR1R_STA & 0x01)
    {
        DMA_UR1R_STA &= ~0x01;
        rx_data = uart_rx_buff[UART_1][0];
        uart_rx_start_buff(UART_1);

        if (rx_data == 0x7F)
        {
            if (download_count++ > 20)
            {
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            download_count = 0;
        }

        if (uart1_irq_handler != NULL)
        {
            uart1_irq_handler(rx_data);
        }
    }

    if (DMA_UR1R_STA & 0x02)
    {
        DMA_UR1R_STA &= ~0x02;
        uart_rx_start_buff(UART_1);
    }
}

void DMA_UART2_IRQHandler(void) interrupt 8
{
    uint8 rx_data;

    if (DMA_UR2R_STA & 0x01)
    {
        DMA_UR2R_STA &= ~0x01;
        rx_data = uart_rx_buff[UART_2][0];
        uart_rx_start_buff(UART_2);
        if (uart2_irq_handler != NULL)
        {
            uart2_irq_handler(rx_data);
        }
    }

    if (DMA_UR2R_STA & 0x02)
    {
        DMA_UR2R_STA &= ~0x02;
        uart_rx_start_buff(UART_2);
    }
}

void DMA_UART3_IRQHandler(void) interrupt 17
{
    uint8 rx_data;

    if (DMA_UR3R_STA & 0x01)
    {
        DMA_UR3R_STA &= ~0x01;
        rx_data = uart_rx_buff[UART_3][0];
        uart_rx_start_buff(UART_3);
        if (uart3_irq_handler != NULL)
        {
            uart3_irq_handler(rx_data);
        }
    }

    if (DMA_UR3R_STA & 0x02)
    {
        DMA_UR3R_STA &= ~0x02;
        uart_rx_start_buff(UART_3);
    }
}

void DMA_UART4_IRQHandler(void) interrupt 18
{
    uint8 rx_data;

    if (DMA_UR4R_STA & 0x01)
    {
        DMA_UR4R_STA &= ~0x01;
        rx_data = uart_rx_buff[UART_4][0];
        uart_rx_start_buff(UART_4);

        if (RxLine < 200U)
        {
            DataBuff[RxLine++] = rx_data;
        }
        else
        {
            RxLine = 0;
            memset(DataBuff, 0, sizeof(DataBuff));
        }

        if (rx_data == 0x21)
        {
            USART_PID_Adjust();
            memset(DataBuff, 0, sizeof(DataBuff));
            RxLine = 0;
        }

        if (uart4_irq_handler != NULL)
        {
            uart4_irq_handler(rx_data);
        }
    }

    if (DMA_UR4R_STA & 0x02)
    {
        DMA_UR4R_STA &= ~0x02;
        uart_rx_start_buff(UART_4);
    }
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	// ASCII-cleaned legacy comment.
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  // ASCII-cleaned legacy comment.
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  // ASCII-cleaned legacy comment.
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  // ASCII-cleaned legacy comment.
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

/* Track, right-angle recovery, and roundabout base speeds. */
float speed[5] = {258, 180, 180, 0, 0};

static void reset_pid_runtime(_PID *pid)
{
    pid->err = 0.0f;
    pid->err_sum = 0.0f;
    pid->err_last = 0.0f;
    pid->d_err = 0.0f;
    pid->last = 0.0f;
    pid->out = 0.0f;
    pid->integral_out = 0.0f;
    pid->kp_out = 0.0f;
    pid->ki_out = 0.0f;
    pid->kd_out = 0.0f;
}

void reset_motion_pid_state(void)
{
    reset_pid_runtime(&L_pid);
    reset_pid_runtime(&R_pid);
    reset_pid_runtime(&Turn_PID);

    current_l_pwm_inc = 0.0f;
    current_r_pwm_inc = 0.0f;
    current_l_pwm_inc_last = 0.0f;
    current_r_pwm_inc_last = 0.0f;
    current_l_pwm_duty = 0.0f;
    current_r_pwm_duty = 0.0f;
}



void TM1_Isr() interrupt 3
{
			static int timer_call_count = 0; 
			float override_left_target;
			float override_right_target;

			TIM1_CLEAR_FLAG;

//			angle_project(100);
		
		/********************* Sensor acquisition and safety ********************/
			
			acquire_sensor_data();
			if (inductance4_calibration_active || !inductance4_calibration_valid)
			{
				pwm_state = 0;
			}

		/********************* Gyroscope integration ********************/

            update_gyro_angle_accumulator(&gyro_roll,gyro_roll_sign_rign);
            update_gyro_angle_accumulator(&gyro_roll_cross,gyro_roll_sign_cross);
            update_gyro_angle_accumulator(&gyro_right_angle,gyro_roll_sign_angle);
		/********************* Encoder integration ********************/

			//Encoder integration
			if (timer_call_count < 30) {
			timer_call_count++;
				l_speed_now=0;
				r_speed_now=0;
				mot_inc = 0.0f;
			} 
			update_encoder_speedup_value(&mot_inc_element,encoder_sign);
			
		
	  /********************* Differential speed PID ********************/

		dir_enlarge = 1.0f;
		speed_damping_enlarge = 0.20f;
		
		
		speed_damping = fabs(gyro_data[0]) * speed_damping_enlarge;
		speed_damping = limit_function(speed_damping, 0, 150);

		

		if (element4_get_speed_override(&override_left_target, &override_right_target))
		{
			L_pid.Target = override_left_target;
			R_pid.Target = override_right_target;
		}
		else if (Turn_PID.err > 0)
		{
			L_pid.Target = L_pid.Target_base - dir_enlarge * Turn_PID.out;
			R_pid.Target = R_pid.Target_base + dir_enlarge * Turn_PID.out - speed_damping;
		}
		else
		{
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


	 
	  /********************* PWM command generation ********************/



			
			
		// Normal driving output path.
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


		
	  /********************* Run-state protection ********************/

				//protect
		key_scan_cycle_pwm_state();

		if(pwm_state==2){
		mot_inc=0;
		current_l_pwm_inc=0;
		current_r_pwm_inc=0;
		}	
		
	 /********************* Motor PWM output ********************/
		out_pwm();
	
}




 





// Legacy diagnostic values retained for menu and telemetry compatibility.
float left_value, right_value;
int16 ad_diff;
float ad_sum;
float deviation;
float A_CBH=1;
float B_CBH=1;
float C_CBH=1;


void TM4_Isr() interrupt 20
{
		inductance4_update();
		error = inductance4_calculate_error();
		error = element4_process(error);

		Turn_PID.err=error;
		Turn_PID.out=Turn_PID.kp*Turn_PID.err+
								 Turn_PID.ki*Turn_PID.err*fabs(Turn_PID.err)*2+
								 Turn_PID.kd*(Turn_PID.err-Turn_PID.err_last)+
								 Turn_PID.kp1*gyro_data[0];
		Turn_PID.last=Turn_PID.out;
		Turn_PID.out=limit_function(Turn_PID.out,-dir_loop_limit,dir_loop_limit);

		Turn_PID.err_last=Turn_PID.err;


		TIM4_CLEAR_FLAG;
}







