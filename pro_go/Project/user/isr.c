
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
    if (DMA_UR4R_STA & 0x01)
    {
        DMA_UR4R_STA &= ~0x01;
        uart_rx_start_buff(UART_4);

        if (uart4_irq_handler != NULL)
        {
            uart4_irq_handler(uart_rx_buff[UART_4][0]);
        }
    }

    if (DMA_UR4R_STA & 0x02)
    {
        DMA_UR4R_STA &= ~0x02;
        uart_rx_start_buff(UART_4);
    }
}

#define LED P52
#define DIAGNOSTIC_STREAM_TICKS 1000U

extern volatile uint8 diagnostic_stream_enabled;
extern volatile uint8 diagnostic_stream_due;
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
static vuint8 line_wait_active = 0U;
static vuint8 speed_direction_guard_mask = 0U;

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
float track_turn_ratio=0.0f;
float track_line_speed_scale=0.0f;

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

static void reset_speed_pid_state(void)
{
    reset_pid_runtime(&L_pid);
    reset_pid_runtime(&R_pid);

    current_l_pwm_inc = 0.0f;
    current_r_pwm_inc = 0.0f;
    current_l_pwm_inc_last = 0.0f;
    current_r_pwm_inc_last = 0.0f;
    current_l_pwm_duty = 0.0f;
    current_r_pwm_duty = 0.0f;
}

void reset_motion_pid_state(void)
{
    reset_speed_pid_state();
    reset_pid_runtime(&Turn_PID);
}

static uint8 line_guard_required(void)
{
    return element4_state == ELEMENT4_TRACK
        || element4_state == ELEMENT4_RIGHT_ANGLE_CONFIRM
        || element4_state == ELEMENT4_RING_CONFIRM;
}

static float enforce_target_direction(float pwm_value, float target, uint8 guard_bit)
{
    if ((target > 0.0f && pwm_value < 0.0f)
        || (target < 0.0f && pwm_value > 0.0f)
        || target == 0.0f)
    {
        if (pwm_value != 0.0f)
        {
            speed_direction_guard_mask |= guard_bit;
        }
        return 0.0f;
    }

    return pwm_value;
}

unsigned char motion_line_wait_is_active(void)
{
    return line_wait_active;
}

unsigned char motion_direction_guard_mask(void)
{
    return speed_direction_guard_mask;
}



void TM1_Isr() interrupt 3
{
			static uint16 diagnostic_stream_ticks = 0U;
			float override_left_target;
			float override_right_target;
			float track_base_target;
			float left_pid_delta;
			float right_pid_delta;

			TIM1_CLEAR_FLAG;
			if (diagnostic_stream_enabled)
			{
				if (diagnostic_stream_ticks < DIAGNOSTIC_STREAM_TICKS)
				{
					diagnostic_stream_ticks++;
				}
				if (diagnostic_stream_ticks >= DIAGNOSTIC_STREAM_TICKS)
				{
					diagnostic_stream_ticks = 0U;
					diagnostic_stream_due = 1U;
				}
			}
			else
			{
				diagnostic_stream_ticks = 0U;
				diagnostic_stream_due = 0U;
			}

//			angle_project(100);
		
		/********************* Sensor acquisition and safety ********************/
			
			acquire_sensor_data();
			if (motion_runtime_encoder_test_is_active())
			{
				motion_runtime_encoder_test_tick();
				return;
			}
			if (motion_runtime_motor_test_is_active())
			{
				motion_runtime_motor_test_tick();
				return;
			}
			if (pwm_state == 1U && !motion_runtime_can_run())
			{
				motion_runtime_trigger_protection(
					g_imu_runtime_state == IMU_RUNTIME_READY
						? MOTION_PROTECT_RUN_LOCKED
						: MOTION_PROTECT_IMU);
			}
			if (inductance4_calibration_active || !inductance4_calibration_valid)
			{
				pwm_state = 0;
			}

			if (line_guard_required() && !inductance4_line_is_present())
			{
				if (!line_wait_active)
				{
					line_wait_active = 1U;
					reset_motion_pid_state();
				}
			}
			else if (line_wait_active)
			{
				line_wait_active = 0U;
				reset_motion_pid_state();
			}

		/********************* Gyroscope integration ********************/

            update_gyro_angle_accumulator(&gyro_roll,gyro_roll_sign_rign);
            update_gyro_angle_accumulator(&gyro_roll_cross,gyro_roll_sign_cross);
            update_gyro_angle_accumulator(&gyro_right_angle,gyro_roll_sign_angle);
		/********************* Encoder integration ********************/

			// Encoder integration
			update_encoder_speedup_value(&mot_inc_element,encoder_sign);
			
		
	  /********************* Differential speed PID ********************/

		dir_enlarge = 1.0f;
		speed_damping_enlarge = 0.20f;
		
		
		speed_damping = fabs(gyro_data[0]) * speed_damping_enlarge;
		speed_damping = limit_function(speed_damping, 0, 150);

		

		speed_direction_guard_mask = 0U;
		if (line_wait_active)
		{
			L_pid.Target = 0.0f;
			R_pid.Target = 0.0f;
			current_l_pwm_inc = 0.0f;
			current_r_pwm_inc = 0.0f;
			current_l_pwm_inc_last = 0.0f;
			current_r_pwm_inc_last = 0.0f;
		}
		else
		{
			if (element4_get_speed_override(&override_left_target, &override_right_target))
			{
				L_pid.Target = override_left_target;
				R_pid.Target = override_right_target;
			}
			else
			{
				/* Normal tracking keeps average speed stable and never reverses a wheel. */
				track_line_speed_scale = motion_runtime_line_speed_scale(
					inductance4_get_line_sum());
				track_base_target = L_pid.Target_base - speed_damping;
				if (track_base_target < 0.0f)
				{
					track_base_target = 0.0f;
				}
				track_base_target *= track_line_speed_scale;

				if (dir_loop_limit > 0.0f)
				{
					track_turn_ratio = dir_enlarge * Turn_PID.out / dir_loop_limit;
				}
				else
				{
					track_turn_ratio = 0.0f;
				}
				track_turn_ratio = limit_function(
					track_turn_ratio,
					-g_track_duty_limit,
					g_track_duty_limit);

				L_pid.Target = track_base_target * (1.0f - track_turn_ratio);
				R_pid.Target = track_base_target * (1.0f + track_turn_ratio);
			}

			left_pid_delta = motion_runtime_limit_pid_delta(
				IncPID(l_speed_now, L_pid.Target, &L_pid));
			right_pid_delta = motion_runtime_limit_pid_delta(
				IncPID(r_speed_now, R_pid.Target, &R_pid));
			current_l_pwm_inc = current_l_pwm_inc + left_pid_delta;
			current_r_pwm_inc = current_r_pwm_inc + right_pid_delta;

			current_l_pwm_inc = current_l_pwm_inc_last * 0.2f + current_l_pwm_inc * 0.8f;
			current_r_pwm_inc = current_r_pwm_inc_last * 0.2f + current_r_pwm_inc * 0.8f;

			current_l_pwm_inc = limit_function(current_l_pwm_inc, -1000, 1000);
			current_r_pwm_inc = limit_function(current_r_pwm_inc, -1000, 1000);

			current_l_pwm_inc = enforce_target_direction(
				current_l_pwm_inc, L_pid.Target, 0x01U);
			current_r_pwm_inc = enforce_target_direction(
				current_r_pwm_inc, R_pid.Target, 0x02U);

			current_l_pwm_inc_last = current_l_pwm_inc;
			current_r_pwm_inc_last = current_r_pwm_inc;
		}


	 
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

                current_l_pwm_inc=limit_function(current_l_pwm_inc,-1000,1000);
                current_r_pwm_inc=limit_function(current_r_pwm_inc,-1000,1000);
            }
        #endif
		
		
        current_l_pwm_duty=limit_function(current_l_pwm_duty,-1000,1000);
		current_r_pwm_duty=limit_function(current_r_pwm_duty,-1000,1000);

		motion_runtime_check_feedback(
			L_pid.Target,
			R_pid.Target,
			l_speed_now,
			r_speed_now,
			current_l_pwm_duty,
			current_r_pwm_duty,
			(uint8)(pwm_state == 1U && !line_wait_active));


		
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
		if (line_guard_required() && !inductance4_line_is_present())
		{
			error = 0.0f;
			reset_pid_runtime(&Turn_PID);
		}
		else
		{
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
		}


		TIM4_CLEAR_FLAG;
}







