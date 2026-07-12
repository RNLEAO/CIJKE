

#include "headfile.h"
#include "inductance4.h"
#include "inductance4_menu.h"


		// ASCII-cleaned legacy comment.

uint8 key_value;
int8 key_mode=MENU_PAGE_HOME;
uint8 display_mode=0;



extern int zhijiao_flag;

#define DIAGNOSTIC_UPLOAD_DIVIDER_NORMAL 10U
#define DIAGNOSTIC_UPLOAD_DIVIDER_GUIDE  50U
#define DIAGNOSTIC_TX_BUFFER_SIZE 520U
#define GUIDE_STEP_COUNT          10U
#define GUIDE_COMMAND_SIZE        20U
#define GUIDE_CAPTURE_SAMPLES     40U

typedef enum
{
    GUIDE_IDLE = 0,
    GUIDE_COLLECTING,
    GUIDE_REVIEW_PASS,
    GUIDE_REVIEW_FAIL
} GuideState;

static int8 xdata diagnostic_tx_buffer[DIAGNOSTIC_TX_BUFFER_SIZE];
static uint8 xdata guide_command_buffer[GUIDE_COMMAND_SIZE];
static uint8 xdata guide_receive_buffer[GUIDE_COMMAND_SIZE];
static uint16 xdata guide_old_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_old_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_max[INDUCTANCE4_CHANNEL_COUNT];
static uint8 xdata diagnostic_channel;
static uint8 xdata diagnostic_signal_mask;
static uint8 xdata guide_command_length;
static uint8 xdata guide_step;
static GuideState xdata guide_state;
static uint16 xdata diagnostic_send_offset;
static uint8 xdata guide_window_active;
static uint8 xdata guide_capture_remaining;
static uint8 xdata guide_status_pending;

static void guide_finish_capture(void);

static void guide_reset_window(void)
{
    uint8 channel;

    for (channel = 0U; channel < INDUCTANCE4_CHANNEL_COUNT; channel++)
    {
        guide_step_min[channel] = 4095U;
        guide_step_max[channel] = 0U;
    }
    guide_window_active = 1U;
}

static void guide_update_window(void)
{
    uint8 channel;

    if (guide_state != GUIDE_COLLECTING
        || !guide_window_active
        || guide_capture_remaining == 0U)
    {
        return;
    }

    for (channel = 0U; channel < INDUCTANCE4_CHANNEL_COUNT; channel++)
    {
        if (g_inductance4[channel].filtered < guide_step_min[channel])
        {
            guide_step_min[channel] = g_inductance4[channel].filtered;
        }
        if (g_inductance4[channel].filtered > guide_step_max[channel])
        {
            guide_step_max[channel] = g_inductance4[channel].filtered;
        }
    }

    guide_capture_remaining--;
    if (guide_capture_remaining == 0U)
    {
        guide_finish_capture();
    }
}

static const int8 *guide_sensor_name(uint8 channel)
{
    switch (channel)
    {
        case INDUCTANCE4_L:  return (const int8 *)"L";
        case INDUCTANCE4_LM: return (const int8 *)"LM";
        case INDUCTANCE4_RM: return (const int8 *)"RM";
        case INDUCTANCE4_R:  return (const int8 *)"R";
        default:             return (const int8 *)"UNKNOWN";
    }
}

static const int8 *guide_task_text(void)
{
    switch (guide_step)
    {
        case 0U: return (const int8 *)"L_OVER_WIRE";
        case 1U: return (const int8 *)"LM_OVER_WIRE";
        case 2U: return (const int8 *)"RM_OVER_WIRE";
        case 3U: return (const int8 *)"R_OVER_WIRE";
        case 4U: return (const int8 *)"STRAIGHT_LEFT";
        case 5U: return (const int8 *)"STRAIGHT_RIGHT";
        case 6U: return (const int8 *)"LEFT_ENTRY";
        case 7U: return (const int8 *)"LEFT_TURN";
        case 8U: return (const int8 *)"RIGHT_ENTRY";
        case 9U: return (const int8 *)"RIGHT_TURN";
        default: return (const int8 *)"REVIEW";
    }
}

static const int8 *guide_action_text(void)
{
    switch (guide_step)
    {
        case 0U: return (const int8 *)"L ON WIRE; SEND NEXT";
        case 1U: return (const int8 *)"LM ON WIRE; SEND NEXT";
        case 2U: return (const int8 *)"RM ON WIRE; SEND NEXT";
        case 3U: return (const int8 *)"R ON WIRE; SEND NEXT";
        case 4U: return (const int8 *)"CAR AT LEFT LIMIT; SEND NEXT";
        case 5U: return (const int8 *)"CAR AT RIGHT LIMIT; SEND NEXT";
        case 6U: return (const int8 *)"LEFT CORNER ENTRY; SEND NEXT";
        case 7U: return (const int8 *)"LEFT CORNER TURN; SEND NEXT";
        case 8U: return (const int8 *)"RIGHT CORNER ENTRY; SEND NEXT";
        case 9U: return (const int8 *)"RIGHT CORNER TURN; SEND NEXT";
        default: return (const int8 *)"CHECK REVIEW";
    }
}

static uint8 guide_command_equals(const int8 *expected)
{
    uint8 index = 0U;

    while (expected[index] && index < guide_command_length)
    {
        if ((uint8)expected[index] != guide_command_buffer[index])
        {
            return 0U;
        }
        index++;
    }

    return (index == guide_command_length && expected[index] == '\0');
}

static void guide_send_reply(const char *reply)
{
    wireless_uart_send_string(reply);
}

static void guide_cancel(void)
{
    if (guide_state == GUIDE_IDLE)
    {
        guide_send_reply("OK: GUIDE ALREADY IDLE\r\n");
        return;
    }

    inductance4_calibration_cancel();
    inductance4_set_calibration(guide_old_min, guide_old_max);
    guide_state = GUIDE_IDLE;
    guide_step = 0U;
    guide_status_pending = 0U;
    guide_send_reply("OK: GUIDE CANCELLED, OLD CAL RESTORED\r\n");
}

static void guide_start(void)
{
    if (guide_state != GUIDE_IDLE)
    {
        inductance4_calibration_cancel();
        inductance4_set_calibration(guide_old_min, guide_old_max);
    }

    pwm_state = 0U;
    Pwmout = 0U;
    inductance4_get_calibration(guide_old_min, guide_old_max);
    inductance4_guided_calibration_start();
    guide_state = GUIDE_COLLECTING;
    guide_step = 0U;
    guide_window_active = 0U;
    guide_capture_remaining = 0U;
    guide_status_pending = 1U;
    guide_send_reply("OK: GUIDE STARTED, STEP 1 L_OVER_WIRE\r\n");
}

static void guide_finish_capture(void)
{
    inductance4_calibration_capture_values(guide_step_min);
    inductance4_calibration_capture_values(guide_step_max);
    inductance4_get_calibration_extrema(guide_capture_min, guide_capture_max);
    guide_window_active = 0U;
    guide_step++;

    if (guide_step < GUIDE_STEP_COUNT)
    {
        guide_status_pending = 1U;
        guide_send_reply("OK: STEP SAVED, FOLLOW NEXT ACTION\r\n");
        return;
    }

    if (inductance4_calibration_finish())
    {
        guide_state = GUIDE_REVIEW_PASS;
        guide_status_pending = 1U;
        guide_send_reply("OK: REVIEW PASS, SEND SAVE OR CANCEL\r\n");
    }
    else
    {
        guide_state = GUIDE_REVIEW_FAIL;
        inductance4_set_calibration(guide_old_min, guide_old_max);
        guide_status_pending = 1U;
        guide_send_reply("ERROR: REVIEW FAIL, SEND CAL START TO RETRY\r\n");
    }
}

static void guide_capture_next(void)
{
    if (guide_state != GUIDE_COLLECTING)
    {
        guide_send_reply("ERROR: SEND CAL START FIRST\r\n");
        return;
    }
    if (guide_capture_remaining > 0U)
    {
        guide_send_reply("WAIT: CAPTURE RUNNING, HOLD STILL\r\n");
        return;
    }

    pwm_state = 0U;
    Pwmout = 0U;
    guide_reset_window();
    guide_capture_remaining = GUIDE_CAPTURE_SAMPLES;
    guide_status_pending = 1U;
    guide_send_reply("OK: CAPTURING, HOLD STILL ABOUT 1 SECOND\r\n");
}

static void guide_save(void)
{
    if (guide_state != GUIDE_REVIEW_PASS)
    {
        guide_send_reply("ERROR: GUIDE NOT PASSED, CANNOT SAVE\r\n");
        return;
    }

    if (inductance4_save_config())
    {
        guide_state = GUIDE_IDLE;
        guide_send_reply("OK: CAL SAVED TO FLASH\r\n");
    }
    else
    {
        guide_send_reply("ERROR: FLASH SAVE FAILED\r\n");
    }
}

static void guide_process_command(void)
{
    if (guide_command_equals((const int8 *)"CAL START"))
    {
        guide_start();
    }
    else if (guide_command_equals((const int8 *)"NEXT")
             || guide_command_equals((const int8 *)"N"))
    {
        guide_capture_next();
    }
    else if (guide_command_equals((const int8 *)"SAVE"))
    {
        guide_save();
    }
    else if (guide_command_equals((const int8 *)"CANCEL"))
    {
        guide_cancel();
    }
    else if (guide_command_equals((const int8 *)"STATUS"))
    {
        guide_status_pending = 1U;
        guide_send_reply("OK: STATUS REQUESTED\r\n");
    }
    else
    {
        guide_send_reply("ERROR: USE CAL START / NEXT / STATUS / SAVE / CANCEL\r\n");
    }

    guide_command_length = 0U;
}

static void guide_poll_commands(void)
{
    uint8 count;
    uint8 index;
    uint8 value;

    count = (uint8)wireless_uart_read_buffer(guide_receive_buffer, GUIDE_COMMAND_SIZE);
    for (index = 0U; index < count; index++)
    {
        value = guide_receive_buffer[index];
        if (value >= 'a' && value <= 'z')
        {
            value = (uint8)(value - 'a' + 'A');
        }

        if (value == '\r' || value == '\n')
        {
            if (guide_command_length > 0U)
            {
                guide_process_command();
            }
        }
        else if (guide_command_length < (GUIDE_COMMAND_SIZE - 1U))
        {
            guide_command_buffer[guide_command_length++] = value;
        }
    }

    if (guide_command_length > 0U)
    {
        if (guide_command_equals((const int8 *)"NEXT")
            || guide_command_equals((const int8 *)"N")
            || guide_command_equals((const int8 *)"SAVE")
            || guide_command_equals((const int8 *)"CANCEL")
            || guide_command_equals((const int8 *)"STATUS")
            || guide_command_equals((const int8 *)"CAL START"))
        {
            guide_process_command();
        }
    }
}

static const int8 *diagnostic_signal_text(void)
{
    return (15U == diagnostic_signal_mask)
        ? (const int8 *)"OK"
        : (const int8 *)"BAD";
}

static const int8 *diagnostic_calibration_text(void)
{
    if (inductance4_calibration_active)
    {
        return (const int8 *)"RUN";
    }

    return inductance4_calibration_valid
        ? (const int8 *)"READY"
        : (const int8 *)"NEEDED";
}

static const int8 *diagnostic_motor_text(void)
{
    return pwm_state
        ? (const int8 *)"RUN"
        : (const int8 *)"STOP";
}

static void diagnostic_append_u32(const int8 *label, uint32 value)
{
    diagnostic_send_offset += (uint16)zf_sprintf(
        &diagnostic_tx_buffer[diagnostic_send_offset],
        (const int8 *)"%s%u",
        label,
        value);
}

static void diagnostic_append_i32(const int8 *label, int32 value)
{
    diagnostic_send_offset += (uint16)zf_sprintf(
        &diagnostic_tx_buffer[diagnostic_send_offset],
        (const int8 *)"%s%d",
        label,
        value);
}

static void diagnostic_append_text(const int8 *label, const int8 *value)
{
    diagnostic_send_offset += (uint16)zf_sprintf(
        &diagnostic_tx_buffer[diagnostic_send_offset],
        (const int8 *)"%s%s",
        label,
        value);
}

static void send_diagnostic_frame(void)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_u32((const int8 *)"RAW: L=", (uint32)g_inductance4[INDUCTANCE4_L].filtered);
    diagnostic_append_u32((const int8 *)" LM=", (uint32)g_inductance4[INDUCTANCE4_LM].filtered);
    diagnostic_append_u32((const int8 *)" RM=", (uint32)g_inductance4[INDUCTANCE4_RM].filtered);
    diagnostic_append_u32((const int8 *)" R=", (uint32)g_inductance4[INDUCTANCE4_R].filtered);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_i32((const int8 *)"NORM: L=", (int32)g_inductance4[INDUCTANCE4_L].normalized);
    diagnostic_append_i32((const int8 *)" LM=", (int32)g_inductance4[INDUCTANCE4_LM].normalized);
    diagnostic_append_i32((const int8 *)" RM=", (int32)g_inductance4[INDUCTANCE4_RM].normalized);
    diagnostic_append_i32((const int8 *)" R=", (int32)g_inductance4[INDUCTANCE4_R].normalized);
    diagnostic_append_i32((const int8 *)" ERR=", (int32)error);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"STATUS: SIGNAL=", diagnostic_signal_text());
    diagnostic_append_u32((const int8 *)" MASK=", (uint32)diagnostic_signal_mask);
    diagnostic_append_text((const int8 *)" CAL=", diagnostic_calibration_text());
    diagnostic_append_text((const int8 *)" MOTOR=", diagnostic_motor_text());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n\r\n");

    if (guide_state == GUIDE_COLLECTING)
    {
        pwm_state = 0U;
        Pwmout = 0U;
        diagnostic_append_u32((const int8 *)"GUIDE: STEP=", (uint32)(guide_step + 1U));
        diagnostic_append_u32((const int8 *)"/", (uint32)GUIDE_STEP_COUNT);
        diagnostic_append_text((const int8 *)" TASK=", guide_task_text());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\nACTION: ");
        diagnostic_append_text(
            (const int8 *)"",
            guide_capture_remaining > 0U
                ? (const int8 *)"HOLD STILL; CAPTURING"
                : guide_action_text());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
        if (guide_step > 0U)
        {
            diagnostic_append_u32((const int8 *)"CAPTURED: L=", (uint32)guide_capture_min[INDUCTANCE4_L]);
            diagnostic_append_u32((const int8 *)"-", (uint32)guide_capture_max[INDUCTANCE4_L]);
            diagnostic_append_u32((const int8 *)" LM=", (uint32)guide_capture_min[INDUCTANCE4_LM]);
            diagnostic_append_u32((const int8 *)"-", (uint32)guide_capture_max[INDUCTANCE4_LM]);
            diagnostic_append_u32((const int8 *)" RM=", (uint32)guide_capture_min[INDUCTANCE4_RM]);
            diagnostic_append_u32((const int8 *)"-", (uint32)guide_capture_max[INDUCTANCE4_RM]);
            diagnostic_append_u32((const int8 *)" R=", (uint32)guide_capture_min[INDUCTANCE4_R]);
            diagnostic_append_u32((const int8 *)"-", (uint32)guide_capture_max[INDUCTANCE4_R]);
            diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
        }
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
    }
    else if (guide_state == GUIDE_REVIEW_PASS)
    {
        diagnostic_append_text((const int8 *)"GUIDE: REVIEW=PASS ACTION=SEND SAVE OR CANCEL",
                               (const int8 *)"\r\n");
    }
    else if (guide_state == GUIDE_REVIEW_FAIL)
    {
        diagnostic_append_text((const int8 *)"GUIDE: REVIEW=FAIL CHANNEL=",
                               guide_sensor_name(inductance4_calibration_failed_channel));
        diagnostic_append_text((const int8 *)" ACTION=SEND CAL START TO RETRY",
                               (const int8 *)"\r\n");
    }

    if (guide_state == GUIDE_REVIEW_PASS || guide_state == GUIDE_REVIEW_FAIL)
    {
        diagnostic_append_u32((const int8 *)"REVIEW L: MIN=", (uint32)guide_capture_min[INDUCTANCE4_L]);
        diagnostic_append_u32((const int8 *)" MAX=", (uint32)guide_capture_max[INDUCTANCE4_L]);
        diagnostic_append_u32((const int8 *)" SPAN=", (uint32)(guide_capture_max[INDUCTANCE4_L] - guide_capture_min[INDUCTANCE4_L]));
        diagnostic_append_u32((const int8 *)" | LM: MIN=", (uint32)guide_capture_min[INDUCTANCE4_LM]);
        diagnostic_append_u32((const int8 *)" MAX=", (uint32)guide_capture_max[INDUCTANCE4_LM]);
        diagnostic_append_u32((const int8 *)" SPAN=", (uint32)(guide_capture_max[INDUCTANCE4_LM] - guide_capture_min[INDUCTANCE4_LM]));
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
        diagnostic_append_u32((const int8 *)"REVIEW RM: MIN=", (uint32)guide_capture_min[INDUCTANCE4_RM]);
        diagnostic_append_u32((const int8 *)" MAX=", (uint32)guide_capture_max[INDUCTANCE4_RM]);
        diagnostic_append_u32((const int8 *)" SPAN=", (uint32)(guide_capture_max[INDUCTANCE4_RM] - guide_capture_min[INDUCTANCE4_RM]));
        diagnostic_append_u32((const int8 *)" | R: MIN=", (uint32)guide_capture_min[INDUCTANCE4_R]);
        diagnostic_append_u32((const int8 *)" MAX=", (uint32)guide_capture_max[INDUCTANCE4_R]);
        diagnostic_append_u32((const int8 *)" SPAN=", (uint32)(guide_capture_max[INDUCTANCE4_R] - guide_capture_min[INDUCTANCE4_R]));
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n\r\n");
    }

    if (diagnostic_send_offset >= DIAGNOSTIC_TX_BUFFER_SIZE)
    {
        return;
    }
    uart_write_buffer(
        WIRELESS_UART_INDEX,
        (uint8 *)diagnostic_tx_buffer,
        diagnostic_send_offset);
}

static void upload_inductance_diagnostics(void)
{
    static uint8 upload_divider = 0;
    diagnostic_signal_mask = 0U;

    if (guide_state == GUIDE_IDLE)
    {
        upload_divider++;
        if (upload_divider < DIAGNOSTIC_UPLOAD_DIVIDER_NORMAL)
        {
            return;
        }
        upload_divider = 0U;
    }
    else
    {
        upload_divider = 0U;
        if (!guide_status_pending)
        {
            return;
        }
        guide_status_pending = 0U;
    }

    for (diagnostic_channel = 0U;
         diagnostic_channel < INDUCTANCE4_CHANNEL_COUNT;
         diagnostic_channel++)
    {
        if (g_inductance4[diagnostic_channel].filtered > 4U
            && g_inductance4[diagnostic_channel].filtered < 4091U)
        {
            diagnostic_signal_mask |= (uint8)(1U << diagnostic_channel);
        }

    }
    send_diagnostic_frame();
}


		// ASCII-cleaned legacy comment.
		// ASCII-cleaned legacy comment.
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
		adc_init(ADC_P11, ADC_12BIT);	  
		delay_init();	
		
		
//			while(1) {
//				vbat_in=adc_once(ADC_P11, ADC_12BIT);
//				adc_vbat = ((float)vbat_in / 4095.0f) * 3.3f * (10000.0f + 3000.0f) / 3000.0f;

//				if (adc_vbat > adc_vbat_tar) {
//						pwm_state_charge=1;
//						pwm_state=1;

// ASCII-cleaned legacy comment.
//			}
		
	// ASCII-cleaned legacy comment.
		imu660ra_init();
		lcd_init();  		
		delay_init();
		

		
		init();
		
		change_speed_Target_base(speed[0]);
		//flash
		iap_init();                     
		load_all_params_from_flash();
		inductance4_load_config();
		element4_init();

		/* Start control interrupts only after every module is ready. */
		pit_timer_ms(TIM_1, 5);
		pit_timer_ms(TIM_4, 10);
		

		P52=0;


	while(1)
	{
			#if 1

            guide_poll_commands();
            guide_update_window();
            if (guide_state != GUIDE_IDLE)
            {
                pwm_state = 0U;
                Pwmout = 0U;
                negative_pressure_set_enabled(0);
            }
	
			lcd_show_status(pwm_state);

			key_value = fetch_ui_key_event();
			current_key = key_value; 

				// ASCII-cleaned legacy comment.
				if (key_value == KEY_EVENT_PAGE_NEXT) {
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode >= (MENU_PAGE_COUNT - 1)) {
								key_mode = MENU_PAGE_HOME;
						} else { // ASCII-cleaned legacy comment.
								key_mode++;
						}
				}
				// ASCII-cleaned legacy comment.
				else if (key_value == KEY_EVENT_PAGE_PREV) { // ASCII-cleaned legacy comment.
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode <= MENU_PAGE_HOME) {
								key_mode = (MENU_PAGE_COUNT - 1);
						} else { // ASCII-cleaned legacy comment.
								key_mode--;
						}
				}
			

					
				if (key_value == KEY_EVENT_SAVE_ALL) {
          save_all_params_to_flash();
					inductance4_save_config();
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
        case MENU_PAGE_HOME:
            lcd_show_font(72, 48, 32, 32, Ci_32x32, BLACK, WHITE);
            lcd_show_font(104, 48, 32, 32, Jian_32x32, BLACK, WHITE);
            lcd_show_font(136, 48, 32, 32, Ke_32x32, BLACK, WHITE);
            break;
        case MENU_PAGE_OVERVIEW: display_submenu_check(key_value); break;
        case MENU_PAGE_LEFT_MOTOR: display_motor(&L_pid, l_speed_now, current_l_pwm_duty, key_value, 0); break;
        case MENU_PAGE_RIGHT_MOTOR: display_motor(&R_pid, r_speed_now, current_r_pwm_duty, key_value, 1); break;
        case MENU_PAGE_TURN_PID: display_t(key_value); break;
        case MENU_PAGE_ERROR_WEIGHTS: display_submenu_ee(key_value); break;
        case MENU_PAGE_GYRO: display_gyro(key_value); break;
        case MENU_PAGE_GYRO_DATA: display_g(key_value); break;
        case MENU_PAGE_STRAIGHT: display_straight_param(key_value); break;
        case MENU_PAGE_RIGHT_ANGLE: display_right_angle_param(key_value); break;
        case MENU_PAGE_RING_DEBUG: display_circle_debug_menu(key_value); break;
        case MENU_PAGE_RING_ADVANCED: display_circle_advanced_menu(key_value); break;
        case MENU_PAGE_SPEED: display_speed_menu(key_value); break;
        case MENU_PAGE_CHARGE: display_submenu_charge_debug(key_value); break;
        case MENU_PAGE_INDUCTANCE4_CALIBRATION: display_inductance4_calibration(key_value); break;
        case MENU_PAGE_INDUCTANCE4_DATA: display_inductance4_data(); break;
        case MENU_PAGE_NEGATIVE_PRESSURE_AUTO: display_negative_pressure_auto(key_value); break;
        case MENU_PAGE_NEGATIVE_PRESSURE_OUTPUT: display_negative_pressure_output(key_value); break;
        default:
            key_mode = MENU_PAGE_HOME;
            break;
    }

            if (guide_state != GUIDE_IDLE)
            {
                negative_pressure_set_enabled(0);
            }

            delay_ms(20);
            upload_inductance_diagnostics();

		
		

			#endif

			#if 0
	
			lcd_show_status(pwm_state);

			key_value = key_scan(1);
			current_key = key_scan(1); 

				// ASCII-cleaned legacy comment.
				if (key_value == 4) {
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode >= (MENU_PAGE_COUNT - 1)) {
								key_mode = -1;
						} else { // ASCII-cleaned legacy comment.
						}
				}
				// ASCII-cleaned legacy comment.
				else if (key_value == KEY_EVENT_PAGE_PREV) { // ASCII-cleaned legacy comment.
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode <= -1) {
								key_mode = (MENU_PAGE_COUNT - 1);
						} else { // ASCII-cleaned legacy comment.
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
