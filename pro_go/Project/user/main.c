

#include "headfile.h"
#include "inductance4.h"
#include "inductance4_menu.h"


		// ASCII-cleaned legacy comment.

uint8 key_value;
int8 key_mode=0;                 
uint8 menu_sign=17;
uint8 display_mode=0;



extern int zhijiao_flag;

#define DIAGNOSTIC_TX_BUFFER_SIZE 900U
#define GUIDE_STEP_COUNT          10U
#define GUIDE_COMMAND_SIZE        32U
#define GUIDE_REPLY_SIZE         128U
#define GUIDE_CAPTURE_SAMPLES     40U
#define GUIDE_COMMAND_IDLE_POLLS   3U

typedef enum
{
    GUIDE_IDLE = 0,
    GUIDE_COLLECTING,
    GUIDE_REVIEW_PASS,
    GUIDE_REVIEW_FAIL
} GuideState;

static int8 xdata diagnostic_tx_buffer[DIAGNOSTIC_TX_BUFFER_SIZE];
static int8 xdata guide_reply_buffer[GUIDE_REPLY_SIZE];
static uint8 xdata guide_command_buffer[GUIDE_COMMAND_SIZE];
static uint8 xdata guide_receive_buffer[GUIDE_COMMAND_SIZE];
static uint16 xdata guide_old_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_old_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_max[INDUCTANCE4_CHANNEL_COUNT];
static uint8 xdata diagnostic_channel;
static uint8 xdata diagnostic_adc_mask;
static uint8 xdata guide_command_length;
static uint8 xdata guide_step;
static GuideState xdata guide_state;
static uint16 xdata diagnostic_send_offset;
static uint8 xdata guide_window_active;
static uint8 xdata guide_capture_remaining;
static uint8 xdata guide_status_pending;
static uint8 xdata diagnostic_compact_pending;
static uint8 xdata diagnostic_full_pending;
static MotorTestResult xdata motor_test_report_pending;
static EncoderTestResult xdata encoder_test_report_pending;

volatile uint8 diagnostic_stream_enabled = 0U;
volatile uint8 diagnostic_stream_due = 0U;

static void guide_finish_capture(void);
static void guide_send_reply(const char *reply);
static void guide_send_motor_test_started(MotorTestSide side);
static void guide_send_runtime_config(void);
static uint8 send_compact_status_frame(void);
static uint8 send_diagnostic_frame(void);
static uint8 send_motor_test_result_frame(MotorTestResult result);
static uint8 send_encoder_test_result_frame(EncoderTestResult result);

static const int8 *motor_test_result_text(MotorTestResult result)
{
    switch (result)
    {
        case MOTOR_TEST_RESULT_RUNNING: return (const int8 *)"RUNNING";
        case MOTOR_TEST_RESULT_DONE: return (const int8 *)"DONE";
        case MOTOR_TEST_RESULT_STOPPED: return (const int8 *)"STOPPED";
        case MOTOR_TEST_RESULT_LEFT_STALL: return (const int8 *)"L_STALL";
        case MOTOR_TEST_RESULT_RIGHT_STALL: return (const int8 *)"R_STALL";
        case MOTOR_TEST_RESULT_LEFT_DIRECTION: return (const int8 *)"L_DIR";
        case MOTOR_TEST_RESULT_RIGHT_DIRECTION: return (const int8 *)"R_DIR";
        case MOTOR_TEST_RESULT_IMU: return (const int8 *)"IMU";
        case MOTOR_TEST_RESULT_PROTECT: return (const int8 *)"PROTECT";
        case MOTOR_TEST_RESULT_ENCODER_MODE: return (const int8 *)"ENC_MODE";
        case MOTOR_TEST_RESULT_ENCODER_NOISE: return (const int8 *)"ENC_NOISE";
        default: return (const int8 *)"IDLE";
    }
}

static void motor_test_report_event(void)
{
    MotorTestResult event = motion_runtime_motor_test_take_event();

    if (event != MOTOR_TEST_RESULT_IDLE)
    {
        motor_test_report_pending = event;
    }
}

static void encoder_test_report_event(void)
{
    EncoderTestResult event = motion_runtime_encoder_test_take_event();

    if (event != ENCODER_TEST_RESULT_IDLE)
    {
        encoder_test_report_pending = event;
    }
}

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

static void guide_send_motor_test_started(MotorTestSide side)
{
    uint32 reply_length;
    const int8 *side_text = side == MOTOR_TEST_SIDE_LEFT
        ? (const int8 *)"L"
        : (side == MOTOR_TEST_SIDE_RIGHT
            ? (const int8 *)"R"
            : (const int8 *)"B");

    reply_length = zf_sprintf(
        guide_reply_buffer,
        (const int8 *)"OK: MTEST %s START PWM=%u PRECHECK=%uMS RUN=%uMS AUTO_STOP\r\n",
        side_text,
        (uint32)motion_runtime_motor_test_pwm_value(),
        (uint32)MOTOR_TEST_PRECHECK_MS,
        (uint32)MOTOR_TEST_DURATION_MS);
    if (reply_length >= GUIDE_REPLY_SIZE)
    {
        reply_length = GUIDE_REPLY_SIZE - 1U;
    }
    guide_reply_buffer[reply_length] = '\0';
    guide_send_reply((const char *)guide_reply_buffer);
}

static void guide_send_runtime_config(void)
{
    uint32 reply_length = zf_sprintf(
        guide_reply_buffer,
        (const int8 *)"CFG: PWM_LIMIT=%u MTEST_PWM=%u MTEST_B_PWM=%u MTEST_MS=%u PRECHECK_MS=%u LDIR=%u RDIR=%u\r\n",
        (uint32)MOTOR_PWM_LIMIT_VALUE,
        (uint32)MOTOR_TEST_PWM_VALUE,
        (uint32)MOTOR_TEST_BOTH_PWM_VALUE,
        (uint32)MOTOR_TEST_DURATION_MS,
        (uint32)MOTOR_TEST_PRECHECK_MS,
        (uint32)LEFT_MOTOR_FORWARD_LEVEL,
        (uint32)RIGHT_MOTOR_FORWARD_LEVEL);
    if (reply_length >= GUIDE_REPLY_SIZE)
    {
        reply_length = GUIDE_REPLY_SIZE - 1U;
    }
    guide_reply_buffer[reply_length] = '\0';
    guide_send_reply((const char *)guide_reply_buffer);
}

static void guide_rearm_wireless_receiver(void)
{
    uint8 flush_pass;

    uart_rx_interrupt(WIRELESS_UART_INDEX, 0U);
    guide_command_length = 0U;
    for (flush_pass = 0U; flush_pass < 4U; flush_pass++)
    {
        if (wireless_uart_read_buffer(
                guide_receive_buffer,
                GUIDE_COMMAND_SIZE) == 0U)
        {
            break;
        }
    }
    uart_rx_start_buff(WIRELESS_UART_INDEX);
    uart_rx_interrupt(WIRELESS_UART_INDEX, 1U);
}

static void guide_cancel(void)
{
	motion_runtime_motor_test_stop();
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
	motion_runtime_motor_test_stop();
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
	motion_runtime_motor_test_stop();
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
	motion_runtime_motor_test_stop();
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
    if (guide_command_equals((const int8 *)"STOP"))
    {
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        reset_motion_pid_state();
        motion_runtime_force_stop();
        guide_send_reply("OK: MOTOR STOPPED\r\n");
    }
    else if (guide_command_equals((const int8 *)"CLEAR"))
    {
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        if (motion_runtime_clear_protection())
        {
            guide_send_reply("OK: PROTECTION CLEARED, RUN REMAINS LOCKED\r\n");
        }
        else
        {
            guide_send_reply("ERROR: IMU660RB NOT READY\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"RUN"))
    {
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        if (motion_runtime_can_run())
        {
            pwm_state = 1U;
            Pwmout = 1U;
            guide_send_reply("OK: MOTOR RUN\r\n");
        }
        else
        {
            pwm_state = 0U;
            Pwmout = 0U;
            guide_send_reply("ERROR: RUN LOCKED FOR IMU AND ENCODER DIAGNOSIS\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"IMU CAL"))
    {
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        motion_runtime_force_stop();
        if (motion_runtime_calibrate_imu(400U, 5U))
        {
            guide_send_reply("OK: IMU660RB CALIBRATED, KEEP CAR STILL DURING BOOT\r\n");
        }
        else
        {
            guide_send_reply("ERROR: IMU660RB CALIBRATION UNSTABLE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"ELEMENTS OFF"))
    {
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        element4_set_enabled(0U);
        guide_send_reply("OK: ELEMENTS OFF\r\n");
    }
    else if (guide_command_equals((const int8 *)"MTEST STOP"))
    {
		motion_runtime_encoder_test_stop();
		if (motion_runtime_motor_test_stop())
		{
			guide_send_reply("OK: MTEST STOPPED\r\n");
		}
		else
		{
			guide_send_reply("OK: MTEST ALREADY STOPPED\r\n");
		}
    }
    else if (guide_command_equals((const int8 *)"MTEST L")
             || guide_command_equals((const int8 *)"MTEST R")
             || guide_command_equals((const int8 *)"MTEST B"))
    {
		MotorTestSide side = guide_command_equals((const int8 *)"MTEST L")
			? MOTOR_TEST_SIDE_LEFT
			: (guide_command_equals((const int8 *)"MTEST R")
				? MOTOR_TEST_SIDE_RIGHT
				: MOTOR_TEST_SIDE_BOTH);

		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		reset_motion_pid_state();
		if (guide_state != GUIDE_IDLE)
		{
			guide_send_reply("ERROR: MTEST DISABLED DURING CALIBRATION GUIDE\r\n");
		}
		else if (element4_is_enabled())
		{
			guide_send_reply("ERROR: MTEST REQUIRES ELEMENTS OFF\r\n");
		}
		else if (g_imu_runtime_state != IMU_RUNTIME_READY)
		{
			guide_send_reply("ERROR: MTEST REQUIRES IMU STATE OK\r\n");
		}
		else if (g_motion_protect_reason != MOTION_PROTECT_NONE)
		{
			guide_send_reply("ERROR: MTEST PROTECTION ACTIVE; SEND CLEAR\r\n");
		}
		else
		{
			interrupt_global_disable();
			if (motion_runtime_motor_test_start(side))
			{
				interrupt_global_enable();
				guide_send_motor_test_started(side);
			}
			else
			{
				interrupt_global_enable();
				guide_send_reply("ERROR: MTEST START REJECTED BY SAFETY STATE\r\n");
			}
		}
    }
    else if (guide_command_equals((const int8 *)"ETEST STOP"))
    {
		if (motion_runtime_encoder_test_stop())
		{
			guide_send_reply("OK: ETEST STOPPED\r\n");
		}
		else
		{
			guide_send_reply("OK: ETEST ALREADY STOPPED\r\n");
		}
    }
    else if (guide_command_equals((const int8 *)"ETEST L")
             || guide_command_equals((const int8 *)"ETEST R"))
    {
		MotorTestSide side = guide_command_equals((const int8 *)"ETEST L")
			? MOTOR_TEST_SIDE_LEFT
			: MOTOR_TEST_SIDE_RIGHT;

		motion_runtime_motor_test_stop();
		motion_runtime_encoder_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		reset_motion_pid_state();
		motion_runtime_force_stop();
		if (guide_state != GUIDE_IDLE)
		{
			guide_send_reply("ERROR: ETEST DISABLED DURING CALIBRATION GUIDE\r\n");
		}
		else if (element4_is_enabled())
		{
			guide_send_reply("ERROR: ETEST REQUIRES ELEMENTS OFF\r\n");
		}
		else if (motion_runtime_encoder_mode_mask() != 0x03U)
		{
			guide_send_reply("ERROR: ETEST ENCODER MODE INVALID\r\n");
		}
		else
		{
			interrupt_global_disable();
			if (motion_runtime_encoder_test_start(side))
			{
				interrupt_global_enable();
				guide_send_reply(side == MOTOR_TEST_SIDE_LEFT
					? "OK: ETEST L START NO_PWM TIME=15S; TURN LEFT WHEEL BY HAND\r\n"
					: "OK: ETEST R START NO_PWM TIME=15S; TURN RIGHT WHEEL BY HAND\r\n");
			}
			else
			{
				interrupt_global_enable();
				guide_send_reply("ERROR: ETEST START REJECTED BY SAFETY STATE\r\n");
			}
		}
    }
    else if (guide_command_equals((const int8 *)"ELEMENTS ON"))
    {
		motion_runtime_motor_test_stop();
        if (!g_motion_run_unlocked)
        {
            element4_set_enabled(0U);
            guide_send_reply("ERROR: ELEMENTS LOCKED UNTIL BASE TRACKING PASSES\r\n");
        }
        else
        {
            element4_set_enabled(1U);
            guide_send_reply("OK: ELEMENTS ON\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"CAL START"))
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
        if (guide_state == GUIDE_IDLE)
        {
            diagnostic_compact_pending = 1U;
        }
        else
        {
            guide_status_pending = 1U;
        }
    }
    else if (guide_command_equals((const int8 *)"STATUS FULL"))
    {
        if (guide_state == GUIDE_IDLE)
        {
            diagnostic_full_pending = 1U;
        }
        else
        {
            guide_status_pending = 1U;
        }
    }
    else if (guide_command_equals((const int8 *)"STREAM ON"))
    {
        diagnostic_stream_due = 0U;
        diagnostic_stream_enabled = 1U;
        guide_send_reply("OK: STREAM ON PERIOD=5S; SEND STREAM OFF TO STOP\r\n");
    }
    else if (guide_command_equals((const int8 *)"STREAM OFF"))
    {
        diagnostic_stream_enabled = 0U;
        diagnostic_stream_due = 0U;
        guide_send_reply("OK: STREAM OFF\r\n");
    }
    else
    {
        guide_send_reply("ERROR: USE STATUS / STATUS FULL / STREAM ON / STREAM OFF / STOP / CLEAR / RUN / MTEST L / MTEST R / MTEST B / MTEST STOP / ETEST L / ETEST R / ETEST STOP / IMU CAL / ELEMENTS OFF / CAL START / NEXT / SAVE / CANCEL\r\n");
    }

    guide_command_length = 0U;
}

static void guide_poll_commands(void)
{
    static uint8 command_idle_polls = 0U;
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
            command_idle_polls = 0U;
        }
        else if (guide_command_length < (GUIDE_COMMAND_SIZE - 1U))
        {
            guide_command_buffer[guide_command_length++] = value;
        }
    }

    if (guide_command_length == 0U)
    {
        command_idle_polls = 0U;
    }
    else if (count > 0U)
    {
        command_idle_polls = 0U;
    }
    else
    {
        if (command_idle_polls < GUIDE_COMMAND_IDLE_POLLS)
        {
            command_idle_polls++;
        }
        if (command_idle_polls >= GUIDE_COMMAND_IDLE_POLLS)
        {
            command_idle_polls = 0U;
            guide_process_command();
        }
    }
}

static const int8 *diagnostic_line_text(void)
{
    return inductance4_line_is_present()
        ? (const int8 *)"FOUND"
        : (const int8 *)"LOST";
}

static const int8 *diagnostic_adc_text(void)
{
    return (15U == diagnostic_adc_mask)
        ? (const int8 *)"OK"
        : (const int8 *)"BAD";
}

static const int8 *diagnostic_guard_text(void)
{
    switch (motion_direction_guard_mask())
    {
        case 0x01U: return (const int8 *)"L";
        case 0x02U: return (const int8 *)"R";
        case 0x03U: return (const int8 *)"LR";
        default: return (const int8 *)"NONE";
    }
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
    if (pwm_state == 2U)
    {
        return (const int8 *)"PROTECT";
    }
    if (motion_runtime_motor_test_is_active())
    {
        return (const int8 *)"MTEST";
    }
    if (motion_runtime_encoder_test_is_active())
    {
        return (const int8 *)"ETEST";
    }
    if (!g_motion_run_unlocked)
    {
        return (const int8 *)"LOCKED";
    }
    if (pwm_state == 1U)
    {
        if (motion_line_wait_is_active())
        {
            return (const int8 *)"WAIT_LINE";
        }
        return (const int8 *)"RUN";
    }
    return (const int8 *)"STOP";
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

static uint8 send_compact_status_frame(void)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_text((const int8 *)"STATUS: IMU=",
                           (const int8 *)motion_runtime_imu_state_text());
    diagnostic_append_text((const int8 *)" MOTOR=", diagnostic_motor_text());
    diagnostic_append_text((const int8 *)" PROTECT=",
                           (const int8 *)motion_runtime_protect_reason_text());
    diagnostic_append_text((const int8 *)" ELEMENTS=",
                           element4_is_enabled()
                               ? (const int8 *)"ON"
                               : (const int8 *)"OFF");
    diagnostic_append_text((const int8 *)" STREAM=",
                           diagnostic_stream_enabled
                               ? (const int8 *)"ON"
                               : (const int8 *)"OFF");
    diagnostic_append_u32((const int8 *)" RUNLOCK=",
                          (uint32)!g_motion_run_unlocked);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"MTEST: SIDE=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_text((const int8 *)" RESULT=",
                           (const int8 *)motion_runtime_motor_test_result_text());
    diagnostic_append_u32((const int8 *)" PWM=",
                          (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32((const int8 *)" RUN_MS=",
                          (uint32)MOTOR_TEST_DURATION_MS);
    diagnostic_append_u32((const int8 *)" PULSES=",
                          motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32((const int8 *)" PEAK=",
                          (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"ETEST: SIDE=",
                           (const int8 *)motion_runtime_encoder_test_side_text());
    diagnostic_append_text((const int8 *)" RESULT=",
                           (const int8 *)motion_runtime_encoder_test_result_text());
    diagnostic_append_u32((const int8 *)" REMAIN_MS=",
                          (uint32)motion_runtime_encoder_test_remaining_ms());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ENC: LRAW=", (uint32)g_encoder_left_raw);
    diagnostic_append_u32((const int8 *)" RRAW=", (uint32)g_encoder_right_raw);
    diagnostic_append_i32((const int8 *)" LSIGNED=", g_encoder_left_signed);
    diagnostic_append_i32((const int8 *)" RSIGNED=", g_encoder_right_signed);
    diagnostic_append_u32((const int8 *)" T0CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x01U) ? 1U : 0U));
    diagnostic_append_u32((const int8 *)" T3CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x02U) ? 1U : 0U));
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_i32((const int8 *)"OUTPUT: LPWM=", (int32)g_motor_left_applied_pwm);
    diagnostic_append_i32((const int8 *)" RPWM=", (int32)g_motor_right_applied_pwm);
    diagnostic_append_text((const int8 *)" LINE=", diagnostic_line_text());
    diagnostic_append_text((const int8 *)" ADC=", diagnostic_adc_text());
    diagnostic_append_u32((const int8 *)" MASK=", (uint32)diagnostic_adc_mask);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (diagnostic_send_offset >= DIAGNOSTIC_TX_BUFFER_SIZE)
    {
        return 0U;
    }
    uart_write_buffer(
        WIRELESS_UART_INDEX,
        (uint8 *)diagnostic_tx_buffer,
        diagnostic_send_offset);
    return 1U;
}

static uint8 send_motor_test_result_frame(MotorTestResult result)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_text((const int8 *)"MTEST RESULT: SIDE=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_text((const int8 *)" CMD=FORWARD RESULT=",
                           motor_test_result_text(result));
    diagnostic_append_u32((const int8 *)" PWM=",
                          (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32((const int8 *)" RUN_MS=",
                          (uint32)MOTOR_TEST_DURATION_MS);
    diagnostic_append_u32((const int8 *)" PULSES=",
                          motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32((const int8 *)" PEAK=",
                          (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"MTEST ENC: LTOTAL=",
                          motion_runtime_motor_test_left_total());
    diagnostic_append_u32((const int8 *)" RTOTAL=",
                          motion_runtime_motor_test_right_total());
    diagnostic_append_u32((const int8 *)" LPEAK=",
                          (uint32)motion_runtime_motor_test_left_peak());
    diagnostic_append_u32((const int8 *)" RPEAK=",
                          (uint32)motion_runtime_motor_test_right_peak());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        diagnostic_append_u32((const int8 *)"MTEST BAL: DIFF=",
                              motion_runtime_motor_test_difference());
        diagnostic_append_u32((const int8 *)" MATCH_X1000=",
                              (uint32)motion_runtime_motor_test_balance_x1000());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
    }

    diagnostic_append_u32((const int8 *)"MTEST PRECHECK: LPEAK=",
                          (uint32)motion_runtime_motor_test_left_idle_peak());
    diagnostic_append_u32((const int8 *)" RPEAK=",
                          (uint32)motion_runtime_motor_test_right_idle_peak());
    diagnostic_append_u32((const int8 *)" T0CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x01U) ? 1U : 0U));
    diagnostic_append_u32((const int8 *)" T3CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x02U) ? 1U : 0U));
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"MTEST SAFE: PROTECT=",
                           (const int8 *)motion_runtime_protect_reason_text());
    diagnostic_append_i32((const int8 *)" LPWM=", (int32)g_motor_left_applied_pwm);
    diagnostic_append_i32((const int8 *)" RPWM=", (int32)g_motor_right_applied_pwm);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (result == MOTOR_TEST_RESULT_DONE)
    {
        diagnostic_append_text((const int8 *)"ACTION: TEST COMPLETE; TURN DRIVER OFF",
                               (const int8 *)"\r\n");
    }
    else
    {
        diagnostic_append_text((const int8 *)"ACTION: TEST FAILED; TURN DRIVER OFF; CHECK BEFORE CLEAR",
                               (const int8 *)"\r\n");
    }

    if (diagnostic_send_offset >= DIAGNOSTIC_TX_BUFFER_SIZE)
    {
        return 0U;
    }
    uart_write_buffer(
        WIRELESS_UART_INDEX,
        (uint8 *)diagnostic_tx_buffer,
        diagnostic_send_offset);
    return 1U;
}

static uint8 send_encoder_test_result_frame(EncoderTestResult result)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_text((const int8 *)"ETEST RESULT: SIDE=",
                           (const int8 *)motion_runtime_encoder_test_side_text());
    diagnostic_append_text((const int8 *)" RESULT=",
                           (const int8 *)motion_runtime_encoder_test_result_text());
    diagnostic_append_text((const int8 *)" NO_PWM=1",
                           (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ETEST ENC: LTOTAL=",
                          motion_runtime_encoder_test_left_total());
    diagnostic_append_u32((const int8 *)" RTOTAL=",
                          motion_runtime_encoder_test_right_total());
    diagnostic_append_u32((const int8 *)" LPEAK=",
                          (uint32)motion_runtime_encoder_test_left_peak());
    diagnostic_append_u32((const int8 *)" RPEAK=",
                          (uint32)motion_runtime_encoder_test_right_peak());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ETEST MODE: T0CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x01U) ? 1U : 0U));
    diagnostic_append_u32((const int8 *)" T3CT=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x02U) ? 1U : 0U));
    diagnostic_append_i32((const int8 *)" LPWM=", (int32)g_motor_left_applied_pwm);
    diagnostic_append_i32((const int8 *)" RPWM=", (int32)g_motor_right_applied_pwm);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (result == ENCODER_TEST_RESULT_DONE)
    {
        diagnostic_append_text((const int8 *)"ACTION: CAPTURE COMPLETE; KEEP DRIVER OFF",
                               (const int8 *)"\r\n");
    }
    else if (result == ENCODER_TEST_RESULT_STOPPED)
    {
        diagnostic_append_text((const int8 *)"ACTION: ETEST STOPPED; KEEP DRIVER OFF",
                               (const int8 *)"\r\n");
    }
    else
    {
        diagnostic_append_text((const int8 *)"ACTION: ETEST FAILED; CHECK ENCODER MODE",
                               (const int8 *)"\r\n");
    }

    if (diagnostic_send_offset >= DIAGNOSTIC_TX_BUFFER_SIZE)
    {
        return 0U;
    }
    uart_write_buffer(
        WIRELESS_UART_INDEX,
        (uint8 *)diagnostic_tx_buffer,
        diagnostic_send_offset);
    return 1U;
}

static uint8 send_diagnostic_frame(void)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_u32((const int8 *)"RAW: L=", (uint32)g_inductance4[INDUCTANCE4_L].filtered);
    diagnostic_append_u32((const int8 *)" LM=", (uint32)g_inductance4[INDUCTANCE4_LM].filtered);
    diagnostic_append_u32((const int8 *)" RM=", (uint32)g_inductance4[INDUCTANCE4_RM].filtered);
    diagnostic_append_u32((const int8 *)" R=", (uint32)g_inductance4[INDUCTANCE4_R].filtered);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"MTEST: SIDE=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_u32((const int8 *)" REMAIN_MS=",
                          (uint32)motion_runtime_motor_test_remaining_ms());
    diagnostic_append_u32(
        (const int8 *)" PWM=",
        (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32(
        (const int8 *)" PULSES=",
        motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32(
        (const int8 *)" PEAK=",
        (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)" RESULT=",
                           (const int8 *)motion_runtime_motor_test_result_text());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_i32((const int8 *)"NORM: L=", (int32)g_inductance4[INDUCTANCE4_L].normalized);
    diagnostic_append_i32((const int8 *)" LM=", (int32)g_inductance4[INDUCTANCE4_LM].normalized);
    diagnostic_append_i32((const int8 *)" RM=", (int32)g_inductance4[INDUCTANCE4_RM].normalized);
    diagnostic_append_i32((const int8 *)" R=", (int32)g_inductance4[INDUCTANCE4_R].normalized);
    diagnostic_append_i32((const int8 *)" ERR_X1000=", (int32)(error * 1000.0f));
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"STATUS: LINE=", diagnostic_line_text());
    diagnostic_append_u32((const int8 *)" SUM=", (uint32)inductance4_get_line_sum());
    diagnostic_append_text((const int8 *)" ADC=", diagnostic_adc_text());
    diagnostic_append_u32((const int8 *)" MASK=", (uint32)diagnostic_adc_mask);
    diagnostic_append_text((const int8 *)" CAL=", diagnostic_calibration_text());
    diagnostic_append_text((const int8 *)" MOTOR=", diagnostic_motor_text());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"IMU: TYPE=660RB STATE=",
                           (const int8 *)motion_runtime_imu_state_text());
    diagnostic_append_i32((const int8 *)" GX_X10=", (int32)(g_imu_gyro_x_dps * 10.0f));
    diagnostic_append_i32((const int8 *)" GY_X10=", (int32)(g_imu_gyro_y_dps * 10.0f));
    diagnostic_append_i32((const int8 *)" GZ_X10=", (int32)(g_imu_gyro_z_dps * 10.0f));
    diagnostic_append_i32((const int8 *)" TURN_X10=", (int32)(g_imu_turn_rate_dps * 10.0f));
    diagnostic_append_i32((const int8 *)" BZ_X10=", (int32)(g_imu_bias_z_dps * 10.0f));
    diagnostic_append_text((const int8 *)" AXIS=Z", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ENC: LRAW=", (uint32)g_encoder_left_raw);
    diagnostic_append_u32((const int8 *)" RRAW=", (uint32)g_encoder_right_raw);
    diagnostic_append_i32((const int8 *)" LSIGNED=", g_encoder_left_signed);
    diagnostic_append_i32((const int8 *)" RSIGNED=", g_encoder_right_signed);
    diagnostic_append_u32((const int8 *)" LPHASE=", (uint32)g_encoder_left_phase);
    diagnostic_append_u32((const int8 *)" RPHASE=", (uint32)g_encoder_right_phase);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"SAFE: PROTECT=",
                           (const int8 *)motion_runtime_protect_reason_text());
    diagnostic_append_text((const int8 *)" ELEMENTS=",
                           element4_is_enabled()
                               ? (const int8 *)"ON"
                               : (const int8 *)"OFF");
    diagnostic_append_u32((const int8 *)" RUNLOCK=", (uint32)!g_motion_run_unlocked);
    diagnostic_append_i32((const int8 *)" DUTY_X1000=", (int32)(track_turn_ratio * 1000.0f));
    diagnostic_append_i32((const int8 *)" LINE_SCALE_X1000=", (int32)(track_line_speed_scale * 1000.0f));
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (guide_state == GUIDE_IDLE)
    {
        diagnostic_append_i32((const int8 *)"CTRL: ERR_X1000=", (int32)(error * 1000.0f));
        diagnostic_append_i32((const int8 *)" TURN_X10=", (int32)(Turn_PID.out * 10.0f));
        diagnostic_append_text((const int8 *)" ELEMENT=", (const int8 *)element4_state_name());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

        diagnostic_append_i32((const int8 *)"SPEED: LT_X10=", (int32)(L_pid.Target * 10.0f));
        diagnostic_append_i32((const int8 *)" RT_X10=", (int32)(R_pid.Target * 10.0f));
        diagnostic_append_i32((const int8 *)" LS_X10=", (int32)(l_speed_now * 10.0f));
        diagnostic_append_i32((const int8 *)" RS_X10=", (int32)(r_speed_now * 10.0f));
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

        diagnostic_append_i32((const int8 *)"OUTPUT: LPWM=", (int32)current_l_pwm_duty);
        diagnostic_append_i32((const int8 *)" RPWM=", (int32)current_r_pwm_duty);
        diagnostic_append_i32((const int8 *)" LAPPLIED=", (int32)g_motor_left_applied_pwm);
        diagnostic_append_i32((const int8 *)" RAPPLIED=", (int32)g_motor_right_applied_pwm);
        diagnostic_append_text((const int8 *)" GUARD=", diagnostic_guard_text());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
        diagnostic_append_u32((const int8 *)"COUNT: LSAT=", (uint32)g_motor_left_saturation_count);
        diagnostic_append_u32((const int8 *)" RSAT=", (uint32)g_motor_right_saturation_count);
        diagnostic_append_u32((const int8 *)" LREV=", (uint32)g_motor_left_reversal_count);
        diagnostic_append_u32((const int8 *)" RREV=", (uint32)g_motor_right_reversal_count);
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n\r\n");
    }
    else
    {
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
    }

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
        return 0U;
    }
    uart_write_buffer(
        WIRELESS_UART_INDEX,
        (uint8 *)diagnostic_tx_buffer,
        diagnostic_send_offset);
    return 1U;
}

static void upload_inductance_diagnostics(void)
{
    diagnostic_adc_mask = 0U;

    for (diagnostic_channel = 0U;
         diagnostic_channel < INDUCTANCE4_CHANNEL_COUNT;
         diagnostic_channel++)
    {
        if (g_inductance4[diagnostic_channel].filtered > 4U
            && g_inductance4[diagnostic_channel].filtered < 4091U)
        {
            diagnostic_adc_mask |= (uint8)(1U << diagnostic_channel);
        }

    }
    if (encoder_test_report_pending != ENCODER_TEST_RESULT_IDLE)
    {
        if (send_encoder_test_result_frame(encoder_test_report_pending))
        {
            encoder_test_report_pending = ENCODER_TEST_RESULT_IDLE;
        }
        return;
    }
    if (motor_test_report_pending != MOTOR_TEST_RESULT_IDLE)
    {
        if (send_motor_test_result_frame(motor_test_report_pending))
        {
            motor_test_report_pending = MOTOR_TEST_RESULT_IDLE;
        }
        return;
    }

    if (guide_state != GUIDE_IDLE)
    {
        if (guide_status_pending && send_diagnostic_frame())
        {
            guide_status_pending = 0U;
        }
        return;
    }

    if (diagnostic_compact_pending)
    {
        if (send_compact_status_frame())
        {
            diagnostic_compact_pending = 0U;
        }
        return;
    }

    if (diagnostic_full_pending)
    {
        if (send_diagnostic_frame())
        {
            diagnostic_full_pending = 0U;
        }
        return;
    }

    if (diagnostic_stream_enabled && diagnostic_stream_due)
    {
        if (send_diagnostic_frame())
        {
            diagnostic_stream_due = 0U;
        }
    }
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
		lcd_init();  		
		delay_init();
		

		
		init();
		negative_pressure_init();
		motion_runtime_force_stop();
		if (motion_runtime_init_imu())
		{
			motion_runtime_calibrate_imu(400U, 5U);
		}
		
		//flash
		iap_init();                     
		load_all_params_from_flash();
		inductance4_load_config();
		element4_init();
		element4_set_enabled(0U);
		motion_runtime_set_run_unlocked(0U);
		change_speed_Target_base(0);
		reset_motion_pid_state();
		pwm_state = 0U;
		Pwmout = 0U;
		guide_rearm_wireless_receiver();

		if (g_imu_runtime_state == IMU_RUNTIME_READY)
		{
			wireless_uart_send_string(
				"BOOT: IMU660RB=OK AXIS=Z MOTOR=LOCKED ELEMENTS=OFF\r\n");
		}
		else
		{
			wireless_uart_send_string(
				"BOOT: IMU660RB=ERROR MOTOR=LOCKED ELEMENTS=OFF\r\n");
		}
		guide_send_runtime_config();

		/* Start control interrupts only after every module is ready. */
		pit_timer_ms(TIM_1, 5);
		pit_timer_ms(TIM_4, 10);
		

		P52=0;


	while(1)
	{
			#if 1

			encoder_test_report_event();
			motor_test_report_event();
            upload_inductance_diagnostics();
            if (encoder_test_report_pending == ENCODER_TEST_RESULT_IDLE
                && motor_test_report_pending == MOTOR_TEST_RESULT_IDLE)
            {
                guide_poll_commands();
            }
            guide_update_window();
            if (guide_state != GUIDE_IDLE)
            {
                pwm_state = 0U;
                Pwmout = 0U;
            }
	
			lcd_show_status(pwm_state);

			key_value = fetch_ui_key_event();
			current_key = key_value; 

				// ASCII-cleaned legacy comment.
				if (key_value == KEY_EVENT_PAGE_NEXT) {
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode >= (menu_sign - 1)) { 
								key_mode = 0;
						} else { // ASCII-cleaned legacy comment.
								key_mode++;
						}
				}
				// ASCII-cleaned legacy comment.
				else if (key_value == KEY_EVENT_PAGE_PREV) { // ASCII-cleaned legacy comment.
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode <= 0) {
								key_mode = (menu_sign - 1);
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
        case 0:
            lcd_show_font(72, 48, 32, 32, Ci_32x32, BLACK, WHITE);
            lcd_show_font(104, 48, 32, 32, Jian_32x32, BLACK, WHITE);
            lcd_show_font(136, 48, 32, 32, Ke_32x32, BLACK, WHITE);
            break;
        case 1:  display_submenu_check(key_value); break;
        case 2:  display_motor(&L_pid, l_speed_now, current_l_pwm_duty, key_value, 0); break;
        case 3:  display_motor(&R_pid, r_speed_now, current_r_pwm_duty, key_value, 1); break;
        case 4:  display_t(key_value); break;
        case 5:  display_submenu_ee(key_value); break;
        case 6:  display_gyro(key_value); break;
        case 7:  display_g(key_value); break;
        case 8:  display_straight_param(key_value); break;
        case 9:  display_right_angle_param(key_value); break;
        case 10: display_circle_debug_menu(key_value); break;
        case 11: display_circle_advanced_menu(key_value); break;
        case 12: display_speed_menu(key_value); break;
        case 13: display_submenu_charge_debug(key_value); break;
        case 14: display_inductance4_calibration(key_value); break;
        case 15: display_inductance4_data(); break;
        case 16: display_negative_pressure_bench(key_value); break;
        default:
            key_mode = 0;
            break;
    }

            delay_ms(20);
		
		

			#endif

			#if 0
	
			lcd_show_status(pwm_state);

			key_value = key_scan(1);
			current_key = key_scan(1); 

				// ASCII-cleaned legacy comment.
				if (key_value == 4) {
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode >= (menu_sign - 1)) { 
								key_mode = -1;
						} else { // ASCII-cleaned legacy comment.
						}
				}
				// ASCII-cleaned legacy comment.
				else if (key_value == KEY_EVENT_PAGE_PREV) { // ASCII-cleaned legacy comment.
						lcd_clear(WHITE); // ASCII-cleaned legacy comment.
						if (key_mode <= -1) {
								key_mode = (menu_sign - 1);
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
