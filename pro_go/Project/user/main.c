

#include "headfile.h"
#include "inductance4.h"
#include "inductance4_menu.h"


		// ASCII-cleaned legacy comment.

uint8 key_value;
int8 key_mode=0;                 
uint8 menu_sign=17;
uint8 display_mode=0;



extern int zhijiao_flag;

#define DIAGNOSTIC_TX_BUFFER_SIZE 512U
#define GUIDE_STEP_COUNT          10U
#define GUIDE_COMMAND_SIZE        32U
#define GUIDE_REPLY_SIZE         128U
#define GUIDE_CAPTURE_SAMPLES     40U
#define GUIDE_COMMAND_IDLE_POLLS   3U
#if !RACE_MINIMAL_BUILD
#define SCOPE_CHANNEL_COUNT        8U
#define SCOPE_PACKET_SIZE         40U
#define SCOPE_ARM_TICKS          400U
#define SCOPE_TEST_TICKS         400U
#endif
#define IMU_INIT_RETRY_COUNT       3U

#if !RACE_MINIMAL_BUILD
typedef enum
{
    GUIDE_IDLE = 0,
    GUIDE_COLLECTING,
    GUIDE_REVIEW_PASS,
    GUIDE_REVIEW_FAIL
} GuideState;

typedef enum
{
    SCOPE_OFF = 0,
    SCOPE_ARMED,
    SCOPE_ACTIVE,
    SCOPE_TEST_ARMED,
    SCOPE_TEST_ACTIVE,
    SCOPE_RESULT_HELD
} ScopeState;
#endif

typedef struct
{
    TrackTestResult result;
    uint16 sample_count;
    int32 left_average_x10;
    int32 right_average_x10;
    int32 left_final_x10;
    int32 right_final_x10;
    uint16 match_x1000;
    uint16 start_sample_count;
    uint32 start_left_total;
    uint32 start_right_total;
    uint16 left_pwm_final;
    uint16 right_pwm_final;
    uint16 left_saturation_count;
    uint16 right_saturation_count;
    uint16 left_reversal_count;
    uint16 right_reversal_count;
    const int8 *result_text;
    const int8 *protect_text;
    uint8 mode;
    int8 t12_direction;
    uint8 t12_half_active;
    uint8 t12_start_release_reason;
    uint16 t12_start_release_sample_count;
    uint32 t12_start_release_left_total;
    uint32 t12_start_release_right_total;
    uint8 t12_exit_trigger_mask;
    uint16 t12_exit_angle_x10;
    uint16 t12_exit_half_ticks;
    uint8 t12_exit_norm_l;
    uint8 t12_exit_norm_lm;
    uint8 t12_exit_norm_rm;
    uint8 t12_exit_norm_r;
    int16 t12_exit_error_x1000;
    uint16 t12_exit_sum;
    uint8 t12_post_valid;
    uint8 t12_post_delay_ticks;
    uint16 t12_post_angle_x10;
    uint8 t12_post_norm_l;
    uint8 t12_post_norm_lm;
    uint8 t12_post_norm_rm;
    uint8 t12_post_norm_r;
    int16 t12_post_error_x1000;
    uint16 t12_post_sum;
} TrackResultSnapshot;

static int8 xdata diagnostic_tx_buffer[DIAGNOSTIC_TX_BUFFER_SIZE];
static int8 xdata guide_reply_buffer[GUIDE_REPLY_SIZE];
#if !RACE_MINIMAL_BUILD
static uint8 xdata scope_packet[SCOPE_PACKET_SIZE];
static float xdata scope_tx_values[SCOPE_CHANNEL_COUNT];
#endif
static TrackResultSnapshot xdata track_result_snapshot;
static uint8 xdata guide_command_buffer[GUIDE_COMMAND_SIZE];
static uint8 xdata guide_receive_buffer[GUIDE_COMMAND_SIZE];
#if !RACE_MINIMAL_BUILD
static uint16 xdata guide_old_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_old_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_capture_max[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_min[INDUCTANCE4_CHANNEL_COUNT];
static uint16 xdata guide_step_max[INDUCTANCE4_CHANNEL_COUNT];
#endif
static uint8 xdata diagnostic_channel;
static uint8 xdata diagnostic_adc_mask;
static uint8 xdata guide_command_length;
#if !RACE_MINIMAL_BUILD
static uint8 xdata guide_step;
static GuideState xdata guide_state;
static uint8 xdata guide_window_active;
static uint8 xdata guide_capture_remaining;
static uint8 xdata guide_status_pending;
#endif
static uint16 xdata diagnostic_send_offset;
static uint8 xdata diagnostic_compact_pending;
static MotorTestResult xdata motor_test_report_pending;
#if !RACE_MINIMAL_BUILD
static EncoderTestResult xdata encoder_test_report_pending;
#endif
static TrackTestResult xdata track_test_report_pending;
#if !RACE_MINIMAL_BUILD
static ScopeState scope_state = SCOPE_OFF;
#endif
static uint8 track_result_valid = 0U;

#if !RACE_MINIMAL_BUILD
extern volatile uint8 g_scope_arm_active;
extern volatile uint16 g_scope_arm_ticks;
extern volatile uint8 g_scope_capture_enabled;
extern volatile uint8 g_scope_test_mode;
extern volatile uint8 g_scope_snapshot_ready;
extern volatile float xdata g_scope_snapshot[SCOPE_CHANNEL_COUNT];
#endif

#if !RACE_MINIMAL_BUILD
static void guide_finish_capture(void);
#endif
static void guide_send_reply(const char *reply);
static void guide_send_ttest_error(const int8 *reason);
static void guide_send_motor_test_started(MotorTestSide side);
static void guide_send_track_test_started(uint8 mode);
static void guide_send_runtime_config(void);
static const int8 *guide_track_test_precheck(void);
#if !RACE_MINIMAL_BUILD
static void scope_service_arm(void);
static uint8 send_scope_frame(void);
#endif
static uint8 send_compact_status_frame(void);
static uint8 send_motor_test_result_frame(MotorTestResult result);
#if !RACE_MINIMAL_BUILD
static uint8 send_encoder_test_result_frame(EncoderTestResult result);
#endif
static uint8 send_track_test_result_frame(TrackTestResult result);

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

#if !RACE_MINIMAL_BUILD
static void encoder_test_report_event(void)
{
    EncoderTestResult event = motion_runtime_encoder_test_take_event();

    if (event != ENCODER_TEST_RESULT_IDLE)
    {
        encoder_test_report_pending = event;
    }
}
#endif

static void track_test_report_event(void)
{
    TrackTestResult event = motion_runtime_track_test_take_event();

    if (event != TRACK_TEST_RESULT_IDLE)
    {
        track_result_snapshot.result = event;
        track_result_snapshot.sample_count = motion_runtime_track_test_sample_count();
        track_result_snapshot.left_average_x10 = motion_runtime_track_test_left_average_x10();
        track_result_snapshot.right_average_x10 = motion_runtime_track_test_right_average_x10();
        track_result_snapshot.left_final_x10 = motion_runtime_track_test_left_final_x10();
        track_result_snapshot.right_final_x10 = motion_runtime_track_test_right_final_x10();
        track_result_snapshot.match_x1000 = motion_runtime_track_test_match_x1000();
        track_result_snapshot.start_sample_count = g_track_test_start_sample_count;
        track_result_snapshot.start_left_total = g_track_test_start_left_total;
        track_result_snapshot.start_right_total = g_track_test_start_right_total;
        track_result_snapshot.left_pwm_final = motion_runtime_track_test_left_pwm_final();
        track_result_snapshot.right_pwm_final = motion_runtime_track_test_right_pwm_final();
        track_result_snapshot.left_saturation_count = g_motor_left_saturation_count;
        track_result_snapshot.right_saturation_count = g_motor_right_saturation_count;
        track_result_snapshot.left_reversal_count = g_motor_left_reversal_count;
        track_result_snapshot.right_reversal_count = g_motor_right_reversal_count;
        track_result_snapshot.mode = g_track_test_mode;
        track_result_snapshot.t12_direction = g_track_test_t12_direction;
        track_result_snapshot.t12_half_active = g_track_test_t12_half_active;
        track_result_snapshot.t12_start_release_reason = g_track_t12_start_release_reason;
        track_result_snapshot.t12_start_release_sample_count = g_track_t12_start_release_sample_count;
        track_result_snapshot.t12_start_release_left_total = g_track_t12_start_release_left_total;
        track_result_snapshot.t12_start_release_right_total = g_track_t12_start_release_right_total;
        track_result_snapshot.t12_exit_trigger_mask = g_track_t12_exit_trigger_mask;
        track_result_snapshot.t12_exit_angle_x10 = g_track_t12_exit_angle_x10;
        track_result_snapshot.t12_exit_half_ticks = g_track_t12_exit_half_ticks;
        track_result_snapshot.t12_exit_norm_l = g_track_t12_exit_norm_l;
        track_result_snapshot.t12_exit_norm_lm = g_track_t12_exit_norm_lm;
        track_result_snapshot.t12_exit_norm_rm = g_track_t12_exit_norm_rm;
        track_result_snapshot.t12_exit_norm_r = g_track_t12_exit_norm_r;
        track_result_snapshot.t12_exit_error_x1000 = g_track_t12_exit_error_x1000;
        track_result_snapshot.t12_exit_sum = g_track_t12_exit_sum;
        track_result_snapshot.t12_post_valid = g_track_t12_post_valid;
        track_result_snapshot.t12_post_delay_ticks = g_track_t12_post_delay_ticks;
        track_result_snapshot.t12_post_angle_x10 = g_track_t12_post_angle_x10;
        track_result_snapshot.t12_post_norm_l = g_track_t12_post_norm_l;
        track_result_snapshot.t12_post_norm_lm = g_track_t12_post_norm_lm;
        track_result_snapshot.t12_post_norm_rm = g_track_t12_post_norm_rm;
        track_result_snapshot.t12_post_norm_r = g_track_t12_post_norm_r;
        track_result_snapshot.t12_post_error_x1000 = g_track_t12_post_error_x1000;
        track_result_snapshot.t12_post_sum = g_track_t12_post_sum;
        track_result_snapshot.result_text =
            (const int8 *)motion_runtime_track_test_result_text();
        track_result_snapshot.protect_text =
            (const int8 *)motion_runtime_protect_reason_text();
        track_result_valid = 1U;

#if !RACE_MINIMAL_BUILD
        if (scope_state == SCOPE_ACTIVE)
        {
            interrupt_global_disable();
            g_scope_capture_enabled = 0U;
            g_scope_snapshot_ready = 0U;
            interrupt_global_enable();
            scope_state = SCOPE_RESULT_HELD;
        }
        else
        {
            track_test_report_pending = event;
        }
#else
        track_test_report_pending = event;
#endif
    }
}

#if !RACE_MINIMAL_BUILD
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
#endif

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
#if RACE_MINIMAL_BUILD
    wireless_uart_send_string(reply);
#else
    if (scope_state != SCOPE_ACTIVE
        && scope_state != SCOPE_TEST_ACTIVE)
    {
        wireless_uart_send_string(reply);
    }
#endif
}

static void guide_prepare_imu_service(void)
{
	motion_runtime_track_test_stop();
	motion_runtime_encoder_test_stop();
	motion_runtime_motor_test_stop();
    motion_runtime_set_run_unlocked(0U);
    element4_set_enabled(0U);
    negative_pressure_set_enabled(0U);
    change_speed_Target_base(0);
    reset_motion_pid_state();
    pwm_state = 0U;
    Pwmout = 0U;
    motion_runtime_force_stop();
}

static uint8 imu_init_with_retry(void)
{
    uint8 attempt;

    for (attempt = 0U; attempt < IMU_INIT_RETRY_COUNT; attempt++)
    {
        if (motion_runtime_init_imu())
        {
            return 1U;
        }
        system_delay_ms(20U);
    }
    return 0U;
}

static void guide_send_imu_diagnostic(void)
{
    uint8 id_6b;
    uint8 id_6a;
    const int8 *address_text;
    uint32 reply_length;

    imu660rb_prepare_iic_bus();
    id_6b = imu660rb_get_chip_id_at(IMU660RB_DEV_ADDR);
    id_6a = imu660rb_get_chip_id_at(IMU660RB_DEV_ADDR_ALT);
    address_text = imu660rb_get_iic_address() == IMU660RB_DEV_ADDR_ALT
        ? (const int8 *)"6A"
        : (const int8 *)"6B";
    reply_length = zf_sprintf(
        guide_reply_buffer,
        (const int8 *)"IMU:I2C ID6B=%u ID6A=%u USE=%s EXPECT=107 STATE=%s\r\n",
        (uint32)id_6b,
        (uint32)id_6a,
        address_text,
        (const int8 *)motion_runtime_imu_state_text());

    if (reply_length >= GUIDE_REPLY_SIZE)
    {
        reply_length = GUIDE_REPLY_SIZE - 1U;
    }
    guide_reply_buffer[reply_length] = '\0';
    guide_send_reply((const char *)guide_reply_buffer);
}

static void guide_send_ttest_error(const int8 *reason)
{
    uint32 reply_length = zf_sprintf(
        guide_reply_buffer,
        (const int8 *)"ERR:TTEST %s\r\n",
        reason);

    if (reply_length >= GUIDE_REPLY_SIZE)
    {
        reply_length = GUIDE_REPLY_SIZE - 1U;
    }
    guide_reply_buffer[reply_length] = '\0';
    guide_send_reply((const char *)guide_reply_buffer);
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
        (const int8 *)"OK:MTEST %s PWM=%u PRE=%u T=%uMS\r\n",
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

static void guide_send_track_test_started(uint8 mode)
{
    guide_send_reply(mode == TRACK_TEST_MODE_T12
        ? "OK:TTEST Z09-10/T12R8 V120 T3000 E22 P25.5 F150 R30 X13/100 A170/200 BK600 B70/55 LP25\r\n"
        : "OK:TTEST Z07/T10R8 V215 T3000 P15 N300/M80 R500 BK600\r\n");
}

static void guide_send_runtime_config(void)
{
    uint32 reply_length;

    reply_length = zf_sprintf(
        guide_reply_buffer,
        (const int8 *)"CFG:PWM=%u ML=%u MB=%u MT=%u PRE=%u LD=%u RD=%u\r\n",
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

    guide_send_reply(
        "CFG2:RACE8 T10V215 T12V120 T3000 L150 R500 P25.5 X13/100 A170/200 BK600 B70/55 LP25\r\n");
}

static const int8 *guide_track_test_precheck(void)
{
#if !RACE_MINIMAL_BUILD
    if (guide_state != GUIDE_IDLE)
    {
        return (const int8 *)"GUIDE";
    }
#endif
    if (motion_runtime_track_test_is_active())
    {
        return (const int8 *)"BUSY";
    }
    if (g_motion_run_unlocked)
    {
        return (const int8 *)"RUNLOCK";
    }
    if (element4_is_enabled())
    {
        return (const int8 *)"ELEM";
    }
    if (negative_pressure_enabled
        || negative_pressure_armed
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || negative_pressure_real_output_percent != 0U)
    {
        return (const int8 *)"FAN";
    }
    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        return (const int8 *)"IMU";
    }
    if (g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        return (const int8 *)"SAFE;CLEAR";
    }
    if (motion_runtime_encoder_mode_mask() != 0x03U)
    {
        return (const int8 *)"ENCMODE";
    }
    if (!inductance4_calibration_valid)
    {
        return (const int8 *)"CAL";
    }
    if (!inductance4_line_is_present())
    {
        return (const int8 *)"LINE";
    }
    if (!motion_runtime_motor_test_both_passed())
    {
        return (const int8 *)"MTESTB";
    }
    return NULL;
}

#if !RACE_MINIMAL_BUILD
static void scope_service_arm(void)
{
    uint8 due;
    uint8 started;
    const int8 *reason;

    if (scope_state == SCOPE_TEST_ACTIVE)
    {
        interrupt_global_disable();
        due = (uint8)(g_scope_arm_active && g_scope_arm_ticks == 0U);
        if (due)
        {
            g_scope_arm_active = 0U;
            g_scope_capture_enabled = 0U;
            g_scope_test_mode = 0U;
            g_scope_snapshot_ready = 0U;
        }
        interrupt_global_enable();
        if (due)
        {
            scope_state = SCOPE_OFF;
        }
        return;
    }

    if (scope_state != SCOPE_ARMED
        && scope_state != SCOPE_TEST_ARMED)
    {
        return;
    }

    interrupt_global_disable();
    due = (uint8)(g_scope_arm_active && g_scope_arm_ticks == 0U);
    interrupt_global_enable();
    if (!due)
    {
        return;
    }

    if (scope_state == SCOPE_TEST_ARMED)
    {
        interrupt_global_disable();
        g_scope_snapshot_ready = 0U;
        g_scope_test_mode = 1U;
        g_scope_capture_enabled = 1U;
        g_scope_arm_ticks = SCOPE_TEST_TICKS;
        g_scope_arm_active = 1U;
        scope_state = SCOPE_TEST_ACTIVE;
        interrupt_global_enable();
        return;
    }

    reason = guide_track_test_precheck();
    if (reason != NULL)
    {
        interrupt_global_disable();
        g_scope_arm_active = 0U;
        g_scope_capture_enabled = 0U;
        g_scope_test_mode = 0U;
        g_scope_snapshot_ready = 0U;
        interrupt_global_enable();
        scope_state = SCOPE_OFF;
        guide_send_ttest_error(reason);
        return;
    }

    interrupt_global_disable();
    g_scope_arm_active = 0U;
    g_scope_snapshot_ready = 0U;
    g_scope_test_mode = 0U;
    track_result_valid = 0U;
    started = motion_runtime_track_test_start();
    if (started)
    {
        g_scope_capture_enabled = 1U;
        scope_state = SCOPE_ACTIVE;
    }
    else
    {
        g_scope_capture_enabled = 0U;
        scope_state = SCOPE_OFF;
    }
    interrupt_global_enable();

    if (!started)
    {
        guide_send_ttest_error((const int8 *)"STATE");
    }
}
#endif

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

#if !RACE_MINIMAL_BUILD
static void guide_cancel(void)
{
	motion_runtime_track_test_stop();
	motion_runtime_encoder_test_stop();
	motion_runtime_motor_test_stop();
    if (guide_state == GUIDE_IDLE)
    {
        guide_send_reply("OK:GUIDE IDLE\r\n");
        return;
    }

    inductance4_calibration_cancel();
    inductance4_set_calibration(guide_old_min, guide_old_max);
    guide_state = GUIDE_IDLE;
    guide_step = 0U;
    guide_status_pending = 0U;
    guide_send_reply("OK:CANCEL OLD=RESTORED\r\n");
}

static void guide_start(void)
{
	motion_runtime_track_test_stop();
	motion_runtime_encoder_test_stop();
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
    guide_send_reply("OK:GUIDE 1 L_OVER_WIRE\r\n");
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
        guide_send_reply("OK:STEP SAVED\r\n");
        return;
    }

    if (inductance4_calibration_finish())
    {
        guide_state = GUIDE_REVIEW_PASS;
        guide_status_pending = 1U;
        guide_send_reply("OK:REVIEW PASS\r\n");
    }
    else
    {
        guide_state = GUIDE_REVIEW_FAIL;
        inductance4_set_calibration(guide_old_min, guide_old_max);
        guide_status_pending = 1U;
        guide_send_reply("ERR:REVIEW;CAL START\r\n");
    }
}

static void guide_capture_next(void)
{
	motion_runtime_track_test_stop();
	motion_runtime_encoder_test_stop();
	motion_runtime_motor_test_stop();
    if (guide_state != GUIDE_COLLECTING)
    {
        guide_send_reply("ERR:CAL START FIRST\r\n");
        return;
    }
    if (guide_capture_remaining > 0U)
    {
        guide_send_reply("WAIT:CAPTURE HOLD\r\n");
        return;
    }

    pwm_state = 0U;
    Pwmout = 0U;
    guide_reset_window();
    guide_capture_remaining = GUIDE_CAPTURE_SAMPLES;
    guide_status_pending = 1U;
    guide_send_reply("OK:CAPTURE 1S HOLD\r\n");
}

static void guide_save(void)
{
	motion_runtime_track_test_stop();
	motion_runtime_encoder_test_stop();
	motion_runtime_motor_test_stop();
    if (guide_state != GUIDE_REVIEW_PASS)
    {
        guide_send_reply("ERR:GUIDE NOT PASS\r\n");
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
#endif

#if !RACE_MINIMAL_BUILD
static void guide_process_command(void)
{
    if ((scope_state == SCOPE_ACTIVE
         || scope_state == SCOPE_TEST_ACTIVE)
        && !guide_command_equals((const int8 *)"STOP")
        && !guide_command_equals((const int8 *)"TTEST STOP"))
    {
        guide_command_length = 0U;
        return;
    }
    if ((scope_state == SCOPE_ARMED
         || scope_state == SCOPE_TEST_ARMED)
        && !guide_command_equals((const int8 *)"STOP")
        && !guide_command_equals((const int8 *)"TTEST STOP"))
    {
        guide_send_reply("ERR:SCOPE ARMED;STOP\r\n");
        guide_command_length = 0U;
        return;
    }

    if (guide_command_equals((const int8 *)"STOP"))
    {
        if (scope_state == SCOPE_ARMED
            || scope_state == SCOPE_TEST_ARMED
            || scope_state == SCOPE_TEST_ACTIVE)
        {
            interrupt_global_disable();
            g_scope_arm_active = 0U;
            g_scope_arm_ticks = 0U;
            g_scope_capture_enabled = 0U;
            g_scope_test_mode = 0U;
            g_scope_snapshot_ready = 0U;
			interrupt_global_enable();
			scope_state = SCOPE_OFF;
		}
		else if (scope_state == SCOPE_ACTIVE)
		{
			interrupt_global_disable();
			g_scope_capture_enabled = 0U;
			g_scope_snapshot_ready = 0U;
			interrupt_global_enable();
			motion_runtime_track_test_stop();
			track_test_report_event();
		}
		else
		{
			motion_runtime_track_test_stop();
		}
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
		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        if (motion_runtime_clear_protection())
        {
            guide_send_reply("OK:CLEAR LOCK=1\r\n");
        }
        else
        {
            guide_send_reply("ERROR: IMU660RB NOT READY\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"RUN"))
    {
        if (motion_runtime_track_test_is_active())
        {
            guide_send_ttest_error((const int8 *)"RUN;STOP FIRST");
        }
        else
        {
			motion_runtime_encoder_test_stop();
			motion_runtime_motor_test_stop();
            pwm_state = 0U;
            Pwmout = 0U;
            change_speed_Target_base(0);
            reset_motion_pid_state();
            motion_runtime_force_stop();
            guide_send_reply("ERR:RUN USE TTEST\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"IMU DIAG"))
    {
        guide_prepare_imu_service();
        guide_send_imu_diagnostic();
    }
    else if (guide_command_equals((const int8 *)"IMU INIT"))
    {
        guide_prepare_imu_service();
        if (imu_init_with_retry())
        {
            guide_send_reply("OK:IMU INIT;SEND IMU CAL\r\n");
        }
        else
        {
            guide_send_reply("ERR:IMU MISSING\r\n");
        }
        guide_send_imu_diagnostic();
    }
    else if (guide_command_equals((const int8 *)"IMU CAL"))
    {
        guide_prepare_imu_service();
        if (g_imu_runtime_state == IMU_RUNTIME_MISSING
            && !imu_init_with_retry())
        {
            guide_send_reply("ERR:IMU MISSING;USE IMU DIAG\r\n");
            guide_send_imu_diagnostic();
        }
        else if (motion_runtime_calibrate_imu(400U, 5U))
        {
            guide_send_reply("OK:IMU CAL\r\n");
        }
        else
        {
            guide_send_reply("ERR:IMU UNSTABLE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"ELEMENTS OFF"))
    {
		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        element4_set_enabled(0U);
        guide_send_reply("OK: ELEMENTS OFF\r\n");
    }
    else if (guide_command_equals((const int8 *)"SCOPE TEST"))
    {
		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        change_speed_Target_base(0);
        reset_motion_pid_state();
        motion_runtime_force_stop();
        motion_runtime_set_run_unlocked(0U);
        element4_set_enabled(0U);
        negative_pressure_set_enabled(0U);
        diagnostic_compact_pending = 0U;
        guide_status_pending = 0U;
        encoder_test_report_pending = ENCODER_TEST_RESULT_IDLE;
        motor_test_report_pending = MOTOR_TEST_RESULT_IDLE;
        track_test_report_pending = TRACK_TEST_RESULT_IDLE;
        track_result_valid = 0U;
        scope_state = SCOPE_TEST_ARMED;
        interrupt_global_disable();
        g_scope_capture_enabled = 0U;
        g_scope_test_mode = 0U;
        g_scope_snapshot_ready = 0U;
        g_scope_arm_ticks = SCOPE_ARM_TICKS;
        g_scope_arm_active = 1U;
        interrupt_global_enable();
        guide_send_reply(
            "OK:SCOPE TEST ARM=2000MS BIN=V2/8CH/50HZ MOTOR=LOCKED\r\n");
    }
    else if (guide_command_equals((const int8 *)"TTEST STOP"))
    {
        uint8 stopped = 0U;

        if (scope_state == SCOPE_ARMED
            || scope_state == SCOPE_TEST_ARMED
            || scope_state == SCOPE_TEST_ACTIVE)
        {
            interrupt_global_disable();
            g_scope_arm_active = 0U;
            g_scope_arm_ticks = 0U;
            g_scope_capture_enabled = 0U;
            g_scope_test_mode = 0U;
            g_scope_snapshot_ready = 0U;
            interrupt_global_enable();
            scope_state = SCOPE_OFF;
            stopped = 1U;
        }
        else if (scope_state == SCOPE_ACTIVE)
        {
            interrupt_global_disable();
            g_scope_capture_enabled = 0U;
            g_scope_snapshot_ready = 0U;
            interrupt_global_enable();
            stopped = motion_runtime_track_test_stop();
            track_test_report_event();
        }
        else
        {
            stopped = motion_runtime_track_test_stop();
        }

        if (stopped)
        {
            guide_send_reply("OK:TTEST STOP LOCK=1\r\n");
        }
        else
        {
            guide_send_reply("OK:TTEST IDLE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"TTEST SCOPE"))
    {
        const int8 *reason;

		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		change_speed_Target_base(0);
		reset_motion_pid_state();
		motion_runtime_force_stop();

        reason = guide_track_test_precheck();
        if (reason != NULL)
        {
            guide_send_ttest_error(reason);
        }
        else
        {
            diagnostic_compact_pending = 0U;
            guide_status_pending = 0U;
            encoder_test_report_pending = ENCODER_TEST_RESULT_IDLE;
            motor_test_report_pending = MOTOR_TEST_RESULT_IDLE;
            track_test_report_pending = TRACK_TEST_RESULT_IDLE;
            track_result_valid = 0U;
            scope_state = SCOPE_ARMED;
            interrupt_global_disable();
            g_scope_capture_enabled = 0U;
            g_scope_test_mode = 0U;
            g_scope_snapshot_ready = 0U;
            g_scope_arm_ticks = SCOPE_ARM_TICKS;
            g_scope_arm_active = 1U;
            interrupt_global_enable();
            guide_send_reply(
                "OK:SCOPE Z07/T10TRACK ARM=2000MS BIN=V2/8CH/50HZ TRESULT=AFTER\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"TTEST T12")
             || guide_command_equals((const int8 *)"TTEST"))
    {
        const int8 *reason;
        uint8 track_mode;

        track_mode = guide_command_equals((const int8 *)"TTEST T12")
            ? TRACK_TEST_MODE_T12
            : TRACK_TEST_MODE_T10;

		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		change_speed_Target_base(0);
		reset_motion_pid_state();
		motion_runtime_force_stop();

        reason = guide_track_test_precheck();
        if (reason != NULL)
        {
            guide_send_ttest_error(reason);
        }
        else
        {
            interrupt_global_disable();
            g_scope_arm_active = 0U;
            g_scope_arm_ticks = 0U;
            g_scope_capture_enabled = 0U;
            g_scope_test_mode = 0U;
            g_scope_snapshot_ready = 0U;
            scope_state = SCOPE_OFF;
            track_result_valid = 0U;
            if (motion_runtime_track_test_start_mode(track_mode))
            {
                interrupt_global_enable();
                guide_send_track_test_started(track_mode);
            }
            else
            {
                interrupt_global_enable();
                guide_send_ttest_error((const int8 *)"STATE");
            }
        }
    }
    else if (guide_command_equals((const int8 *)"TRESULT"))
    {
        if (track_result_valid)
        {
            track_test_report_pending = track_result_snapshot.result;
        }
        else
        {
            guide_send_reply("ERR:TRESULT NONE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"MTEST STOP"))
    {
		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		if (motion_runtime_motor_test_stop())
		{
			guide_send_reply("OK:MTEST STOP\r\n");
		}
		else
		{
			guide_send_reply("OK:MTEST IDLE\r\n");
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

		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		reset_motion_pid_state();
		if (guide_state != GUIDE_IDLE)
		{
			guide_send_reply("ERR:MTEST GUIDE\r\n");
		}
		else if (element4_is_enabled())
		{
			guide_send_reply("ERR:MTEST ELEM\r\n");
		}
		else if (g_imu_runtime_state != IMU_RUNTIME_READY)
		{
			guide_send_reply("ERR:MTEST IMU\r\n");
		}
		else if (g_motion_protect_reason != MOTION_PROTECT_NONE)
		{
			guide_send_reply("ERR:MTEST SAFE;CLEAR\r\n");
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
				guide_send_reply("ERR:MTEST STATE\r\n");
			}
		}
    }
    else if (guide_command_equals((const int8 *)"ETEST STOP"))
    {
		motion_runtime_track_test_stop();
		if (motion_runtime_encoder_test_stop())
		{
			guide_send_reply("OK:ETEST STOP\r\n");
		}
		else
		{
			guide_send_reply("OK:ETEST IDLE\r\n");
		}
    }
    else if (guide_command_equals((const int8 *)"ETEST L")
             || guide_command_equals((const int8 *)"ETEST R"))
    {
		MotorTestSide side = guide_command_equals((const int8 *)"ETEST L")
			? MOTOR_TEST_SIDE_LEFT
			: MOTOR_TEST_SIDE_RIGHT;

		motion_runtime_track_test_stop();
		motion_runtime_motor_test_stop();
		motion_runtime_encoder_test_stop();
		pwm_state = 0U;
		Pwmout = 0U;
		reset_motion_pid_state();
		motion_runtime_force_stop();
		if (guide_state != GUIDE_IDLE)
		{
			guide_send_reply("ERR:ETEST GUIDE\r\n");
		}
		else if (element4_is_enabled())
		{
			guide_send_reply("ERR:ETEST ELEM\r\n");
		}
		else if (motion_runtime_encoder_mode_mask() != 0x03U)
		{
			guide_send_reply("ERR:ETEST ENCMODE\r\n");
		}
		else
		{
			interrupt_global_disable();
			if (motion_runtime_encoder_test_start(side))
			{
				interrupt_global_enable();
				guide_send_reply(side == MOTOR_TEST_SIDE_LEFT
					? "OK:ETEST L 15S HAND\r\n"
					: "OK:ETEST R 15S HAND\r\n");
			}
			else
			{
				interrupt_global_enable();
				guide_send_reply("ERR:ETEST STATE\r\n");
			}
		}
    }
    else if (guide_command_equals((const int8 *)"ELEMENTS ON"))
    {
		motion_runtime_track_test_stop();
		motion_runtime_encoder_test_stop();
		motion_runtime_motor_test_stop();
        if (!g_motion_run_unlocked)
        {
            element4_set_enabled(0U);
            guide_send_reply("ERR:ELEM LOCK\r\n");
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
            diagnostic_compact_pending = 1U;
        }
        else
        {
            guide_status_pending = 1U;
        }
    }
    else if (guide_command_equals((const int8 *)"STREAM ON"))
    {
        guide_send_reply("ERR:STREAM DISABLED\r\n");
    }
    else if (guide_command_equals((const int8 *)"STREAM OFF"))
    {
        guide_send_reply("OK: STREAM OFF\r\n");
    }
    else
    {
        guide_send_reply("ERR:CMD\r\n");
    }

    guide_command_length = 0U;
}
#else
static void guide_process_command(void)
{
    if (guide_command_equals((const int8 *)"STOP"))
    {
        motion_runtime_track_test_stop();
        motion_runtime_encoder_test_stop();
        motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        reset_motion_pid_state();
        motion_runtime_force_stop();
        guide_send_reply("OK:MOTOR STOPPED\r\n");
    }
    else if (guide_command_equals((const int8 *)"CLEAR"))
    {
        motion_runtime_track_test_stop();
        motion_runtime_encoder_test_stop();
        motion_runtime_motor_test_stop();
        if (motion_runtime_clear_protection())
        {
            guide_send_reply("OK:CLEAR LOCK=1\r\n");
        }
        else
        {
            guide_send_reply("ERR:IMU NOT READY\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"IMU DIAG"))
    {
        guide_prepare_imu_service();
        guide_send_imu_diagnostic();
    }
    else if (guide_command_equals((const int8 *)"IMU INIT"))
    {
        guide_prepare_imu_service();
        guide_send_reply(imu_init_with_retry()
            ? "OK:IMU INIT;SEND IMU CAL\r\n"
            : "ERR:IMU MISSING\r\n");
        guide_send_imu_diagnostic();
    }
    else if (guide_command_equals((const int8 *)"IMU CAL"))
    {
        guide_prepare_imu_service();
        if (g_imu_runtime_state == IMU_RUNTIME_MISSING
            && !imu_init_with_retry())
        {
            guide_send_reply("ERR:IMU MISSING;USE IMU DIAG\r\n");
        }
        else if (motion_runtime_calibrate_imu(400U, 5U))
        {
            guide_send_reply("OK:IMU CAL\r\n");
        }
        else
        {
            guide_send_reply("ERR:IMU UNSTABLE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"ELEMENTS OFF"))
    {
        motion_runtime_track_test_stop();
        motion_runtime_encoder_test_stop();
        motion_runtime_motor_test_stop();
        element4_set_enabled(0U);
        guide_send_reply("OK:ELEMENTS OFF\r\n");
    }
    else if (guide_command_equals((const int8 *)"TTEST STOP"))
    {
        guide_send_reply(motion_runtime_track_test_stop()
            ? "OK:TTEST STOP LOCK=1\r\n"
            : "OK:TTEST IDLE\r\n");
    }
    else if (guide_command_equals((const int8 *)"TTEST T12")
             || guide_command_equals((const int8 *)"TTEST"))
    {
        const int8 *reason;
        uint8 track_mode = guide_command_equals((const int8 *)"TTEST T12")
            ? TRACK_TEST_MODE_T12
            : TRACK_TEST_MODE_T10;

        motion_runtime_encoder_test_stop();
        motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        change_speed_Target_base(0);
        reset_motion_pid_state();
        motion_runtime_force_stop();
        reason = guide_track_test_precheck();
        if (reason != NULL)
        {
            guide_send_ttest_error(reason);
        }
        else
        {
            track_result_valid = 0U;
            if (motion_runtime_track_test_start_mode(track_mode))
            {
                guide_send_track_test_started(track_mode);
            }
            else
            {
                guide_send_ttest_error((const int8 *)"STATE");
            }
        }
    }
    else if (guide_command_equals((const int8 *)"MTEST STOP"))
    {
        guide_send_reply(motion_runtime_motor_test_stop()
            ? "OK:MTEST STOP\r\n"
            : "OK:MTEST IDLE\r\n");
    }
    else if (guide_command_equals((const int8 *)"MTEST B"))
    {
        motion_runtime_track_test_stop();
        motion_runtime_encoder_test_stop();
        motion_runtime_motor_test_stop();
        pwm_state = 0U;
        Pwmout = 0U;
        reset_motion_pid_state();
        if (element4_is_enabled())
        {
            guide_send_reply("ERR:MTEST ELEM\r\n");
        }
        else if (g_imu_runtime_state != IMU_RUNTIME_READY)
        {
            guide_send_reply("ERR:MTEST IMU\r\n");
        }
        else if (g_motion_protect_reason != MOTION_PROTECT_NONE)
        {
            guide_send_reply("ERR:MTEST SAFE;CLEAR\r\n");
        }
        else if (motion_runtime_motor_test_start(MOTOR_TEST_SIDE_BOTH))
        {
            guide_send_motor_test_started(MOTOR_TEST_SIDE_BOTH);
        }
        else
        {
            guide_send_reply("ERR:MTEST STATE\r\n");
        }
    }
    else if (guide_command_equals((const int8 *)"STATUS")
             || guide_command_equals((const int8 *)"STATUS FULL"))
    {
        diagnostic_compact_pending = 1U;
    }
    else
    {
        guide_send_reply("ERR:RACE8 CMD\r\n");
    }

    guide_command_length = 0U;
}
#endif

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
    if (motion_runtime_track_test_is_active())
    {
        return (const int8 *)"TTEST";
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

#if !RACE_MINIMAL_BUILD
static uint8 send_scope_frame(void)
{
    uint8 channel;
    uint8 index;
    uint8 checksum = 0xAAU;
    uint8 *bytes;

    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    interrupt_global_disable();
    if (!g_scope_snapshot_ready)
    {
        interrupt_global_enable();
        return 0U;
    }
    for (channel = 0U; channel < SCOPE_CHANNEL_COUNT; channel++)
    {
        scope_tx_values[channel] = g_scope_snapshot[channel];
    }
    g_scope_snapshot_ready = 0U;
    interrupt_global_enable();

    scope_packet[0] = 0xAAU;
    scope_packet[1] = 0U;
    scope_packet[2] = 0x10U;
    scope_packet[3] = SCOPE_CHANNEL_COUNT;
    scope_packet[4] = 0x01U;
    scope_packet[5] = 0U;
    scope_packet[6] = 0U;
    scope_packet[7] = 0U;
    for (channel = 0U; channel < SCOPE_CHANNEL_COUNT; channel++)
    {
        bytes = (uint8 *)&scope_tx_values[channel];
        index = (uint8)(8U + channel * 4U);
        scope_packet[index] = bytes[0];
        scope_packet[index + 1U] = bytes[1];
        scope_packet[index + 2U] = bytes[2];
        scope_packet[index + 3U] = bytes[3];
    }
    for (index = 2U; index < SCOPE_PACKET_SIZE; index++)
    {
        checksum += scope_packet[index];
    }
    scope_packet[1] = checksum;
    return (uint8)(wireless_uart_send_buffer(
        scope_packet,
        SCOPE_PACKET_SIZE) == 0U);
}
#endif

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
    diagnostic_append_text((const int8 *)" STREAM=", (const int8 *)"OFF");
    diagnostic_append_u32((const int8 *)" RUNLOCK=",
                          (uint32)!g_motion_run_unlocked);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"MT:S=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_text((const int8 *)" R=",
                           (const int8 *)motion_runtime_motor_test_result_text());
    diagnostic_append_u32((const int8 *)" P=",
                          (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32((const int8 *)" T=",
                          (uint32)MOTOR_TEST_DURATION_MS);
    diagnostic_append_u32((const int8 *)" N=",
                          motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32((const int8 *)" PK=",
                          (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"ET:S=",
                           (const int8 *)motion_runtime_encoder_test_side_text());
    diagnostic_append_text((const int8 *)" R=",
                           (const int8 *)motion_runtime_encoder_test_result_text());
    diagnostic_append_u32((const int8 *)" MS=",
                           (uint32)motion_runtime_encoder_test_remaining_ms());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"TT:R=",
                           (const int8 *)motion_runtime_track_test_result_text());
    diagnostic_append_u32((const int8 *)" MS=",
                          (uint32)motion_runtime_track_test_remaining_ms());
    diagnostic_append_u32((const int8 *)" V=",
                          (uint32)TRACK_TEST_TARGET_VALUE);
    diagnostic_append_u32((const int8 *)" B=",
                          (uint32)motion_runtime_motor_test_both_passed());
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
    diagnostic_append_text((const int8 *)"MTR:S=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_text((const int8 *)" R=",
                           motor_test_result_text(result));
    diagnostic_append_u32((const int8 *)" P=",
                          (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32((const int8 *)" T=",
                          (uint32)MOTOR_TEST_DURATION_MS);
    diagnostic_append_u32((const int8 *)" N=",
                          motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32((const int8 *)" PK=",
                          (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"MTE:L=",
                          motion_runtime_motor_test_left_total());
    diagnostic_append_u32((const int8 *)" R=",
                          motion_runtime_motor_test_right_total());
    diagnostic_append_u32((const int8 *)" LP=",
                          (uint32)motion_runtime_motor_test_left_peak());
    diagnostic_append_u32((const int8 *)" RP=",
                          (uint32)motion_runtime_motor_test_right_peak());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        diagnostic_append_u32((const int8 *)"MTB:D=",
                              motion_runtime_motor_test_difference());
        diagnostic_append_u32((const int8 *)" M=",
                              (uint32)motion_runtime_motor_test_balance_x1000());
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
    }

    diagnostic_append_u32((const int8 *)"MTP:L=",
                          (uint32)motion_runtime_motor_test_left_idle_peak());
    diagnostic_append_u32((const int8 *)" R=",
                          (uint32)motion_runtime_motor_test_right_idle_peak());
    diagnostic_append_u32((const int8 *)" T0=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x01U) ? 1U : 0U));
    diagnostic_append_u32((const int8 *)" T3=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x02U) ? 1U : 0U));
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_text((const int8 *)"MTS:P=",
                           (const int8 *)motion_runtime_protect_reason_text());
    diagnostic_append_i32((const int8 *)" L=", (int32)g_motor_left_applied_pwm);
    diagnostic_append_i32((const int8 *)" R=", (int32)g_motor_right_applied_pwm);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (result == MOTOR_TEST_RESULT_DONE)
    {
        diagnostic_append_text((const int8 *)"ACT:DONE DRIVER=OFF",
                               (const int8 *)"\r\n");
    }
    else
    {
        diagnostic_append_text((const int8 *)"ACT:FAIL DRIVER=OFF CHECK/CLEAR",
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

static uint8 send_track_test_result_frame(TrackTestResult result)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_text(
                           track_result_snapshot.mode == TRACK_TEST_MODE_T12
                               ? (const int8 *)"TTR:Z09-10/T12R8 R="
                               : (const int8 *)"TTR:Z07/T10R8 R=",
                           track_result_snapshot.result_text);
    if (track_result_snapshot.mode == TRACK_TEST_MODE_T12)
    {
        diagnostic_append_text((const int8 *)" D=",
                               track_result_snapshot.t12_direction < 0
                                   ? (const int8 *)"R"
                                   : (track_result_snapshot.t12_direction > 0
                                       ? (const int8 *)"L"
                                       : (const int8 *)"?"));
        diagnostic_append_u32((const int8 *)" H=",
                              (uint32)track_result_snapshot.t12_half_active);
    }
    diagnostic_append_u32((const int8 *)" N=",
                          (uint32)track_result_snapshot.sample_count);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (track_result_snapshot.mode == TRACK_TEST_MODE_T12)
    {
        diagnostic_append_u32((const int8 *)"T12S:K=",
                              (uint32)track_result_snapshot.t12_start_release_reason);
        diagnostic_append_u32((const int8 *)" N=",
                              (uint32)track_result_snapshot.t12_start_release_sample_count);
        diagnostic_append_u32((const int8 *)" L=",
                              track_result_snapshot.t12_start_release_left_total);
        diagnostic_append_u32((const int8 *)" R=",
                              track_result_snapshot.t12_start_release_right_total);
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

        diagnostic_append_u32((const int8 *)"T12X:K=",
                              (uint32)track_result_snapshot.t12_exit_trigger_mask);
        diagnostic_append_u32((const int8 *)" A10=",
                              (uint32)track_result_snapshot.t12_exit_angle_x10);
        diagnostic_append_u32((const int8 *)" HT=",
                              (uint32)track_result_snapshot.t12_exit_half_ticks);
        diagnostic_append_u32((const int8 *)" L=",
                              (uint32)track_result_snapshot.t12_exit_norm_l);
        diagnostic_append_u32((const int8 *)" LM=",
                              (uint32)track_result_snapshot.t12_exit_norm_lm);
        diagnostic_append_u32((const int8 *)" RM=",
                              (uint32)track_result_snapshot.t12_exit_norm_rm);
        diagnostic_append_u32((const int8 *)" R=",
                              (uint32)track_result_snapshot.t12_exit_norm_r);
        diagnostic_append_i32((const int8 *)" E=",
                              (int32)track_result_snapshot.t12_exit_error_x1000);
        diagnostic_append_u32((const int8 *)" S=",
                              (uint32)track_result_snapshot.t12_exit_sum);
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

        diagnostic_append_u32((const int8 *)"T12P:V=",
                              (uint32)track_result_snapshot.t12_post_valid);
        diagnostic_append_u32((const int8 *)" DT=",
                              (uint32)track_result_snapshot.t12_post_delay_ticks);
        diagnostic_append_u32((const int8 *)" A10=",
                              (uint32)track_result_snapshot.t12_post_angle_x10);
        diagnostic_append_u32((const int8 *)" L=",
                              (uint32)track_result_snapshot.t12_post_norm_l);
        diagnostic_append_u32((const int8 *)" LM=",
                              (uint32)track_result_snapshot.t12_post_norm_lm);
        diagnostic_append_u32((const int8 *)" RM=",
                              (uint32)track_result_snapshot.t12_post_norm_rm);
        diagnostic_append_u32((const int8 *)" R=",
                              (uint32)track_result_snapshot.t12_post_norm_r);
        diagnostic_append_i32((const int8 *)" E=",
                              (int32)track_result_snapshot.t12_post_error_x1000);
        diagnostic_append_u32((const int8 *)" S=",
                              (uint32)track_result_snapshot.t12_post_sum);
        diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");
    }

    diagnostic_append_i32((const int8 *)"TTS:LA10=",
                          track_result_snapshot.left_average_x10);
    diagnostic_append_i32((const int8 *)" RA10=",
                          track_result_snapshot.right_average_x10);
    diagnostic_append_i32((const int8 *)" LF10=",
                          track_result_snapshot.left_final_x10);
    diagnostic_append_i32((const int8 *)" RF10=",
                          track_result_snapshot.right_final_x10);
    diagnostic_append_u32((const int8 *)" M1000=",
                          (uint32)track_result_snapshot.match_x1000);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"TS5:N=",
                          (uint32)track_result_snapshot.start_sample_count);
    diagnostic_append_u32((const int8 *)" L=",
                          track_result_snapshot.start_left_total);
    diagnostic_append_u32((const int8 *)" R=",
                          track_result_snapshot.start_right_total);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"TTP:LF=",
                          (uint32)track_result_snapshot.left_pwm_final);
    diagnostic_append_u32((const int8 *)" RF=",
                          (uint32)track_result_snapshot.right_pwm_final);
    diagnostic_append_u32((const int8 *)" LS=",
                          (uint32)track_result_snapshot.left_saturation_count);
    diagnostic_append_u32((const int8 *)" RS=",
                          (uint32)track_result_snapshot.right_saturation_count);
    diagnostic_append_u32((const int8 *)" LR=",
                          (uint32)track_result_snapshot.left_reversal_count);
    diagnostic_append_u32((const int8 *)" RR=",
                          (uint32)track_result_snapshot.right_reversal_count);
    diagnostic_append_text((const int8 *)" P=",
                           track_result_snapshot.protect_text);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (result == TRACK_TEST_RESULT_DONE)
    {
        diagnostic_append_text(
            (const int8 *)"ACT:DONE LOCK=1 DRIVER=OFF",
            (const int8 *)"\r\n");
    }
    else if (result == TRACK_TEST_RESULT_STOPPED)
    {
        diagnostic_append_text(
            (const int8 *)"ACT:STOP LOCK=1",
            (const int8 *)"\r\n");
    }
    else
    {
        diagnostic_append_text(
            (const int8 *)"ACT:FAIL LOCK=1 CHECK/CLEAR",
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

#if !RACE_MINIMAL_BUILD
static uint8 send_encoder_test_result_frame(EncoderTestResult result)
{
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 0U;
    }

    diagnostic_send_offset = 0U;
    diagnostic_append_text((const int8 *)"ETR:S=",
                           (const int8 *)motion_runtime_encoder_test_side_text());
    diagnostic_append_text((const int8 *)" R=",
                           (const int8 *)motion_runtime_encoder_test_result_text());
    diagnostic_append_text((const int8 *)" P=0",
                           (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ETE:L=",
                          motion_runtime_encoder_test_left_total());
    diagnostic_append_u32((const int8 *)" R=",
                          motion_runtime_encoder_test_right_total());
    diagnostic_append_u32((const int8 *)" LP=",
                          (uint32)motion_runtime_encoder_test_left_peak());
    diagnostic_append_u32((const int8 *)" RP=",
                          (uint32)motion_runtime_encoder_test_right_peak());
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    diagnostic_append_u32((const int8 *)"ETM:T0=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x01U) ? 1U : 0U));
    diagnostic_append_u32((const int8 *)" T3=",
                          (uint32)((motion_runtime_encoder_mode_mask() & 0x02U) ? 1U : 0U));
    diagnostic_append_i32((const int8 *)" L=", (int32)g_motor_left_applied_pwm);
    diagnostic_append_i32((const int8 *)" R=", (int32)g_motor_right_applied_pwm);
    diagnostic_append_text((const int8 *)"", (const int8 *)"\r\n");

    if (result == ENCODER_TEST_RESULT_DONE)
    {
        diagnostic_append_text((const int8 *)"ACT:DONE DRIVER=OFF",
                               (const int8 *)"\r\n");
    }
    else if (result == ENCODER_TEST_RESULT_STOPPED)
    {
        diagnostic_append_text((const int8 *)"ACT:STOP DRIVER=OFF",
                               (const int8 *)"\r\n");
    }
    else
    {
        diagnostic_append_text((const int8 *)"ACT:FAIL ENCMODE",
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
#endif

#if 0
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

    diagnostic_append_text((const int8 *)"MT:S=",
                           (const int8 *)motion_runtime_motor_test_side_text());
    diagnostic_append_u32((const int8 *)" MS=",
                          (uint32)motion_runtime_motor_test_remaining_ms());
    diagnostic_append_u32(
        (const int8 *)" P=",
        (uint32)motion_runtime_motor_test_pwm_value());
    diagnostic_append_u32(
        (const int8 *)" N=",
        motion_runtime_motor_test_pulse_total());
    diagnostic_append_u32(
        (const int8 *)" PK=",
        (uint32)motion_runtime_motor_test_peak_raw());
    diagnostic_append_text((const int8 *)" R=",
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
        diagnostic_append_text((const int8 *)"GUIDE:PASS ACTION=SAVE/CANCEL",
                               (const int8 *)"\r\n");
    }
    else if (guide_state == GUIDE_REVIEW_FAIL)
    {
        diagnostic_append_text((const int8 *)"GUIDE:FAIL CH=",
                               guide_sensor_name(inductance4_calibration_failed_channel));
        diagnostic_append_text((const int8 *)" ACTION=CAL START",
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
#endif

static void upload_inductance_diagnostics(void)
{
#if !RACE_MINIMAL_BUILD
    if (scope_state == SCOPE_ACTIVE
        || scope_state == SCOPE_TEST_ACTIVE)
    {
        send_scope_frame();
        return;
    }
#endif

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
#if !RACE_MINIMAL_BUILD
    if (encoder_test_report_pending != ENCODER_TEST_RESULT_IDLE)
    {
        if (send_encoder_test_result_frame(encoder_test_report_pending))
        {
            encoder_test_report_pending = ENCODER_TEST_RESULT_IDLE;
        }
        return;
    }
#endif
    if (motor_test_report_pending != MOTOR_TEST_RESULT_IDLE)
    {
        if (send_motor_test_result_frame(motor_test_report_pending))
        {
            motor_test_report_pending = MOTOR_TEST_RESULT_IDLE;
        }
        return;
    }
    if (track_test_report_pending != TRACK_TEST_RESULT_IDLE)
    {
        if (send_track_test_result_frame(track_test_report_pending))
        {
            track_test_report_pending = TRACK_TEST_RESULT_IDLE;
        }
        return;
    }

#if !RACE_MINIMAL_BUILD
    if (guide_state != GUIDE_IDLE)
    {
        if (guide_status_pending && send_compact_status_frame())
        {
            guide_status_pending = 0U;
        }
        return;
    }
#endif

    if (diagnostic_compact_pending)
    {
        if (send_compact_status_frame())
        {
            diagnostic_compact_pending = 0U;
        }
        return;
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
#if !RACE_MINIMAL_BUILD
		lcd_init();
#endif
		delay_init();
		

		
		init();
		negative_pressure_init();
		motion_runtime_force_stop();
		if (imu_init_with_retry())
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
				"BOOT:IMU=OK AXIS=Z LOCK=1 ELEM=0\r\n");
		}
		else
		{
			wireless_uart_send_string(
				"BOOT:IMU=ERR LOCK=1 ELEM=0\r\n");
			guide_send_imu_diagnostic();
		}
		guide_send_runtime_config();

		/* Start control interrupts only after every module is ready. */
		pit_timer_ms(TIM_1, 5);
		pit_timer_ms(TIM_4, 10);
		

		P52=0;


	while(1)
	{
			#if RACE_MINIMAL_BUILD
			motor_test_report_event();
			track_test_report_event();
			upload_inductance_diagnostics();
			if (motor_test_report_pending == MOTOR_TEST_RESULT_IDLE
				&& track_test_report_pending == TRACK_TEST_RESULT_IDLE)
			{
				guide_poll_commands();
			}
			delay_ms(2);
			#else

			encoder_test_report_event();
			motor_test_report_event();
			track_test_report_event();
            scope_service_arm();
            upload_inductance_diagnostics();
            if (encoder_test_report_pending == ENCODER_TEST_RESULT_IDLE
                && motor_test_report_pending == MOTOR_TEST_RESULT_IDLE
                && track_test_report_pending == TRACK_TEST_RESULT_IDLE)
            {
                guide_poll_commands();
            }
            guide_update_window();
            if (guide_state != GUIDE_IDLE)
            {
                pwm_state = 0U;
                Pwmout = 0U;
            }
			if (scope_state == SCOPE_ARMED
				|| scope_state == SCOPE_ACTIVE
				|| scope_state == SCOPE_TEST_ARMED
				|| scope_state == SCOPE_TEST_ACTIVE)
			{
				continue;
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
