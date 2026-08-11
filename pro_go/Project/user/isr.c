
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
#if !RACE_MINIMAL_BUILD
#define SCOPE_CAPTURE_DIVIDER 4U

volatile uint8 g_scope_arm_active = 0U;
volatile uint16 g_scope_arm_ticks = 0U;
volatile uint8 g_scope_capture_enabled = 0U;
volatile uint8 g_scope_test_mode = 0U;
volatile uint8 g_scope_snapshot_ready = 0U;
volatile float xdata g_scope_snapshot[8];
static uint8 scope_capture_ticks = 0U;
static uint8 scope_test_phase = 0U;
#endif

/* T12 is an explicit half-circle test mode; T10 never enters this state machine. */
#define TRACK_T12_APPROACH                 0U
#define TRACK_T12_ENTRY_CONFIRM            1U
#define TRACK_T12_HALF                     2U
#define TRACK_T12_HANDOFF                  3U
#define TRACK_T12_ARM_CONFIRM_TICKS        3U
#define TRACK_T12_ENTRY_CONFIRM_TICKS      3U
#define TRACK_T12_EXIT_CONFIRM_TICKS       2U
#define TRACK_T12_ENTRY_TIMEOUT_TICKS    100U
#define TRACK_T12_HALF_MIN_TICKS           40U
#define TRACK_T12_HALF_ANGLE_MIN         160.0f
#define TRACK_T12_HALF_ANGLE_EXIT         195.0f
#define TRACK_T12_HALF_MAX_TICKS          230U
#define TRACK_T12_HALF_RAMP_TICKS          24U
#define TRACK_T12_HANDOFF_TICKS            15U
#define TRACK_T12_HALF_ENTRY_RATIO          0.22f
#define TRACK_T12_HALF_RATIO                0.285f
#define TRACK_T12_HALF_TAPER_START_DEG     110.0f
#define TRACK_T12_HALF_TAPER_END_DEG       170.0f
#define TRACK_T12_HALF_TAIL_RATIO            0.16f
#define TRACK_T12_EXIT_RATIO                0.08f
#define TRACK_T12_LINE_FEEDBACK             0.30f
#define TRACK_T12_TURN_FILTER_ALPHA          0.25f
#define TRACK_T12_START_SYNC_GAIN            0.45f
#define TRACK_T12_START_SYNC_LIMIT           0.25f
#define TRACK_T12_START_BALANCE_X1000      700U
#define TRACK_T12_START_FALLBACK_X1000     450U
#define TRACK_T12_START_FALLBACK_SAMPLES    40U
#define TRACK_T12_ENTRY_SUM_MIN             80U
#define TRACK_T12_ENTRY_INNER_MIN            8U
#define TRACK_T12_ENTRY_SIDE_DIFF           30U
#define TRACK_T12_ENTRY_ERROR_MIN            0.22f
#define TRACK_T12_CONFIRM_SUM_MIN           70U
#define TRACK_T12_CONFIRM_INNER_MIN          6U
#define TRACK_T12_CONFIRM_SIDE_DIFF         20U
#define TRACK_T12_CONFIRM_ERROR_MIN          0.18f

static uint8 track_t12_state = TRACK_T12_APPROACH;
static uint8 track_t12_arm_ticks = 0U;
static uint8 track_t12_entry_ticks = 0U;
static uint8 track_t12_exit_ticks = 0U;
static uint8 track_t12_exit_trigger_pending = 0U;
static uint16 track_t12_state_ticks = 0U;
static uint16 track_t12_entry_timeout = 0U;
static float track_t12_angle = 0.0f;
static float track_t12_turn_filtered = 0.0f;

volatile uint8 xdata g_track_t12_start_release_reason = 0U;
volatile uint16 xdata g_track_t12_start_release_sample_count = 0U;
volatile uint32 xdata g_track_t12_start_release_left_total = 0UL;
volatile uint32 xdata g_track_t12_start_release_right_total = 0UL;
volatile uint16 xdata g_track_t12_approach_max_sum = 0U;
volatile uint16 xdata g_track_t12_approach_max_error_x1000 = 0U;
volatile uint16 xdata g_track_t12_approach_max_side_diff = 0U;
volatile uint8 xdata g_track_t12_entry_source = TRACK_T12_ENTRY_SOURCE_NONE;
volatile uint8 xdata g_track_t12_entry_norm_l = 0U;
volatile uint8 xdata g_track_t12_entry_norm_lm = 0U;
volatile uint8 xdata g_track_t12_entry_norm_rm = 0U;
volatile uint8 xdata g_track_t12_entry_norm_r = 0U;
volatile int16 xdata g_track_t12_entry_error_x1000 = 0;
volatile uint16 xdata g_track_t12_entry_sum = 0U;
volatile uint8 xdata g_track_t12_exit_trigger_mask = 0U;
volatile uint16 xdata g_track_t12_exit_angle_x10 = 0U;
volatile uint16 xdata g_track_t12_exit_half_ticks = 0U;
volatile uint8 xdata g_track_t12_exit_norm_l = 0U;
volatile uint8 xdata g_track_t12_exit_norm_lm = 0U;
volatile uint8 xdata g_track_t12_exit_norm_rm = 0U;
volatile uint8 xdata g_track_t12_exit_norm_r = 0U;
volatile int16 xdata g_track_t12_exit_error_x1000 = 0;
volatile uint16 xdata g_track_t12_exit_sum = 0U;
volatile uint8 xdata g_track_t12_post_valid = 0U;
volatile uint8 xdata g_track_t12_post_delay_ticks = 0U;
volatile uint16 xdata g_track_t12_post_angle_x10 = 0U;
volatile uint8 xdata g_track_t12_post_norm_l = 0U;
volatile uint8 xdata g_track_t12_post_norm_lm = 0U;
volatile uint8 xdata g_track_t12_post_norm_rm = 0U;
volatile uint8 xdata g_track_t12_post_norm_r = 0U;
volatile int16 xdata g_track_t12_post_error_x1000 = 0;
volatile uint16 xdata g_track_t12_post_sum = 0U;

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

static void prime_track_t10_closed_loop(void)
{
    reset_speed_pid_state();
    current_l_pwm_inc = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
    current_r_pwm_inc = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
    current_l_pwm_inc_last = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
    current_r_pwm_inc_last = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
    current_l_pwm_duty = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
    current_r_pwm_duty = TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM;
}

void reset_motion_pid_state(void)
{
    reset_speed_pid_state();
    reset_pid_runtime(&Turn_PID);
}

void reset_track_test_steering_state(void)
{
	track_turn_ratio = 0.0f;
	track_t12_state = TRACK_T12_APPROACH;
	track_t12_arm_ticks = 0U;
	track_t12_entry_ticks = 0U;
	track_t12_exit_ticks = 0U;
	track_t12_exit_trigger_pending = 0U;
	track_t12_state_ticks = 0U;
	track_t12_entry_timeout = 0U;
	track_t12_angle = 0.0f;
	track_t12_turn_filtered = 0.0f;
}

void reset_track_test_exit_diagnostic(void)
{
	g_track_t12_start_release_reason = 0U;
	g_track_t12_start_release_sample_count = 0U;
	g_track_t12_start_release_left_total = 0UL;
	g_track_t12_start_release_right_total = 0UL;
	g_track_t12_approach_max_sum = 0U;
	g_track_t12_approach_max_error_x1000 = 0U;
	g_track_t12_approach_max_side_diff = 0U;
	g_track_t12_entry_source = TRACK_T12_ENTRY_SOURCE_NONE;
	g_track_t12_entry_norm_l = 0U;
	g_track_t12_entry_norm_lm = 0U;
	g_track_t12_entry_norm_rm = 0U;
	g_track_t12_entry_norm_r = 0U;
	g_track_t12_entry_error_x1000 = 0;
	g_track_t12_entry_sum = 0U;
	g_track_t12_exit_trigger_mask = 0U;
	g_track_t12_exit_angle_x10 = 0U;
	g_track_t12_exit_half_ticks = 0U;
	g_track_t12_exit_norm_l = 0U;
	g_track_t12_exit_norm_lm = 0U;
	g_track_t12_exit_norm_rm = 0U;
	g_track_t12_exit_norm_r = 0U;
	g_track_t12_exit_error_x1000 = 0;
	g_track_t12_exit_sum = 0U;
	g_track_t12_post_valid = 0U;
	g_track_t12_post_delay_ticks = 0U;
	g_track_t12_post_angle_x10 = 0U;
	g_track_t12_post_norm_l = 0U;
	g_track_t12_post_norm_lm = 0U;
	g_track_t12_post_norm_rm = 0U;
	g_track_t12_post_norm_r = 0U;
	g_track_t12_post_error_x1000 = 0;
	g_track_t12_post_sum = 0U;
}

static void track_t12_update_approach_diagnostic(void)
{
	uint16 sum = inductance4_get_line_sum();
	uint16 left_pair = (uint16)g_inductance4[INDUCTANCE4_L].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_LM].normalized;
	uint16 right_pair = (uint16)g_inductance4[INDUCTANCE4_RM].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_R].normalized;
	uint16 side_diff;
	uint16 error_x1000 = (uint16)(fabs(error) * 1000.0f);

	if (left_pair > right_pair)
	{
		side_diff = left_pair - right_pair;
	}
	else
	{
		side_diff = right_pair - left_pair;
	}

	if (sum > g_track_t12_approach_max_sum)
	{
		g_track_t12_approach_max_sum = sum;
	}
	if (error_x1000 > g_track_t12_approach_max_error_x1000)
	{
		g_track_t12_approach_max_error_x1000 = error_x1000;
	}
	if (side_diff > g_track_t12_approach_max_side_diff)
	{
		g_track_t12_approach_max_side_diff = side_diff;
	}
}

static void track_t12_start_half(int8 direction, uint8 source)
{
	track_t12_state = TRACK_T12_HALF;
	track_t12_state_ticks = 0U;
	track_t12_exit_ticks = 0U;
	track_t12_exit_trigger_pending = 0U;
	track_t12_angle = 0.0f;
	track_t12_turn_filtered = (float)direction
		* TRACK_T12_HALF_ENTRY_RATIO;
	g_track_test_t12_direction = direction;
	g_track_test_t12_half_active = 1U;
	g_track_t12_entry_source = source;
	g_track_t12_entry_norm_l = (uint8)g_inductance4[INDUCTANCE4_L].normalized;
	g_track_t12_entry_norm_lm = (uint8)g_inductance4[INDUCTANCE4_LM].normalized;
	g_track_t12_entry_norm_rm = (uint8)g_inductance4[INDUCTANCE4_RM].normalized;
	g_track_t12_entry_norm_r = (uint8)g_inductance4[INDUCTANCE4_R].normalized;
	g_track_t12_entry_error_x1000 = (int16)(error * 1000.0f);
	g_track_t12_entry_sum = inductance4_get_line_sum();
}

static int8 track_t12_candidate_direction(void)
{
	uint16 sum = inductance4_get_line_sum();
	uint16 left_pair = (uint16)g_inductance4[INDUCTANCE4_L].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_LM].normalized;
	uint16 right_pair = (uint16)g_inductance4[INDUCTANCE4_RM].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_R].normalized;

	/* Relative pair dominance follows the current 120-160 normal line-sum range. */
	if (!inductance4_line_is_present() || sum < TRACK_T12_ENTRY_SUM_MIN)
	{
		return 0;
	}
	if (g_inductance4[INDUCTANCE4_RM].normalized
			>= TRACK_T12_ENTRY_INNER_MIN
		&& right_pair >= (uint16)(left_pair + TRACK_T12_ENTRY_SIDE_DIFF)
		&& error <= -TRACK_T12_ENTRY_ERROR_MIN)
	{
		return -1;
	}
	if (g_inductance4[INDUCTANCE4_LM].normalized
			>= TRACK_T12_ENTRY_INNER_MIN
		&& left_pair >= (uint16)(right_pair + TRACK_T12_ENTRY_SIDE_DIFF)
		&& error >= TRACK_T12_ENTRY_ERROR_MIN)
	{
		return 1;
	}
	return 0;
}

static uint8 track_t12_entry_matches(int8 direction)
{
	uint16 sum = inductance4_get_line_sum();
	uint16 left_pair = (uint16)g_inductance4[INDUCTANCE4_L].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_LM].normalized;
	uint16 right_pair = (uint16)g_inductance4[INDUCTANCE4_RM].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_R].normalized;

	if (!inductance4_line_is_present() || sum < TRACK_T12_CONFIRM_SUM_MIN)
	{
		return 0U;
	}
	if (direction < 0)
	{
		return g_inductance4[INDUCTANCE4_RM].normalized
				>= TRACK_T12_CONFIRM_INNER_MIN
			&& right_pair >= (uint16)(left_pair + TRACK_T12_CONFIRM_SIDE_DIFF)
			&& error <= -TRACK_T12_CONFIRM_ERROR_MIN;
	}
	return g_inductance4[INDUCTANCE4_LM].normalized
			>= TRACK_T12_CONFIRM_INNER_MIN
		&& left_pair >= (uint16)(right_pair + TRACK_T12_CONFIRM_SIDE_DIFF)
		&& error >= TRACK_T12_CONFIRM_ERROR_MIN;
}

static uint8 track_t12_exit_matches(int8 direction)
{
	uint16 sum = inductance4_get_line_sum();
	uint16 left_pair = (uint16)g_inductance4[INDUCTANCE4_L].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_LM].normalized;
	uint16 right_pair = (uint16)g_inductance4[INDUCTANCE4_RM].normalized
		+ (uint16)g_inductance4[INDUCTANCE4_R].normalized;

	/* A180 is brief in motion; use a wider dynamic window than the static sample. */
	if (sum < 60U || sum > 380U)
	{
		return 0U;
	}
	if (direction < 0)
	{
		return g_inductance4[INDUCTANCE4_RM].normalized <= 100
			&& g_inductance4[INDUCTANCE4_R].normalized <= 75
			&& left_pair >= (uint16)(right_pair + 20U)
			&& error >= 0.25f;
	}
	return g_inductance4[INDUCTANCE4_L].normalized <= 100
		&& g_inductance4[INDUCTANCE4_LM].normalized <= 75
		&& right_pair >= (uint16)(left_pair + 20U)
		&& error <= -0.25f;
}

static uint8 track_t12_exit_trigger_now(void)
{
	uint8 trigger_mask = 0U;

	if (track_t12_angle >= TRACK_T12_HALF_ANGLE_EXIT)
	{
		trigger_mask |= TRACK_T12_EXIT_TRIGGER_ANGLE;
	}
	if (track_t12_state_ticks >= TRACK_T12_HALF_MAX_TICKS)
	{
		trigger_mask |= TRACK_T12_EXIT_TRIGGER_TIME;
	}
	if (track_t12_angle >= TRACK_T12_HALF_ANGLE_MIN
		&& track_t12_exit_matches(g_track_test_t12_direction))
	{
		trigger_mask |= TRACK_T12_EXIT_TRIGGER_SENSOR;
	}
	return trigger_mask;
}

static void track_t12_capture_exit_diagnostic(uint8 post_handoff)
{
	uint16 angle_x10 = (uint16)(track_t12_angle * 10.0f);
	int16 error_x1000 = (int16)(error * 1000.0f);
	uint16 sum = inductance4_get_line_sum();

	if (post_handoff)
	{
		g_track_t12_post_delay_ticks = (uint8)track_t12_state_ticks;
		g_track_t12_post_angle_x10 = angle_x10;
		g_track_t12_post_norm_l = (uint8)g_inductance4[INDUCTANCE4_L].normalized;
		g_track_t12_post_norm_lm = (uint8)g_inductance4[INDUCTANCE4_LM].normalized;
		g_track_t12_post_norm_rm = (uint8)g_inductance4[INDUCTANCE4_RM].normalized;
		g_track_t12_post_norm_r = (uint8)g_inductance4[INDUCTANCE4_R].normalized;
		g_track_t12_post_error_x1000 = error_x1000;
		g_track_t12_post_sum = sum;
		g_track_t12_post_valid = 1U;
		return;
	}

	g_track_t12_exit_angle_x10 = angle_x10;
	g_track_t12_exit_half_ticks = track_t12_state_ticks;
	g_track_t12_exit_norm_l = (uint8)g_inductance4[INDUCTANCE4_L].normalized;
	g_track_t12_exit_norm_lm = (uint8)g_inductance4[INDUCTANCE4_LM].normalized;
	g_track_t12_exit_norm_rm = (uint8)g_inductance4[INDUCTANCE4_RM].normalized;
	g_track_t12_exit_norm_r = (uint8)g_inductance4[INDUCTANCE4_R].normalized;
	g_track_t12_exit_error_x1000 = error_x1000;
	g_track_t12_exit_sum = sum;
}

static uint8 track_t12_start_ready(void)
{
	uint32 smaller_total;
	uint32 larger_total;
	uint8 release_reason = 0U;

	if (g_track_t12_start_release_reason != 0U)
	{
		return 1U;
	}
	if (g_track_test_start_left_total < TRACK_TEST_T12_START_SYNC_RELEASE_COUNT
		|| g_track_test_start_right_total < TRACK_TEST_T12_START_SYNC_RELEASE_COUNT)
	{
		return 0U;
	}

	smaller_total = g_track_test_start_left_total < g_track_test_start_right_total
		? g_track_test_start_left_total : g_track_test_start_right_total;
	larger_total = g_track_test_start_left_total > g_track_test_start_right_total
		? g_track_test_start_left_total : g_track_test_start_right_total;
	if (smaller_total * 1000UL
		>= larger_total * TRACK_T12_START_BALANCE_X1000)
	{
		release_reason = TRACK_T12_START_RELEASE_BALANCED;
	}
	if (release_reason == 0U)
	{
		return 0U;
	}
	if (g_track_t12_start_release_reason == 0U)
	{
		g_track_t12_start_release_sample_count = g_track_test_start_sample_count;
		g_track_t12_start_release_left_total = g_track_test_start_left_total;
		g_track_t12_start_release_right_total = g_track_test_start_right_total;
		g_track_t12_start_release_reason = release_reason;
	}
	return 1U;
}

static float track_line_turn_ratio_raw(float track_error)
{
	uint16 sum = inductance4_get_line_sum();

	/* Keep the Z07 straight heading when Z09 raises only the outer-field sum. */
	if (sum >= 300U
		&& g_inductance4[INDUCTANCE4_LM].normalized < 80
		&& g_inductance4[INDUCTANCE4_RM].normalized < 80
		&& fabs(track_error) >= 0.20f)
	{
		return 0.0f;
	}
	if (fabs(track_error) <= TRACK_TEST_TURN_DEADBAND)
	{
		return 0.0f;
	}
	return limit_function(
		track_error * TRACK_TEST_T12_LINE_TURN_GAIN,
		-TRACK_TEST_TURN_RATIO_LIMIT,
		TRACK_TEST_TURN_RATIO_LIMIT);
}

static float track_t10_test_turn_ratio(float track_error)
{
	if (fabs(track_error) <= TRACK_TEST_TURN_DEADBAND)
	{
		return 0.0f;
	}
	return limit_function(
		track_error * TRACK_TEST_T10_TURN_GAIN,
		-TRACK_TEST_TURN_RATIO_LIMIT,
		TRACK_TEST_TURN_RATIO_LIMIT);
}

#if TRACK_TEST_START_ASSIST_ENABLED
static void track_test_apply_soft_start_sync(void)
{
	uint32 left_total;
	uint32 right_total;
	uint32 larger_total;
	uint32 smaller_total;
	uint16 start_sample_limit;
	float imbalance;
	float trim;

	/* T12 keeps its existing release gate; T10 uses only its short startup
	 * window and then returns to sensor-only steering. */
	if (g_track_test_mode == TRACK_TEST_MODE_T12
		&& g_track_t12_start_release_reason != 0U)
	{
		return;
	}
	start_sample_limit = g_track_test_mode == TRACK_TEST_MODE_T12
		? TRACK_TEST_T12_START_MONITOR_SAMPLES
		: TRACK_TEST_START_SYNC_SAMPLES;
	if (g_track_test_start_sample_count >= start_sample_limit)
	{
		return;
	}
	left_total = g_track_test_start_left_total;
	right_total = g_track_test_start_right_total;
	if (left_total == right_total)
	{
		return;
	}
	larger_total = left_total > right_total ? left_total : right_total;
	smaller_total = left_total < right_total ? left_total : right_total;
	if (larger_total < TRACK_TEST_T10_START_RELEASE_COUNT)
	{
		return;
	}

	imbalance = (float)(larger_total - smaller_total) / (float)larger_total;
	trim = limit_function(
		imbalance * TRACK_T12_START_SYNC_GAIN,
		0.0f,
		TRACK_T12_START_SYNC_LIMIT);
	if (left_total > right_total)
	{
		L_pid.Target *= 1.0f - trim;
		R_pid.Target *= 1.0f + trim;
	}
	else
	{
		L_pid.Target *= 1.0f + trim;
		R_pid.Target *= 1.0f - trim;
	}
}
#endif

static void track_t12_update(void)
{
	if (track_t12_state_ticks < 65535U)
	{
		track_t12_state_ticks++;
	}

	switch (track_t12_state)
	{
		case TRACK_T12_APPROACH:
		{
			int8 candidate = track_t12_candidate_direction();
			track_t12_update_approach_diagnostic();
			if (g_track_test_t12_force_direction != 0)
			{
				track_t12_start_half(
					g_track_test_t12_force_direction,
					TRACK_T12_ENTRY_SOURCE_FORCE);
				break;
			}
			if (candidate != 0)
			{
				if (g_track_test_t12_direction == 0
					|| g_track_test_t12_direction == candidate)
				{
					g_track_test_t12_direction = candidate;
					if (track_t12_arm_ticks < TRACK_T12_ARM_CONFIRM_TICKS)
					{
						track_t12_arm_ticks++;
					}
					if (track_t12_arm_ticks >= TRACK_T12_ARM_CONFIRM_TICKS)
					{
						track_t12_state = TRACK_T12_ENTRY_CONFIRM;
						track_t12_state_ticks = 0U;
						track_t12_entry_timeout = TRACK_T12_ENTRY_TIMEOUT_TICKS;
						track_t12_entry_ticks = 0U;
					}
				}
				else
				{
					track_t12_arm_ticks = 0U;
					g_track_test_t12_direction = 0;
				}
			}
			else
			{
				track_t12_arm_ticks = 0U;
				g_track_test_t12_direction = 0;
			}
			break;
		}

		case TRACK_T12_ENTRY_CONFIRM:
			if (track_t12_entry_timeout > 0U)
			{
				track_t12_entry_timeout--;
			}
			if (track_t12_entry_matches(g_track_test_t12_direction))
			{
				if (track_t12_entry_ticks < TRACK_T12_ENTRY_CONFIRM_TICKS)
				{
					track_t12_entry_ticks++;
				}
				if (track_t12_entry_ticks >= TRACK_T12_ENTRY_CONFIRM_TICKS)
				{
					track_t12_start_half(
						g_track_test_t12_direction,
						TRACK_T12_ENTRY_SOURCE_AUTO);
				}
			}
			else
			{
				track_t12_entry_ticks = 0U;
			}
			if (track_t12_state == TRACK_T12_ENTRY_CONFIRM
				&& track_t12_entry_timeout == 0U)
			{
				track_t12_state = TRACK_T12_APPROACH;
				track_t12_arm_ticks = 0U;
				track_t12_entry_ticks = 0U;
				g_track_test_t12_direction = 0;
			}
			break;

		case TRACK_T12_HALF:
		{
			uint8 exit_trigger_mask = 0U;

			track_t12_angle += fabs(g_imu_turn_rate_dps) * 0.01f;
			if (track_t12_state_ticks >= TRACK_T12_HALF_MIN_TICKS)
			{
				exit_trigger_mask = track_t12_exit_trigger_now();
			}
			if (exit_trigger_mask != 0U)
			{
				track_t12_exit_trigger_pending |= exit_trigger_mask;
				if (track_t12_exit_ticks < TRACK_T12_EXIT_CONFIRM_TICKS)
				{
					track_t12_exit_ticks++;
				}
				if (track_t12_exit_ticks >= TRACK_T12_EXIT_CONFIRM_TICKS)
				{
					g_track_t12_exit_trigger_mask = track_t12_exit_trigger_pending;
					track_t12_capture_exit_diagnostic(0U);
					track_t12_state = TRACK_T12_HANDOFF;
					track_t12_state_ticks = 0U;
				}
			}
			else
			{
				track_t12_exit_ticks = 0U;
				track_t12_exit_trigger_pending = 0U;
			}
			break;
		}

		case TRACK_T12_HANDOFF:
			if (!g_track_t12_post_valid)
			{
				track_t12_angle += fabs(g_imu_turn_rate_dps) * 0.01f;
				if (track_t12_state_ticks >= TRACK_T12_HANDOFF_TICKS)
				{
					track_t12_capture_exit_diagnostic(1U);
				}
			}
			break;
		default:
			track_t12_state = TRACK_T12_APPROACH;
			break;
	}
}

static float track_t12_turn_ratio(float track_error)
{
	float half_ratio;
	float handoff_ratio;
	float requested_ratio;
	float taper;

	if (!track_t12_start_ready())
	{
		/* Keep the Z09 approach tangent until the T12 release gate is ready. */
		track_t12_turn_filtered = 0.0f;
		return 0.0f;
	}

	if (track_t12_state == TRACK_T12_HALF)
	{
		half_ratio = TRACK_T12_HALF_ENTRY_RATIO;
		if (track_t12_state_ticks < TRACK_T12_HALF_RAMP_TICKS)
		{
			half_ratio += (TRACK_T12_HALF_RATIO - TRACK_T12_HALF_ENTRY_RATIO)
				* (float)track_t12_state_ticks / (float)TRACK_T12_HALF_RAMP_TICKS;
		}
		else
		{
			half_ratio = TRACK_T12_HALF_RATIO;
		}
		/* Ease the latter half of the arc so the exit does not overshoot. */
		if (track_t12_angle > TRACK_T12_HALF_TAPER_START_DEG)
		{
			taper = (track_t12_angle - TRACK_T12_HALF_TAPER_START_DEG)
				/ (TRACK_T12_HALF_TAPER_END_DEG
					- TRACK_T12_HALF_TAPER_START_DEG);
			taper = limit_function(taper, 0.0f, 1.0f);
			taper = taper * taper * (3.0f - 2.0f * taper);
			half_ratio -= (half_ratio - TRACK_T12_HALF_TAIL_RATIO) * taper;
		}
		requested_ratio = (float)g_track_test_t12_direction * half_ratio
			+ TRACK_T12_LINE_FEEDBACK * track_line_turn_ratio_raw(track_error);
	}
	else if (track_t12_state == TRACK_T12_HANDOFF
		&& track_t12_state_ticks < TRACK_T12_HANDOFF_TICKS)
	{
		handoff_ratio = (float)(TRACK_T12_HANDOFF_TICKS - track_t12_state_ticks)
			/ (float)TRACK_T12_HANDOFF_TICKS;
		requested_ratio = -(float)g_track_test_t12_direction * TRACK_T12_EXIT_RATIO
			* handoff_ratio + track_line_turn_ratio_raw(track_error)
			* (1.0f - handoff_ratio);
	}
	else
	{
		requested_ratio = track_line_turn_ratio_raw(track_error);
	}

	track_t12_turn_filtered += TRACK_T12_TURN_FILTER_ALPHA
		* (requested_ratio - track_t12_turn_filtered);
	return track_t12_turn_filtered;
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
			float override_left_target;
			float override_right_target;
			float track_base_target;
			float left_pid_delta;
			float right_pid_delta;
			float left_speed_feedback;
			float right_speed_feedback;
			float track_t10_sync_ratio;
			uint8 track_t10_start_was_active;

			TIM1_CLEAR_FLAG;
			#if !RACE_MINIMAL_BUILD
			if (g_scope_arm_active && g_scope_arm_ticks > 0U)
			{
				g_scope_arm_ticks--;
			}
			#endif

//			angle_project(100);
		
		/********************* Sensor acquisition and safety ********************/
			
			acquire_sensor_data();
			negative_pressure_tick();
			if (motion_runtime_stall_diag_is_active())
			{
				motion_runtime_stall_diag_tick();
				return;
			}
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

			if (motion_runtime_track_test_is_active())
			{
				line_wait_active = 0U;
			}
			else if (line_guard_required() && !inductance4_line_is_present())
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
			if (motion_runtime_track_test_is_active())
			{
				/* T10 uses immediate bounded P steering around the proven speed PI. */
				track_line_speed_scale = 1.0f;
				track_base_target = L_pid.Target_base;
				if (g_track_test_mode == TRACK_TEST_MODE_T10)
				{
					track_t10_start_was_active =
						motion_runtime_track_t10_startup_is_active();
					motion_runtime_track_t10_startup_tick();
					if (track_t10_start_was_active
						&& !motion_runtime_track_t10_startup_is_active())
					{
						prime_track_t10_closed_loop();
					}
				}
#if TRACK_TEST_STEERING_ENABLED
				if (g_track_test_mode == TRACK_TEST_MODE_T12)
				{
					track_turn_ratio = track_t12_turn_ratio(error);
				}
				else
				{
					track_turn_ratio = track_t10_test_turn_ratio(error)
						* motion_runtime_track_t10_steering_scale();
				}
#else
				track_turn_ratio = 0.0f;
#endif
				L_pid.Target = track_base_target * (1.0f - track_turn_ratio);
				R_pid.Target = track_base_target * (1.0f + track_turn_ratio);
				if (g_track_test_mode == TRACK_TEST_MODE_T10)
				{
					track_t10_sync_ratio = motion_runtime_track_t10_sync_ratio();
					L_pid.Target *= 1.0f - track_t10_sync_ratio;
					R_pid.Target *= 1.0f + track_t10_sync_ratio;
				}
#if TRACK_TEST_START_ASSIST_ENABLED
				if (g_track_test_mode == TRACK_TEST_MODE_T12)
				{
					track_test_apply_soft_start_sync();
				}
#endif
			}
			else if (element4_get_speed_override(&override_left_target, &override_right_target))
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

			/* Track tests only command forward motion. Magnitude feedback prevents
			 * a transient encoder phase sample from turning overspeed into acceleration. */
			if (motion_runtime_track_test_is_active())
			{
				left_speed_feedback = fabs(l_speed_now);
				right_speed_feedback = fabs(r_speed_now);
			}
			else
			{
				left_speed_feedback = l_speed_now;
				right_speed_feedback = r_speed_now;
			}

			left_pid_delta = motion_runtime_limit_pid_delta(
				IncPID(left_speed_feedback, L_pid.Target, &L_pid));
			right_pid_delta = motion_runtime_limit_pid_delta(
				IncPID(right_speed_feedback, R_pid.Target, &R_pid));
			current_l_pwm_inc = current_l_pwm_inc + left_pid_delta;
			current_r_pwm_inc = current_r_pwm_inc + right_pid_delta;

			current_l_pwm_inc = current_l_pwm_inc_last * 0.2f + current_l_pwm_inc * 0.8f;
			current_r_pwm_inc = current_r_pwm_inc_last * 0.2f + current_r_pwm_inc * 0.8f;

			current_l_pwm_inc = limit_function(
				current_l_pwm_inc,
				-MOTOR_PWM_LIMIT_VALUE,
				MOTOR_PWM_LIMIT_VALUE);
			current_r_pwm_inc = limit_function(
				current_r_pwm_inc,
				-MOTOR_PWM_LIMIT_VALUE,
				MOTOR_PWM_LIMIT_VALUE);

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
#if TRACK_TEST_START_ASSIST_ENABLED
		if (motion_runtime_track_t10_startup_is_active()
			&& L_pid.Target_base > 0.0f)
		{
			current_l_pwm_inc = motion_runtime_track_t10_left_start_pwm();
			current_r_pwm_inc = motion_runtime_track_t10_right_start_pwm();
			current_l_pwm_inc_last = current_l_pwm_inc;
			current_r_pwm_inc_last = current_r_pwm_inc;
			current_l_pwm_duty = current_l_pwm_inc;
			current_r_pwm_duty = current_r_pwm_inc;
		}
		if (motion_runtime_track_test_is_active()
			&& g_track_test_mode == TRACK_TEST_MODE_T12
			&& L_pid.Target_base > 0.0f)
		{
			if (g_track_test_start_sample_count < TRACK_TEST_T12_START_MONITOR_SAMPLES)
			{
				/* Each wheel gets the same slew-limited breakaway floor independently. */
				if (g_track_test_start_left_total < TRACK_TEST_T12_START_SYNC_RELEASE_COUNT
					&& current_l_pwm_duty < TRACK_TEST_START_BREAKAWAY_PWM)
				{
					current_l_pwm_duty = TRACK_TEST_START_BREAKAWAY_PWM;
				}
				if (g_track_test_start_right_total < TRACK_TEST_T12_START_SYNC_RELEASE_COUNT
					&& current_r_pwm_duty < TRACK_TEST_START_BREAKAWAY_PWM)
				{
					current_r_pwm_duty = TRACK_TEST_START_BREAKAWAY_PWM;
				}
			}
		}
#endif
		
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

                current_l_pwm_inc=limit_function(
                    current_l_pwm_inc,
                    -MOTOR_PWM_LIMIT_VALUE,
                    MOTOR_PWM_LIMIT_VALUE);
                current_r_pwm_inc=limit_function(
                    current_r_pwm_inc,
                    -MOTOR_PWM_LIMIT_VALUE,
                    MOTOR_PWM_LIMIT_VALUE);
            }
        #endif
		
		
        current_l_pwm_duty=limit_function(
            current_l_pwm_duty,
            -MOTOR_PWM_LIMIT_VALUE,
            MOTOR_PWM_LIMIT_VALUE);
		current_r_pwm_duty=limit_function(
            current_r_pwm_duty,
            -MOTOR_PWM_LIMIT_VALUE,
            MOTOR_PWM_LIMIT_VALUE);

		motion_runtime_check_feedback(
			L_pid.Target,
			R_pid.Target,
			l_speed_now,
			r_speed_now,
			g_motor_left_applied_pwm,
			g_motor_right_applied_pwm,
			(uint8)(pwm_state == 1U && !line_wait_active));


		
	  /********************* Run-state protection ********************/

				//protect
		#if !RACE_MINIMAL_BUILD
		if (!motion_runtime_track_test_is_active() && !g_scope_arm_active)
		{
			key_scan_cycle_pwm_state();
		}
		#endif

		if(pwm_state==2){
		mot_inc=0;
		current_l_pwm_inc=0;
		current_r_pwm_inc=0;
		}	
		
	 /********************* Motor PWM output ********************/
		out_pwm();
		#if !RACE_MINIMAL_BUILD
		if (g_scope_capture_enabled
			&& (motion_runtime_track_test_is_active() || g_scope_test_mode))
		{
			if (++scope_capture_ticks >= SCOPE_CAPTURE_DIVIDER)
			{
				scope_capture_ticks = 0U;
				if (g_scope_test_mode)
				{
					g_scope_snapshot[0] = (float)scope_test_phase;
					g_scope_snapshot[1] = 100.0f - (float)scope_test_phase;
					g_scope_snapshot[2] = 20.0f;
					g_scope_snapshot[3] = 40.0f;
					g_scope_snapshot[4] = 60.0f;
					g_scope_snapshot[5] = 80.0f;
					g_scope_snapshot[6] = 30.0f;
					g_scope_snapshot[7] = 70.0f;
					if (++scope_test_phase > 100U)
					{
						scope_test_phase = 0U;
					}
				}
				else
				{
					g_scope_snapshot[0] = L_pid.Target_base;
					g_scope_snapshot[1] = l_speed_now;
					g_scope_snapshot[2] = L_pid.Target_base;
					g_scope_snapshot[3] = r_speed_now;
					g_scope_snapshot[4] = g_motor_left_applied_pwm / 10.0f;
					g_scope_snapshot[5] = g_motor_right_applied_pwm / 10.0f;
					g_scope_snapshot[6] = (float)g_encoder_left_raw;
					g_scope_snapshot[7] = (float)g_encoder_right_raw;
				}
				g_scope_snapshot_ready = 1U;
			}
		}
		else
		{
			scope_capture_ticks = 0U;
			scope_test_phase = 0U;
		}
		#endif
		motion_runtime_track_test_tick();
	
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
			if (motion_runtime_track_test_is_active()
				&& g_track_test_mode == TRACK_TEST_MODE_T12
				&& track_t12_start_ready())
			{
				track_t12_update();
			}

			if (motion_runtime_track_test_is_active())
			{
				Turn_PID.err = error;
				Turn_PID.out = 0.0f;
				Turn_PID.last = 0.0f;
				Turn_PID.err_last = error;
			}
			else
			{
			Turn_PID.err=error;
			Turn_PID.out=Turn_PID.kp*Turn_PID.err+
							 Turn_PID.ki*Turn_PID.err*fabs(Turn_PID.err)*2+
								 Turn_PID.kd*(Turn_PID.err-Turn_PID.err_last)+
								 Turn_PID.kp1*gyro_data[0];
			Turn_PID.last=Turn_PID.out;
			Turn_PID.out=limit_function(Turn_PID.out,-dir_loop_limit,dir_loop_limit);

			Turn_PID.err_last=Turn_PID.err;
			}
		}


		TIM4_CLEAR_FLAG;
}







