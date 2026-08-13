#include "headfile.h"

#define IMU_CALIBRATION_DISCARD_SAMPLES    20U
#define IMU_CALIBRATION_MAX_SPAN_DPS       12.0f
#define IMU_CALIBRATION_MAX_BIAS_DPS       30.0f

#define MOTOR_PWM_LIMIT           MOTOR_PWM_LIMIT_VALUE
#define MOTOR_REVERSE_DEADTIME_TICKS       10U
#define MOTOR_STALL_PWM_MIN               350.0f
#define MOTOR_STALL_TARGET_MIN             20.0f
#define MOTOR_STALL_RAW_MAX                 1U
#define MOTOR_STALL_TICKS                  60U
#define MOTOR_DIRECTION_SPEED_MIN           5.0f
#define MOTOR_DIRECTION_TARGET_MIN         20.0f
#define MOTOR_DIRECTION_FAULT_TICKS        20U
#define MOTOR_SATURATION_THRESHOLD (MOTOR_PWM_LIMIT_VALUE - 5.0f)
#define MOTOR_SATURATION_FAULT_TICKS      100U
#define ENCODER_RAW_SPIKE_LIMIT         10000U
#define ENCODER_SPIKE_FAULT_TICKS           3U

#define MOTOR_TEST_CONTROL_PERIOD_MS           5U
#define RAW_DRIVE_DURATION_TICKS       (RAW_DRIVE_DURATION_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define MOTOR_TEST_PWM                    ((float)MOTOR_TEST_PWM_VALUE)
#define MOTOR_TEST_BOTH_PWM               ((float)MOTOR_TEST_BOTH_PWM_VALUE)
#define MOTOR_TEST_DURATION_TICKS         (MOTOR_TEST_DURATION_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define MOTOR_TEST_PRECHECK_TICKS     (MOTOR_TEST_PRECHECK_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define MOTOR_TEST_BOOST_DURATION_TICKS (MOTOR_TEST_BOOST_DURATION_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define MOTOR_TEST_BOOST_STALL_GRACE_TICKS   10U
#define MOTOR_TEST_STALL_TICKS               20U
#define MOTOR_TEST_DIRECTION_FAULT_TICKS      3U
#define MOTOR_TEST_NOISE_FAULT_TICKS          3U
#define MOTOR_TEST_IDLE_RAW_MAX               4U
#define MOTOR_TEST_RUNNING_RAW_MAX         2000U
#define MOTOR_TEST_UNTESTED_RAW_MAX          20U
#define MOTOR_TEST_ENCODER_MODE_MASK        0x03U
#define STALL_DIAG_PRECHECK_TICKS  (STALL_DIAG_PRECHECK_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define STALL_DIAG_STAGE_TICKS        (STALL_DIAG_STAGE_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define STALL_DIAG_MOVE_TOTAL_MIN          8UL
#define STALL_DIAG_MOVE_PEAK_MIN             2U
#define STALL_DIAG_IDLE_RAW_MAX               4U
#define STALL_DIAG_NOISE_FAULT_TICKS          3U
#define ENCODER_TEST_DURATION_TICKS     (ENCODER_TEST_DURATION_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define TRACK_TEST_DURATION_TICKS       (TRACK_TEST_DURATION_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define TRACK_TEST_LINE_LOST_TICKS      (TRACK_TEST_LINE_LOST_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define TRACK_TEST_STARTUP_GRACE_TICKS  (TRACK_TEST_STARTUP_GRACE_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#if TRACK_TEST_START_ASSIST_ENABLED
#define TRACK_TEST_TARGET_RAMP_TICKS    (TRACK_TEST_TARGET_RAMP_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define TRACK_TEST_DECEL_RAMP_TICKS     (TRACK_TEST_DECEL_RAMP_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#define TRACK_TEST_T10_STEER_BLEND_TICKS (TRACK_TEST_T10_STEER_BLEND_MS / MOTOR_TEST_CONTROL_PERIOD_MS)
#endif
#define TRACK_TEST_SPEED_KP                 5.9f
#define TRACK_TEST_SPEED_KI                 0.50f
#if TRACK_TEST_START_ASSIST_ENABLED
#define TRACK_TEST_PID_RISE_LIMIT          40.0f
#define TRACK_TEST_PID_FALL_LIMIT          80.0f
#endif

#if RAW_DRIVE_PWM_VALUE > PWM_DUTY_MAX
#error RAW drive PWM exceeds the driver duty scale.
#endif

#if MOTOR_TEST_BOOST_PWM_VALUE > PWM_DUTY_MAX
#error Motor-test boost PWM exceeds the driver duty scale.
#endif

typedef struct
{
    float applied_abs_pwm;
    int8 applied_sign;
    int8 pending_sign;
    uint8 deadtime_ticks;
} MotorOutputState;

volatile uint8 g_imu_runtime_state = IMU_RUNTIME_MISSING;
volatile uint8 g_motion_run_unlocked = 0U;
volatile uint8 g_motion_protect_reason = MOTION_PROTECT_NONE;

float g_imu_gyro_x_dps = 0.0f;
float g_imu_gyro_y_dps = 0.0f;
float g_imu_gyro_z_dps = 0.0f;
float g_imu_bias_x_dps = 0.0f;
float g_imu_bias_y_dps = 0.0f;
float g_imu_bias_z_dps = 0.0f;
float g_imu_turn_rate_dps = 0.0f;

uint16 g_encoder_left_raw = 0U;
uint16 g_encoder_right_raw = 0U;
int32 g_encoder_left_signed = 0;
int32 g_encoder_right_signed = 0;
uint8 g_encoder_left_phase = 0U;
uint8 g_encoder_right_phase = 0U;

float g_motor_left_applied_pwm = 0.0f;
float g_motor_right_applied_pwm = 0.0f;
uint16 g_motor_left_saturation_count = 0U;
uint16 g_motor_right_saturation_count = 0U;
uint16 g_motor_left_reversal_count = 0U;
uint16 g_motor_right_reversal_count = 0U;

volatile uint8 g_motor_test_side = MOTOR_TEST_SIDE_NONE;
volatile uint8 g_motor_test_result = MOTOR_TEST_RESULT_IDLE;
volatile uint16 g_motor_test_ticks_remaining = 0U;

volatile uint8 g_encoder_test_side = MOTOR_TEST_SIDE_NONE;
volatile uint8 g_encoder_test_result = ENCODER_TEST_RESULT_IDLE;
volatile uint16 g_encoder_test_ticks_remaining = 0U;

float g_track_duty_limit = 0.45f;
float g_speed_pid_delta_limit = 40.0f;
float g_motor_pwm_slew_per_tick = 25.0f;

static MotorOutputState left_output_state = {0.0f, 0, 0, 0U};
static MotorOutputState right_output_state = {0.0f, 0, 0, 0U};
static volatile uint8 raw_drive_active = 0U;
static volatile uint8 raw_drive_event = RAW_DRIVE_EVENT_NONE;
static volatile uint16 raw_drive_ticks_remaining = 0U;
static volatile uint16 raw_drive_sample_count = 0U;
static volatile uint32 raw_drive_left_total = 0U;
static volatile uint32 raw_drive_right_total = 0U;
static volatile uint16 raw_drive_left_peak = 0U;
static volatile uint16 raw_drive_right_peak = 0U;

static uint8 left_stall_ticks = 0U;
static uint8 right_stall_ticks = 0U;
static uint8 left_direction_fault_ticks = 0U;
static uint8 right_direction_fault_ticks = 0U;
static uint8 left_saturation_ticks = 0U;
static uint8 right_saturation_ticks = 0U;
static uint8 encoder_spike_ticks = 0U;
static uint8 motor_test_left_stall_ticks = 0U;
static uint8 motor_test_right_stall_ticks = 0U;
static uint8 motor_test_left_direction_fault_ticks = 0U;
static uint8 motor_test_right_direction_fault_ticks = 0U;
static uint8 motor_test_noise_ticks = 0U;
static uint8 motor_test_precheck_ticks_remaining = 0U;
static volatile uint8 motor_test_active = 0U;
static volatile uint8 motor_test_event = MOTOR_TEST_RESULT_IDLE;
static volatile uint32 motor_test_left_total = 0U;
static volatile uint32 motor_test_right_total = 0U;
static volatile uint16 motor_test_left_peak = 0U;
static volatile uint16 motor_test_right_peak = 0U;
static volatile uint16 motor_test_left_idle_peak = 0U;
static volatile uint16 motor_test_right_idle_peak = 0U;
static volatile uint8 motor_test_boost_active = 0U;
static volatile uint8 motor_test_boost_ticks_remaining = 0U;
static volatile uint16 motor_test_boost_sample_count = 0U;
static volatile uint32 motor_test_boost_left_total = 0U;
static volatile uint32 motor_test_boost_right_total = 0U;
static volatile uint16 motor_test_boost_left_peak = 0U;
static volatile uint16 motor_test_boost_right_peak = 0U;
static volatile uint16 motor_test_hold_sample_count = 0U;
static volatile uint32 motor_test_hold_left_total = 0U;
static volatile uint32 motor_test_hold_right_total = 0U;
static volatile uint16 motor_test_hold_left_peak = 0U;
static volatile uint16 motor_test_hold_right_peak = 0U;
static volatile uint8 stall_diag_active = 0U;
static volatile uint8 stall_diag_event = STALL_DIAG_RESULT_IDLE;
static volatile uint8 stall_diag_requested_side = MOTOR_TEST_SIDE_NONE;
static volatile uint8 stall_diag_active_side = MOTOR_TEST_SIDE_NONE;
static volatile uint8 stall_diag_stage = 0U;
static volatile uint8 stall_diag_stage_ticks_remaining = 0U;
static volatile uint8 stall_diag_noise_ticks = 0U;
static volatile uint8 stall_diag_pass_mask = 0U;
static volatile uint8 g_stall_diag_result = STALL_DIAG_RESULT_IDLE;
static volatile uint16 stall_diag_applied_pwm = 0U;
static volatile uint32 stall_diag_active_total = 0UL;
static volatile uint32 stall_diag_left_total = 0UL;
static volatile uint32 stall_diag_right_total = 0UL;
static volatile uint16 stall_diag_active_peak = 0U;
static volatile uint16 stall_diag_left_peak = 0U;
static volatile uint16 stall_diag_right_peak = 0U;
static volatile uint16 stall_diag_left_breakaway_pwm = 0U;
static volatile uint16 stall_diag_right_breakaway_pwm = 0U;
static volatile uint8 encoder_test_active = 0U;
static volatile uint8 encoder_test_event = ENCODER_TEST_RESULT_IDLE;
static volatile uint32 encoder_test_left_total = 0U;
static volatile uint32 encoder_test_right_total = 0U;
static volatile uint16 encoder_test_left_peak = 0U;
static volatile uint16 encoder_test_right_peak = 0U;
volatile uint8 g_track_test_result = TRACK_TEST_RESULT_IDLE;
volatile uint8 g_track_test_mode = TRACK_TEST_MODE_T10;
volatile int8 g_track_test_t12_direction = 0;
volatile int8 g_track_test_t12_force_direction = 0;
volatile uint8 g_track_test_t12_half_active = 0U;
volatile uint16 g_track_test_ticks_remaining = 0U;
volatile uint16 g_track_test_start_sample_count = 0U;
volatile uint32 g_track_test_start_left_total = 0U;
volatile uint32 g_track_test_start_right_total = 0U;
volatile uint8 g_track_test_t10_start_release_mask = 0U;
volatile uint16 g_track_test_t10_start_release_sample_count = 0U;
volatile uint32 g_track_test_t10_start_release_left_total = 0U;
volatile uint32 g_track_test_t10_start_release_right_total = 0U;
volatile uint8 g_track_test_t10_start_stage = 0U;
volatile uint16 g_track_test_t10_start_peak_pwm = 0U;
volatile uint8 g_track_test_t10_right_start_stage = 0U;
volatile uint16 g_track_test_t10_right_start_peak_pwm = 0U;
static volatile uint8 track_test_active = 0U;
static volatile uint8 track_test_event = TRACK_TEST_RESULT_IDLE;
static volatile uint8 motor_test_both_passed = 0U;
static volatile uint8 track_test_line_lost_ticks = 0U;
static volatile uint16 track_test_sample_count = 0U;
static float track_test_left_speed_sum = 0.0f;
static float track_test_right_speed_sum = 0.0f;
static float track_test_left_speed_final = 0.0f;
static float track_test_right_speed_final = 0.0f;
static uint16 track_test_left_pwm_final = 0U;
static uint16 track_test_right_pwm_final = 0U;
#if TRACK_TEST_START_ASSIST_ENABLED
static float track_test_ramped_target = 0.0f;
static float track_test_target_value = 0.0f;
static float track_test_ramp_step = 0.0f;
static float track_test_decel_step = 0.0f;
static uint8 track_test_t10_start_active = 0U;
static uint8 track_test_t10_left_confirm_ticks = 0U;
static uint8 track_test_t10_right_confirm_ticks = 0U;
static uint8 track_test_t10_steer_blend_ticks = 0U;
#endif
static uint8 track_test_pid_saved = 0U;
static float track_test_saved_left_kp = 0.0f;
static float track_test_saved_left_ki = 0.0f;
static float track_test_saved_left_kd = 0.0f;
static float track_test_saved_right_kp = 0.0f;
static float track_test_saved_right_ki = 0.0f;
static float track_test_saved_right_kd = 0.0f;

static float motion_abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float motion_clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int8 motion_sign_float(float value)
{
    if (value > 0.0f)
    {
        return 1;
    }
    if (value < 0.0f)
    {
        return -1;
    }
    return 0;
}

static float motion_approach(float current, float target, float maximum_step)
{
    if (current < target)
    {
        current += maximum_step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= maximum_step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static void motion_motor_test_finish(
    MotorTestResult result,
    MotionProtectReason protect_reason,
    uint8 post_event)
{
    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        motor_test_both_passed = (uint8)(
            result == MOTOR_TEST_RESULT_DONE
            && protect_reason == MOTION_PROTECT_NONE);
    }
    motor_test_active = 0U;
    g_motor_test_ticks_remaining = 0U;
    motor_test_left_stall_ticks = 0U;
    motor_test_right_stall_ticks = 0U;
    motor_test_left_direction_fault_ticks = 0U;
    motor_test_right_direction_fault_ticks = 0U;
    motor_test_noise_ticks = 0U;
    motor_test_precheck_ticks_remaining = 0U;
    motor_test_boost_active = 0U;
    motor_test_boost_ticks_remaining = 0U;
    motion_runtime_force_stop();
    g_motor_test_result = (uint8)result;
    if (post_event)
    {
        motor_test_event = (uint8)result;
    }
    if (protect_reason != MOTION_PROTECT_NONE)
    {
        motion_runtime_trigger_protection(protect_reason);
    }
}

static void motion_encoder_test_finish(
    EncoderTestResult result,
    uint8 post_event)
{
    encoder_test_active = 0U;
    g_encoder_test_ticks_remaining = 0U;
    motion_runtime_force_stop();
    g_encoder_test_result = (uint8)result;
    if (post_event)
    {
        encoder_test_event = (uint8)result;
    }
}

static void motion_track_test_finish(
    TrackTestResult result,
    uint8 post_event)
{
    track_test_active = 0U;
    g_track_test_ticks_remaining = 0U;
    track_test_line_lost_ticks = 0U;
    g_motion_run_unlocked = 0U;
    pwm_state = 0U;
    Pwmout = 0U;
    change_speed_Target_base(0);

    if (track_test_pid_saved)
    {
        L_pid.kp = track_test_saved_left_kp;
        L_pid.ki = track_test_saved_left_ki;
        L_pid.kd = track_test_saved_left_kd;
        R_pid.kp = track_test_saved_right_kp;
        R_pid.ki = track_test_saved_right_ki;
        R_pid.kd = track_test_saved_right_kd;
        track_test_pid_saved = 0U;
    }

    reset_motion_pid_state();
    reset_track_test_steering_state();
    motion_runtime_force_stop();
    g_track_test_result = (uint8)result;
    if (post_event)
    {
        track_test_event = (uint8)result;
    }
}

static void motion_reset_feedback_guards(void)
{
    left_stall_ticks = 0U;
    right_stall_ticks = 0U;
    left_direction_fault_ticks = 0U;
    right_direction_fault_ticks = 0U;
    left_saturation_ticks = 0U;
    right_saturation_ticks = 0U;
    encoder_spike_ticks = 0U;
}

uint8 motion_runtime_init_imu(void)
{
    g_imu_runtime_state = IMU_RUNTIME_MISSING;
    g_imu_turn_rate_dps = 0.0f;
    gyro_data[0] = 0.0f;

    if (imu660rb_init() != 0U)
    {
        return 0U;
    }

    g_imu_runtime_state = IMU_RUNTIME_CALIBRATING;
    return 1U;
}

uint8 motion_runtime_calibrate_imu(uint16 sample_count, uint16 sample_delay_ms)
{
    uint16 sample;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float min_x = 100000.0f;
    float min_y = 100000.0f;
    float min_z = 100000.0f;
    float max_x = -100000.0f;
    float max_y = -100000.0f;
    float max_z = -100000.0f;

    if (g_imu_runtime_state == IMU_RUNTIME_MISSING || sample_count == 0U)
    {
        return 0U;
    }

    g_imu_runtime_state = IMU_RUNTIME_CALIBRATING;
    gyro_data[0] = 0.0f;

    for (sample = 0U; sample < IMU_CALIBRATION_DISCARD_SAMPLES; sample++)
    {
        imu660rb_get_gyro();
        system_delay_ms(sample_delay_ms);
    }

    for (sample = 0U; sample < sample_count; sample++)
    {
        imu660rb_get_gyro();
        g_imu_gyro_x_dps = imu660rb_gyro_transition(imu660rb_gyro_x);
        g_imu_gyro_y_dps = imu660rb_gyro_transition(imu660rb_gyro_y);
        g_imu_gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

        sum_x += g_imu_gyro_x_dps;
        sum_y += g_imu_gyro_y_dps;
        sum_z += g_imu_gyro_z_dps;

        if (g_imu_gyro_x_dps < min_x) min_x = g_imu_gyro_x_dps;
        if (g_imu_gyro_y_dps < min_y) min_y = g_imu_gyro_y_dps;
        if (g_imu_gyro_z_dps < min_z) min_z = g_imu_gyro_z_dps;
        if (g_imu_gyro_x_dps > max_x) max_x = g_imu_gyro_x_dps;
        if (g_imu_gyro_y_dps > max_y) max_y = g_imu_gyro_y_dps;
        if (g_imu_gyro_z_dps > max_z) max_z = g_imu_gyro_z_dps;

        system_delay_ms(sample_delay_ms);
    }

    g_imu_bias_x_dps = sum_x / (float)sample_count;
    g_imu_bias_y_dps = sum_y / (float)sample_count;
    g_imu_bias_z_dps = sum_z / (float)sample_count;

    if ((max_x - min_x) > IMU_CALIBRATION_MAX_SPAN_DPS
        || (max_y - min_y) > IMU_CALIBRATION_MAX_SPAN_DPS
        || (max_z - min_z) > IMU_CALIBRATION_MAX_SPAN_DPS
        || motion_abs_float(g_imu_bias_x_dps) > IMU_CALIBRATION_MAX_BIAS_DPS
        || motion_abs_float(g_imu_bias_y_dps) > IMU_CALIBRATION_MAX_BIAS_DPS
        || motion_abs_float(g_imu_bias_z_dps) > IMU_CALIBRATION_MAX_BIAS_DPS)
    {
        g_imu_runtime_state = IMU_RUNTIME_UNSTABLE;
        g_imu_turn_rate_dps = 0.0f;
        gyro_data[0] = 0.0f;
        return 0U;
    }

    g_imu_runtime_state = IMU_RUNTIME_READY;
    g_imu_turn_rate_dps = 0.0f;
    gyro_data[0] = 0.0f;
    return 1U;
}

void motion_runtime_update_imu(void)
{
    float corrected_turn_rate;

    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        g_imu_turn_rate_dps = 0.0f;
        gyro_data[0] = 0.0f;
        return;
    }

    imu660rb_get_gyro();
    g_imu_gyro_x_dps = imu660rb_gyro_transition(imu660rb_gyro_x);
    g_imu_gyro_y_dps = imu660rb_gyro_transition(imu660rb_gyro_y);
    g_imu_gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

    /* The module is flat with its text facing upward, so yaw is the Z axis. */
    corrected_turn_rate = g_imu_gyro_z_dps - g_imu_bias_z_dps;
    g_imu_turn_rate_dps = 0.25f * corrected_turn_rate
        + 0.75f * g_imu_turn_rate_dps;
    gyro_data[0] = g_imu_turn_rate_dps;
}

void motion_runtime_set_encoder_sample(
    uint16 left_raw,
    uint16 right_raw,
    int32 left_signed,
    int32 right_signed,
    uint8 left_phase,
    uint8 right_phase)
{
    g_encoder_left_raw = left_raw;
    g_encoder_right_raw = right_raw;
    g_encoder_left_signed = left_signed;
    g_encoder_right_signed = right_signed;
    g_encoder_left_phase = left_phase;
    g_encoder_right_phase = right_phase;
}

float motion_runtime_limit_pid_delta(float delta_pwm)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    if (track_test_active)
    {
        return motion_clamp_float(
            delta_pwm,
            -TRACK_TEST_PID_FALL_LIMIT,
            TRACK_TEST_PID_RISE_LIMIT);
    }
#endif
    return motion_clamp_float(
        delta_pwm,
        -g_speed_pid_delta_limit,
        g_speed_pid_delta_limit);
}

float motion_runtime_line_speed_scale(uint16 line_sum)
{
    if (line_sum <= 50U)
    {
        return 0.0f;
    }
    if (line_sum >= 140U)
    {
        return 1.0f;
    }

    return 0.35f + 0.65f * ((float)(line_sum - 50U) / 90.0f);
}

void motion_runtime_trigger_protection(MotionProtectReason reason)
{
    if (reason == MOTION_PROTECT_NONE)
    {
        return;
    }

    if (g_motion_protect_reason == MOTION_PROTECT_NONE)
    {
        g_motion_protect_reason = (uint8)reason;
    }
    g_motion_run_unlocked = 0U;
    motor_test_both_passed = 0U;
    change_speed_Target_base(0);
    pwm_state = 2U;
    Pwmout = 2U;
    motion_runtime_force_stop();
}

uint8 motion_runtime_clear_protection(void)
{
    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        return 0U;
    }

    motion_runtime_stall_diag_stop();
    g_motion_protect_reason = MOTION_PROTECT_NONE;
    g_motion_run_unlocked = 0U;
    pwm_state = 0U;
    Pwmout = 0U;
    change_speed_Target_base(0);
    reset_motion_pid_state();
    motion_reset_feedback_guards();
    motion_runtime_force_stop();
    return 1U;
}

void motion_runtime_set_run_unlocked(uint8 unlocked)
{
    g_motion_run_unlocked = unlocked ? 1U : 0U;
    if (!g_motion_run_unlocked)
    {
        pwm_state = 0U;
        Pwmout = 0U;
        change_speed_Target_base(0);
        motion_runtime_force_stop();
    }
}

uint8 motion_runtime_can_run(void)
{
    return g_motion_run_unlocked
        && g_imu_runtime_state == IMU_RUNTIME_READY
        && g_motion_protect_reason == MOTION_PROTECT_NONE;
}

void motion_runtime_check_feedback(
    float left_target,
    float right_target,
    float left_speed,
    float right_speed,
    float left_applied_pwm,
    float right_applied_pwm,
    uint8 motor_running)
{
    if (!motor_running)
    {
        motion_reset_feedback_guards();
        return;
    }

    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        motion_runtime_trigger_protection(MOTION_PROTECT_IMU);
        return;
    }

    if (g_encoder_left_raw > ENCODER_RAW_SPIKE_LIMIT
        || g_encoder_right_raw > ENCODER_RAW_SPIKE_LIMIT)
    {
        if (encoder_spike_ticks < 255U) encoder_spike_ticks++;
        if (encoder_spike_ticks >= ENCODER_SPIKE_FAULT_TICKS)
        {
            motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_SPIKE);
            return;
        }
    }
    else
    {
        encoder_spike_ticks = 0U;
    }

    if (track_test_active
        && g_track_test_ticks_remaining
            > (TRACK_TEST_DURATION_TICKS - TRACK_TEST_STARTUP_GRACE_TICKS))
    {
        left_stall_ticks = 0U;
        right_stall_ticks = 0U;
    }
    else
    {
        if (motion_abs_float(left_applied_pwm) >= MOTOR_STALL_PWM_MIN
            && motion_abs_float(left_target) >= MOTOR_STALL_TARGET_MIN
            && g_encoder_left_raw <= MOTOR_STALL_RAW_MAX)
        {
            if (left_stall_ticks < 255U) left_stall_ticks++;
            if (left_stall_ticks >= MOTOR_STALL_TICKS)
            {
                motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_LEFT_STALL);
                return;
            }
        }
        else
        {
            left_stall_ticks = 0U;
        }

        if (motion_abs_float(right_applied_pwm) >= MOTOR_STALL_PWM_MIN
            && motion_abs_float(right_target) >= MOTOR_STALL_TARGET_MIN
            && g_encoder_right_raw <= MOTOR_STALL_RAW_MAX)
        {
            if (right_stall_ticks < 255U) right_stall_ticks++;
            if (right_stall_ticks >= MOTOR_STALL_TICKS)
            {
                motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_RIGHT_STALL);
                return;
            }
        }
        else
        {
            right_stall_ticks = 0U;
        }
    }

    if ((left_target > MOTOR_DIRECTION_TARGET_MIN
            && left_speed < -MOTOR_DIRECTION_SPEED_MIN)
        || (left_target < -MOTOR_DIRECTION_TARGET_MIN
            && left_speed > MOTOR_DIRECTION_SPEED_MIN))
    {
        if (left_direction_fault_ticks < 255U) left_direction_fault_ticks++;
        if (left_direction_fault_ticks >= MOTOR_DIRECTION_FAULT_TICKS)
        {
            motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_LEFT_DIRECTION);
            return;
        }
    }
    else
    {
        left_direction_fault_ticks = 0U;
    }

    if ((right_target > MOTOR_DIRECTION_TARGET_MIN
            && right_speed < -MOTOR_DIRECTION_SPEED_MIN)
        || (right_target < -MOTOR_DIRECTION_TARGET_MIN
            && right_speed > MOTOR_DIRECTION_SPEED_MIN))
    {
        if (right_direction_fault_ticks < 255U) right_direction_fault_ticks++;
        if (right_direction_fault_ticks >= MOTOR_DIRECTION_FAULT_TICKS)
        {
            motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_RIGHT_DIRECTION);
            return;
        }
    }
    else
    {
        right_direction_fault_ticks = 0U;
    }

    if (motion_abs_float(left_applied_pwm) >= MOTOR_SATURATION_THRESHOLD)
    {
        if (g_motor_left_saturation_count < 65535U) g_motor_left_saturation_count++;
        if (left_saturation_ticks < 255U) left_saturation_ticks++;
    }
    else
    {
        left_saturation_ticks = 0U;
    }

    if (motion_abs_float(right_applied_pwm) >= MOTOR_SATURATION_THRESHOLD)
    {
        if (g_motor_right_saturation_count < 65535U) g_motor_right_saturation_count++;
        if (right_saturation_ticks < 255U) right_saturation_ticks++;
    }
    else
    {
        right_saturation_ticks = 0U;
    }

    if ((left_saturation_ticks >= MOTOR_SATURATION_FAULT_TICKS
            && motion_abs_float(left_speed) < MOTOR_DIRECTION_SPEED_MIN)
        || (right_saturation_ticks >= MOTOR_SATURATION_FAULT_TICKS
            && motion_abs_float(right_speed) < MOTOR_DIRECTION_SPEED_MIN))
    {
        motion_runtime_trigger_protection(MOTION_PROTECT_SPEED_SATURATION);
    }
}

static void motion_apply_left_output(float request)
{
    int8 desired_sign = motion_sign_float(request);
    float desired_abs = motion_abs_float(request);
    float slew_step = g_motor_pwm_slew_per_tick;

    desired_abs = motion_clamp_float(desired_abs, 0.0f, MOTOR_PWM_LIMIT);

    if (left_output_state.deadtime_ticks > 0U)
    {
        left_output_state.pending_sign = desired_sign;
        left_output_state.deadtime_ticks--;
        left_output_state.applied_abs_pwm = 0.0f;
        g_motor_left_applied_pwm = 0.0f;
        pwm_set_duty(LEFT_MOTOR_PWM, 0U);
        if (left_output_state.deadtime_ticks == 0U)
        {
            left_output_state.applied_sign = left_output_state.pending_sign;
        }
        return;
    }

    if (desired_sign != 0
        && left_output_state.applied_sign != 0
        && desired_sign != left_output_state.applied_sign)
    {
        left_output_state.pending_sign = desired_sign;
        left_output_state.deadtime_ticks = MOTOR_REVERSE_DEADTIME_TICKS;
        left_output_state.applied_abs_pwm = 0.0f;
        g_motor_left_applied_pwm = 0.0f;
        if (g_motor_left_reversal_count < 65535U) g_motor_left_reversal_count++;
        pwm_set_duty(LEFT_MOTOR_PWM, 0U);
        return;
    }

    if (desired_sign == 0)
    {
        left_output_state.applied_abs_pwm = 0.0f;
        g_motor_left_applied_pwm = 0.0f;
        pwm_set_duty(LEFT_MOTOR_PWM, 0U);
        return;
    }

    if (left_output_state.applied_sign == 0)
    {
        left_output_state.applied_sign = desired_sign;
    }

#if TRACK_TEST_START_ASSIST_ENABLED
    if (track_test_active
        && g_track_test_mode == TRACK_TEST_MODE_T10
        && track_test_t10_start_active)
    {
        slew_step = TRACK_TEST_T10_LEFT_BREAKAWAY_PWM
            / (float)TRACK_TEST_T10_BREAKAWAY_RAMP_TICKS;
    }
#endif

    left_output_state.applied_abs_pwm = motion_approach(
        left_output_state.applied_abs_pwm,
        desired_abs,
        slew_step);

    LEFT_MOTOR_DIR = left_output_state.applied_sign > 0
        ? LEFT_MOTOR_FORWARD_LEVEL
        : (1U - LEFT_MOTOR_FORWARD_LEVEL);
    pwm_set_duty(LEFT_MOTOR_PWM, (uint32)left_output_state.applied_abs_pwm);
    g_motor_left_applied_pwm = left_output_state.applied_sign > 0
        ? left_output_state.applied_abs_pwm
        : -left_output_state.applied_abs_pwm;
}

static void motion_apply_right_output(float request)
{
    int8 desired_sign = motion_sign_float(request);
    float desired_abs = motion_abs_float(request);
    float slew_step = g_motor_pwm_slew_per_tick;

    desired_abs = motion_clamp_float(desired_abs, 0.0f, MOTOR_PWM_LIMIT);

    if (right_output_state.deadtime_ticks > 0U)
    {
        right_output_state.pending_sign = desired_sign;
        right_output_state.deadtime_ticks--;
        right_output_state.applied_abs_pwm = 0.0f;
        g_motor_right_applied_pwm = 0.0f;
        pwm_set_duty(RIGHT_MOTOR_PWM, 0U);
        if (right_output_state.deadtime_ticks == 0U)
        {
            right_output_state.applied_sign = right_output_state.pending_sign;
        }
        return;
    }

    if (desired_sign != 0
        && right_output_state.applied_sign != 0
        && desired_sign != right_output_state.applied_sign)
    {
        right_output_state.pending_sign = desired_sign;
        right_output_state.deadtime_ticks = MOTOR_REVERSE_DEADTIME_TICKS;
        right_output_state.applied_abs_pwm = 0.0f;
        g_motor_right_applied_pwm = 0.0f;
        if (g_motor_right_reversal_count < 65535U) g_motor_right_reversal_count++;
        pwm_set_duty(RIGHT_MOTOR_PWM, 0U);
        return;
    }

    if (desired_sign == 0)
    {
        right_output_state.applied_abs_pwm = 0.0f;
        g_motor_right_applied_pwm = 0.0f;
        pwm_set_duty(RIGHT_MOTOR_PWM, 0U);
        return;
    }

    if (right_output_state.applied_sign == 0)
    {
        right_output_state.applied_sign = desired_sign;
    }

#if TRACK_TEST_START_ASSIST_ENABLED
    if (track_test_active
        && g_track_test_mode == TRACK_TEST_MODE_T10
        && track_test_t10_start_active)
    {
        slew_step = motion_runtime_track_t10_right_start_pwm()
            / (float)TRACK_TEST_T10_BREAKAWAY_RAMP_TICKS;
    }
#endif

    right_output_state.applied_abs_pwm = motion_approach(
        right_output_state.applied_abs_pwm,
        desired_abs,
        slew_step);

    RIGHT_MOTOR_DIR = right_output_state.applied_sign > 0
        ? RIGHT_MOTOR_FORWARD_LEVEL
        : (1U - RIGHT_MOTOR_FORWARD_LEVEL);
    pwm_set_duty(RIGHT_MOTOR_PWM, (uint32)right_output_state.applied_abs_pwm);
    g_motor_right_applied_pwm = right_output_state.applied_sign > 0
        ? right_output_state.applied_abs_pwm
        : -right_output_state.applied_abs_pwm;
}

void motion_runtime_apply_outputs(
    float left_requested_pwm,
    float right_requested_pwm,
    uint8 motor_running)
{
    if (!motor_running)
    {
        motion_runtime_force_stop();
        return;
    }

    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        motion_runtime_trigger_protection(MOTION_PROTECT_IMU);
        return;
    }
    if (!g_motion_run_unlocked)
    {
        motion_runtime_trigger_protection(MOTION_PROTECT_RUN_LOCKED);
        return;
    }
    if (g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        motion_runtime_force_stop();
        return;
    }

    motion_apply_left_output(left_requested_pwm);
    motion_apply_right_output(right_requested_pwm);
}

void motion_runtime_force_stop(void)
{
    pwm_set_duty(LEFT_MOTOR_PWM, 0U);
    pwm_set_duty(RIGHT_MOTOR_PWM, 0U);
    LEFT_MOTOR_DIR = 0;
    RIGHT_MOTOR_DIR = 0;

    left_output_state.applied_abs_pwm = 0.0f;
    left_output_state.applied_sign = 0;
    left_output_state.pending_sign = 0;
    left_output_state.deadtime_ticks = 0U;
    right_output_state.applied_abs_pwm = 0.0f;
    right_output_state.applied_sign = 0;
    right_output_state.pending_sign = 0;
    right_output_state.deadtime_ticks = 0U;
    g_motor_left_applied_pwm = 0.0f;
    g_motor_right_applied_pwm = 0.0f;
    raw_drive_active = 0U;
    raw_drive_ticks_remaining = 0U;
}

uint8 motion_runtime_raw_drive_start(void)
{
    if (raw_drive_active
        || stall_diag_active
        || motor_test_active
        || encoder_test_active
        || track_test_active
        || g_motion_run_unlocked
        || g_imu_runtime_state != IMU_RUNTIME_READY
        || g_motion_protect_reason != MOTION_PROTECT_NONE
        || element4_is_enabled()
        || negative_pressure_enabled
        || negative_pressure_armed
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || negative_pressure_real_output_percent != 0U)
    {
        return 0U;
    }

    interrupt_global_disable();
    motion_runtime_force_stop();
    ctimer_count_clean(MOTOR1_ENCODER);
    ctimer_count_clean(MOTOR2_ENCODER);
    g_encoder_left_raw = 0U;
    g_encoder_right_raw = 0U;
    g_encoder_left_signed = 0;
    g_encoder_right_signed = 0;
    raw_drive_event = RAW_DRIVE_EVENT_NONE;
    raw_drive_ticks_remaining = RAW_DRIVE_DURATION_TICKS;
    raw_drive_sample_count = 0U;
    raw_drive_left_total = 0UL;
    raw_drive_right_total = 0UL;
    raw_drive_left_peak = 0U;
    raw_drive_right_peak = 0U;
    LEFT_MOTOR_DIR = LEFT_MOTOR_FORWARD_LEVEL;
    RIGHT_MOTOR_DIR = RIGHT_MOTOR_FORWARD_LEVEL;
    pwm_set_duty(LEFT_MOTOR_PWM, RAW_DRIVE_PWM_VALUE);
    pwm_set_duty(RIGHT_MOTOR_PWM, RAW_DRIVE_PWM_VALUE);
    left_output_state.applied_abs_pwm = (float)RAW_DRIVE_PWM_VALUE;
    left_output_state.applied_sign = 1;
    right_output_state.applied_abs_pwm = (float)RAW_DRIVE_PWM_VALUE;
    right_output_state.applied_sign = 1;
    g_motor_left_applied_pwm = (float)RAW_DRIVE_PWM_VALUE;
    g_motor_right_applied_pwm = (float)RAW_DRIVE_PWM_VALUE;
    raw_drive_active = 1U;
    interrupt_global_enable();
    return 1U;
}

uint8 motion_runtime_motor_protection_is_clearable(void)
{
    switch ((MotionProtectReason)g_motion_protect_reason)
    {
        case MOTION_PROTECT_ENCODER_LEFT_STALL:
        case MOTION_PROTECT_ENCODER_RIGHT_STALL:
        case MOTION_PROTECT_ENCODER_LEFT_DIRECTION:
        case MOTION_PROTECT_ENCODER_RIGHT_DIRECTION:
        case MOTION_PROTECT_ENCODER_SPIKE:
        case MOTION_PROTECT_SPEED_SATURATION:
        case MOTION_PROTECT_ENCODER_MODE:
        case MOTION_PROTECT_ENCODER_NOISE:
            return 1U;
        default:
            return 0U;
    }
}

uint8 motion_runtime_clear_motor_protection(void)
{
    if (!motion_runtime_motor_protection_is_clearable())
    {
        return 0U;
    }
    return motion_runtime_clear_protection();
}

uint8 motion_runtime_raw_drive_stop(void)
{
    uint8 was_active;

    interrupt_global_disable();
    was_active = raw_drive_active;
    motion_runtime_force_stop();
    if (was_active)
    {
        raw_drive_event = RAW_DRIVE_EVENT_STOPPED;
    }
    interrupt_global_enable();
    return was_active;
}

void motion_runtime_raw_drive_tick(void)
{
    uint16 left_raw;
    uint16 right_raw;

    if (!raw_drive_active)
    {
        return;
    }

    left_raw = g_encoder_left_raw;
    right_raw = g_encoder_right_raw;
    if (raw_drive_sample_count < 65535U)
    {
        raw_drive_sample_count++;
    }
    if (raw_drive_left_total <= (0xFFFFFFFFUL - (uint32)left_raw))
    {
        raw_drive_left_total += (uint32)left_raw;
    }
    else
    {
        raw_drive_left_total = 0xFFFFFFFFUL;
    }
    if (raw_drive_right_total <= (0xFFFFFFFFUL - (uint32)right_raw))
    {
        raw_drive_right_total += (uint32)right_raw;
    }
    else
    {
        raw_drive_right_total = 0xFFFFFFFFUL;
    }
    if (left_raw > raw_drive_left_peak)
    {
        raw_drive_left_peak = left_raw;
    }
    if (right_raw > raw_drive_right_peak)
    {
        raw_drive_right_peak = right_raw;
    }
    if (g_imu_runtime_state != IMU_RUNTIME_READY
        || g_motion_protect_reason != MOTION_PROTECT_NONE
        || g_motion_run_unlocked
        || element4_is_enabled()
        || negative_pressure_enabled
        || negative_pressure_armed
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || negative_pressure_real_output_percent != 0U)
    {
        motion_runtime_force_stop();
        raw_drive_event = RAW_DRIVE_EVENT_ABORTED;
        return;
    }

    if (raw_drive_ticks_remaining > 0U)
    {
        raw_drive_ticks_remaining--;
    }
    if (raw_drive_ticks_remaining == 0U)
    {
        motion_runtime_force_stop();
        raw_drive_event = RAW_DRIVE_EVENT_DONE;
    }
}

uint8 motion_runtime_raw_drive_is_active(void)
{
    return raw_drive_active;
}

uint8 motion_runtime_raw_drive_take_event(void)
{
    uint8 event;

    interrupt_global_disable();
    event = raw_drive_event;
    raw_drive_event = RAW_DRIVE_EVENT_NONE;
    interrupt_global_enable();
    return event;
}

uint16 motion_runtime_raw_drive_sample_count(void)
{
    uint16 value;

    interrupt_global_disable();
    value = raw_drive_sample_count;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_raw_drive_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = raw_drive_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_raw_drive_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = raw_drive_right_total;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_raw_drive_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = raw_drive_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_raw_drive_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = raw_drive_right_peak;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_raw_drive_difference(void)
{
    uint32 left_total = motion_runtime_raw_drive_left_total();
    uint32 right_total = motion_runtime_raw_drive_right_total();

    return left_total > right_total
        ? left_total - right_total
        : right_total - left_total;
}

uint16 motion_runtime_raw_drive_balance_x1000(void)
{
    uint32 left_total = motion_runtime_raw_drive_left_total();
    uint32 right_total = motion_runtime_raw_drive_right_total();
    uint32 smaller = left_total < right_total ? left_total : right_total;
    uint32 larger = left_total > right_total ? left_total : right_total;

    if (larger == 0UL)
    {
        return 0U;
    }
    return (uint16)(((float)smaller * 1000.0f) / (float)larger);
}

uint32 motion_runtime_raw_drive_left_average_x10(void)
{
    uint16 sample_count = motion_runtime_raw_drive_sample_count();
    uint32 total = motion_runtime_raw_drive_left_total();

    if (sample_count == 0U)
    {
        return 0UL;
    }
    return (total / sample_count) * 10UL
        + ((total % sample_count) * 10UL) / sample_count;
}

uint32 motion_runtime_raw_drive_right_average_x10(void)
{
    uint16 sample_count = motion_runtime_raw_drive_sample_count();
    uint32 total = motion_runtime_raw_drive_right_total();

    if (sample_count == 0U)
    {
        return 0UL;
    }
    return (total / sample_count) * 10UL
        + ((total % sample_count) * 10UL) / sample_count;
}

static void motion_motor_test_apply_direct(uint16 pwm)
{
    uint8 test_left = (uint8)(g_motor_test_side != MOTOR_TEST_SIDE_RIGHT);
    uint8 test_right = (uint8)(g_motor_test_side != MOTOR_TEST_SIDE_LEFT);

    LEFT_MOTOR_DIR = LEFT_MOTOR_FORWARD_LEVEL;
    RIGHT_MOTOR_DIR = RIGHT_MOTOR_FORWARD_LEVEL;
    pwm_set_duty(LEFT_MOTOR_PWM, test_left ? (uint32)pwm : 0UL);
    pwm_set_duty(RIGHT_MOTOR_PWM, test_right ? (uint32)pwm : 0UL);
    left_output_state.applied_abs_pwm = test_left ? (float)pwm : 0.0f;
    left_output_state.applied_sign = test_left ? 1 : 0;
    left_output_state.pending_sign = 0;
    left_output_state.deadtime_ticks = 0U;
    right_output_state.applied_abs_pwm = test_right ? (float)pwm : 0.0f;
    right_output_state.applied_sign = test_right ? 1 : 0;
    right_output_state.pending_sign = 0;
    right_output_state.deadtime_ticks = 0U;
    g_motor_left_applied_pwm = test_left ? (float)pwm : 0.0f;
    g_motor_right_applied_pwm = test_right ? (float)pwm : 0.0f;
}

uint8 motion_runtime_motor_test_start(MotorTestSide side)
{
    if ((side != MOTOR_TEST_SIDE_LEFT
            && side != MOTOR_TEST_SIDE_RIGHT
            && side != MOTOR_TEST_SIDE_BOTH)
        || stall_diag_active
        || raw_drive_active
        || g_motion_run_unlocked
        || g_imu_runtime_state != IMU_RUNTIME_READY
        || g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        return 0U;
    }

    motion_runtime_force_stop();
    ctimer_count_clean(MOTOR1_ENCODER);
    ctimer_count_clean(MOTOR2_ENCODER);
    g_encoder_left_raw = 0U;
    g_encoder_right_raw = 0U;
    g_encoder_left_signed = 0;
    g_encoder_right_signed = 0;
    motor_test_active = 0U;
    g_motor_test_ticks_remaining = 0U;
    motor_test_left_stall_ticks = 0U;
    motor_test_right_stall_ticks = 0U;
    motor_test_left_direction_fault_ticks = 0U;
    motor_test_right_direction_fault_ticks = 0U;
    motor_test_noise_ticks = 0U;
    motor_test_precheck_ticks_remaining = MOTOR_TEST_PRECHECK_TICKS;
    motor_test_event = MOTOR_TEST_RESULT_IDLE;
    motor_test_left_total = 0U;
    motor_test_right_total = 0U;
    motor_test_left_peak = 0U;
    motor_test_right_peak = 0U;
    motor_test_left_idle_peak = 0U;
    motor_test_right_idle_peak = 0U;
    motor_test_boost_active = 0U;
    motor_test_boost_ticks_remaining = 0U;
    motor_test_boost_sample_count = 0U;
    motor_test_boost_left_total = 0UL;
    motor_test_boost_right_total = 0UL;
    motor_test_boost_left_peak = 0U;
    motor_test_boost_right_peak = 0U;
    motor_test_hold_sample_count = 0U;
    motor_test_hold_left_total = 0UL;
    motor_test_hold_right_total = 0UL;
    motor_test_hold_left_peak = 0U;
    motor_test_hold_right_peak = 0U;
    if (side == MOTOR_TEST_SIDE_BOTH)
    {
        motor_test_both_passed = 0U;
    }

    g_motor_test_side = (uint8)side;
    g_motor_test_result = MOTOR_TEST_RESULT_RUNNING;
    g_motor_test_ticks_remaining = MOTOR_TEST_DURATION_TICKS;
    motor_test_active = 1U;
    return 1U;
}

uint8 motion_runtime_motor_test_stop(void)
{
    uint8 was_active;

    interrupt_global_disable();
    was_active = motor_test_active;
    if (was_active)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_STOPPED,
            MOTION_PROTECT_NONE,
            0U);
    }
    else
    {
        motion_runtime_force_stop();
    }
    interrupt_global_enable();
    return was_active;
}

void motion_runtime_motor_test_tick(void)
{
    uint16 boost_elapsed_ticks;
    uint16 left_raw;
    uint16 right_raw;
    uint16 tested_raw;
    uint16 untested_raw;
    uint8 test_left;
    uint8 test_right;

    if (!motor_test_active)
    {
        return;
    }

    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_IMU,
            MOTION_PROTECT_IMU,
            1U);
        return;
    }
    if (g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_PROTECT,
            MOTION_PROTECT_NONE,
            1U);
        return;
    }
    if (g_motion_run_unlocked
        || (g_motor_test_side != MOTOR_TEST_SIDE_LEFT
            && g_motor_test_side != MOTOR_TEST_SIDE_RIGHT
            && g_motor_test_side != MOTOR_TEST_SIDE_BOTH))
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_PROTECT,
            MOTION_PROTECT_RUN_LOCKED,
            1U);
        return;
    }

    if (motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_ENCODER_MODE,
            MOTION_PROTECT_ENCODER_MODE,
            1U);
        return;
    }

    left_raw = g_encoder_left_raw;
    right_raw = g_encoder_right_raw;

    if (motor_test_precheck_ticks_remaining > 0U)
    {
        motion_apply_left_output(0.0f);
        motion_apply_right_output(0.0f);

        if (left_raw > motor_test_left_idle_peak)
        {
            motor_test_left_idle_peak = left_raw;
        }
        if (right_raw > motor_test_right_idle_peak)
        {
            motor_test_right_idle_peak = right_raw;
        }

        if (left_raw > MOTOR_TEST_IDLE_RAW_MAX
            || right_raw > MOTOR_TEST_IDLE_RAW_MAX)
        {
            if (motor_test_noise_ticks < 255U)
            {
                motor_test_noise_ticks++;
            }
        }
        else
        {
            motor_test_noise_ticks = 0U;
        }

        if (motor_test_noise_ticks >= MOTOR_TEST_NOISE_FAULT_TICKS)
        {
            motion_motor_test_finish(
                MOTOR_TEST_RESULT_ENCODER_NOISE,
                MOTION_PROTECT_ENCODER_NOISE,
                1U);
            return;
        }

        motor_test_precheck_ticks_remaining--;
        if (motor_test_precheck_ticks_remaining == 0U)
        {
            ctimer_count_clean(MOTOR1_ENCODER);
            ctimer_count_clean(MOTOR2_ENCODER);
            g_encoder_left_raw = 0U;
            g_encoder_right_raw = 0U;
            g_encoder_left_signed = 0;
            g_encoder_right_signed = 0;
            motor_test_noise_ticks = 0U;
            motor_test_boost_active = 1U;
            motor_test_boost_ticks_remaining = MOTOR_TEST_BOOST_DURATION_TICKS;
            motion_motor_test_apply_direct(MOTOR_TEST_BOOST_PWM_VALUE);
        }
        return;
    }

    test_left = (uint8)(g_motor_test_side != MOTOR_TEST_SIDE_RIGHT);
    test_right = (uint8)(g_motor_test_side != MOTOR_TEST_SIDE_LEFT);

    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        tested_raw = 0U;
        untested_raw = 0U;
    }
    else if (g_motor_test_side == MOTOR_TEST_SIDE_LEFT)
    {
        tested_raw = left_raw;
        untested_raw = right_raw;
    }
    else
    {
        tested_raw = right_raw;
        untested_raw = left_raw;
    }

    if (motor_test_boost_active)
    {
        motion_motor_test_apply_direct(MOTOR_TEST_BOOST_PWM_VALUE);
        if (motor_test_boost_sample_count < 65535U)
        {
            motor_test_boost_sample_count++;
        }
        if (motor_test_boost_left_total <= (0xFFFFFFFFUL - (uint32)left_raw))
        {
            motor_test_boost_left_total += (uint32)left_raw;
        }
        else
        {
            motor_test_boost_left_total = 0xFFFFFFFFUL;
        }
        if (motor_test_boost_right_total <= (0xFFFFFFFFUL - (uint32)right_raw))
        {
            motor_test_boost_right_total += (uint32)right_raw;
        }
        else
        {
            motor_test_boost_right_total = 0xFFFFFFFFUL;
        }
        if (left_raw > motor_test_boost_left_peak)
        {
            motor_test_boost_left_peak = left_raw;
        }
        if (right_raw > motor_test_boost_right_peak)
        {
            motor_test_boost_right_peak = right_raw;
        }
    }
    else
    {
        motion_motor_test_apply_direct(
            g_motor_test_side == MOTOR_TEST_SIDE_BOTH
                ? MOTOR_TEST_BOTH_PWM_VALUE : MOTOR_TEST_PWM_VALUE);
        if (motor_test_hold_sample_count < 65535U)
        {
            motor_test_hold_sample_count++;
        }
        if (motor_test_hold_left_total <= (0xFFFFFFFFUL - (uint32)left_raw))
        {
            motor_test_hold_left_total += (uint32)left_raw;
        }
        else
        {
            motor_test_hold_left_total = 0xFFFFFFFFUL;
        }
        if (motor_test_hold_right_total <= (0xFFFFFFFFUL - (uint32)right_raw))
        {
            motor_test_hold_right_total += (uint32)right_raw;
        }
        else
        {
            motor_test_hold_right_total = 0xFFFFFFFFUL;
        }
        if (left_raw > motor_test_hold_left_peak)
        {
            motor_test_hold_left_peak = left_raw;
        }
        if (right_raw > motor_test_hold_right_peak)
        {
            motor_test_hold_right_peak = right_raw;
        }
    }

    if (motor_test_left_total <= (0xFFFFFFFFUL - (uint32)left_raw))
    {
        motor_test_left_total += (uint32)left_raw;
    }
    else
    {
        motor_test_left_total = 0xFFFFFFFFUL;
    }
    if (motor_test_right_total <= (0xFFFFFFFFUL - (uint32)right_raw))
    {
        motor_test_right_total += (uint32)right_raw;
    }
    else
    {
        motor_test_right_total = 0xFFFFFFFFUL;
    }
    if (left_raw > motor_test_left_peak)
    {
        motor_test_left_peak = left_raw;
    }
    if (right_raw > motor_test_right_peak)
    {
        motor_test_right_peak = right_raw;
    }

    if ((g_motor_test_side == MOTOR_TEST_SIDE_BOTH
            && (left_raw > MOTOR_TEST_RUNNING_RAW_MAX
                || right_raw > MOTOR_TEST_RUNNING_RAW_MAX))
        || (g_motor_test_side != MOTOR_TEST_SIDE_BOTH
            && (tested_raw > MOTOR_TEST_RUNNING_RAW_MAX
                || untested_raw > MOTOR_TEST_UNTESTED_RAW_MAX)))
    {
        if (motor_test_noise_ticks < 255U)
        {
            motor_test_noise_ticks++;
        }
    }
    else
    {
        motor_test_noise_ticks = 0U;
    }
    if (motor_test_noise_ticks >= MOTOR_TEST_NOISE_FAULT_TICKS)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_ENCODER_NOISE,
            MOTION_PROTECT_ENCODER_NOISE,
            1U);
        return;
    }

    boost_elapsed_ticks = MOTOR_TEST_BOOST_DURATION_TICKS
        - motor_test_boost_ticks_remaining;
    if (!motor_test_boost_active
        || boost_elapsed_ticks >= MOTOR_TEST_BOOST_STALL_GRACE_TICKS)
    {
        if (test_left && left_raw <= MOTOR_STALL_RAW_MAX)
        {
            if (motor_test_left_stall_ticks < 255U)
            {
                motor_test_left_stall_ticks++;
            }
        }
        else
        {
            motor_test_left_stall_ticks = 0U;
        }
        if (test_right && right_raw <= MOTOR_STALL_RAW_MAX)
        {
            if (motor_test_right_stall_ticks < 255U)
            {
                motor_test_right_stall_ticks++;
            }
        }
        else
        {
            motor_test_right_stall_ticks = 0U;
        }

        if (test_left
            && g_encoder_left_signed < -(int32)MOTOR_STALL_RAW_MAX)
        {
            if (motor_test_left_direction_fault_ticks < 255U)
            {
                motor_test_left_direction_fault_ticks++;
            }
        }
        else
        {
            motor_test_left_direction_fault_ticks = 0U;
        }
        if (test_right
            && g_encoder_right_signed < -(int32)MOTOR_STALL_RAW_MAX)
        {
            if (motor_test_right_direction_fault_ticks < 255U)
            {
                motor_test_right_direction_fault_ticks++;
            }
        }
        else
        {
            motor_test_right_direction_fault_ticks = 0U;
        }

        if (motor_test_left_direction_fault_ticks
            >= MOTOR_TEST_DIRECTION_FAULT_TICKS)
        {
            motion_motor_test_finish(
                MOTOR_TEST_RESULT_LEFT_DIRECTION,
                MOTION_PROTECT_ENCODER_LEFT_DIRECTION,
                1U);
            return;
        }
        if (motor_test_right_direction_fault_ticks
            >= MOTOR_TEST_DIRECTION_FAULT_TICKS)
        {
            motion_motor_test_finish(
                MOTOR_TEST_RESULT_RIGHT_DIRECTION,
                MOTION_PROTECT_ENCODER_RIGHT_DIRECTION,
                1U);
            return;
        }
        if (motor_test_left_stall_ticks >= MOTOR_TEST_STALL_TICKS)
        {
            motion_motor_test_finish(
                MOTOR_TEST_RESULT_LEFT_STALL,
                MOTION_PROTECT_ENCODER_LEFT_STALL,
                1U);
            return;
        }
        if (motor_test_right_stall_ticks >= MOTOR_TEST_STALL_TICKS)
        {
            motion_motor_test_finish(
                MOTOR_TEST_RESULT_RIGHT_STALL,
                MOTION_PROTECT_ENCODER_RIGHT_STALL,
                1U);
            return;
        }
    }

    if (motor_test_boost_active)
    {
        if (motor_test_boost_ticks_remaining > 0U)
        {
            motor_test_boost_ticks_remaining--;
        }
        if (motor_test_boost_ticks_remaining == 0U)
        {
            motor_test_boost_active = 0U;
            motor_test_left_stall_ticks = 0U;
            motor_test_right_stall_ticks = 0U;
            motion_motor_test_apply_direct(
                g_motor_test_side == MOTOR_TEST_SIDE_BOTH
                    ? MOTOR_TEST_BOTH_PWM_VALUE : MOTOR_TEST_PWM_VALUE);
        }
        return;
    }

    if (g_motor_test_ticks_remaining > 0U)
    {
        g_motor_test_ticks_remaining--;
    }
    if (g_motor_test_ticks_remaining == 0U)
    {
        motion_motor_test_finish(
            MOTOR_TEST_RESULT_DONE,
            MOTION_PROTECT_NONE,
            1U);
    }
}

uint8 motion_runtime_motor_test_is_active(void)
{
    return motor_test_active;
}

uint16 motion_runtime_motor_test_remaining_ms(void)
{
    return (uint16)(
        g_motor_test_ticks_remaining * MOTOR_TEST_CONTROL_PERIOD_MS);
}

uint16 motion_runtime_motor_test_pwm_value(void)
{
    return g_motor_test_side == MOTOR_TEST_SIDE_BOTH
        ? MOTOR_TEST_BOTH_PWM_VALUE
        : MOTOR_TEST_PWM_VALUE;
}

uint32 motion_runtime_motor_test_pulse_total(void)
{
    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        uint32 left_total = motion_runtime_motor_test_left_total();
        uint32 right_total = motion_runtime_motor_test_right_total();

        return left_total <= (0xFFFFFFFFUL - right_total)
            ? left_total + right_total
            : 0xFFFFFFFFUL;
    }
    return g_motor_test_side == MOTOR_TEST_SIDE_RIGHT
        ? motion_runtime_motor_test_right_total()
        : motion_runtime_motor_test_left_total();
}

uint16 motion_runtime_motor_test_peak_raw(void)
{
    if (g_motor_test_side == MOTOR_TEST_SIDE_BOTH)
    {
        uint16 left_peak = motion_runtime_motor_test_left_peak();
        uint16 right_peak = motion_runtime_motor_test_right_peak();

        return left_peak > right_peak ? left_peak : right_peak;
    }
    return g_motor_test_side == MOTOR_TEST_SIDE_RIGHT
        ? motion_runtime_motor_test_right_peak()
        : motion_runtime_motor_test_left_peak();
}

uint32 motion_runtime_motor_test_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_right_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_difference(void)
{
    uint32 left_total = motion_runtime_motor_test_left_total();
    uint32 right_total = motion_runtime_motor_test_right_total();

    return left_total > right_total
        ? left_total - right_total
        : right_total - left_total;
}

uint16 motion_runtime_motor_test_balance_x1000(void)
{
    uint32 left_total = motion_runtime_motor_test_left_total();
    uint32 right_total = motion_runtime_motor_test_right_total();
    uint32 smaller = left_total < right_total ? left_total : right_total;
    uint32 larger = left_total > right_total ? left_total : right_total;

    if (larger == 0U)
    {
        return 0U;
    }
    return (uint16)(((float)smaller * 1000.0f) / (float)larger);
}

uint16 motion_runtime_motor_test_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_right_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_left_idle_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_left_idle_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_right_idle_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_right_idle_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_boost_sample_count(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_boost_sample_count;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_boost_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_boost_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_boost_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_boost_right_total;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_boost_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_boost_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_boost_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_boost_right_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_hold_sample_count(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_hold_sample_count;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_hold_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_hold_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_motor_test_hold_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = motor_test_hold_right_total;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_hold_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_hold_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_motor_test_hold_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = motor_test_hold_right_peak;
    interrupt_global_enable();
    return value;
}

uint8 motion_runtime_encoder_mode_mask(void)
{
    uint8 mask = 0U;

    if (ctimer_count_mode_active(MOTOR1_ENCODER))
    {
        mask |= 0x01U;
    }
    if (ctimer_count_mode_active(MOTOR2_ENCODER))
    {
        mask |= 0x02U;
    }
    return mask;
}

MotorTestResult motion_runtime_motor_test_take_event(void)
{
    MotorTestResult event;

    interrupt_global_disable();
    event = (MotorTestResult)motor_test_event;
    motor_test_event = MOTOR_TEST_RESULT_IDLE;
    interrupt_global_enable();
    return event;
}

const char *motion_runtime_motor_test_side_text(void)
{
    switch (g_motor_test_side)
    {
        case MOTOR_TEST_SIDE_LEFT: return "L";
        case MOTOR_TEST_SIDE_RIGHT: return "R";
        case MOTOR_TEST_SIDE_BOTH: return "B";
        default: return "NONE";
    }
}

const char *motion_runtime_motor_test_result_text(void)
{
    switch (g_motor_test_result)
    {
        case MOTOR_TEST_RESULT_RUNNING: return "RUNNING";
        case MOTOR_TEST_RESULT_DONE: return "DONE";
        case MOTOR_TEST_RESULT_STOPPED: return "STOPPED";
        case MOTOR_TEST_RESULT_LEFT_STALL: return "L_STALL";
        case MOTOR_TEST_RESULT_RIGHT_STALL: return "R_STALL";
        case MOTOR_TEST_RESULT_LEFT_DIRECTION: return "L_DIR";
        case MOTOR_TEST_RESULT_RIGHT_DIRECTION: return "R_DIR";
        case MOTOR_TEST_RESULT_IMU: return "IMU";
        case MOTOR_TEST_RESULT_PROTECT: return "PROTECT";
        case MOTOR_TEST_RESULT_ENCODER_MODE: return "ENC_MODE";
        case MOTOR_TEST_RESULT_ENCODER_NOISE: return "ENC_NOISE";
        default: return "IDLE";
    }
}

uint8 motion_runtime_motor_test_both_passed(void)
{
    return motor_test_both_passed;
}

static uint16 motion_stall_diag_stage_pwm(uint8 stage)
{
    switch (stage)
    {
        case 1U: return STALL_DIAG_PWM_LOW_VALUE;
        case 2U: return STALL_DIAG_PWM_MID_VALUE;
        case 3U: return STALL_DIAG_PWM_HIGH_VALUE;
        default: return 0U;
    }
}

static void motion_stall_diag_apply_output(uint16 pwm)
{
    motion_runtime_force_stop();
    if (stall_diag_active_side == MOTOR_TEST_SIDE_LEFT)
    {
        LEFT_MOTOR_DIR = LEFT_MOTOR_FORWARD_LEVEL;
        pwm_set_duty(LEFT_MOTOR_PWM, (uint32)pwm);
        left_output_state.applied_abs_pwm = (float)pwm;
        left_output_state.applied_sign = 1;
        g_motor_left_applied_pwm = (float)pwm;
    }
    else if (stall_diag_active_side == MOTOR_TEST_SIDE_RIGHT)
    {
        RIGHT_MOTOR_DIR = RIGHT_MOTOR_FORWARD_LEVEL;
        pwm_set_duty(RIGHT_MOTOR_PWM, (uint32)pwm);
        right_output_state.applied_abs_pwm = (float)pwm;
        right_output_state.applied_sign = 1;
        g_motor_right_applied_pwm = (float)pwm;
    }
}

static void motion_stall_diag_reset_encoder_sample(void)
{
    ctimer_count_clean(MOTOR1_ENCODER);
    ctimer_count_clean(MOTOR2_ENCODER);
    g_encoder_left_raw = 0U;
    g_encoder_right_raw = 0U;
    g_encoder_left_signed = 0;
    g_encoder_right_signed = 0;
}

static void motion_stall_diag_begin_precheck(MotorTestSide side)
{
    motion_runtime_force_stop();
    motion_stall_diag_reset_encoder_sample();
    stall_diag_active_side = (uint8)side;
    stall_diag_stage = 0U;
    stall_diag_stage_ticks_remaining = STALL_DIAG_PRECHECK_TICKS;
    stall_diag_noise_ticks = 0U;
    stall_diag_applied_pwm = 0U;
    stall_diag_active_total = 0UL;
    stall_diag_active_peak = 0U;
}

static void motion_stall_diag_begin_stage(uint8 stage)
{
    motion_stall_diag_reset_encoder_sample();
    stall_diag_stage = stage;
    stall_diag_stage_ticks_remaining = STALL_DIAG_STAGE_TICKS;
    stall_diag_noise_ticks = 0U;
    stall_diag_active_total = 0UL;
    stall_diag_active_peak = 0U;
    stall_diag_applied_pwm = motion_stall_diag_stage_pwm(stage);
    motion_stall_diag_apply_output(stall_diag_applied_pwm);
}

static void motion_stall_diag_finish(
    StallDiagResult result,
    MotionProtectReason protect_reason,
    uint8 post_event)
{
    stall_diag_active = 0U;
    stall_diag_stage_ticks_remaining = 0U;
    stall_diag_noise_ticks = 0U;
    motion_runtime_force_stop();
    g_stall_diag_result = (uint8)result;
    if (post_event)
    {
        stall_diag_event = (uint8)result;
    }
    if (protect_reason != MOTION_PROTECT_NONE)
    {
        motion_runtime_trigger_protection(protect_reason);
    }
}

static void motion_stall_diag_finish_side(void)
{
    uint8 moved = (uint8)(
        stall_diag_active_total >= STALL_DIAG_MOVE_TOTAL_MIN
        && stall_diag_active_peak >= STALL_DIAG_MOVE_PEAK_MIN);

    if (moved)
    {
        if (stall_diag_active_side == MOTOR_TEST_SIDE_LEFT)
        {
            stall_diag_left_breakaway_pwm = stall_diag_applied_pwm;
            stall_diag_pass_mask |= 0x01U;
            if (stall_diag_requested_side == MOTOR_TEST_SIDE_BOTH)
            {
                motion_stall_diag_begin_precheck(MOTOR_TEST_SIDE_RIGHT);
                return;
            }
        }
        else
        {
            stall_diag_right_breakaway_pwm = stall_diag_applied_pwm;
            stall_diag_pass_mask |= 0x02U;
        }
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_PASS,
            MOTION_PROTECT_NONE,
            1U);
        return;
    }

    if (stall_diag_stage < 3U)
    {
        motion_stall_diag_begin_stage((uint8)(stall_diag_stage + 1U));
        return;
    }

    if (stall_diag_active_side == MOTOR_TEST_SIDE_LEFT)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_LEFT_STALL,
            MOTION_PROTECT_ENCODER_LEFT_STALL,
            1U);
    }
    else
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_RIGHT_STALL,
            MOTION_PROTECT_ENCODER_RIGHT_STALL,
            1U);
    }
}

uint8 motion_runtime_stall_diag_start(MotorTestSide side)
{
    MotorTestSide first_side;

    if ((side != MOTOR_TEST_SIDE_LEFT
            && side != MOTOR_TEST_SIDE_RIGHT
            && side != MOTOR_TEST_SIDE_BOTH)
        || stall_diag_active
        || raw_drive_active
        || motor_test_active
        || encoder_test_active
        || track_test_active
        || g_motion_run_unlocked
        || g_imu_runtime_state != IMU_RUNTIME_READY
        || g_motion_protect_reason != MOTION_PROTECT_NONE
        || motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK)
    {
        return 0U;
    }

    first_side = side == MOTOR_TEST_SIDE_BOTH
        ? MOTOR_TEST_SIDE_LEFT : side;
    stall_diag_requested_side = (uint8)side;
    stall_diag_pass_mask = 0U;
    stall_diag_left_total = 0UL;
    stall_diag_right_total = 0UL;
    stall_diag_left_peak = 0U;
    stall_diag_right_peak = 0U;
    stall_diag_left_breakaway_pwm = 0U;
    stall_diag_right_breakaway_pwm = 0U;
    stall_diag_event = STALL_DIAG_RESULT_IDLE;
    g_stall_diag_result = STALL_DIAG_RESULT_RUNNING;
    stall_diag_active = 1U;
    motion_stall_diag_begin_precheck(first_side);
    return 1U;
}

uint8 motion_runtime_stall_diag_stop(void)
{
    uint8 was_active;

    interrupt_global_disable();
    was_active = stall_diag_active;
    if (was_active)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_STOPPED,
            MOTION_PROTECT_NONE,
            0U);
    }
    else
    {
        motion_runtime_force_stop();
    }
    interrupt_global_enable();
    return was_active;
}

void motion_runtime_stall_diag_tick(void)
{
    uint16 active_raw;
    uint16 inactive_raw;

    if (!stall_diag_active)
    {
        return;
    }
    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_IMU,
            MOTION_PROTECT_IMU,
            1U);
        return;
    }
    if (g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_PROTECT,
            MOTION_PROTECT_NONE,
            1U);
        return;
    }
    if (g_motion_run_unlocked)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_PROTECT,
            MOTION_PROTECT_RUN_LOCKED,
            1U);
        return;
    }
    if (motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_ENCODER_MODE,
            MOTION_PROTECT_ENCODER_MODE,
            1U);
        return;
    }

    if (stall_diag_stage == 0U)
    {
        if (g_encoder_left_raw > STALL_DIAG_IDLE_RAW_MAX
            || g_encoder_right_raw > STALL_DIAG_IDLE_RAW_MAX)
        {
            if (stall_diag_noise_ticks < 255U)
            {
                stall_diag_noise_ticks++;
            }
        }
        else
        {
            stall_diag_noise_ticks = 0U;
        }
        if (stall_diag_noise_ticks >= STALL_DIAG_NOISE_FAULT_TICKS)
        {
            motion_stall_diag_finish(
                STALL_DIAG_RESULT_ENCODER_NOISE,
                MOTION_PROTECT_ENCODER_NOISE,
                1U);
            return;
        }
        if (stall_diag_stage_ticks_remaining > 0U)
        {
            stall_diag_stage_ticks_remaining--;
        }
        if (stall_diag_stage_ticks_remaining == 0U)
        {
            motion_stall_diag_begin_stage(1U);
        }
        return;
    }

    active_raw = stall_diag_active_side == MOTOR_TEST_SIDE_LEFT
        ? g_encoder_left_raw : g_encoder_right_raw;
    inactive_raw = stall_diag_active_side == MOTOR_TEST_SIDE_LEFT
        ? g_encoder_right_raw : g_encoder_left_raw;

    if (stall_diag_active_total <= (0xFFFFFFFFUL - (uint32)active_raw))
    {
        stall_diag_active_total += (uint32)active_raw;
    }
    else
    {
        stall_diag_active_total = 0xFFFFFFFFUL;
    }
    if (active_raw > stall_diag_active_peak)
    {
        stall_diag_active_peak = active_raw;
    }
    if (stall_diag_active_side == MOTOR_TEST_SIDE_LEFT)
    {
        if (stall_diag_left_total <= (0xFFFFFFFFUL - (uint32)active_raw))
        {
            stall_diag_left_total += (uint32)active_raw;
        }
        else
        {
            stall_diag_left_total = 0xFFFFFFFFUL;
        }
        if (active_raw > stall_diag_left_peak)
        {
            stall_diag_left_peak = active_raw;
        }
    }
    else
    {
        if (stall_diag_right_total <= (0xFFFFFFFFUL - (uint32)active_raw))
        {
            stall_diag_right_total += (uint32)active_raw;
        }
        else
        {
            stall_diag_right_total = 0xFFFFFFFFUL;
        }
        if (active_raw > stall_diag_right_peak)
        {
            stall_diag_right_peak = active_raw;
        }
    }

    if (active_raw > MOTOR_TEST_RUNNING_RAW_MAX
        || inactive_raw > MOTOR_TEST_UNTESTED_RAW_MAX)
    {
        if (stall_diag_noise_ticks < 255U)
        {
            stall_diag_noise_ticks++;
        }
    }
    else
    {
        stall_diag_noise_ticks = 0U;
    }
    if (stall_diag_noise_ticks >= STALL_DIAG_NOISE_FAULT_TICKS)
    {
        motion_stall_diag_finish(
            STALL_DIAG_RESULT_ENCODER_NOISE,
            MOTION_PROTECT_ENCODER_NOISE,
            1U);
        return;
    }

    if (stall_diag_stage_ticks_remaining > 0U)
    {
        stall_diag_stage_ticks_remaining--;
    }
    if (stall_diag_stage_ticks_remaining == 0U)
    {
        motion_stall_diag_finish_side();
    }
}

uint8 motion_runtime_stall_diag_is_active(void)
{
    return stall_diag_active;
}

StallDiagResult motion_runtime_stall_diag_take_event(void)
{
    StallDiagResult event;

    interrupt_global_disable();
    event = (StallDiagResult)stall_diag_event;
    stall_diag_event = STALL_DIAG_RESULT_IDLE;
    interrupt_global_enable();
    return event;
}

StallDiagResult motion_runtime_stall_diag_result(void)
{
    return (StallDiagResult)g_stall_diag_result;
}

const char *motion_runtime_stall_diag_result_text(void)
{
    switch (g_stall_diag_result)
    {
        case STALL_DIAG_RESULT_RUNNING: return "RUNNING";
        case STALL_DIAG_RESULT_PASS: return "PASS";
        case STALL_DIAG_RESULT_STOPPED: return "STOPPED";
        case STALL_DIAG_RESULT_LEFT_STALL: return "L_STALL";
        case STALL_DIAG_RESULT_RIGHT_STALL: return "R_STALL";
        case STALL_DIAG_RESULT_IMU: return "IMU";
        case STALL_DIAG_RESULT_PROTECT: return "PROTECT";
        case STALL_DIAG_RESULT_ENCODER_MODE: return "ENC_MODE";
        case STALL_DIAG_RESULT_ENCODER_NOISE: return "ENC_NOISE";
        default: return "IDLE";
    }
}

const char *motion_runtime_stall_diag_requested_side_text(void)
{
    switch (stall_diag_requested_side)
    {
        case MOTOR_TEST_SIDE_LEFT: return "L";
        case MOTOR_TEST_SIDE_RIGHT: return "R";
        case MOTOR_TEST_SIDE_BOTH: return "B";
        default: return "NONE";
    }
}

const char *motion_runtime_stall_diag_active_side_text(void)
{
    switch (stall_diag_active_side)
    {
        case MOTOR_TEST_SIDE_LEFT: return "L";
        case MOTOR_TEST_SIDE_RIGHT: return "R";
        default: return "NONE";
    }
}

uint8 motion_runtime_stall_diag_stage(void)
{
    return stall_diag_stage;
}

uint16 motion_runtime_stall_diag_applied_pwm(void)
{
    return stall_diag_applied_pwm;
}

uint32 motion_runtime_stall_diag_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = stall_diag_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_stall_diag_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = stall_diag_right_total;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_stall_diag_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = stall_diag_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_stall_diag_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = stall_diag_right_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_stall_diag_left_breakaway_pwm(void)
{
    uint16 value;

    interrupt_global_disable();
    value = stall_diag_left_breakaway_pwm;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_stall_diag_right_breakaway_pwm(void)
{
    uint16 value;

    interrupt_global_disable();
    value = stall_diag_right_breakaway_pwm;
    interrupt_global_enable();
    return value;
}

uint8 motion_runtime_stall_diag_pass_mask(void)
{
    return stall_diag_pass_mask;
}

uint8 motion_runtime_track_test_start(void)
{
    g_track_test_t12_force_direction = 0;
    return motion_runtime_track_test_start_mode(TRACK_TEST_MODE_T10);
}

void motion_runtime_set_track_test_t12_force_direction(int8 direction)
{
    if (direction > 0)
    {
        g_track_test_t12_force_direction = 1;
    }
    else if (direction < 0)
    {
        g_track_test_t12_force_direction = -1;
    }
    else
    {
        g_track_test_t12_force_direction = 0;
    }
}

uint8 motion_runtime_track_test_start_mode(uint8 mode)
{
    uint16 target_value;

    if (track_test_active
        || raw_drive_active
        || stall_diag_active
        || motor_test_active
        || encoder_test_active
        || g_motion_run_unlocked
        || g_imu_runtime_state != IMU_RUNTIME_READY
        || g_motion_protect_reason != MOTION_PROTECT_NONE
        || element4_is_enabled()
        || !inductance4_calibration_valid
        || !inductance4_line_is_present()
        || motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK
        || negative_pressure_enabled
        || negative_pressure_armed
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || negative_pressure_real_output_percent != 0U)
    {
        return 0U;
    }
    if (mode != TRACK_TEST_MODE_T10 && mode != TRACK_TEST_MODE_T12)
    {
        return 0U;
    }

    motion_runtime_force_stop();
    ctimer_count_clean(MOTOR1_ENCODER);
    ctimer_count_clean(MOTOR2_ENCODER);
    g_encoder_left_raw = 0U;
    g_encoder_right_raw = 0U;
    g_encoder_left_signed = 0;
    g_encoder_right_signed = 0;
    l_encoder = 0.0f;
    r_encoder = 0.0f;
    l_encoder_last = 0.0f;
    r_encoder_last = 0.0f;
    l_speed_now = 0.0f;
    r_speed_now = 0.0f;

    track_test_saved_left_kp = L_pid.kp;
    track_test_saved_left_ki = L_pid.ki;
    track_test_saved_left_kd = L_pid.kd;
    track_test_saved_right_kp = R_pid.kp;
    track_test_saved_right_ki = R_pid.ki;
    track_test_saved_right_kd = R_pid.kd;
    track_test_pid_saved = 1U;
    L_pid.kp = TRACK_TEST_SPEED_KP;
    L_pid.ki = TRACK_TEST_SPEED_KI;
    L_pid.kd = 0.0f;
    R_pid.kp = TRACK_TEST_SPEED_KP;
    R_pid.ki = TRACK_TEST_SPEED_KI;
    R_pid.kd = 0.0f;

    reset_motion_pid_state();
    g_track_test_t12_direction = 0;
    g_track_test_t12_half_active = 0U;
    reset_track_test_exit_diagnostic();
    reset_track_test_steering_state();
    target_value = mode == TRACK_TEST_MODE_T12
        ? TRACK_TEST_T12_TARGET_VALUE
        : TRACK_TEST_T10_TARGET_VALUE;
#if TRACK_TEST_START_ASSIST_ENABLED
    track_test_target_value = (float)target_value;
    track_test_ramp_step = track_test_target_value
        / (float)TRACK_TEST_TARGET_RAMP_TICKS;
    track_test_decel_step = track_test_target_value
        / (float)TRACK_TEST_DECEL_RAMP_TICKS;
    change_speed_Target_base(0);
    track_test_ramped_target = 0.0f;
#else
    change_speed_Target_base((int)target_value);
#endif
    current_l_pwm_inc = 0.0f;
    current_r_pwm_inc = 0.0f;
    current_l_pwm_inc_last = 0.0f;
    current_r_pwm_inc_last = 0.0f;
    current_l_pwm_duty = 0.0f;
    current_r_pwm_duty = 0.0f;

    motion_reset_feedback_guards();
    g_motor_left_saturation_count = 0U;
    g_motor_right_saturation_count = 0U;
    g_motor_left_reversal_count = 0U;
    g_motor_right_reversal_count = 0U;
    track_test_line_lost_ticks = 0U;
    track_test_sample_count = 0U;
    track_test_left_speed_sum = 0.0f;
    track_test_right_speed_sum = 0.0f;
    track_test_left_speed_final = 0.0f;
    track_test_right_speed_final = 0.0f;
    track_test_left_pwm_final = 0U;
    track_test_right_pwm_final = 0U;
    g_track_test_start_sample_count = 0U;
    g_track_test_start_left_total = 0U;
    g_track_test_start_right_total = 0U;
    g_track_test_t10_start_release_mask = 0U;
    g_track_test_t10_start_release_sample_count = 0U;
    g_track_test_t10_start_release_left_total = 0U;
    g_track_test_t10_start_release_right_total = 0U;
    g_track_test_t10_start_stage = mode == TRACK_TEST_MODE_T10 ? 1U : 0U;
    g_track_test_t10_start_peak_pwm = mode == TRACK_TEST_MODE_T10
        ? (uint16)TRACK_TEST_T10_LEFT_BREAKAWAY_PWM : 0U;
    g_track_test_t10_right_start_stage = mode == TRACK_TEST_MODE_T10 ? 1U : 0U;
    g_track_test_t10_right_start_peak_pwm = mode == TRACK_TEST_MODE_T10
        ? (uint16)TRACK_TEST_T10_RIGHT_BREAKAWAY_PWM : 0U;
#if TRACK_TEST_START_ASSIST_ENABLED
    track_test_t10_start_active = mode == TRACK_TEST_MODE_T10 ? 1U : 0U;
    track_test_t10_left_confirm_ticks = 0U;
    track_test_t10_right_confirm_ticks = 0U;
    track_test_t10_steer_blend_ticks = mode == TRACK_TEST_MODE_T10
        ? 0U : TRACK_TEST_T10_STEER_BLEND_TICKS;
#endif
    track_test_event = TRACK_TEST_RESULT_IDLE;
    g_track_test_mode = mode;

    g_track_test_result = TRACK_TEST_RESULT_RUNNING;
    g_track_test_ticks_remaining = TRACK_TEST_DURATION_TICKS;
    g_motion_run_unlocked = 1U;
    pwm_state = 1U;
    Pwmout = 1U;
    track_test_active = 1U;
    return 1U;
}

uint8 motion_runtime_track_test_stop(void)
{
    uint8 was_active;

    interrupt_global_disable();
    was_active = track_test_active;
    if (was_active)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_STOPPED, 0U);
    }
    interrupt_global_enable();
    return was_active;
}

uint8 motion_runtime_track_test_abort_start_sync(void)
{
    if (!track_test_active || g_track_test_mode != TRACK_TEST_MODE_T12)
    {
        return 0U;
    }

    motion_track_test_finish(TRACK_TEST_RESULT_START_SYNC, 1U);
    return 1U;
}

void motion_runtime_track_test_tick(void)
{
    float left_speed;
    float right_speed;
    float left_pwm;
    float right_pwm;
#if TRACK_TEST_START_ASSIST_ENABLED
    uint16 start_monitor_samples;
#endif

    if (!track_test_active)
    {
        return;
    }

    if (g_imu_runtime_state != IMU_RUNTIME_READY)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_IMU, 1U);
        return;
    }
    if (motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK)
    {
        motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_MODE);
        motion_track_test_finish(TRACK_TEST_RESULT_PROTECT, 1U);
        return;
    }
    if (element4_is_enabled()
        || negative_pressure_enabled
        || negative_pressure_armed
        || negative_pressure_state != NEGATIVE_PRESSURE_STATE_OFF
        || negative_pressure_real_output_percent != 0U)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_PROTECT, 1U);
        return;
    }
    if (g_motion_protect_reason != MOTION_PROTECT_NONE)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_PROTECT, 1U);
        return;
    }
    if (!g_motion_run_unlocked || pwm_state != 1U)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_STOPPED, 1U);
        return;
    }

    if (!inductance4_line_is_present())
    {
        if (track_test_line_lost_ticks < 255U)
        {
            track_test_line_lost_ticks++;
        }
    }
    else
    {
        track_test_line_lost_ticks = 0U;
    }
    if (track_test_line_lost_ticks >= TRACK_TEST_LINE_LOST_TICKS)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_LINE_LOST, 1U);
        return;
    }

    left_speed = motion_abs_float(l_speed_now);
    right_speed = motion_abs_float(r_speed_now);
    left_pwm = motion_abs_float(g_motor_left_applied_pwm);
    right_pwm = motion_abs_float(g_motor_right_applied_pwm);

    track_test_left_speed_sum += left_speed;
    track_test_right_speed_sum += right_speed;
    track_test_left_speed_final = left_speed;
    track_test_right_speed_final = right_speed;
    track_test_left_pwm_final = (uint16)left_pwm;
    track_test_right_pwm_final = (uint16)right_pwm;
    if (track_test_sample_count < 65535U)
    {
        track_test_sample_count++;
    }

#if TRACK_TEST_START_ASSIST_ENABLED
    start_monitor_samples = g_track_test_mode == TRACK_TEST_MODE_T12
        ? TRACK_TEST_T12_START_MONITOR_SAMPLES
        : TRACK_TEST_START_SYNC_SAMPLES;
    if (g_track_test_start_sample_count < start_monitor_samples)
    {
        g_track_test_start_left_total += (uint32)g_encoder_left_raw;
        g_track_test_start_right_total += (uint32)g_encoder_right_raw;
        g_track_test_start_sample_count++;
    }

    if (track_test_ramped_target < track_test_target_value
        && g_track_test_start_sample_count <= TRACK_TEST_TARGET_RAMP_TICKS)
    {
        track_test_ramped_target += track_test_ramp_step;
        if (g_track_test_start_sample_count >= TRACK_TEST_TARGET_RAMP_TICKS
            || track_test_ramped_target > track_test_target_value)
        {
            track_test_ramped_target = track_test_target_value;
        }
        change_speed_Target_base((int)track_test_ramped_target);
    }
    if (g_track_test_ticks_remaining <= TRACK_TEST_DECEL_RAMP_TICKS)
    {
        track_test_ramped_target -= track_test_decel_step;
        if (track_test_ramped_target < 0.0f)
        {
            track_test_ramped_target = 0.0f;
        }
        change_speed_Target_base((int)track_test_ramped_target);
    }
#endif
    if (g_track_test_ticks_remaining > 0U)
    {
        g_track_test_ticks_remaining--;
    }
    if (g_track_test_ticks_remaining == 0U)
    {
        motion_track_test_finish(TRACK_TEST_RESULT_DONE, 1U);
    }
}

uint8 motion_runtime_track_test_is_active(void)
{
    return track_test_active;
}

void motion_runtime_track_t10_startup_tick(void)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    uint32 left_total;
    uint32 right_total;
    uint16 sample_count;

    if (!track_test_active || g_track_test_mode != TRACK_TEST_MODE_T10)
    {
        return;
    }

    if (!track_test_t10_start_active)
    {
        if (track_test_t10_steer_blend_ticks < TRACK_TEST_T10_STEER_BLEND_TICKS)
        {
            track_test_t10_steer_blend_ticks++;
        }
        return;
    }

    left_total = g_track_test_start_left_total + (uint32)g_encoder_left_raw;
    right_total = g_track_test_start_right_total + (uint32)g_encoder_right_raw;
    sample_count = g_track_test_start_sample_count < 65535U
        ? (uint16)(g_track_test_start_sample_count + 1U)
        : g_track_test_start_sample_count;

    if ((g_track_test_t10_start_release_mask & 0x01U) == 0U)
    {
        if (g_encoder_left_raw >= TRACK_TEST_T10_START_RAW_MIN)
        {
            if (track_test_t10_left_confirm_ticks < 255U)
            {
                track_test_t10_left_confirm_ticks++;
            }
        }
        else
        {
            track_test_t10_left_confirm_ticks = 0U;
        }
        if (track_test_t10_left_confirm_ticks >= TRACK_TEST_T10_START_CONFIRM_TICKS
            && left_total >= TRACK_TEST_T10_START_MOVE_TOTAL)
        {
            g_track_test_t10_start_release_mask |= 0x01U;
        }
    }

    if ((g_track_test_t10_start_release_mask & 0x02U) == 0U)
    {
        if (g_encoder_right_raw >= TRACK_TEST_T10_START_RAW_MIN)
        {
            if (track_test_t10_right_confirm_ticks < 255U)
            {
                track_test_t10_right_confirm_ticks++;
            }
        }
        else
        {
            track_test_t10_right_confirm_ticks = 0U;
        }
        if (track_test_t10_right_confirm_ticks >= TRACK_TEST_T10_START_CONFIRM_TICKS
            && right_total >= TRACK_TEST_T10_START_MOVE_TOTAL)
        {
            g_track_test_t10_start_release_mask |= 0x02U;
        }
    }

    if ((g_track_test_t10_start_release_mask & 0x02U) == 0U)
    {
        if (sample_count >= TRACK_TEST_T10_FINAL_SAMPLE)
        {
            g_track_test_t10_right_start_stage = 3U;
            g_track_test_t10_right_start_peak_pwm =
                (uint16)TRACK_TEST_T10_RIGHT_FINAL_PWM;
        }
        else if (sample_count >= TRACK_TEST_T10_RETRY_SAMPLE)
        {
            g_track_test_t10_right_start_stage = 2U;
            g_track_test_t10_right_start_peak_pwm =
                (uint16)TRACK_TEST_T10_RIGHT_RETRY_PWM;
        }
    }

    if ((g_track_test_t10_start_release_mask & 0x01U) == 0U)
    {
        if (sample_count >= TRACK_TEST_T10_FINAL_SAMPLE)
        {
            g_track_test_t10_start_stage = 3U;
            g_track_test_t10_start_peak_pwm =
                (uint16)TRACK_TEST_T10_LEFT_FINAL_PWM;
        }
        else if (sample_count >= TRACK_TEST_T10_RETRY_SAMPLE)
        {
            g_track_test_t10_start_stage = 2U;
            g_track_test_t10_start_peak_pwm =
                (uint16)TRACK_TEST_T10_LEFT_RETRY_PWM;
        }
    }

    if (g_track_test_t10_start_release_mask == 0x03U
        && track_test_ramped_target >= TRACK_TEST_T10_RELEASE_TARGET)
    {
        track_test_t10_start_active = 0U;
        track_test_t10_steer_blend_ticks = 0U;
        g_track_test_t10_start_release_sample_count = sample_count;
        g_track_test_t10_start_release_left_total = left_total;
        g_track_test_t10_start_release_right_total = right_total;
        return;
    }

    if (sample_count >= TRACK_TEST_START_SYNC_SAMPLES)
    {
        if ((g_track_test_t10_start_release_mask & 0x01U) == 0U)
        {
            motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_LEFT_STALL);
        }
        else if ((g_track_test_t10_start_release_mask & 0x02U) == 0U)
        {
            motion_runtime_trigger_protection(MOTION_PROTECT_ENCODER_RIGHT_STALL);
        }
    }
#endif
}

uint8 motion_runtime_track_t10_startup_is_active(void)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    return track_test_active
        && g_track_test_mode == TRACK_TEST_MODE_T10
        && track_test_t10_start_active;
#else
    return 0U;
#endif
}

float motion_runtime_track_t10_left_start_pwm(void)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    if (!motion_runtime_track_t10_startup_is_active())
    {
        return 0.0f;
    }
    if ((g_track_test_t10_start_release_mask & 0x01U) != 0U)
    {
        return TRACK_TEST_T10_LEFT_BREAKAWAY_PWM;
    }
    if (g_track_test_t10_start_stage >= 3U)
    {
        return TRACK_TEST_T10_LEFT_FINAL_PWM;
    }
    if (g_track_test_t10_start_stage >= 2U)
    {
        return TRACK_TEST_T10_LEFT_RETRY_PWM;
    }
    return TRACK_TEST_T10_LEFT_BREAKAWAY_PWM;
#else
    return 0.0f;
#endif
}

float motion_runtime_track_t10_right_start_pwm(void)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    if (!motion_runtime_track_t10_startup_is_active())
    {
        return 0.0f;
    }
    if ((g_track_test_t10_start_release_mask & 0x02U) != 0U)
    {
        return TRACK_TEST_T10_RIGHT_BREAKAWAY_PWM;
    }
    if (g_track_test_t10_right_start_stage >= 3U)
    {
        return TRACK_TEST_T10_RIGHT_FINAL_PWM;
    }
    if (g_track_test_t10_right_start_stage >= 2U)
    {
        return TRACK_TEST_T10_RIGHT_RETRY_PWM;
    }
    return TRACK_TEST_T10_RIGHT_BREAKAWAY_PWM;
#else
    return 0.0f;
#endif
}

float motion_runtime_track_t10_steering_scale(void)
{
#if TRACK_TEST_START_ASSIST_ENABLED
    if (motion_runtime_track_t10_startup_is_active())
    {
        return 0.0f;
    }
    if (track_test_t10_steer_blend_ticks >= TRACK_TEST_T10_STEER_BLEND_TICKS)
    {
        return 1.0f;
    }
    return (float)track_test_t10_steer_blend_ticks
        / (float)TRACK_TEST_T10_STEER_BLEND_TICKS;
#else
    return 1.0f;
#endif
}

uint16 motion_runtime_track_test_remaining_ms(void)
{
    return (uint16)(
        g_track_test_ticks_remaining * MOTOR_TEST_CONTROL_PERIOD_MS);
}

uint16 motion_runtime_track_test_sample_count(void)
{
    return track_test_sample_count;
}

int32 motion_runtime_track_test_left_average_x10(void)
{
    if (track_test_sample_count == 0U)
    {
        return 0;
    }
    return (int32)(
        track_test_left_speed_sum * 10.0f / (float)track_test_sample_count);
}

int32 motion_runtime_track_test_right_average_x10(void)
{
    if (track_test_sample_count == 0U)
    {
        return 0;
    }
    return (int32)(
        track_test_right_speed_sum * 10.0f / (float)track_test_sample_count);
}

int32 motion_runtime_track_test_left_final_x10(void)
{
    return (int32)(track_test_left_speed_final * 10.0f);
}

int32 motion_runtime_track_test_right_final_x10(void)
{
    return (int32)(track_test_right_speed_final * 10.0f);
}

uint16 motion_runtime_track_test_left_pwm_final(void)
{
    return track_test_left_pwm_final;
}

uint16 motion_runtime_track_test_right_pwm_final(void)
{
    return track_test_right_pwm_final;
}

uint16 motion_runtime_track_test_match_x1000(void)
{
    float left_average;
    float right_average;
    float smaller;
    float larger;

    if (track_test_sample_count == 0U)
    {
        return 0U;
    }
    left_average = track_test_left_speed_sum / (float)track_test_sample_count;
    right_average = track_test_right_speed_sum / (float)track_test_sample_count;
    smaller = left_average < right_average ? left_average : right_average;
    larger = left_average > right_average ? left_average : right_average;
    if (larger <= 0.0f)
    {
        return 0U;
    }
    return (uint16)(smaller * 1000.0f / larger);
}

TrackTestResult motion_runtime_track_test_take_event(void)
{
    TrackTestResult event;

    interrupt_global_disable();
    event = (TrackTestResult)track_test_event;
    track_test_event = TRACK_TEST_RESULT_IDLE;
    interrupt_global_enable();
    return event;
}

const char *motion_runtime_track_test_result_text(void)
{
    switch (g_track_test_result)
    {
        case TRACK_TEST_RESULT_RUNNING: return "RUNNING";
        case TRACK_TEST_RESULT_DONE: return "DONE";
        case TRACK_TEST_RESULT_STOPPED: return "STOPPED";
        case TRACK_TEST_RESULT_LINE_LOST: return "LINE_LOST";
        case TRACK_TEST_RESULT_IMU: return "IMU";
        case TRACK_TEST_RESULT_PROTECT: return "PROTECT";
        case TRACK_TEST_RESULT_START_SYNC: return "START_SYNC";
        default: return "IDLE";
    }
}

uint8 motion_runtime_encoder_test_start(MotorTestSide side)
{
    if ((side != MOTOR_TEST_SIDE_LEFT && side != MOTOR_TEST_SIDE_RIGHT)
        || g_motion_run_unlocked
        || motor_test_active
        || track_test_active
        || stall_diag_active
        || raw_drive_active)
    {
        return 0U;
    }

    motion_runtime_force_stop();
    ctimer_count_clean(MOTOR1_ENCODER);
    ctimer_count_clean(MOTOR2_ENCODER);
    g_encoder_left_raw = 0U;
    g_encoder_right_raw = 0U;
    g_encoder_left_signed = 0;
    g_encoder_right_signed = 0;
    encoder_test_active = 0U;
    encoder_test_event = ENCODER_TEST_RESULT_IDLE;
    encoder_test_left_total = 0U;
    encoder_test_right_total = 0U;
    encoder_test_left_peak = 0U;
    encoder_test_right_peak = 0U;
    g_encoder_test_side = (uint8)side;
    g_encoder_test_result = ENCODER_TEST_RESULT_RUNNING;
    g_encoder_test_ticks_remaining = ENCODER_TEST_DURATION_TICKS;
    encoder_test_active = 1U;
    return 1U;
}

uint8 motion_runtime_encoder_test_stop(void)
{
    uint8 was_active;

    interrupt_global_disable();
    was_active = encoder_test_active;
    if (was_active)
    {
        motion_encoder_test_finish(
            ENCODER_TEST_RESULT_STOPPED,
            1U);
    }
    else
    {
        motion_runtime_force_stop();
    }
    interrupt_global_enable();
    return was_active;
}

void motion_runtime_encoder_test_tick(void)
{
    uint16 left_raw;
    uint16 right_raw;

    if (!encoder_test_active)
    {
        return;
    }

    motion_runtime_force_stop();
    if (motion_runtime_encoder_mode_mask() != MOTOR_TEST_ENCODER_MODE_MASK)
    {
        motion_encoder_test_finish(
            ENCODER_TEST_RESULT_ENCODER_MODE,
            1U);
        return;
    }

    left_raw = g_encoder_left_raw;
    right_raw = g_encoder_right_raw;
    if (encoder_test_left_total <= (0xFFFFFFFFUL - (uint32)left_raw))
    {
        encoder_test_left_total += (uint32)left_raw;
    }
    else
    {
        encoder_test_left_total = 0xFFFFFFFFUL;
    }
    if (encoder_test_right_total <= (0xFFFFFFFFUL - (uint32)right_raw))
    {
        encoder_test_right_total += (uint32)right_raw;
    }
    else
    {
        encoder_test_right_total = 0xFFFFFFFFUL;
    }
    if (left_raw > encoder_test_left_peak)
    {
        encoder_test_left_peak = left_raw;
    }
    if (right_raw > encoder_test_right_peak)
    {
        encoder_test_right_peak = right_raw;
    }

    if (g_encoder_test_ticks_remaining > 0U)
    {
        g_encoder_test_ticks_remaining--;
    }
    if (g_encoder_test_ticks_remaining == 0U)
    {
        motion_encoder_test_finish(
            ENCODER_TEST_RESULT_DONE,
            1U);
    }
}

uint8 motion_runtime_encoder_test_is_active(void)
{
    return encoder_test_active;
}

uint16 motion_runtime_encoder_test_remaining_ms(void)
{
    return (uint16)(
        g_encoder_test_ticks_remaining * MOTOR_TEST_CONTROL_PERIOD_MS);
}

uint32 motion_runtime_encoder_test_left_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = encoder_test_left_total;
    interrupt_global_enable();
    return value;
}

uint32 motion_runtime_encoder_test_right_total(void)
{
    uint32 value;

    interrupt_global_disable();
    value = encoder_test_right_total;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_encoder_test_left_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = encoder_test_left_peak;
    interrupt_global_enable();
    return value;
}

uint16 motion_runtime_encoder_test_right_peak(void)
{
    uint16 value;

    interrupt_global_disable();
    value = encoder_test_right_peak;
    interrupt_global_enable();
    return value;
}

EncoderTestResult motion_runtime_encoder_test_take_event(void)
{
    EncoderTestResult event;

    interrupt_global_disable();
    event = (EncoderTestResult)encoder_test_event;
    encoder_test_event = ENCODER_TEST_RESULT_IDLE;
    interrupt_global_enable();
    return event;
}

const char *motion_runtime_encoder_test_side_text(void)
{
    switch (g_encoder_test_side)
    {
        case MOTOR_TEST_SIDE_LEFT: return "L";
        case MOTOR_TEST_SIDE_RIGHT: return "R";
        default: return "NONE";
    }
}

const char *motion_runtime_encoder_test_result_text(void)
{
    switch (g_encoder_test_result)
    {
        case ENCODER_TEST_RESULT_RUNNING: return "RUNNING";
        case ENCODER_TEST_RESULT_DONE: return "DONE";
        case ENCODER_TEST_RESULT_STOPPED: return "STOPPED";
        case ENCODER_TEST_RESULT_ENCODER_MODE: return "ENC_MODE";
        default: return "IDLE";
    }
}

const char *motion_runtime_imu_state_text(void)
{
    switch (g_imu_runtime_state)
    {
        case IMU_RUNTIME_CALIBRATING: return "CAL";
        case IMU_RUNTIME_READY: return "OK";
        case IMU_RUNTIME_UNSTABLE: return "UNSTABLE";
        default: return "MISSING";
    }
}

const char *motion_runtime_protect_reason_text(void)
{
    switch (g_motion_protect_reason)
    {
        case MOTION_PROTECT_RUN_LOCKED: return "RUN_LOCKED";
        case MOTION_PROTECT_IMU: return "IMU";
        case MOTION_PROTECT_ENCODER_LEFT_STALL: return "L_STALL";
        case MOTION_PROTECT_ENCODER_RIGHT_STALL: return "R_STALL";
        case MOTION_PROTECT_ENCODER_LEFT_DIRECTION: return "L_DIR";
        case MOTION_PROTECT_ENCODER_RIGHT_DIRECTION: return "R_DIR";
        case MOTION_PROTECT_ENCODER_SPIKE: return "ENC_SPIKE";
        case MOTION_PROTECT_SPEED_SATURATION: return "SATURATION";
        case MOTION_PROTECT_ENCODER_MODE: return "ENC_MODE";
        case MOTION_PROTECT_ENCODER_NOISE: return "ENC_NOISE";
        default: return "NONE";
    }
}
