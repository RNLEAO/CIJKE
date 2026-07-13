#include "headfile.h"

#define IMU_CALIBRATION_DISCARD_SAMPLES    20U
#define IMU_CALIBRATION_MAX_SPAN_DPS       12.0f
#define IMU_CALIBRATION_MAX_BIAS_DPS       30.0f

#define MOTOR_PWM_LIMIT                  1000.0f
#define MOTOR_REVERSE_DEADTIME_TICKS       10U
#define MOTOR_STALL_PWM_MIN               350.0f
#define MOTOR_STALL_TARGET_MIN             20.0f
#define MOTOR_STALL_RAW_MAX                 1U
#define MOTOR_STALL_TICKS                  60U
#define MOTOR_DIRECTION_SPEED_MIN           5.0f
#define MOTOR_DIRECTION_TARGET_MIN         20.0f
#define MOTOR_DIRECTION_FAULT_TICKS        20U
#define MOTOR_SATURATION_THRESHOLD        995.0f
#define MOTOR_SATURATION_FAULT_TICKS      100U
#define ENCODER_RAW_SPIKE_LIMIT         10000U
#define ENCODER_SPIKE_FAULT_TICKS           3U

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

float g_track_duty_limit = 0.45f;
float g_speed_pid_delta_limit = 40.0f;
float g_motor_pwm_slew_per_tick = 25.0f;

static MotorOutputState left_output_state = {0.0f, 0, 0, 0U};
static MotorOutputState right_output_state = {0.0f, 0, 0, 0U};

static uint8 left_stall_ticks = 0U;
static uint8 right_stall_ticks = 0U;
static uint8 left_direction_fault_ticks = 0U;
static uint8 right_direction_fault_ticks = 0U;
static uint8 left_saturation_ticks = 0U;
static uint8 right_saturation_ticks = 0U;
static uint8 encoder_spike_ticks = 0U;

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

    g_motion_protect_reason = MOTION_PROTECT_NONE;
    pwm_state = 0U;
    Pwmout = 0U;
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
    float left_requested_pwm,
    float right_requested_pwm,
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

    if (motion_abs_float(g_motor_left_applied_pwm) >= MOTOR_STALL_PWM_MIN
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

    if (motion_abs_float(g_motor_right_applied_pwm) >= MOTOR_STALL_PWM_MIN
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

    if (motion_abs_float(left_requested_pwm) >= MOTOR_SATURATION_THRESHOLD)
    {
        if (g_motor_left_saturation_count < 65535U) g_motor_left_saturation_count++;
        if (left_saturation_ticks < 255U) left_saturation_ticks++;
    }
    else
    {
        left_saturation_ticks = 0U;
    }

    if (motion_abs_float(right_requested_pwm) >= MOTOR_SATURATION_THRESHOLD)
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

    left_output_state.applied_abs_pwm = motion_approach(
        left_output_state.applied_abs_pwm,
        desired_abs,
        g_motor_pwm_slew_per_tick);

    LEFT_MOTOR_DIR = left_output_state.applied_sign > 0 ? 1 : 0;
    pwm_set_duty(LEFT_MOTOR_PWM, (uint32)left_output_state.applied_abs_pwm);
    g_motor_left_applied_pwm = left_output_state.applied_sign > 0
        ? left_output_state.applied_abs_pwm
        : -left_output_state.applied_abs_pwm;
}

static void motion_apply_right_output(float request)
{
    int8 desired_sign = motion_sign_float(request);
    float desired_abs = motion_abs_float(request);

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

    right_output_state.applied_abs_pwm = motion_approach(
        right_output_state.applied_abs_pwm,
        desired_abs,
        g_motor_pwm_slew_per_tick);

    RIGHT_MOTOR_DIR = right_output_state.applied_sign > 0 ? 1 : 0;
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
        default: return "NONE";
    }
}
