#ifndef CIJIANKE_MOTION_RUNTIME_H
#define CIJIANKE_MOTION_RUNTIME_H

#include "zf_common_typedef.h"

#define RACE_MINIMAL_BUILD       1U
#define MOTOR_PWM_LIMIT_VALUE   2000.0f
#define MOTOR_TEST_PWM_VALUE    2000U
#define MOTOR_TEST_BOTH_PWM_VALUE 2000U
#define MOTOR_TEST_DURATION_MS  1000U
#define MOTOR_TEST_PRECHECK_MS  50U
#define STALL_DIAG_PRECHECK_MS  50U
#define STALL_DIAG_STAGE_MS     100U
#define STALL_DIAG_PWM_LOW_VALUE  600U
#define STALL_DIAG_PWM_MID_VALUE 1000U
#define STALL_DIAG_PWM_HIGH_VALUE 1400U
#define ENCODER_TEST_DURATION_MS 15000U
#define TRACK_TEST_T10_TARGET_VALUE 180U
#define TRACK_TEST_T12_TARGET_VALUE 120U
#define TRACK_TEST_TARGET_VALUE TRACK_TEST_T10_TARGET_VALUE
#define TRACK_TEST_DURATION_MS  3000U
#define TRACK_TEST_LINE_LOST_MS   80U
#define TRACK_TEST_STARTUP_GRACE_MS 500U
#define TRACK_TEST_START_ASSIST_ENABLED 1U
#define TRACK_TEST_STEERING_ENABLED 1U
#define TRACK_TEST_T10_TURN_GAIN 0.12f
#define TRACK_TEST_T12_LINE_TURN_GAIN 0.12f
#define TRACK_TEST_TURN_DEADBAND 0.02f
#define TRACK_TEST_TURN_RATIO_LIMIT 0.10f
#define TRACK_TEST_MODE_T10 0U
#define TRACK_TEST_MODE_T12 1U
#define TRACK_TEST_T10_CLOSED_LOOP_SEED_PWM 1000.0f
#if TRACK_TEST_START_ASSIST_ENABLED
#define TRACK_TEST_TARGET_RAMP_MS 500U
#define TRACK_TEST_DECEL_RAMP_MS 500U
#define TRACK_TEST_START_SYNC_SAMPLES 100U
#define TRACK_TEST_T12_START_MONITOR_SAMPLES 140U
#define TRACK_TEST_START_BREAKAWAY_PWM 600.0f
#define TRACK_TEST_T10_START_RELEASE_COUNT 100U
#define TRACK_TEST_T12_START_SYNC_RELEASE_COUNT 300U
#define TRACK_TEST_T10_LEFT_BREAKAWAY_PWM 1400.0f
#define TRACK_TEST_T10_LEFT_RETRY_PWM 1700.0f
#define TRACK_TEST_T10_LEFT_FINAL_PWM 2000.0f
#define TRACK_TEST_T10_RIGHT_BREAKAWAY_PWM 600.0f
#define TRACK_TEST_T10_RIGHT_RETRY_PWM 1000.0f
#define TRACK_TEST_T10_RIGHT_FINAL_PWM 1400.0f
#define TRACK_TEST_T10_BREAKAWAY_RAMP_TICKS 10U
#define TRACK_TEST_T10_RETRY_SAMPLE 40U
#define TRACK_TEST_T10_FINAL_SAMPLE 60U
#define TRACK_TEST_T10_START_MOVE_TOTAL 8U
#define TRACK_TEST_T10_START_RAW_MIN 2U
#define TRACK_TEST_T10_START_CONFIRM_TICKS 2U
#define TRACK_TEST_T10_RELEASE_TARGET 60.0f
#define TRACK_TEST_T10_STEER_BLEND_MS 100U
#define TRACK_TEST_T10_SYNC_MS 200U
#define TRACK_TEST_T10_SYNC_GAIN 0.30f
#define TRACK_TEST_T10_SYNC_LIMIT 0.12f
#define TRACK_TEST_T10_SYNC_MIN_TOTAL 8U
#endif

typedef enum
{
    IMU_RUNTIME_MISSING = 0,
    IMU_RUNTIME_CALIBRATING,
    IMU_RUNTIME_READY,
    IMU_RUNTIME_UNSTABLE
} ImuRuntimeState;

typedef enum
{
    MOTION_PROTECT_NONE = 0,
    MOTION_PROTECT_RUN_LOCKED,
    MOTION_PROTECT_IMU,
    MOTION_PROTECT_ENCODER_LEFT_STALL,
    MOTION_PROTECT_ENCODER_RIGHT_STALL,
    MOTION_PROTECT_ENCODER_LEFT_DIRECTION,
    MOTION_PROTECT_ENCODER_RIGHT_DIRECTION,
    MOTION_PROTECT_ENCODER_SPIKE,
    MOTION_PROTECT_SPEED_SATURATION,
    MOTION_PROTECT_ENCODER_MODE,
    MOTION_PROTECT_ENCODER_NOISE
} MotionProtectReason;

typedef enum
{
    MOTOR_TEST_SIDE_NONE = 0,
    MOTOR_TEST_SIDE_LEFT,
    MOTOR_TEST_SIDE_RIGHT,
    MOTOR_TEST_SIDE_BOTH
} MotorTestSide;

typedef enum
{
    MOTOR_TEST_RESULT_IDLE = 0,
    MOTOR_TEST_RESULT_RUNNING,
    MOTOR_TEST_RESULT_DONE,
    MOTOR_TEST_RESULT_STOPPED,
    MOTOR_TEST_RESULT_LEFT_STALL,
    MOTOR_TEST_RESULT_RIGHT_STALL,
    MOTOR_TEST_RESULT_LEFT_DIRECTION,
    MOTOR_TEST_RESULT_RIGHT_DIRECTION,
    MOTOR_TEST_RESULT_IMU,
    MOTOR_TEST_RESULT_PROTECT,
    MOTOR_TEST_RESULT_ENCODER_MODE,
    MOTOR_TEST_RESULT_ENCODER_NOISE
} MotorTestResult;

typedef enum
{
    ENCODER_TEST_RESULT_IDLE = 0,
    ENCODER_TEST_RESULT_RUNNING,
    ENCODER_TEST_RESULT_DONE,
    ENCODER_TEST_RESULT_STOPPED,
    ENCODER_TEST_RESULT_ENCODER_MODE
} EncoderTestResult;

typedef enum
{
    TRACK_TEST_RESULT_IDLE = 0,
    TRACK_TEST_RESULT_RUNNING,
    TRACK_TEST_RESULT_DONE,
    TRACK_TEST_RESULT_STOPPED,
    TRACK_TEST_RESULT_LINE_LOST,
    TRACK_TEST_RESULT_IMU,
    TRACK_TEST_RESULT_PROTECT
} TrackTestResult;

typedef enum
{
    STALL_DIAG_RESULT_IDLE = 0,
    STALL_DIAG_RESULT_RUNNING,
    STALL_DIAG_RESULT_PASS,
    STALL_DIAG_RESULT_STOPPED,
    STALL_DIAG_RESULT_LEFT_STALL,
    STALL_DIAG_RESULT_RIGHT_STALL,
    STALL_DIAG_RESULT_IMU,
    STALL_DIAG_RESULT_PROTECT,
    STALL_DIAG_RESULT_ENCODER_MODE,
    STALL_DIAG_RESULT_ENCODER_NOISE
} StallDiagResult;

extern volatile uint8 g_imu_runtime_state;
extern volatile uint8 g_motion_run_unlocked;
extern volatile uint8 g_motion_protect_reason;

extern float g_imu_gyro_x_dps;
extern float g_imu_gyro_y_dps;
extern float g_imu_gyro_z_dps;
extern float g_imu_bias_x_dps;
extern float g_imu_bias_y_dps;
extern float g_imu_bias_z_dps;
extern float g_imu_turn_rate_dps;

extern uint16 g_encoder_left_raw;
extern uint16 g_encoder_right_raw;
extern int32 g_encoder_left_signed;
extern int32 g_encoder_right_signed;
extern uint8 g_encoder_left_phase;
extern uint8 g_encoder_right_phase;

extern float g_motor_left_applied_pwm;
extern float g_motor_right_applied_pwm;
extern uint16 g_motor_left_saturation_count;
extern uint16 g_motor_right_saturation_count;
extern uint16 g_motor_left_reversal_count;
extern uint16 g_motor_right_reversal_count;

extern volatile uint8 g_motor_test_side;
extern volatile uint8 g_motor_test_result;
extern volatile uint16 g_motor_test_ticks_remaining;

extern volatile uint8 g_encoder_test_side;
extern volatile uint8 g_encoder_test_result;
extern volatile uint16 g_encoder_test_ticks_remaining;

extern volatile uint8 g_track_test_result;
extern volatile uint8 g_track_test_mode;
extern volatile int8 g_track_test_t12_direction;
extern volatile int8 g_track_test_t12_force_direction;
extern volatile uint8 g_track_test_t12_half_active;
extern volatile uint16 g_track_test_ticks_remaining;
extern volatile uint16 g_track_test_start_sample_count;
extern volatile uint32 g_track_test_start_left_total;
extern volatile uint32 g_track_test_start_right_total;
extern volatile uint8 g_track_test_t10_start_release_mask;
extern volatile uint16 g_track_test_t10_start_release_sample_count;
extern volatile uint32 g_track_test_t10_start_release_left_total;
extern volatile uint32 g_track_test_t10_start_release_right_total;
extern volatile uint8 g_track_test_t10_start_stage;
extern volatile uint16 g_track_test_t10_start_peak_pwm;
extern volatile uint8 g_track_test_t10_right_start_stage;
extern volatile uint16 g_track_test_t10_right_start_peak_pwm;
extern volatile uint16 g_track_test_t10_sync_sample_count;
extern volatile uint32 g_track_test_t10_sync_left_total;
extern volatile uint32 g_track_test_t10_sync_right_total;
extern volatile int16 g_track_test_t10_sync_final_x1000;
extern volatile uint16 g_track_test_t10_sync_peak_x1000;

extern float g_track_duty_limit;
extern float g_speed_pid_delta_limit;
extern float g_motor_pwm_slew_per_tick;

uint8 motion_runtime_init_imu(void);
uint8 motion_runtime_calibrate_imu(uint16 sample_count, uint16 sample_delay_ms);
void motion_runtime_update_imu(void);

void motion_runtime_set_encoder_sample(
    uint16 left_raw,
    uint16 right_raw,
    int32 left_signed,
    int32 right_signed,
    uint8 left_phase,
    uint8 right_phase);

float motion_runtime_limit_pid_delta(float delta_pwm);
float motion_runtime_line_speed_scale(uint16 line_sum);
void motion_runtime_check_feedback(
    float left_target,
    float right_target,
    float left_speed,
    float right_speed,
    float left_applied_pwm,
    float right_applied_pwm,
    uint8 motor_running);

void motion_runtime_apply_outputs(
    float left_requested_pwm,
    float right_requested_pwm,
    uint8 motor_running);
void motion_runtime_force_stop(void);

uint8 motion_runtime_motor_test_start(MotorTestSide side);
uint8 motion_runtime_motor_test_stop(void);
void motion_runtime_motor_test_tick(void);
uint8 motion_runtime_motor_test_is_active(void);
uint16 motion_runtime_motor_test_remaining_ms(void);
uint16 motion_runtime_motor_test_pwm_value(void);
uint32 motion_runtime_motor_test_pulse_total(void);
uint16 motion_runtime_motor_test_peak_raw(void);
uint32 motion_runtime_motor_test_left_total(void);
uint32 motion_runtime_motor_test_right_total(void);
uint32 motion_runtime_motor_test_difference(void);
uint16 motion_runtime_motor_test_balance_x1000(void);
uint16 motion_runtime_motor_test_left_peak(void);
uint16 motion_runtime_motor_test_right_peak(void);
uint16 motion_runtime_motor_test_left_idle_peak(void);
uint16 motion_runtime_motor_test_right_idle_peak(void);
uint8 motion_runtime_encoder_mode_mask(void);
MotorTestResult motion_runtime_motor_test_take_event(void);
const char *motion_runtime_motor_test_side_text(void);
const char *motion_runtime_motor_test_result_text(void);
uint8 motion_runtime_motor_test_both_passed(void);

uint8 motion_runtime_stall_diag_start(MotorTestSide side);
uint8 motion_runtime_stall_diag_stop(void);
void motion_runtime_stall_diag_tick(void);
uint8 motion_runtime_stall_diag_is_active(void);
StallDiagResult motion_runtime_stall_diag_take_event(void);
StallDiagResult motion_runtime_stall_diag_result(void);
const char *motion_runtime_stall_diag_result_text(void);
const char *motion_runtime_stall_diag_requested_side_text(void);
const char *motion_runtime_stall_diag_active_side_text(void);
uint8 motion_runtime_stall_diag_stage(void);
uint16 motion_runtime_stall_diag_applied_pwm(void);
uint32 motion_runtime_stall_diag_left_total(void);
uint32 motion_runtime_stall_diag_right_total(void);
uint16 motion_runtime_stall_diag_left_peak(void);
uint16 motion_runtime_stall_diag_right_peak(void);
uint16 motion_runtime_stall_diag_left_breakaway_pwm(void);
uint16 motion_runtime_stall_diag_right_breakaway_pwm(void);
uint8 motion_runtime_stall_diag_pass_mask(void);

uint8 motion_runtime_encoder_test_start(MotorTestSide side);
uint8 motion_runtime_encoder_test_stop(void);
void motion_runtime_encoder_test_tick(void);
uint8 motion_runtime_encoder_test_is_active(void);
uint16 motion_runtime_encoder_test_remaining_ms(void);
uint32 motion_runtime_encoder_test_left_total(void);
uint32 motion_runtime_encoder_test_right_total(void);
uint16 motion_runtime_encoder_test_left_peak(void);
uint16 motion_runtime_encoder_test_right_peak(void);
EncoderTestResult motion_runtime_encoder_test_take_event(void);
const char *motion_runtime_encoder_test_side_text(void);
const char *motion_runtime_encoder_test_result_text(void);

uint8 motion_runtime_track_test_start(void);
uint8 motion_runtime_track_test_start_mode(uint8 mode);
void motion_runtime_set_track_test_t12_force_direction(int8 direction);
uint8 motion_runtime_track_test_stop(void);
void motion_runtime_track_test_tick(void);
uint8 motion_runtime_track_test_is_active(void);
void motion_runtime_track_t10_startup_tick(void);
uint8 motion_runtime_track_t10_startup_is_active(void);
float motion_runtime_track_t10_left_start_pwm(void);
float motion_runtime_track_t10_right_start_pwm(void);
float motion_runtime_track_t10_steering_scale(void);
float motion_runtime_track_t10_sync_ratio(void);
uint16 motion_runtime_track_test_remaining_ms(void);
uint16 motion_runtime_track_test_sample_count(void);
int32 motion_runtime_track_test_left_average_x10(void);
int32 motion_runtime_track_test_right_average_x10(void);
int32 motion_runtime_track_test_left_final_x10(void);
int32 motion_runtime_track_test_right_final_x10(void);
uint16 motion_runtime_track_test_left_pwm_final(void);
uint16 motion_runtime_track_test_right_pwm_final(void);
uint16 motion_runtime_track_test_match_x1000(void);
TrackTestResult motion_runtime_track_test_take_event(void);
const char *motion_runtime_track_test_result_text(void);

void motion_runtime_set_run_unlocked(uint8 unlocked);
uint8 motion_runtime_can_run(void);
void motion_runtime_trigger_protection(MotionProtectReason reason);
uint8 motion_runtime_clear_protection(void);

const char *motion_runtime_imu_state_text(void);
const char *motion_runtime_protect_reason_text(void);

#endif
