#ifndef CIJIANKE_MOTION_RUNTIME_H
#define CIJIANKE_MOTION_RUNTIME_H

#include "zf_common_typedef.h"

#define MOTOR_TEST_PWM_VALUE    300U
#define MOTOR_TEST_DURATION_MS  500U
#define ENCODER_TEST_DURATION_MS 15000U

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
    MOTOR_TEST_SIDE_RIGHT
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
    float left_requested_pwm,
    float right_requested_pwm,
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
uint32 motion_runtime_motor_test_pulse_total(void);
uint16 motion_runtime_motor_test_peak_raw(void);
uint32 motion_runtime_motor_test_left_total(void);
uint32 motion_runtime_motor_test_right_total(void);
uint16 motion_runtime_motor_test_left_peak(void);
uint16 motion_runtime_motor_test_right_peak(void);
uint16 motion_runtime_motor_test_left_idle_peak(void);
uint16 motion_runtime_motor_test_right_idle_peak(void);
uint8 motion_runtime_encoder_mode_mask(void);
MotorTestResult motion_runtime_motor_test_take_event(void);
const char *motion_runtime_motor_test_side_text(void);
const char *motion_runtime_motor_test_result_text(void);

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

void motion_runtime_set_run_unlocked(uint8 unlocked);
uint8 motion_runtime_can_run(void);
void motion_runtime_trigger_protection(MotionProtectReason reason);
uint8 motion_runtime_clear_protection(void);

const char *motion_runtime_imu_state_text(void);
const char *motion_runtime_protect_reason_text(void);

#endif
