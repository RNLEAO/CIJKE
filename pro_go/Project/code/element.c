#include "headfile.h"
#include "inductance4.h"

extern void reset_motion_pid_state(void);

#define ELEMENT4_CONFIRM_TICKS          3U
#define ELEMENT4_RECOVER_TICKS         25U
#define ELEMENT4_RIGHT_ANGLE_TIMEOUT  140U
#define ELEMENT4_RING_TIMEOUT         500U
#define ELEMENT4_LINE_SUM_MIN          35.0f
#define ELEMENT4_LINE_ERROR_MAX         0.45f

Element4State element4_state = ELEMENT4_TRACK;
int8 element4_direction = 0;

uint8 cir_flag = 0;
uint8 encoder_sign = 0;
uint8 yuansu_flag = 1;
int16 circle_exit_count = 0;
uint16 temp_flag_speed = 0;
uint16 cir_angle_flag = 0;
uint8 circle_enter_case = 0;
float temp_flag = 0.0f;
float run_mode = 0.0f;

float in_circle_LR = 176.0f;
float in_circle_MID = 0.0f;
float in_circle_LRMID = 18.0f;
float ring_error = 0.72f;
float ring_exit_error = 0.45f;
float ring_inc_element12 = 0.21f;
float ring_inc_element56 = 0.30f;
float ring_inc_element67 = 0.35f;
float ring_angle_23 = 15.0f;
float ring_angle_34 = 120.0f;
float ring_angle_45 = 280.0f;
float temp_flag_tar = 30.0f;

char right_angle_flag = 0;
int right_angle_count = 0;
float straight_err_threshold = 0.28f;
float straight_integral_threshold = 0.15f;

float right_angle_outer_low = 40.0f;
float right_angle_inner_high = 90.0f;
float right_angle_inner_low = 70.0f;
float right_angle_inner_diff = 45.0f;
float right_angle_min_angle = 65.0f;
float right_angle_target_angle = 82.0f;
float right_angle_reverse_speed = 120.0f;
float right_angle_forward_speed = 100.0f;

float ring_outer_high = 88.0f;
float ring_inner_low = 14.0f;
float ring_inner_active = 25.0f;
float ring_side_diff = 18.0f;

volatile unsigned char hall_triggered = 0;
volatile unsigned int hall_timer_count = 0;
volatile unsigned int hall_count = 0;

static uint16 state_ticks = 0;
static uint8 confirm_ticks = 0;
static uint8 recover_ticks = 0;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8 line_reacquired(float track_error)
{
    float total = L + LM + RM + R;

    return total >= ELEMENT4_LINE_SUM_MIN
        && absolute_float(track_error) <= ELEMENT4_LINE_ERROR_MAX;
}

static int8 detect_right_angle_direction(void)
{
    if (L < right_angle_outer_low || R < right_angle_outer_low)
    {
        return 0;
    }

    if (LM >= right_angle_inner_high
        && RM <= right_angle_inner_low
        && (LM - RM) >= right_angle_inner_diff)
    {
        return 1;
    }

    if (RM >= right_angle_inner_high
        && LM <= right_angle_inner_low
        && (RM - LM) >= right_angle_inner_diff)
    {
        return -1;
    }

    return 0;
}

static int8 detect_ring_direction(void)
{
    float left_side;
    float right_side;
    float side_delta;

    if (L < ring_outer_high || R < ring_outer_high)
    {
        return 0;
    }

    if (!((LM <= ring_inner_low && RM >= ring_inner_active)
        || (RM <= ring_inner_low && LM >= ring_inner_active)))
    {
        return 0;
    }

    left_side = L + 1.4f * LM;
    right_side = R + 1.4f * RM;
    side_delta = left_side - right_side;

    if (absolute_float(side_delta) < ring_side_diff)
    {
        return 0;
    }

    return side_delta > 0.0f ? 1 : -1;
}

static void enter_track_state(void)
{
    element4_state = ELEMENT4_TRACK;
    element4_direction = 0;
    state_ticks = 0;
    confirm_ticks = 0;
    recover_ticks = 0;

    cir_flag = 0;
    cir_angle_flag = 0;
    circle_enter_case = 0;
    right_angle_flag = 0;
    right_angle_count = 0;
    encoder_sign = 0;
    gyro_roll_sign_rign = 0;
    gyro_roll_sign_angle = 0;
    mot_inc_element = 0.0f;
    gyro_roll = 0.0f;
    gyro_right_angle = 0.0f;
    temp_flag = 0.0f;
    change_speed_Target_base((int)speed[0]);
    reset_motion_pid_state();
}

static void start_right_angle(int8 direction)
{
    element4_state = ELEMENT4_RIGHT_ANGLE_TURN;
    element4_direction = direction;
    state_ticks = 0;
    confirm_ticks = 0;
    recover_ticks = 0;

    right_angle_flag = 1;
    right_angle_count = 0;
    gyro_right_angle = 0.0f;
    gyro_roll_sign_angle = 1;
    encoder_sign = 1;
    mot_inc_element = 0.0f;
    reset_motion_pid_state();
}

static void start_ring(int8 direction)
{
    element4_state = ELEMENT4_RING_ENTER;
    element4_direction = direction;
    state_ticks = 0;
    confirm_ticks = 0;
    recover_ticks = 0;

    cir_flag = 1;
    cir_angle_flag = direction > 0 ? 1 : 2;
    circle_enter_case = 1;
    gyro_roll = 0.0f;
    gyro_roll_sign_rign = 0;
    encoder_sign = 1;
    mot_inc_element = 0.0f;
    change_speed_Target_base((int)speed[2]);
    reset_motion_pid_state();
}

void element4_init(void)
{
    enter_track_state();
}

float element4_process(float track_error)
{
    int8 detected_direction;

    state_ticks++;

    switch (element4_state)
    {
        case ELEMENT4_TRACK:
            detected_direction = detect_right_angle_direction();
            if (detected_direction != 0)
            {
                element4_state = ELEMENT4_RIGHT_ANGLE_CONFIRM;
                element4_direction = detected_direction;
                confirm_ticks = 1;
                state_ticks = 0;
                return track_error;
            }

            detected_direction = detect_ring_direction();
            if (detected_direction != 0)
            {
                element4_state = ELEMENT4_RING_CONFIRM;
                element4_direction = detected_direction;
                confirm_ticks = 1;
                state_ticks = 0;
            }
            return track_error;

        case ELEMENT4_RIGHT_ANGLE_CONFIRM:
            detected_direction = detect_right_angle_direction();
            if (detected_direction == element4_direction)
            {
                confirm_ticks++;
                if (confirm_ticks >= ELEMENT4_CONFIRM_TICKS)
                {
                    start_right_angle(element4_direction);
                }
            }
            else
            {
                enter_track_state();
            }
            return track_error;

        case ELEMENT4_RIGHT_ANGLE_TURN:
            right_angle_count = (int)state_ticks;

            if ((absolute_float(gyro_right_angle) >= right_angle_min_angle
                    && line_reacquired(track_error))
                || absolute_float(gyro_right_angle) >= right_angle_target_angle
                || state_ticks >= ELEMENT4_RIGHT_ANGLE_TIMEOUT)
            {
                element4_state = ELEMENT4_RIGHT_ANGLE_RECOVER;
                right_angle_flag = 2;
                state_ticks = 0;
                recover_ticks = 0;
                gyro_roll_sign_angle = 0;
                encoder_sign = 0;
                mot_inc_element = 0.0f;
                change_speed_Target_base((int)speed[1]);
                reset_motion_pid_state();
            }
            return track_error;

        case ELEMENT4_RIGHT_ANGLE_RECOVER:
            if (line_reacquired(track_error))
            {
                recover_ticks++;
            }
            else
            {
                recover_ticks = 0;
            }

            if (recover_ticks >= ELEMENT4_RECOVER_TICKS
                || state_ticks >= ELEMENT4_RIGHT_ANGLE_TIMEOUT)
            {
                enter_track_state();
            }
            return track_error;

        case ELEMENT4_RING_CONFIRM:
            detected_direction = detect_ring_direction();
            if (detected_direction == element4_direction)
            {
                confirm_ticks++;
                if (confirm_ticks >= ELEMENT4_CONFIRM_TICKS)
                {
                    start_ring(element4_direction);
                }
            }
            else
            {
                enter_track_state();
            }
            return track_error;

        case ELEMENT4_RING_ENTER:
            if (mot_inc_element >= ring_inc_element12)
            {
                element4_state = ELEMENT4_RING_HOLD;
                cir_flag = 2;
                state_ticks = 0;
                encoder_sign = 0;
                gyro_roll = 0.0f;
                gyro_roll_sign_rign = 1;
            }
            else if (state_ticks >= ELEMENT4_RING_TIMEOUT)
            {
                enter_track_state();
            }
            return track_error + (float)element4_direction * 0.25f * ring_error;

        case ELEMENT4_RING_HOLD:
            if (absolute_float(gyro_roll) >= ring_angle_34)
            {
                element4_state = ELEMENT4_RING_EXIT;
                cir_flag = 4;
                state_ticks = 0;
            }
            else if (state_ticks >= ELEMENT4_RING_TIMEOUT)
            {
                enter_track_state();
            }
            return (float)element4_direction * ring_error;

        case ELEMENT4_RING_EXIT:
            if (absolute_float(gyro_roll) >= ring_angle_45)
            {
                element4_state = ELEMENT4_RING_RECOVER;
                cir_flag = 6;
                state_ticks = 0;
                recover_ticks = 0;
                gyro_roll_sign_rign = 0;
                encoder_sign = 1;
                mot_inc_element = 0.0f;
            }
            else if (state_ticks >= ELEMENT4_RING_TIMEOUT)
            {
                enter_track_state();
            }
            return (float)element4_direction * ring_exit_error;

        case ELEMENT4_RING_RECOVER:
            if (mot_inc_element >= ring_inc_element56
                && line_reacquired(track_error))
            {
                recover_ticks++;
                temp_flag = (float)recover_ticks;
            }
            else
            {
                recover_ticks = 0;
                temp_flag = 0.0f;
            }

            if (recover_ticks >= (uint8)temp_flag_tar
                || state_ticks >= ELEMENT4_RING_TIMEOUT)
            {
                enter_track_state();
            }
            return track_error;

        default:
            enter_track_state();
            return track_error;
    }
}

uint8 element4_get_speed_override(float *left_target, float *right_target)
{
    if (left_target == NULL || right_target == NULL)
    {
        return 0;
    }

    if (element4_state != ELEMENT4_RIGHT_ANGLE_TURN)
    {
        return 0;
    }

    if (element4_direction > 0)
    {
        *left_target = -right_angle_reverse_speed;
        *right_target = right_angle_forward_speed;
    }
    else
    {
        *left_target = right_angle_forward_speed;
        *right_target = -right_angle_reverse_speed;
    }

    return 1;
}

const char *element4_state_name(void)
{
    switch (element4_state)
    {
        case ELEMENT4_TRACK: return "TRACK";
        case ELEMENT4_RIGHT_ANGLE_CONFIRM: return "RA-CHECK";
        case ELEMENT4_RIGHT_ANGLE_TURN: return "RA-TURN";
        case ELEMENT4_RIGHT_ANGLE_RECOVER: return "RA-REC";
        case ELEMENT4_RING_CONFIRM: return "R-CHECK";
        case ELEMENT4_RING_ENTER: return "R-ENTER";
        case ELEMENT4_RING_HOLD: return "R-HOLD";
        case ELEMENT4_RING_EXIT: return "R-EXIT";
        case ELEMENT4_RING_RECOVER: return "R-REC";
        default: return "UNKNOWN";
    }
}

uint8 element4_negative_pressure_request(void)
{
    return element4_state == ELEMENT4_RING_ENTER
        || element4_state == ELEMENT4_RING_HOLD
        || element4_state == ELEMENT4_RING_EXIT;
}

void check_hall_sensor(void)
{
    if (!hall_triggered)
    {
        if (P36 == 0)
        {
            hall_count++;
            hall_triggered = 1;
            hall_timer_count = 0;
        }
    }
    else
    {
        hall_timer_count++;
        if (hall_timer_count >= 200U)
        {
            hall_triggered = 0;
            hall_timer_count = 0;
        }
    }
}

void Circle_detect(void)
{
}

void Circle_cl(void)
{
}

void right_angle_judge(void)
{
}

void right_angle_cl(void)
{
}
