#include "headfile.h"
#include "control.h"
#include "inductance4.h"



int sample_count = 0;
int state = 0;


float gyro_data[1] = {0.0f};

float roll_accel=0,pitch_accel=0;
float gyro_roll=0,gyro_pitch=0;

float roll = 0.0f, pitch= 0.0f;
float Angle_x=0;
/************ Angle control state *************/
float current_angle=0;
float target_angle=0;
float angle_error;
float turn_control_output;
float base_speed = 0.0f;



/************ Four-channel inductance state *************/
float L_raw = 0,R_raw = 0;
float LM_raw = 0,RM_raw = 0;

/* Four-channel normalized values. MID is retained as a zero-valued legacy symbol. */
float L = 0, R = 0, LM = 0, RM = 0, MID = 0;
uint16  max_AD = 998,min_AD = 1;

uint16  i = 0,j = 0,k1 = 0,temp = 0;

/**************************************************************************
Set the differential-drive target speed for both wheels.
**************************************************************************/
void set_target_speeds(float left_target, float right_target) {
    L_pid.Target = left_target;
    R_pid.Target = right_target;
}

/**************************************************************************
Initialize the four ADC channels, encoders, motor PWM, wireless UART,
and status GPIO outputs.
**************************************************************************/
void init(void)
{

		adc_init(ADC_P00, ADC_12BIT);	
		adc_init(ADC_P01, ADC_12BIT);	
		adc_init(ADC_P05, ADC_12BIT);	  
		adc_init(ADC_P06, ADC_12BIT);	  
		inductance4_init();
		delay_ms(10);

		ctimer_count_init(MOTOR1_ENCODER);
		ctimer_count_init(MOTOR2_ENCODER);
		gpio_init(IO_P35, GPI, GPIO_LOW, GPI_IMPEDANCE);
		gpio_init(IO_P53, GPI, GPIO_LOW, GPI_IMPEDANCE);
		delay_ms(10);

		gpio_init(IO_P64, GPO, GPIO_LOW, GPO_PUSH_PULL);
		gpio_init(IO_P60, GPO, GPIO_LOW, GPO_PUSH_PULL);
		pwm_init(LEFT_MOTOR_PWM, 17000, 0);
		pwm_init(RIGHT_MOTOR_PWM, 17000, 0);
		delay_ms(10);
		
		/* Keep wireless CMD high; low selects configuration mode. */
		gpio_init(IO_P45, GPI, GPIO_HIGH, GPI_PULL_UP);
		wireless_uart_init();
		/* P0.7 is owned by the wireless RTS input. */
		
			
		gpio_mode(P2_6,GPO_PP);
		gpio_mode(P7_4,GPO_PP);
		gpio_mode(P5_2,GPO_PP);

        gpio_init(IO_P70, GPI, GPIO_HIGH, GPI_PULL_UP);
        gpio_init(IO_P71, GPI, GPIO_HIGH, GPI_PULL_UP);
        gpio_init(IO_P72, GPI, GPIO_HIGH, GPI_PULL_UP);
        gpio_init(IO_P73, GPI, GPIO_HIGH, GPI_PULL_UP);
        gpio_init(IO_P75, GPI, GPIO_HIGH, GPI_PULL_UP);
        gpio_init(IO_P76, GPI, GPIO_HIGH, GPI_PULL_UP);


}


/**************************************************************************
??mode:
?mode = 1: ?mode = 0: ?
**************************************************************************/



// --- Key Scan Function (Provided by User - Assumed Unchanged) ---
static volatile uint8 ui_key_event_pending = KEY_EVENT_NONE;

#define UI_DEBOUNCE_TICKS       3
#define UI_LONG_PRESS_TICKS   200

static void post_ui_key_event(uint8 event)
{
    if ((event != KEY_EVENT_NONE) && (ui_key_event_pending == KEY_EVENT_NONE))
    {
        ui_key_event_pending = event;
    }
}

uint8 fetch_ui_key_event(void)
{
    uint8 event = ui_key_event_pending;
    ui_key_event_pending = KEY_EVENT_NONE;
    return event;
}

uint8 key_scan(int mode)
{
    static unsigned int key_pressed_state = 1;
    if (mode) { key_pressed_state = 1; }

    if (key_pressed_state == 1 && (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0))
    {
        delay_ms(10);
        if (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0)
        {
            key_pressed_state = 0;

            if (P70 == 0) return KEY_EVENT_PAGE_PREV;
            if (P71 == 0) return KEY_EVENT_PAGE_NEXT;
            if (P73 == 0) return KEY_EVENT_ITEM_NEXT;
            if (P72 == 0)
            {
                if (P75 == 0) return KEY_EVENT_ADJ_DEC;
                return KEY_EVENT_ADJ_INC;
            }
            return KEY_EVENT_NONE;

        }
    }
    else if (key_pressed_state == 0 && (P70 == 1 && P71 == 1 && P72 == 1 && P73 == 1))
    {
        key_pressed_state = 1;
    }

    return KEY_EVENT_NONE;
}
/**************************************************************************
pwm?mode:
?mode = 1: ?mode = 0: ?
**************************************************************************/
uint8 current_key=0;
uint8 last_key_state =0;
uint8 pwm_state = 0;
char pwm_state_charge = 255;

uint8 Pwmout =0;
void key_scan_cycle_pwm_state(void)
{
    static uint8 raw70 = 1, stable70 = 1, cnt70 = 0;
    static uint8 raw71 = 1, stable71 = 1, cnt71 = 0;
    static uint8 raw72 = 1, stable72 = 1, cnt72 = 0;
    static uint8 raw73 = 1, stable73 = 1, cnt73 = 0;

    static uint8 prev70_down = 0;
    static uint8 prev71_down = 0;
    static uint8 prev72_down = 0;
    static uint8 prev73_down = 0;

    static uint8 key70_long_sent = 0;
    static uint8 key71_long_sent = 0;
    static uint8 key70_combo_used = 0;
    static uint8 key71_combo_used = 0;
    static uint8 combo_sent = 0;

    static uint16 key70_ticks = 0;
    static uint16 key71_ticks = 0;
    static uint16 combo_ticks = 0;

    uint8 cur70 = (P70 == 0) ? 1 : 0;
    uint8 cur71 = (P71 == 0) ? 1 : 0;
    uint8 cur72 = (P72 == 0) ? 1 : 0;
    uint8 cur73 = (P73 == 0) ? 1 : 0;

    if (cur70 != raw70) { raw70 = cur70; cnt70 = UI_DEBOUNCE_TICKS; }
    else if (cnt70 > 0) { cnt70--; if (cnt70 == 0) stable70 = raw70; }

    if (cur71 != raw71) { raw71 = cur71; cnt71 = UI_DEBOUNCE_TICKS; }
    else if (cnt71 > 0) { cnt71--; if (cnt71 == 0) stable71 = raw71; }

    if (cur72 != raw72) { raw72 = cur72; cnt72 = UI_DEBOUNCE_TICKS; }
    else if (cnt72 > 0) { cnt72--; if (cnt72 == 0) stable72 = raw72; }

    if (cur73 != raw73) { raw73 = cur73; cnt73 = UI_DEBOUNCE_TICKS; }
    else if (cnt73 > 0) { cnt73--; if (cnt73 == 0) stable73 = raw73; }

    if (stable72 && !prev72_down)
    {
        if (P75 == 0) post_ui_key_event(KEY_EVENT_ADJ_DEC);
        else post_ui_key_event(KEY_EVENT_ADJ_INC);
    }

    if (stable73 && !prev73_down)
    {
        post_ui_key_event(KEY_EVENT_ITEM_NEXT);
    }

    if (stable70 && stable71)
    {
        key70_combo_used = 1;
        key71_combo_used = 1;

        if (combo_ticks < UI_LONG_PRESS_TICKS) combo_ticks++;
        if (combo_ticks >= UI_LONG_PRESS_TICKS && !combo_sent)
        {
            combo_sent = 1;
            key70_long_sent = 1;
            key71_long_sent = 1;
            pwm_state = 2;
            Pwmout = pwm_state;
            post_ui_key_event(KEY_EVENT_ENTER_CLEAN);
        }
    }
    else
    {
        combo_ticks = 0;
        combo_sent = 0;
    }

    if (stable70 && !stable71)
    {
        if (!prev70_down)
        {
            key70_ticks = 0;
            key70_long_sent = 0;
            key70_combo_used = 0;
        }
        else if (key70_ticks < UI_LONG_PRESS_TICKS)
        {
            key70_ticks++;
        }

        if (key70_ticks >= UI_LONG_PRESS_TICKS && !key70_long_sent)
        {
            key70_long_sent = 1;
            if (pwm_state == 2) pwm_state = 0;
            else if (pwm_state == 1) pwm_state = 0;
            else if (motion_runtime_can_run()) pwm_state = 1;
            else pwm_state = 0;
            Pwmout = pwm_state;
            post_ui_key_event(KEY_EVENT_RUN_TOGGLE);
        }
    }
    else if (!stable70 && prev70_down)
    {
        if (!key70_long_sent && !key70_combo_used)
        {
            post_ui_key_event(KEY_EVENT_PAGE_PREV);
        }
        key70_ticks = 0;
        key70_long_sent = 0;
        key70_combo_used = 0;
    }

    if (stable71 && !stable70)
    {
        if (!prev71_down)
        {
            key71_ticks = 0;
            key71_long_sent = 0;
            key71_combo_used = 0;
        }
        else if (key71_ticks < UI_LONG_PRESS_TICKS)
        {
            key71_ticks++;
        }

        if (key71_ticks >= UI_LONG_PRESS_TICKS && !key71_long_sent)
        {
            key71_long_sent = 1;
            post_ui_key_event(KEY_EVENT_SAVE_ALL);
        }
    }
    else if (!stable71 && prev71_down)
    {
        if (!key71_long_sent && !key71_combo_used)
        {
            post_ui_key_event(KEY_EVENT_PAGE_NEXT);
        }
        key71_ticks = 0;
        key71_long_sent = 0;
        key71_combo_used = 0;
    }

    prev70_down = stable70;
    prev71_down = stable71;
    prev72_down = stable72;
    prev73_down = stable73;
}

static float get_step_scale_from_switch(void)
{
    if (P76 == 0)
        return 50.0f;
    else
        return 1.0f;
}

void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step)
{
    float real_step = step * get_step_scale_from_switch();

    if (key_value == KEY_EVENT_ADJ_INC)
    {
        *parameter += real_step;
    }
    else if (key_value == KEY_EVENT_ADJ_DEC)
    {
        *parameter -= real_step;
    }
}

float low_pass_filter(float current_value, float last_value, float alpha)
{
    return current_value * alpha + last_value * (1 - alpha);
}

void change_speed_Target(int speed)
{
    L_pid.Target = speed;
    R_pid.Target = speed;
}

void change_speed_Target_base(int speed)
{
    L_pid.Target_base = speed;
    R_pid.Target_base = speed;
}





float limit_range(float input, float limit)
{
    if (input > limit)
        return limit;
    else if (input < -limit)
        return -limit;
    else
        return input;
}





/**
 * @brief ? *
 * ?check_value ? * ?>= abs_threshold?(?P52 = 1)? * ?< abs_threshold?(?P52 = 0)? *
 * ? * - ?<math.h> ? * - ?P52 ?1 ? ? *
 * @param check_value ?(?? * @param abs_threshold ?(?? */
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state)
{
    // ASCII-cleaned legacy comment.
    // ASCII-cleaned legacy comment.
		#if 0
	    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1))
    {
        // ASCII-cleaned legacy comment.
    }
    else
    {
        // ASCII-cleaned legacy comment.
        P52 = 0; // ASCII-cleaned legacy comment.
    }
		#endif
		
		
		
		
		
		
		#if 1

		if ((check_value== abs_threshold) && (enable_state == 1))
    {
        // ASCII-cleaned legacy comment.
    }
    else
    {
        // ASCII-cleaned legacy comment.
        P52 = 0; // ASCII-cleaned legacy comment.
    }
		#endif
		


}









	/* ASCII-cleaned legacy section. */






	/* ASCII-cleaned legacy section. */


uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution)
{
    unsigned long sum = 0;
    unsigned short adc_value;
    unsigned short i;

    if (avg_times == 0) {
        avg_times = 1;
    }

    for (i = 0; i < avg_times; i++)
    {
        adc_value = adc_once(channel, resolution);

        sum += adc_value;
    }

    return (uint16)(sum / avg_times);
}

	/* ASCII-cleaned legacy section. */


float limit_float(float value, float min_limit, float max_limit)
{
    float result;

    if (value < min_limit)
    {
        result = min_limit;
    }
    else if (value > max_limit)
    {
        result = max_limit;
    }
    else
    {
        result = value;
    }

    return result;
}

	/* ASCII-cleaned legacy section. */


float normalize_float(float value, float min, float max)
{
    float diff;
    float range;
    float result;

    if (max <= min)
    {
        return 0.0f;
    }
		// ASCII-cleaned legacy comment.
    range = max - min;
		// ASCII-cleaned legacy comment.
    diff = value - min;

    result = (diff * 100.0f) / range;

    // ASCII-cleaned legacy comment.
    if (result > 100.0f)
    {
        result = 100.0f;
    }

    if (result < 0.0f)
    {
        result = 0.0f;
    }

    return result;
}


/**
 * @brief ?(MID) ? *
 * ?70 (?MID=28 ? ?380 (?MID=60 ?? * - ?MID <= 28, ?270.
 * - ?MID >= 60, ?380.
 * - ? ? * ? ?(?^2 ? *
 * @param current_mid_value ? * @return ? */
float calculate_dynamic_target_speed_quadratic(float current_mid_value) {
		#if 1
    int calculated_speed;

    if (current_mid_value <= 26.0f) {
        calculated_speed = 230;
    } else if (current_mid_value >= 30.0f) {
        calculated_speed = 330;
    } else {
        // ASCII-cleaned legacy comment.
        float mid_ratio = (current_mid_value - 26.0f) / 4.0f;
        
        float quartic_factor = mid_ratio * mid_ratio * mid_ratio * mid_ratio; // mid_ratio^4
        
        // ASCII-cleaned legacy comment.
        calculated_speed = (int)(230.0f + quartic_factor * 100.0f);
    }

    return calculated_speed;
		
		#endif
		
		
}

/*********************rgb*********************//*********************rgb*********************//*********************rgb*********************/	



//void set_rgb_pins(int p26_val, int p74_val, int p07_val) {
//    P26 = p26_val;
//    P74 = p74_val;
//    P07 = p07_val;
//}

//void control_rgb_led_conditional(float check_value, float abs_threshold, RgbColorCode_t feedback_color, int enable_state) {
// ASCII-cleaned legacy comment.
// ASCII-cleaned legacy comment.
//	
//		#if 1
//    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1)) {
//        switch (feedback_color) {
//            case RGB_COLOR_OFF:
//                set_rgb_pins(0, 0, 0);
//                break;
//            case RGB_COLOR_WHITE:
//                set_rgb_pins(0, 0, 1);
//                break;
//            case RGB_COLOR_CYAN:
//                set_rgb_pins(0, 1, 0);
//                break;
//            case RGB_COLOR_YELLOW_GREEN:
//                set_rgb_pins(1, 0, 0);
//                break;
//            case RGB_COLOR_MAGENTA:
//                set_rgb_pins(0, 1, 1);
//                break;
//            case RGB_COLOR_GREEN:
//                set_rgb_pins(1, 1, 0);
//                break;
//            case RGB_COLOR_RED:
//                set_rgb_pins(1, 0, 1);
//                break;
//            case RGB_COLOR_BLUE:
//                set_rgb_pins(1, 1, 1);
//                break;
//            default:
// ASCII-cleaned legacy comment.
//                break;
//        }
//    } else {
//        set_rgb_pins(0, 0, 0);
//    }
//		#endif
//		

//		
//}

//void control_rgb_led( RgbColorCode_t feedback_color) {

//        switch (feedback_color) {
//            case RGB_COLOR_OFF:
//                set_rgb_pins(0, 0, 0);
//                break;
//            case RGB_COLOR_WHITE:
//                set_rgb_pins(0, 0, 1);
//                break;
//            case RGB_COLOR_CYAN:
//                set_rgb_pins(0, 1, 0);
//                break;
//            case RGB_COLOR_YELLOW_GREEN:
//                set_rgb_pins(1, 0, 0);
//                break;
//            case RGB_COLOR_MAGENTA:
//                set_rgb_pins(0, 1, 1);
//                break;
//            case RGB_COLOR_GREEN:
//                set_rgb_pins(1, 1, 0);
//                break;
//            case RGB_COLOR_RED:
//                set_rgb_pins(1, 0, 1);
//                break;
//            case RGB_COLOR_BLUE:
//                set_rgb_pins(1, 1, 1);
//                break;
//            default:
// ASCII-cleaned legacy comment.
//                break;
//        }

//}

/* ASCII-cleaned legacy section. */
/* ASCII-cleaned legacy section. */

// ASCII-cleaned legacy comment.
//float dt_my = 0.005f;
// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
//float origin_ax_offset = 0;
//float origin_ay_offset = 0;
//float origin_az_offset = 0;
//float origin_gx_offset = 0;
//float origin_gy_offset = 0;
//float origin_gz_offset = 0;

// ASCII-cleaned legacy comment.
//float roll_v = 0;
//float pitch_v = 0;
//float yaw_v = 0;

// ASCII-cleaned legacy comment.
//float gyro_roll_my = 0;
//float gyro_pitch_my = 0;
//float gyro_yaw = 0;

// ASCII-cleaned legacy comment.
//float acc_roll = 0;
//float acc_pitch = 0;

// ASCII-cleaned legacy comment.
//float k_roll = 0;
//float k_pitch = 0;
//float k_yaw = 0;

// ASCII-cleaned legacy comment.
//float e_P[2][2];
// ASCII-cleaned legacy comment.
//float k_k[2][2];

//void origin_data() {
//    int i;

//    origin_ax_offset = 0;
//    origin_ay_offset = 0;
//    origin_az_offset = 0;
//    origin_gx_offset = 0;
//    origin_gy_offset = 0;
//    origin_gz_offset = 0;

//    for (i = 0; i < 300; i++) {
//        imu660ra_get_acc();
//        imu660ra_get_gyro();

//        origin_ax_offset += imu660ra_acc_transition(imu660ra_acc_x);
//        origin_ay_offset += imu660ra_acc_transition(imu660ra_acc_y);
//        origin_az_offset += imu660ra_acc_transition(imu660ra_acc_z);

//        origin_gx_offset += imu660ra_gyro_transition(imu660ra_gyro_x);
//        origin_gy_offset += imu660ra_gyro_transition(imu660ra_gyro_y);
//        origin_gz_offset += imu660ra_gyro_transition(imu660ra_gyro_z);
//    }

//    origin_ax_offset /= 300.0f;
//    origin_ay_offset /= 300.0f;
//    origin_az_offset /= 300.0f;
//    origin_gx_offset /= 300.0f;
//    origin_gy_offset /= 300.0f;
//    origin_gz_offset /= 300.0f;
//}

//void kalanma_data() {
//    float ax, ay, az;
//    float gx, gy, gz;
//    float sin_roll, cos_roll, sin_pitch, cos_pitch;
//    float acc_ax_adj, acc_ay_adj, acc_az_adj;

//    imu660ra_get_acc();
//    imu660ra_get_gyro();

//    ax = imu660ra_acc_transition(imu660ra_acc_x);
//    ay = imu660ra_acc_transition(imu660ra_acc_y);
//    az = imu660ra_acc_transition(imu660ra_acc_z);

//    gx = imu660ra_gyro_transition(imu660ra_gyro_x);
//    gy = imu660ra_gyro_transition(imu660ra_gyro_y);
//    gz = imu660ra_gyro_transition(imu660ra_gyro_z);

//    sin_roll = (float)sin(k_roll / rad2deg);
//    cos_roll = (float)cos(k_roll / rad2deg);
//    sin_pitch = (float)sin(k_pitch / rad2deg);
//    cos_pitch = (float)cos(k_pitch / rad2deg);

//    roll_v = (gx - origin_gx_offset)
//           + (sin_pitch * sin_roll / cos_pitch) * (gy - origin_gy_offset)
//           + (sin_pitch * cos_roll / cos_pitch) * (gz - origin_gz_offset);
//    pitch_v = cos_roll * (gy - origin_gy_offset) - sin_roll * (gz - origin_gz_offset);
//    yaw_v = (sin_roll / cos_pitch) * (gy - origin_gy_offset)
//          + (cos_roll / cos_pitch) * (gz - origin_gz_offset);

//    roll_v /= 100.0f;
//    pitch_v /= 100.0f;
//    yaw_v /= 100.0f;

//    gyro_roll_my = k_roll + dt_my * roll_v;
//    gyro_pitch_my = k_pitch + dt_my * pitch_v;
// ASCII-cleaned legacy comment.

// ASCII-cleaned legacy comment.
//    e_P[0][0] += 0.0025f;
//    e_P[1][1] += 0.0025f;

// ASCII-cleaned legacy comment.
//    k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3f);
//    k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3f);

// ASCII-cleaned legacy comment.
//    acc_ax_adj = ax - origin_ax_offset;
//    acc_ay_adj = ay - origin_ay_offset;
//    acc_az_adj = az - origin_az_offset;

//    acc_roll = (float)(atan2(acc_ay_adj, acc_az_adj) * rad2deg);
//    acc_pitch = (float)(-atan2(acc_ax_adj,
//        sqrt(acc_ay_adj * acc_ay_adj + acc_az_adj * acc_az_adj)) * rad2deg);

//    k_roll = gyro_roll_my + k_k[0][0] * (acc_roll - gyro_roll_my);
//    k_pitch = gyro_pitch_my + k_k[1][1] * (acc_pitch - gyro_pitch_my);
//    k_yaw = 30.0f * gyro_yaw;

// ASCII-cleaned legacy comment.
//    e_P[0][0] *= (1.0f - k_k[0][0]);
//    e_P[1][1] *= (1.0f - k_k[1][1]);
//}

// ASCII-cleaned legacy comment.
//void init_kalman() {
//    e_P[0][0] = 1.0f; e_P[0][1] = 0.0f;
//    e_P[1][0] = 0.0f; e_P[1][1] = 1.0f;

//    k_k[0][0] = 0.0f; k_k[0][1] = 0.0f;
//    k_k[1][0] = 0.0f; k_k[1][1] = 0.0f;
//}






/**
 * @brief ?encoder_speedup_sign ?encoder_speedup_element ? * ?sign ?1, ?l_speed_now ?r_speed_now ? * ? ?element ? * * @param p_encoder_speedup_element ?encoder_speedup_element ? * @param current_encoder_speedup_sign ?encoder_speedup_sign ? */
void update_encoder_speedup_value(float* p_encoder_speedup_element, 
                                  int current_encoder_speedup_sign) 
{
    if (current_encoder_speedup_sign == 1) {

        *p_encoder_speedup_element += (fabs(l_speed_now) + fabs(r_speed_now)) * 0.5f * 0.00003895f;
    } else {
        *p_encoder_speedup_element = 0.0f;
    }
}


void update_gyro_angle_accumulator(float* p_angle_accumulator,
                                   int sign_flag)
{
    if (sign_flag == 1) {
        *p_angle_accumulator += gyro_data[0] * 0.005f;
    } else {
        *p_angle_accumulator = 0.0f;
    }
}




