#include "headfile.h"
#include "control.h"



int sample_count = 0;  //计数器，用于收集一定数量的数据来计算零漂。
int state = 0;  		   // 状态标志，指示是否已经计算完偏移量


// 转化，去零漂后的物理量
float gyro_data[3] = {0.0f, 0.0f, 0.0f};  // 陀螺仪数据

//互补滤波计算角度值
float roll_accel=0,pitch_accel=0;
float gyro_roll=0,gyro_pitch=0;

float roll = 0.0f, pitch= 0.0f;
float Angle_x=0;//弧度制转角度制

/************角度环参数**************/
float current_angle=0;//当前角度
float target_angle=0;//目标角度
float angle_error;//误差
float turn_control_output;
float base_speed = 0.0f; // 原地转向，基础速度设为 0



/************电感参数**************/
float L_raw = 0,R_raw = 0;//后面四个还没用到
float L = 0,R = 0,LM = 0,RM = 0,MID = 0;//后面四个还没用到
uint16  max_AD = 998,min_AD = 1;//10分辨率电感最大值设为998 12位为4095 

uint16  i = 0,j = 0,k1 = 0,temp = 0;

/**************************************************************************
函数功能：--- 辅助函数：设置左右轮目标速度 ---
入口参数：无
返回  值：wu
**************************************************************************/
void set_target_speeds(float left_target, float right_target) {
    // 这里可以加入对目标速度的上下限限制，例如不允许低于0
    L_pid.Target = left_target;
    R_pid.Target = right_target;
}

/**************************************************************************
函数功能：外设及pid初始化
入口参数：无
返回  值：wu
**************************************************************************/
void init(void)
{

		adc_init(ADC_P00, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P01, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P10, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P05, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P06, ADC_SYSclk_DIV_2);	  
		delay_ms(10);

		ctimer_count_init(MOTOR1_ENCODER);
		ctimer_count_init(MOTOR2_ENCODER);
		delay_ms(10);

		pwm_init(PWMA_CH2P_P62, 17000, 0);
		pwm_init(PWMA_CH1P_P60, 17000, 0);
		pwm_init(PWMA_CH4P_P66, 17000, 0);
		pwm_init(PWMA_CH3P_P64, 17000, 0);
		delay_ms(10);
		
		wireless_uart_init();
		
			
		gpio_mode(P2_6,GPO_PP);
		gpio_mode(P7_4,GPO_PP);
		gpio_mode(P0_7,GPO_PP);
		gpio_mode(P5_2,GPO_PP);


}


/**************************************************************************
按键检测程序,学习板
mode:
控制按键状态是否重置。
mode = 1: 重置按键状态为未按下。实现长按
mode = 0: 正常检测按键状态

**************************************************************************/



// --- Key Scan Function (Provided by User - Assumed Unchanged) ---
uint8 key_scan(int mode) {
    static unsigned int key_pressed_state = 1;
    if (mode) { key_pressed_state = 1; }
    if (key_pressed_state == 1 && (P77 == 0 || P75 == 0 || P76 == 0 || P46 == 0||P45 == 0)) {
        delay_ms(10);
        if (P77 == 0 || P75 == 0 || P76 == 0 || P46 == 0||P45 == 0) {
            key_pressed_state = 0;
            if (P77 == 0) return 1; else if (P75 == 0) return 2;
            else if (P76 == 0) return 3; else if (P46 == 0) return 4;
						else if(P45 == 0)  return 5;
            else return 0;
        }
    } else if (key_pressed_state == 0 && (P77 == 1 && P75 == 1 && P76 == 1 && P46 == 1&& P45 == 1)) {
        key_pressed_state = 1;
    }
    return 0;
}
/**************************************************************************
pwm积分开关，按键模拟拨码开关
mode:
控制按键状态是否重置。
mode = 1: 重置按键状态为未按下。实现长按
mode = 0: 正常检测按键状态

**************************************************************************/
uint8 current_key=0;
uint8 last_key_state =0;
uint8 pwm_state = 0;      // 当前输出状态
char pwm_state_charge = 0;      // 当前输出状态


uint8 Pwmout =0;
#define DEBOUNCE_TICKS 2

void key_scan_cycle_pwm_state(void) // 返回类型改为 void
{
    static uint8 key = 1;
    static uint8 debounce_counter = 0;
    static uint8 last_stable_pin_state = 0;
    uint8 current_pin_state;

    current_pin_state = (P45 == 0) ? 1 : 0;

    if (current_pin_state != last_stable_pin_state)
    {
        debounce_counter = DEBOUNCE_TICKS;
        last_stable_pin_state = current_pin_state;
        return; // void 函数直接 return
    }

    if (debounce_counter > 0)
    {
        debounce_counter--;
        return; // void 函数直接 return
    }

    if (current_pin_state == 1)
    {
        if (key == 1)
        {
            key = 0;
            pwm_state = (pwm_state + 1) % 3;
            Pwmout = pwm_state;
        }
    }
    else
    {
        if (key == 0)
        {
            key = 1;
        }
    }

    // 函数末尾无需显式 return
}




/**
 * @brief 根据按键输入值调整参数 (修改为支持 float 类型)
 *
 * @param key_value 按键输入值 (例如：1, 2, 3, 4 来自 key_scan 函数的返回值)
 * @param parameter   指向要调整的 float 类型参数的指针  <-- 类型修改为 float *
 * @param step        按键步进值，每次按键增加或减少的量  <-- 类型修改为 float
 *example:adjust_parameter_by_key(key_value, &max_AD_float, 0.1f); // 调用函数调整 max_AD_float，步进值为 0.1
 */
void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step) { 
    if (key_value == 2) {
        *parameter += step; // 按键 1：增加参数值
    } else if (key_value == 5) {
        *parameter -= step; // 按键 2：减少参数值
         // 对于浮点数，通常不需要像 uint16_t 那样进行下限判断，因为 float 可以表示负数
         // 您可以根据具体需求添加下限或上限判断
    }
    // 如果需要处理其他按键值，可以在这里添加 else if 分支
}


// 低通滤波函数
float low_pass_filter(float current_value, float last_value, float alpha) {
    return current_value * alpha + last_value * (1 - alpha);
}


/**************************************************************************
函数功能：角度范围限制函数
入口参数：LimitAngle(&Roll);
返回  值：-180-180之间的angle
**************************************************************************/
float LimitAngle(float *angle)
{
    if (*angle >= 180.0f)
    {
        *angle -= 360.0f;
    }
    else if (*angle <= -180.0f)
    {
        *angle += 360.0f;
    }
    return *angle; // 返回修改后的角度值
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      4082驱动测试电机
//  @param      输入电机通道 ，延时时间
//  @return           
//-------------------------------------------------------------------------------------------------------------------
void motor_rotate_and_reverse(int pwm_channel_forward, int pwm_channel_reverse, int rotate_time, int reverse_time, int pwm_duty_max) {
  int duty = 0;

  // 正向转动
  for (duty = 0; duty <= pwm_duty_max; duty += 100) {
    pwm_duty(pwm_channel_forward, duty);
    pwm_duty(pwm_channel_reverse, 0); // 固定反向通道为0
    delay_ms(rotate_time / (pwm_duty_max / 100 + 1)); // 根据rotate_time计算延时
  }

  // 停止
  pwm_duty(pwm_channel_forward, 0);
  pwm_duty(pwm_channel_reverse, 0);
  delay_ms(500); // 停止0.5秒

  // 反向转动
  for (duty = 0; duty <= pwm_duty_max; duty += 100) {
    pwm_duty(pwm_channel_forward, 0); // 固定正向通道为0
    pwm_duty(pwm_channel_reverse, duty);
    delay_ms(reverse_time / (pwm_duty_max / 100 + 1)); // 根据reverse_time计算延时
  }

  // 停止
  pwm_duty(pwm_channel_forward, 0);
  pwm_duty(pwm_channel_reverse, 0);
}







/**
 * @brief 设置左右电机的目标速度
 *
 * 该函数用于分别设置左电机和右电机的目标速度，以便 PID 控制器进行速度控制。
 *
 * @param L_speed 左电机的目标速度值
 * @param R_speed 右电机的目标速度值
 *
 * @note
 * - 该函数会更新全局变量 `L_pid.Target` 和 `R_pid.Target`，分别用于左电机和右电机的 PID 控制。
 * - `L_speed` 和 `R_speed` 的值应该在电机速度的有效范围内。
 */
void change_speed(int speed) {
	
		#if 0
    ang_pid.Target = speed; // 设置左电机的目标速度
		#endif
	
		#if 0
    L_pid.Target = speed; // 设置左电机的目标速度
    R_pid.Target = speed;
		#endif
	
		#if 1

		L_pid.Target_base = speed; // 设置左电机的目标速度
		R_pid.Target_base = speed;
		#endif
}

 

void change_speed_Target(int speed) {
    L_pid.Target = speed; // 设置左电机的目标速度
    R_pid.Target = speed;
}


void change_speed_Target_base(int speed) {
		L_pid.Target_base = speed; // 设置左电机的目标速度
		R_pid.Target_base = speed;
}








// 偏差滑动平均滤波
float filter(float value) 
{
    static float filter_buf[3] = {0};

	filter_buf[2] = filter_buf[1];
	filter_buf[1] = filter_buf[0];
	filter_buf[0] = value;

	return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
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
 * @brief 根据中间电感值计算动态权重。
 *
 * 这个函数实现了根据中间电感读数(M)确定权重因子的逻辑。
 * 权重用于根据机器人是在直线上还是接近/处于弯道来动态调整
 * 水平循迹（侧面电感）的影响。
 * 阈值根据提供的数据进行调整：直道 >= 400 (100%权重),
 * 100-400 线性变化, < 100 (0%权重)。
 *
 * @param M 中间电感传感器的平均读数 (MID)。
 * @return 表示权重的浮点数，范围在 0.0 到 1.0 之间。
 */
float Calculate_Weight_Mid(uint16 M)
{
	
	
		#if OFF//   ON OFF	
	  const float M_low   =  30.0f;   // 脱线下限
    const float M_high  = 500.0f;   // 完全直道上限
    const float k_min   =   0.6f;   // 脱线时最小权重（60%）
    const float k_max   =   1.0f;   // 最大权重（100%）
		static float t=0;
    // 脱线或极小信号时，直接返回 k_min
    if (M <= M_low) {
        return k_min;
    }
    // 完全直道时，直接返回最大
    if (M >= M_high) {
        return k_max;
    }

    // 归一化到 [0,1]
     t = (M - M_low) / (M_high - M_low);

    // Hermite smoothstep: t = t*t*(3 - 2*t)
    t = t * t * (3.0f - 2.0f * t);

    // 插值 k ∈ [k_min, k_max]
    return k_min + (k_max - k_min) * t;
		#endif
	
	
	
		//动态电感bug有点问题，因为当车脱线了，中电感的值小于100，横电感计算出来的误差权重是0，但是这个时候竖向的电感值又很小导致车子的方向环不作用
		#if 0
		float k;

    // 1) M >= 500：完全信任横电感 → k = 100
    if (M >= 500) {
        k = 100.0f;
    }
    // 2) 30 <= M < 500：线性过渡，从 (M=30, k=0) 到 (M=500, k=100)
    else if (M >= 30) {
        // 斜率 m = 100 / (500 - 30) = 100 / 470 ≈ 0.212766
        // 截距 b = -30 * m ≈ -6.38298
        k = (100.0f / 470.0f) * (float)M - (30.0f * 100.0f / 470.0f);
    }
    // 3) M < 30：脱线前依然保留一定横电感权重 → k = 60
    else {
        k = 60.0f;
    }

    // 缩放到 [0.0, 1.0]
    k *= 0.01f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;

    return k;
		
	
		#endif
		
		
		
		#if 1
		
		
		 float k;

    if (M >= 36.0f) {
        // 完全信任横电感
        k = 1.0f;
    }
    else if (M >= 14.0f) {
        // 线性过渡：M 从 14 → 36 时，k 从 0.1 → 1.0
        // 斜率 m = (1.0f - 0.1f) / (36.0f - 14.0f) = 0.9f / 22.0f ≈ 0.040909f
        // 截距 b = 0.1f - 14.0f * m ≈ -0.472727f
        const float m = 0.9f / 22.0f;
        const float b = 0.1f - 14.0f * m;
        k = m * M + b;
    }
    else {
        // 最低也保留 0.1 的横电感权重
        k = 0.1f;
    }

    // 可选：确保数值在 [0.1, 1.0] 范围内
    if (k < 0.1f) k = 0.1f;
    if (k > 1.0f) k = 1.0f;

    return k;
		
		
		
		
		
		#endif
		
		
		
}




/**
 * @brief 根据变量的绝对值是否达到或超过设定的阈值来控制蜂鸣器。
 *
 * 此函数检查输入变量 check_value 的绝对值。
 * 如果绝对值 >= abs_threshold，则打开蜂鸣器 (设置 P52 = 1)。
 * 如果绝对值 < abs_threshold，则关闭蜂鸣器 (设置 P52 = 0)。
 *
 * 前提条件：
 * - 需要包含 <math.h> 头文件。
 * - 变量 P52 必须已经正确定义为控制蜂鸣器的输出引脚，并且 1 表示打开，0 表示关闭。
 *
 * @param check_value 要监控的变量的值 (浮点型)。
 * @param abs_threshold 触发蜂鸣器响起的绝对值阈值 (浮点型)。
 */
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state)
{
    // 检查两个条件：
    // 1. 变量的绝对值是否达到阈值
    // 2. 蜂鸣器是否被启用 (enable_state == 1)
		#if 0
	    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1))
    {
        // 两个条件都满足，打开蜂鸣器
        P52 = 1; // 假设 P52 = 1 表示打开
    }
    else
    {
        // 任何一个条件不满足 (绝对值小于阈值 或 enable_state 不是 1)，关闭蜂鸣器
        P52 = 0; // 假设 P52 = 0 表示关闭
    }
		#endif
		
		
		
		
		
		
		#if 1

		if ((check_value== abs_threshold) && (enable_state == 1))
    {
        // 两个条件都满足，打开蜂鸣器
        P52 = 1; // 假设 P52 = 1 表示打开
    }
    else
    {
        // 任何一个条件不满足 (绝对值小于阈值 或 enable_state 不是 1)，关闭蜂鸣器
        P52 = 0; // 假设 P52 = 0 表示关闭
    }
		#endif
		


}









	/*********************电感处理*********************//*********************电感处理*********************//*********************电感处理*********************/	






	/*********************读取*********************//*********************读取*********************//*********************读取*********************/	


uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution)
{
    unsigned long sum = 0;
    unsigned short adc_value = 0;
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

	/*********************限幅*********************//*********************限幅*********************//*********************限幅*********************/	


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

	/*********************归一化*********************//*********************归一化*********************//*********************归一化*********************/	


float normalize_float(float value, float min, float max)
{
    float diff;
    float range;
    float result;

    if (max <= min)
    {
        return 0.0f;
    }
		//分母
    range = max - min;
		//分子
    diff = value - min;

    result = (diff * 100.0f) / range;

    // 根据 Dianci_Guiyi 的逻辑添加限幅
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
 * @brief 根据中间电感值 (MID) 计算动态目标速度，呈二次函数型加速上升。
 *
 * 速度从270 (当 MID=28 时) 以二次曲线方式增加到 380 (当 MID=60 时)。
 * - 如果 MID <= 28, 速度为 270.
 * - 如果 MID >= 60, 速度为 380.
 * - 否则, 速度按二次函数插值。
 * 具体地, 使用 (归一化MID比例)^2 的方式来实现加速上升效果。
 *
 * @param current_mid_value 当前中间电感传感器的读数。
 * @return 计算得到的目标速度。
 */
float calculate_dynamic_target_speed_quadratic(float current_mid_value) {
		#if 1
    const int MID_min_threshold = 15;   // 修改：原为 28
    const int speed_at_MID_min = 100;
    const int MID_max_threshold = 25;   // 修改：原为 60
    const int speed_at_MID_max = 360;

    int calculated_speed;

    if (current_mid_value <= MID_min_threshold) {
        calculated_speed = speed_at_MID_min;
    } else if (current_mid_value >= MID_max_threshold) {
        calculated_speed = speed_at_MID_max;
    } else {
        // 1. 归一化 MID 值到 0~1
        float mid_ratio = (current_mid_value - (float)MID_min_threshold) / 
                          ((float)MID_max_threshold - (float)MID_min_threshold);
        
        // 2. 使用四次函数加速因子
        float quartic_factor = mid_ratio * mid_ratio * mid_ratio * mid_ratio;  					// mid_ratio^4
        
        // 3. 计算目标速度
        float speed_range = (float)(speed_at_MID_max - speed_at_MID_min);
        calculated_speed = (int)((float)speed_at_MID_min + quartic_factor * speed_range);
    }

    return calculated_speed;
		
		#endif
		
		

		
		
		
}

/*********************rgb*********************//*********************rgb*********************//*********************rgb*********************/	



void set_rgb_pins(int p26_val, int p74_val, int p07_val) {
    P26 = p26_val;
    P74 = p74_val;
    P07 = p07_val;
}

void control_rgb_led_conditional(float check_value, float abs_threshold, RgbColorCode_t feedback_color, int enable_state) {
    // 检查两个条件：
    // 1. 监控值的绝对值是否达到或超过阈值
    // 2. RGB LED 是否被启用 (enable_state == 1)
	
		#if 1
    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1)) {
        switch (feedback_color) {
            case RGB_COLOR_OFF:
                set_rgb_pins(0, 0, 0);
                break;
            case RGB_COLOR_WHITE:
                set_rgb_pins(0, 0, 1);
                break;
            case RGB_COLOR_CYAN:
                set_rgb_pins(0, 1, 0);
                break;
            case RGB_COLOR_YELLOW_GREEN:
                set_rgb_pins(1, 0, 0);
                break;
            case RGB_COLOR_MAGENTA:
                set_rgb_pins(0, 1, 1);
                break;
            case RGB_COLOR_GREEN:
                set_rgb_pins(1, 1, 0);
                break;
            case RGB_COLOR_RED:
                set_rgb_pins(1, 0, 1);
                break;
            case RGB_COLOR_BLUE:
                set_rgb_pins(1, 1, 1);
                break;
            default:
                set_rgb_pins(0, 0, 0); // 安全起见，关闭LED
                break;
        }
    } else {
        set_rgb_pins(0, 0, 0);
    }
		#endif
		

		
}

void control_rgb_led( RgbColorCode_t feedback_color) {

        switch (feedback_color) {
            case RGB_COLOR_OFF:
                set_rgb_pins(0, 0, 0);
                break;
            case RGB_COLOR_WHITE:
                set_rgb_pins(0, 0, 1);
                break;
            case RGB_COLOR_CYAN:
                set_rgb_pins(0, 1, 0);
                break;
            case RGB_COLOR_YELLOW_GREEN:
                set_rgb_pins(1, 0, 0);
                break;
            case RGB_COLOR_MAGENTA:
                set_rgb_pins(0, 1, 1);
                break;
            case RGB_COLOR_GREEN:
                set_rgb_pins(1, 1, 0);
                break;
            case RGB_COLOR_RED:
                set_rgb_pins(1, 0, 1);
                break;
            case RGB_COLOR_BLUE:
                set_rgb_pins(1, 1, 1);
                break;
            default:
                set_rgb_pins(0, 0, 0); // 安全起见，关闭LED
                break;
        }

}

/*********************角度环*********************//*********************角度环*********************//*********************角度环*********************/	
/*******************************卡尔曼滤波********************************//*******************************卡尔曼滤波********************************//*******************************卡尔曼滤波********************************/

///* 微分时间 */
//float dt_my = 0.005f;
//float rad2deg = 57.29578f;  /* 弧度到角度的换算系数 */

///* 初始偏移量 */
//float origin_ax_offset = 0;
//float origin_ay_offset = 0;
//float origin_az_offset = 0;
//float origin_gx_offset = 0;
//float origin_gy_offset = 0;
//float origin_gz_offset = 0;

///* mpu坐标系换算到x和y轴上的速度 */
//float roll_v = 0;
//float pitch_v = 0;
//float yaw_v = 0;

///* 三个状态 */
//float gyro_roll_my = 0;
//float gyro_pitch_my = 0;
//float gyro_yaw = 0;

///* 观测状态值 */
//float acc_roll = 0;
//float acc_pitch = 0;

///* 最后的估计值 */
//float k_roll = 0;
//float k_pitch = 0;
//float k_yaw = 0;

///* 误差协方差矩阵 */
//float e_P[2][2];
///* 卡尔曼增益系数 */
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
//    gyro_yaw = k_yaw / 30.0f + dt_my * (yaw_v + 0.2f);  /* 做角度补偿 */

//    /* Step 2: 计算先验误差协方差矩阵 */
//    e_P[0][0] += 0.0025f;
//    e_P[1][1] += 0.0025f;

//    /* Step 3: 更新卡尔曼增益K */
//    k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3f);
//    k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3f);

//    /* Step 4: 计算观测状态 */
//    acc_ax_adj = ax - origin_ax_offset;
//    acc_ay_adj = ay - origin_ay_offset;
//    acc_az_adj = az - origin_az_offset;

//    acc_roll = (float)(atan2(acc_ay_adj, acc_az_adj) * rad2deg);
//    acc_pitch = (float)(-atan2(acc_ax_adj,
//        sqrt(acc_ay_adj * acc_ay_adj + acc_az_adj * acc_az_adj)) * rad2deg);

//    k_roll = gyro_roll_my + k_k[0][0] * (acc_roll - gyro_roll_my);
//    k_pitch = gyro_pitch_my + k_k[1][1] * (acc_pitch - gyro_pitch_my);
//    k_yaw = 30.0f * gyro_yaw;

//    /* Step 5: 更新协方差矩阵 */
//    e_P[0][0] *= (1.0f - k_k[0][0]);
//    e_P[1][1] *= (1.0f - k_k[1][1]);
//}

///* 初始化矩阵变量（必须单独写） */
//void init_kalman() {
//    e_P[0][0] = 1.0f; e_P[0][1] = 0.0f;
//    e_P[1][0] = 0.0f; e_P[1][1] = 1.0f;

//    k_k[0][0] = 0.0f; k_k[0][1] = 0.0f;
//    k_k[1][0] = 0.0f; k_k[1][1] = 0.0f;
//}






/**
 * @brief 根据 encoder_speedup_sign 更新 encoder_speedup_element 的值。
 * 如果 sign 为 1, 则基于全局的 l_speed_now 和 r_speed_now 进行累加。
 * 否则, 将 element 清零。
 * * @param p_encoder_speedup_element 指向要更新的 encoder_speedup_element 变量的指针。
 * @param current_encoder_speedup_sign 当前的 encoder_speedup_sign 标志位。
 */
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
        // 假设 gyro_data[0] 在此是可访问的
        *p_angle_accumulator += gyro_data[0] * 0.005f;
    } else {
        *p_angle_accumulator = 0.0f;
    }
}

void change_para(){
	
		if(run_mode==1){
		L_pid.kp=10.9;
		L_pid.ki=0.65;
		L_pid.kd=0;
		 
		R_pid.kp=10.9;
		R_pid.ki=0.65;
		R_pid.kd=0;
		
		//	error = (left_mag - right_mag) / (left_mag + right_mag);
		Turn_PID.kp=144;
		Turn_PID.ki=76.1;
		Turn_PID.kd=56;
		Turn_PID.kp1=0.54;
		dir_loop_limit=120; 
		}
		
		else if(run_mode==2){
		L_pid.kp=4.9;
		L_pid.ki=0.2;
		L_pid.kd=0;
		 
		R_pid.kp=4.9;
		R_pid.ki=0.2;
		R_pid.kd=0;

		Turn_PID.kp=194;
		Turn_PID.ki=66.1;
		Turn_PID.kd=66;
		Turn_PID.kp1=0.34;
			
		dir_loop_limit=48; 
		dir_enlarge=15.5;  
		
		}



}








