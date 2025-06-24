#include "headfile.h"
#include "control.h"

// 定义PID控制器结构体变量
_PID R_pid;  // 右电机PID控制器
_PID L_pid;  // 左电机PID控制器
_PID ang_pid;  // 左电机PID控制器

_PID Turn_PID; // 转向PID控制器
_PID Gyro_PID;
/**************************************************************************
函数名称：增量式PID控制器
功能描述：计算增量式PID控制输出
输入参数：Encoder - 当前编码器值
          Target - 目标值
          sptr - PID控制器结构体指针
返回值：PID控制输出值
**************************************************************************/
float IncPID(float Encoder, float Target, _PID* sptr) 
{
    float delta_pwm; // PWM增量值
    
    // 1. 更新当前值和目标值
    sptr->now = Encoder; 
    sptr->Target = Target; 
    
    // 2. 计算当前误差
    sptr->err = sptr->Target - sptr->now; 
    
    // 3. 计算PID各项输出
    // 比例项
    sptr->kp_out = sptr->kp * (sptr->err - sptr->err_last);
    // 积分项
    sptr->ki_out = sptr->ki * sptr->err;
    // 微分项
    sptr->kd_out = sptr->kd * (sptr->err - 2 * sptr->err_last + sptr->d_err);
    
    // 4. 计算总PID输出
    delta_pwm = sptr->kp_out + sptr->ki_out + sptr->kd_out;

    // 5. 更新历史误差值
    sptr->d_err = sptr->err_last; // 保存上上次误差
    sptr->err_last = sptr->err;   // 保存上次误差
    // 6. 返回PWM增量值
    return delta_pwm; 
}


/**************************************************************************
函数名称：位置式PID控制器
功能描述：计算位置式PID控制输出
输入参数：Encoder - 当前编码器值
          Target - 目标值
          sptr - PID控制器结构体指针
返回值：PID控制输出值
**************************************************************************/
float PositionPID(float Encoder, float Target, _PID* sptr)
{
    // 1. 更新当前值和目标值
    sptr->now = Encoder;
    sptr->Target = Target;

    // 2. 计算当前误差
    sptr->err = sptr->Target - sptr->now;

    // 3. 更新积分项
    // 累加当前误差，同时考虑积分限幅
    sptr->err_sum += sptr->err;
	
    // 积分限幅设置为-100到100，可调整
    if (sptr->err_sum > 100.0f) 
        sptr->err_sum = 100.0f;
    else if (sptr->err_sum < -100.0f) 
        sptr->err_sum = -100.0f;

    // 4. 计算PID各项输出
    // 比例项
    sptr->kp_out = sptr->kp * sptr->err;
    // 积分项
    sptr->ki_out = sptr->ki * sptr->err_sum;
    // 微分项
    sptr->kd_out = sptr->kd * (sptr->err - sptr->err_last);

    // 5. 计算总PID输出
    sptr->out = sptr->kp_out + sptr->ki_out + sptr->kd_out;

    // 6. 保存历史误差
    sptr->err_last = sptr->err;

    // 7. 返回控制输出值
    return sptr->out;
}



// 简单一阶低通滤波器，滤掉gyro_data[0]噪声
float gyro_lowpass_filter(float new_value, float old_value, float alpha)
{
    // alpha 越小滤波越强（典型 0.1 ~ 0.3）
    return alpha * new_value + (1.0f - alpha) * old_value;
}

float GyroPositionPID(float gyro_now_raw, float Target, _PID* sptr)
{
    static float gyro_filtered = 0.0f;  // 滤波后的陀螺仪值，保持状态
		float deadzone= 0.0f;  
	
	
    // 1️⃣ 滤波陀螺仪数据
    gyro_filtered = gyro_lowpass_filter(gyro_now_raw, gyro_filtered, 0.2f);  // alpha = 0.2，可调

    // 2️⃣ 更新当前值和目标值
    sptr->now = gyro_filtered;
    sptr->Target = Target;

    // 3️⃣ 计算当前误差
    sptr->err = sptr->Target - sptr->now;

    // 4️⃣ 死区处理（防止噪声误触发）
    deadzone = 1.0f;  // 1 deg/s 死区
    if (fabs(sptr->err) < deadzone)
    {
        sptr->err = 0.0f;
    }

    // 5️⃣ 更新积分项
    sptr->err_sum += sptr->err;
    // 积分限幅
    if (sptr->err_sum > 50.0f) 
        sptr->err_sum = 50.0f;
    else if (sptr->err_sum < -50.0f) 
        sptr->err_sum = -50.0f;

    // 6️⃣ 计算PID各项输出
    sptr->kp_out = sptr->kp * sptr->err;
    sptr->ki_out = sptr->ki * sptr->err_sum;
    sptr->kd_out = sptr->kd * (sptr->err - sptr->err_last);

    // 7️⃣ 最终输出，加上角速度抑制项 kp1 * gyro_filtered
    sptr->out = sptr->kp_out + sptr->ki_out + sptr->kd_out - sptr->kp1 * sptr->now;

    // 8️⃣ 保存历史误差
    sptr->err_last = sptr->err;

    // 9️⃣ 返回控制输出值
    return sptr->out;
}


#include <math.h>  /* for fabs() */

float turn_PstPID(float turn_error,_PID* sptr)
{
    float delta_output;

    /* 更新当前误差 */
    sptr->err = turn_error;

    /* 非线性增强型增量 PID 计算 */
    delta_output = sptr->kp * (sptr->err - sptr->err_last)
                 + sptr->ki * sptr->err
                 + sptr->kd * sptr->err * fabs(sptr->err)
                 + sptr->kd2 * sptr->err * sptr->err * sptr->err;

    /* 叠加输出 */
    sptr->out += delta_output;

    /* 输出限幅 */
    if (sptr->out > sptr->limit_max)
        sptr->out = sptr->limit_max;
    else if (sptr->out < sptr->limit_min)
        sptr->out = sptr->limit_min;

    /* 误差历史更新 */
    sptr->err_last = sptr->err;

    return sptr->out;
}

/**
 * @brief  设置PID控制器的参数
 * @param  sptr: 指向PID结构体的指针
 * @param  p:    比例(Proportional)增益
 * @param  i:    积分(Integral)增益
 * @param  d:    微分(Derivative)增益
 * @param  p1:   额外参数 (例如用于前馈或陀螺仪反馈的增益)
 */
void PID_Set_turn(_PID* sptr, float p, float i, float d, float p1)
{
    // 将传入的参数值赋给结构体对应的成员
    sptr->kp = p;
    sptr->ki = i;
    sptr->kd = d;
    sptr->kp1 = p1;
}

/**
 * @brief  设置PID控制器的参数
 * @param  sptr: 指向PID结构体的指针
 * @param  p:    比例(Proportional)增益
 * @param  i:    积分(Integral)增益
 * @param  d:    微分(Derivative)增益
 * @param  p1:   额外参数 (例如用于前馈或陀螺仪反馈的增益)
 */
void PID_Set_inc(_PID* sptr, float p, float i, float d)
{
    // 将传入的参数值赋给结构体对应的成员
    sptr->kp = p;
    sptr->ki = i;
    sptr->kd = d;
}
