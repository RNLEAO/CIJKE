#include "headfile.h"
#include "control.h"

// 定义PID控制器结构体变量
_PID R_pid;  // 右电机PID控制器
_PID L_pid;  // 左电机PID控制器

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
