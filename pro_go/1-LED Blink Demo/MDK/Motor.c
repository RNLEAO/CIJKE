#include "headfile.h"
#include "Motor.h"

// 编码器值
float r_encoder = 0;
float l_encoder = 0;
float r_encoder_last = 0;
float l_encoder_last = 0;


//电机速度
float r_speed_now = 0;
float l_speed_now = 0;
float speed_now = 0;


// 累计电机增量
float motor_inc = 0;


//<-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|------------------->
//                  -50                 -35                 -25                 -15                 -5                  0                  5                  15                 25                 35                 50
//<-------(-∞,-50)----|----(-50,-35)------|----(-35,-25)------|----(-25,-15)------|----(-15,-5)-------|-----(-5,1)-------|-----(1,5)--------|-----(5,15)-------|-----(15,25)------|-----(25,35)------|-----(35,50)------|-----(50,+∞)----->
//       Kp=340,Ki=40,Kd=18   Kp=270,Ki=34,Kd=18   Kp=215,Ki=28.5,Kd=18   Kp=140,Ki=21.5,Kd=15   Kp=42,Ki=6.6,Kd=4   Kp=10,Ki=1.3,Kd=1   Kp=42,Ki=6.6,Kd=4   Kp=140,Ki=21.5,Kd=15   Kp=215,Ki=28.5,Kd=18   Kp=270,Ki=34,Kd=18   Kp=300,Ki=37,Kd=18   Kp=340,Ki=40,Kd=18


/**
 * @brief  限幅函数 (Clamp Function)
 *         将输入值限制在 [min_val, max_val] 范围内。
 * @param  value    要进行限幅的数值.
 * @param  min_val  允许的最小值.
 * @param  max_val  允许的最大值.
 * @retval float    限幅后的数值.
 */
float limit_function(float value, float min_val, float max_val) {
  if (value < min_val) {
    return min_val; // 如果 value 小于最小值，返回最小值
  } else if (value > max_val) {
    return max_val; // 如果 value 大于最大值，返回最大值
  } else {
    return value;    // 如果 value 在范围内，返回原始值
  }
}









