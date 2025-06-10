#include "headfile.h"


//转向增量式PID
float turn_PstPID(float turn_error,PID_TypDef* sptr)
{
  float	Pwm;
  sptr->Bias = turn_error;                                     // 计算当前误差
	
	Pwm = sptr->Kp * (sptr->Bias-sptr->Last_Bias)+sptr->Ki * sptr->Bias +sptr->Kd * sptr->Bias *fabs(sptr->Bias)+sptr->Kd2*sptr->Bias *sptr->Bias *sptr->Bias ;  //P I D
	
//	if(fabs(sptr->Bias) <= 1.5)
//  Pwm = sptr->Kp * (sptr->Bias-sptr->Last_Bias)+sptr->Ki * sptr->Bias +sptr->Kd * sptr->Bias *fabs(sptr->Bias) * 0.7 ;  //P I D
//	else
//	Pwm = sptr->Kp * (sptr->Bias-sptr->Last_Bias)+sptr->Ki * sptr->Bias +sptr->Kd * sptr->Bias *fabs(sptr->Bias) * 1.1;  
	
//  p=((sptr->Bias-sptr->Last_Bias));
  sptr->Pre_Bias=sptr->Last_Bias;                          // 存储误差，用于下次计算
  sptr->Last_Bias=sptr->Bias;		
	
	 
  return Pwm;                                    // 返回增量值
	
}



