
#ifndef __PID_H_
#define __PID_H_

typedef struct{
       float kp;
       float ki;
       float kd;
       float kp1;
       float kp2;
				float kd2;

       int limit_max;
       int limit_min;

       float integral_max;
       float integral_min;
			 float	MaxOut;
				
       float err;
       float err_sum;
       float err_last;
			 float d_err;
			float last;

       float out;
       float integral_out;
       float kp_out;
       float kd_out;
	     float ki_out;
       float Target;
			 float Target_base;

       float now;
			 

}_PID;

extern _PID R_pid,L_pid,Turn_PID,Gyro_PID;
extern _PID ang_pid;  // ×óµç»úPID¿ØÖÆÆ÷



float IncPID(float Encoder, float Target, _PID* sptr);
float PositionPID(float Encoder, float Target, _PID* sptr);
						 
//void f_Integral_Limit(_PID* sptr);
//void f_Output_Limit(_PID* sptr);

#endif