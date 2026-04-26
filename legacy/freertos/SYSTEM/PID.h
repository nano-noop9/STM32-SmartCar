#ifndef __PID_H
#define __PID_H

typedef struct 
{
	float target_val;//目标值
	float actual_val;//实际值
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
	
} tPid;

//声明函数
float P_realize(tPid * pid,float actual_val);
void PID_init(void);
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
void HW_PID_Tracking(void);
void MPU6050_GoStraight(void);

#endif

