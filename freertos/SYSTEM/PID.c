#include "pid.h"
#include "gpio.h"
#include <stdint.h>
#include "Motor.h"

tPid pidMotor1Speed;//电机1的转速控制
tPid pidMotor2Speed;//电机2的转速控制
tPid pidHW_Tracking;//红外循迹控制
tPid pidFollow;    //定距离跟随PID
tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID

uint8_t HW_ReadBuf[4] = {0};//红外循迹传感器数据
int8_t CurrentHWState = 0;//当前循迹状态
int8_t LastHWState = 0;//上次循迹状态
float HW_PID_Out;
float HW_PID_Out1;
float HW_PID_Out2;


//初始化PID参数
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;//初始化电机1转速PID 结构体
	pidMotor1Speed.target_val=0.0;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=8.0;
	pidMotor1Speed.Ki=2.0;
	pidMotor1Speed.Kd=0.0;
	
	pidMotor2Speed.actual_val=0.0;//初始化电机2转速PID 结构体
	pidMotor2Speed.target_val=0.0;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=10.0;
	pidMotor2Speed.Ki=1.0;
	pidMotor2Speed.Kd=0.0;
	
	pidHW_Tracking.actual_val=0.0;
	pidHW_Tracking.target_val=0.00;//红外循迹PID 的目标值为0
	pidHW_Tracking.err=0.0;
	pidHW_Tracking.err_last=0.0;
	pidHW_Tracking.err_sum=0.0;
	pidHW_Tracking.Kp=-1.50;
	pidHW_Tracking.Ki=0;
	pidHW_Tracking.Kd=0.80;
	
	pidFollow.actual_val=0.0;
	pidFollow.target_val=22.5;//定距离跟随 目标距离22.5cm
	pidFollow.err=0.0;
	pidFollow.err_last=0.0;
	pidFollow.err_sum=0.0;
	pidFollow.Kp=-0.2;//定距离跟随的Kp大小通过估算PID输入输出数据，确定大概大小，然后在调试
	pidFollow.Ki=-0.002;//Ki小一些
	pidFollow.Kd=0;
	
	pidMPU6050YawMovement.actual_val=0.0;
	pidMPU6050YawMovement.target_val=0.00;//设定姿态目标值
	pidMPU6050YawMovement.err=0.0;
	pidMPU6050YawMovement.err_last=0.0;
	pidMPU6050YawMovement.err_sum=0.0;
	pidMPU6050YawMovement.Kp=0.02;//定距离跟随的Kp大小通过估算PID输入输出数据，确定大概大小，然后在调试
	pidMPU6050YawMovement.Ki=0;
	pidMPU6050YawMovement.Kd=0.1;

	
}

//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节   输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//目标值减去实际值等于误差值
	pid->err_sum += pid->err;//误差累计求和
	//使用PID控制
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差:最近一次 赋值给上次
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

void HW_PID_Tracking(void)
{
	//这样比写在if里更高效
	HW_ReadBuf[0] = READ_HW_OUT_1;
	HW_ReadBuf[1] = READ_HW_OUT_2;
	HW_ReadBuf[2] = READ_HW_OUT_3;
	HW_ReadBuf[3] = READ_HW_OUT_4;
	
	if(HW_ReadBuf[0] == 0&&HW_ReadBuf[1] == 0&&HW_ReadBuf[2] == 0&&HW_ReadBuf[3] == 0 )
	{
		CurrentHWState = 0;//前进
	}
	else if(HW_ReadBuf[0] == 0&&HW_ReadBuf[1] == 1&&HW_ReadBuf[2] == 0&&HW_ReadBuf[3] == 0 )
	{
		CurrentHWState = -1;//应该右转
	}
	else if(HW_ReadBuf[0] == 1 && HW_ReadBuf[1] == 0 && HW_ReadBuf[2] == 0 && HW_ReadBuf[3] == 0 )
	{

		CurrentHWState = -2;//快速右转
	}
	else if(HW_ReadBuf[0] == 1&&HW_ReadBuf[1] == 1&&HW_ReadBuf[2] == 0&&HW_ReadBuf[3] == 0 )
	{
		CurrentHWState = -3;//快速右转
	}
	else if(HW_ReadBuf[0] == 0&&HW_ReadBuf[1] == 0&&HW_ReadBuf[2] == 1&&HW_ReadBuf[3] == 0 )
	{
		CurrentHWState = 1;//应该左转
	}
	else if(HW_ReadBuf[0] == 0&&HW_ReadBuf[1] == 0&&HW_ReadBuf[2] == 0&&HW_ReadBuf[3] == 1 )
	{
		CurrentHWState = 2;//快速左转
	}
	else if(HW_ReadBuf[0] == 0&&HW_ReadBuf[1] == 0&&HW_ReadBuf[2] == 1&&HW_ReadBuf[3] == 1 )
	{
		CurrentHWState = 3;//快速左转
	}
//	else 
//	{
//		motorPidSetSpeed(0.5,0.5);
//	}

	//下面加入PID控制

	//PID计算输出目标速度 这个速度，会和基础速度加减
	HW_PID_Out = PID_realize(&pidHW_Tracking,CurrentHWState);
	HW_PID_Out1 = 3 - HW_PID_Out;//左电机速度=基础速度-PID输出
	HW_PID_Out2 = 3 + HW_PID_Out;//右电机速度=基础速度+PID输出

	if(HW_PID_Out1 > 5) HW_PID_Out1	 = 5;	//限制PID输出最大值 避免过大导致电机转速过快
	if(HW_PID_Out1 < 0) HW_PID_Out1 = 0;	//限制PID输出最小值 避免过小导致电机转速过慢
	if(HW_PID_Out2 > 5) HW_PID_Out2	 = 5;	//限制PID输出最大值 避免过大导致电机转速过快
	if(HW_PID_Out2 < 0) HW_PID_Out2 = 0;

	//注意按如下写法需给两个电机一个初目标速度，否则if一直不成立，或初始化LastHWState = 255；
	if(CurrentHWState != LastHWState)//状态没变就不重复发送速度指令，减少不必要的函数调用
	{
		motorPidSetSpeed(HW_PID_Out1,HW_PID_Out2);
	}
	LastHWState = CurrentHWState;//保存当前状态为上次状态
}


float  MPU6050YawMovePidOut = 0.00f; //姿态PID运算输出
float  MPU6050YawMovePidOut1 = 0.00f; //第一个电机控制输出
float  MPU6050YawMovePidOut2 = 0.00f; //第一个电机控制输出
extern float pitch,roll,yaw; // 俯仰角 横滚角 航向角

//void MPU6050_Control(void)
//{
//	MPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

//	MPU6050YawMovePidOut1 = 1.5 - MPU6050YawMovePidOut;//基础速度加减PID输出速度
//	MPU6050YawMovePidOut2 = 1.5 + MPU6050YawMovePidOut;
//	if(MPU6050YawMovePidOut1 >3.5) MPU6050YawMovePidOut1 =3.5;//进行限幅
//	if(MPU6050YawMovePidOut1 <0) MPU6050YawMovePidOut1 =0;
//	if(MPU6050YawMovePidOut2 >3.5) MPU6050YawMovePidOut2 =3.5;
//	if(MPU6050YawMovePidOut2 <0) MPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(MPU6050YawMovePidOut1,MPU6050YawMovePidOut2);

//}

void MPU6050_GoStraight(void)
{
	MPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

	MPU6050YawMovePidOut1 = 1.5 - MPU6050YawMovePidOut;//基础速度加减PID输出速度
	MPU6050YawMovePidOut2 = 1.5 + MPU6050YawMovePidOut;
	if(MPU6050YawMovePidOut1 >3.5) MPU6050YawMovePidOut1 =3.5;//进行限幅
	if(MPU6050YawMovePidOut1 <0) MPU6050YawMovePidOut1 =0;
	if(MPU6050YawMovePidOut2 >3.5) MPU6050YawMovePidOut2 =3.5;
	if(MPU6050YawMovePidOut2 <0) MPU6050YawMovePidOut2 =0;
	motorPidSetSpeed(MPU6050YawMovePidOut1,MPU6050YawMovePidOut2);

}
