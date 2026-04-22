#include "Motor.h"
#include "tim.h"
#include "PID.h"
#include "gpio.h"
#include "oled.h"
#include <stdio.h>

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern float Motor1Speed;
extern float Motor2Speed;

float MotorSetSpeedUpCut=0.5; //用于加速减速的变量

/*******************
*  @brief  设置两个电机转速和方向
*  @param  motor1:电机B设置参数、motor2:设置参数
*  @param  motor1: 输入1~100 对应控制B电机正方向速度在1%-100%、输入-1~-100 对应控制B电机反方向速度在1%-100%、motor2同理
*  @return  无
*
*******************/
void Motor_Set (int motor1,int motor2)
{
	//根据参数正负 设置选择方向(0为反方向)
	if(motor2 < 0) BIN1_RESET;
	   else      BIN1_SET;
	if(motor1 < 0) AIN1_RESET;
		else      AIN1_SET;
	
	//motor2 设置电机B的转速
	if(motor2 < 0)
	{
		if(motor2 < -99) motor2 = -99;//边缘检测
		//负的时候（即反转时），绝对值越小  PWM占空比越大（取反）
		//现在的motor1      -1   -99
		//给寄存器或者函数  99  1 
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -(motor2));//修改定时器1 通道1 PA8 Pulse改变占空比
	}
	else
	{
		if(motor2 > 99) motor2 = 99;
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100-motor2);//修改定时器1 通道1 PA8 Pulse改变占空比，正比于motor2
	}
	
	//motor1 设置电机A的转速
	if(motor1 < 0)
	{
		if(motor1 < -99) motor1 = -99;//超过PWM幅值
		//负的时候绝对值越小  PWM占空比越大
		//现在的motor2      -1   -99
		//给寄存器或者函数   99  1 
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -(motor1));//修改定时器1 通道4 PA11 Pulse改变占空比
	}
	else
	{
		if(motor1 > 99) motor1 = 99;
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100-motor1);//修改定时器1 通道4 PA11 Pulse改变占空比，正比于motor2

	}

}  

/*******************
*  @brief  通过PID控制电机转速
*  @param  Motor1Speed:电机1 目标速度、Motor2Speed:电机2 目标速度
*  @return  无
*
*******************/
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed)
{
	//改变电机PID参数的目标速度
	pidMotor1Speed.target_val = Motor1SetSpeed;
	pidMotor2Speed.target_val = Motor2SetSpeed;
	//根据PID计算 输出作用于电机
	//Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
}

//容易得到
/*
//	motorPidSetSpeed(1,2);//向左转弯
//	motorPidSetSpeed(2,1);//向有转弯
//	motorPidSetSpeed(1,1);//前进
//	motorPidSetSpeed(-1,-1);//后退
//	motorPidSetSpeed(0,0);//停止

//	motorPidSetSpeed(-1,1);//左原地旋转
//	motorPidSetSpeed(1,-1);//右原地旋转
*/

//向前加速函数
void motorSpeedUp(void)
{
	if(MotorSetSpeedUpCut <= MAX_SPEED_UP) MotorSetSpeedUpCut +=0.5 ;  //如果没有超过最大值就增加0.5
	
	motorPidSetSpeed(MotorSetSpeedUpCut,MotorSetSpeedUpCut);//设置到电机
}
//向前减速函数
void motorSpeedCut(void)
{
	if(MotorSetSpeedUpCut >=0.5) MotorSetSpeedUpCut-=0.5;//判断是否速度太小
	motorPidSetSpeed(MotorSetSpeedUpCut,MotorSetSpeedUpCut);//设置到电机
}

//红外循迹
/*
本质是利用哪个红外对管DO被拉高判断黑线的位置
有黑线 - DO高电平 小灯灭
没有黑线 - DO低电平 小灯亮
*/
void HW_tracking(void)
{
		if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
	{
		//printf("应该前进\r\n");
		//OLED_ShowChar(48,4,'Q',16);
		motorPidSetSpeed(1,1);//前运动
	}
	else if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 1&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
	{
		//printf("应该右转\r\n");
		//OLED_ShowChar(48,4,'y',16);
		motorPidSetSpeed(2,0.5);//右边运动
	}
	else if(READ_HW_OUT_1 == 1&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
	{
		//printf("快速右转\r\n");
		//OLED_ShowChar(48,4,'Y',16);
		motorPidSetSpeed(2.5,0.5);//快速右转
	}
	else if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 1&&READ_HW_OUT_4 == 0 )
	{
		//printf("应该左转\r\n");
		//OLED_ShowChar(48,4,'z',16);
		motorPidSetSpeed(0.5,2);//左边运动
	}
	else if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 1 )
	{
		//printf("快速左转\r\n");
		//OLED_ShowChar(48,4,'Z',16);
		motorPidSetSpeed(0.5,2.5);//快速左转
	}
	else 
	{
		//OLED_ShowChar(48,4,' ',16);
	}

}
