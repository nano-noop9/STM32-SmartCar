#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"



#define AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)//设置AIN1 PB13为 低电平
#define AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)//设置AIN1 PB13为 高电平

#define BIN1_RESET 	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)  //设置BIN1 PB3为低电平
#define BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)//设置AIN1 PB13为 高电平

#define MAX_SPEED_UP 3

void Motor_Set (int motor1,int motor2);
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);
void motorSpeedUp(void);
void motorSpeedCut(void);
void HW_tracking(void);

#endif
