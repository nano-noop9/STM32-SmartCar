#ifndef __SONIC_H
#define __SONIC_H

#include "main.h"

// 函数声明
float HC_SR04_Read(void);
void Sonic_Avoid(void);
void Sonic_Follow(void);

// 外部变量声明（供其他文件读取滤波后的超声波数据）
extern float HC_SR04_ReadBuf;  // 存放超声测距距离（滤波后）
extern float Follow_PID_Out;   // 定距离pid跟随的目标速度

// 调试变量（用于诊断超声波问题）
extern uint32_t HC_SR04_Debug_Timeout1;  // 等待Echo上升沿超时次数
extern uint32_t HC_SR04_Debug_Timeout2;  // 测量脉冲宽度超时次数
extern uint32_t HC_SR04_Debug_LastCount; // 上次测量的计数值

#endif

