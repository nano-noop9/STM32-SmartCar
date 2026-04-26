#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "main.h"

void Joystick_Init(void);
void Joystick_ParseData(uint8_t *data, uint16_t len);
void Joystick_Control(void);

/* 新增：摇杆数据是否“新鲜” */
uint8_t Joystick_HasRecentData(void);

#endif
