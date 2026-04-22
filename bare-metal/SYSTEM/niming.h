#ifndef  NIMING_H
#define  NIMING_H
#include "main.h"
//需要发送16位,32位数据，对数据拆分，之后每次发送单个字节
//拆分过程：对变量dwTemp 去地址然后将其转化成char类型指针，最后再取出指针所指向的内容
#define BYTE0(dwTemp)  (*(char *)(&dwTemp))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp) + 3))


void ANO_DT_Send_F1(uint16_t, uint16_t _b, uint16_t _c, uint16_t _d);
void ANO_DT_Send_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d);
void ANO_DT_Send_F3(int16_t _a, int16_t _b, int32_t _c );

#endif 

