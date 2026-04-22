#include "niming.h"
#include "main.h"
#include "usart.h"
uint8_t data_to_send[100];

//通过F1帧发送4个uint16类型的数据
void ANO_DT_Send_F1(uint16_t _a, uint16_t _b, uint16_t _c, uint16_t _d)
{
    uint8_t _cnt = 0;		//计数值
    uint8_t sumcheck = 0;  //和校验
    uint8_t addcheck = 0; //附加和校验
    uint8_t i = 0;
	data_to_send[_cnt++] = 0xAA;//帧头
    data_to_send[_cnt++] = 0xFF;//目标地址
    data_to_send[_cnt++] = 0xF1;//功能码
    data_to_send[_cnt++] = 8; //数据长度
	//单片机为小端模式-低地址存放低位数据，匿名上位机要求先发低位数据，所以先发低地址
	data_to_send[_cnt++] = BYTE0(_a);       
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	 for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];//和校验
        addcheck += sumcheck;//附加校验
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//这里是串口发送函数
}
//，通过F2帧发送4个int16类型的数据
void ANO_DT_Send_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d)   //F2帧  4个  int16 参数
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //和校验
    uint8_t addcheck = 0; //附加和校验
    uint8_t i=0;
   data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF2;
    data_to_send[_cnt++] = 8; //数据长度
	//单片机为小端模式-低地址存放低位数据，匿名上位机要求先发低位数据，所以先发低地址
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//这里是串口发送函数
}
//通过F3帧发送2个int16类型和1个int32类型的数据
void ANO_DT_Send_F3(int16_t _a, int16_t _b, int32_t _c )   //F3帧  2个  int16 参数   1个  int32  参数
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //和校验
    uint8_t addcheck = 0; //附加和校验
    uint8_t i=0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF3;
    data_to_send[_cnt++] = 8; //数据长度
	//单片机为小端模式-低地址存放低位数据，匿名上位机要求先发低位数据，所以先发低地址
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
    data_to_send[_cnt++] = BYTE2(_c);
    data_to_send[_cnt++] = BYTE3(_c);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;

	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//这里是串口发送函数
}

