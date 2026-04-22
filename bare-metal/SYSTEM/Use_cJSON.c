#include "Use_cJSON.h"
#include <stdio.h>
#include "usart.h"
#include <string.h>
#include "PID.h"

extern tPid pidMotor1Speed;//电机1的转速控制
extern tPid pidMotor2Speed;//电机2的转速控制
extern uint8_t Usart1_ReadBuf[256];	//串口1 缓冲数组
extern uint8_t Usart1_ReadCount;	//串口1 接收字节计数

extern float p,i,d,a;

/**
* @brief 通过上位机读写PID参数，设置目标速度
* @param none
* @return none
*/
void PID_RW(void)
{
	 cJSON *cJsonData ,*cJsonVlaue;
	if(Usart_WaitReasFinish() == 0)//是否接收完毕
	{
		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
		{
			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
		    p = cJsonVlaue->valuedouble;
		  	pidMotor2Speed.Kp = p;
		}
		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
		{
			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
		    i = cJsonVlaue->valuedouble;
				pidMotor2Speed.Ki = i;
		}
		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
		{
			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
		    d = cJsonVlaue->valuedouble;
				pidMotor2Speed.Kd = d;
		}
		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
		{
		
			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
		    a = cJsonVlaue->valuedouble;
				pidMotor2Speed.target_val =a;
		}
		if(cJsonData != NULL){
		  cJSON_Delete(cJsonData);//释放空间、但是不能删除cJsonVlaue不然会 出现异常错误
		}
		memset(Usart1_ReadBuf,0,255);//清空接收buf，注意这里不能使用strlen	
	}

}

