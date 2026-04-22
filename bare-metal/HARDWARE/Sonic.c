#include "Sonic.h"
#include "Motor.h"
#include "pid.h"

/*******************
*  @brief  us级延时
*  @param  usdelay:要延时的us时间
*  @return  
*
*******************/
void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000U/1000);//SystemCoreClock:系统频率
  do
  {
    __NOP();
  }
  while (Delay --);
}

/*******************
*  @brief  HC_SR04读取超声波距离
*  @param  无
*  @return 障碍物距离单位:cm (静止表面平整精度更高) 
*注意:两个HC_SR04_Read()函数调用的时间间隔要2ms及以上，测量范围更大 精度更高 
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);//输出15us高电平
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);//高电平输出结束，设置为低电平
	
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)//等待回响高电平
	{
		i++;
		HC_SR04_Delayus(1);
		if(i>100000) return -1;//超时退出循环、防止程序卡死这里
	}
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET)//下面的循环是2us
	{
		i = i+1;
		HC_SR04_Delayus(1);//1us 延时，但是整个循环大概2us左右
		if(i >100000) return -2;//超时退出循环
	}
	Distance = i*2*0.033/2;//这里乘2的原因是上面是2微妙
	return Distance	;
}

/*******************
*  @brief  超声波避障
*  @param  无
*  @return 无
*注意:延时 = 让小车 “执行完这个动作”，不然动作刚一开始就立刻被下一句覆盖，小车会乱抖、不执行、乱转。
*******************/
void Sonic_Avoid(void)
{
	//避障逻辑
	if(HC_SR04_Read() > 25)  //前方无障碍物
	{
		motorPidSetSpeed(1,1);  //前运动
		HAL_Delay(100);
	}
	else  //前方有障碍物
	{
		motorPidSetSpeed(1,-1);  //右边运动 原地	
		HAL_Delay(500);
		
			if(HC_SR04_Read() > 25)  //右边无障碍物
			{
				motorPidSetSpeed(1,1);  //前运动
				HAL_Delay(100);
			}
			else  //右边有障碍物
			{
				motorPidSetSpeed(-1,1);  //左边运动 原地
				HAL_Delay(1000);
			
					if(HC_SR04_Read() >25)  //左边无障碍物
					{
						motorPidSetSpeed(1,1);  //前运动
						HAL_Delay(100);
					}
					else  //左边有障碍物
					{
						motorPidSetSpeed(-1,-1);  //后运动
						HAL_Delay(1000);
						motorPidSetSpeed(1,-1);  //右边运动
						HAL_Delay(50);
					}
			}
	}
}

float HC_SR04_ReadBuf; //存放超声测距距离
float Follow_PID_Out; //定距离pid跟随的目标速度
extern tPid pidFollow;
//PID跟随功能
void Sonic_Follow(void)
{
   HC_SR04_ReadBuf=HC_SR04_Read();//读取前方障碍物距离
	 if(HC_SR04_ReadBuf < 50)
	 {
			 //如果前60cm 有东西就启动跟随
		 Follow_PID_Out = PID_realize(&pidFollow,HC_SR04_ReadBuf);//PID计算输出目标速度 这个速度，会和基础速度加减
		 if(Follow_PID_Out > 4) Follow_PID_Out = 6;//对输出速度限幅
		 if(Follow_PID_Out < -4) Follow_PID_Out = -6;
		 motorPidSetSpeed(Follow_PID_Out,Follow_PID_Out);//速度作用与电机上
	 }
	 else motorPidSetSpeed(0,0);//如果前面60cm 没有东西就停止
		HAL_Delay(10);//读取超声波传感器不能过快
}
