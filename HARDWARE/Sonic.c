#include "Sonic.h"
#include "Motor.h"
#include "pid.h"
#include "cmsis_os.h"

/********** 超声波数据滤波器 **********/
#define FILTER_SIZE 5  // 滤波器缓冲区大小（5点移动平均）

// 滤波器结构体
typedef struct {
    float buffer[FILTER_SIZE];  // 存储最近5次测量值
    uint8_t index;              // 当前写入位置（循环使用）
    uint8_t count;              // 已存储的有效数据个数
} UltrasonicFilter_t;

static UltrasonicFilter_t filter = {0};  // 静态滤波器实例

// 调试变量：记录超声波读取状态
uint32_t HC_SR04_Debug_Timeout1 = 0;  // 等待Echo上升沿超时次数
uint32_t HC_SR04_Debug_Timeout2 = 0;  // 测量脉冲宽度超时次数
uint32_t HC_SR04_Debug_LastCount = 0; // 上次测量的计数值

/**
 * @brief  超声波数据滤波函数（移动平均滤波）
 * @param  raw_value: 原始测量值（单位：cm）
 * @return 滤波后的距离值（单位：cm）
 * @note   拒绝明显无效的读数（0或>400cm），返回上次有效值
 */
float FilterDistance(float raw_value)
{
    // 拒绝无效读数（0表示测量失败，>400cm超出HC-SR04量程）
    if(raw_value <= 0 || raw_value > 400) {
        // 如果有历史数据，返回上一次的有效值
        if(filter.count > 0) 
		{
			//找到上一次写入数据的位置
            uint8_t last_idx = (filter.index + FILTER_SIZE - 1) % FILTER_SIZE;
            return filter.buffer[last_idx];
        }
        return 0;  // 没有历史数据则返回0
    }

    // 将新数据存入循环缓冲区
    filter.buffer[filter.index] = raw_value;
    filter.index = (filter.index + 1) % FILTER_SIZE;  // 循环索引
    if(filter.count < FILTER_SIZE) filter.count++;    // 累计有效数据个数

    // 计算移动平均值
    float sum = 0;
    for(uint8_t i = 0; i < filter.count; i++) 
	{
        sum += filter.buffer[i];
    }
    return sum / filter.count;  // 返回平均值
}

/*******************
*  @brief  us级延时（FreeRTOS环境下的简化版本）
*  @param  usdelay:要延时的us时间
*  @return
*  @note   在FreeRTOS中，时钟节拍中断会影响精度，但对于HC-SR04足够用
*******************/
void HC_SR04_Delayus(uint32_t usdelay)
{
  // 使用更保守的计算，考虑FreeRTOS中断开销
  __IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000U / 1000);
  do
  {
    __NOP();
  }
  while (Delay --);
}

/*******************
*  @brief  HC_SR04读取超声波距离（FreeRTOS优化版+调试）
*  @param  无
*  @return 障碍物距离单位:cm (静止表面平整精度更高)
*  @note   1. 两次调用间隔需>=60ms（HC-SR04恢复时间要求）
*          2. 不能在轮询中使用osDelay，会破坏微秒级时序
*          3. 通过外层函数控制调用频率，避免长时间占用CPU
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;

	// 发送15us触发脉冲
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);

	// 等待回声信号开始（Echo引脚变高）
	// 注意：这里不能用osDelay，必须连续轮询以捕获上升沿
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)
	{
		i++;
		HC_SR04_Delayus(1);
		// 超时保护：约10ms后退出
		if(i > 10000) {
			HC_SR04_Debug_Timeout1++;  // 记录超时次数
			return 0;  // 返回0表示无效读数
		}
	}

	// 测量回声脉冲宽度（Echo引脚高电平持续时间）
	// 注意：这里也不能用osDelay，必须精确计数
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET)
	{
		i = i+1;
		HC_SR04_Delayus(1);  // 1us延时，实际循环约2us
		// 超时保护：约30ms后退出（对应最大测距约5m）
		if(i > 30000) {
			HC_SR04_Debug_Timeout2++;  // 记录超时次数
			return 0;  // 返回0表示超出量程
		}
	}

	// 记录本次计数值（用于调试）
	HC_SR04_Debug_LastCount = i;

	// 计算距离：i*2us * 0.033cm/us / 2（来回距离）
	Distance = i*2*0.033/2;

	// 有效性检查：HC-SR04有效范围2-400cm
	if(Distance < 2 || Distance > 400) return 0;

	return Distance;
}

/*******************
*  @brief  超声波避障（改进版：添加滤波和时序控制）
*  @param  无
*  @return 无
*  @note   1. 强制60ms最小读取间隔（HC-SR04硬件要求）
*          2. 使用滤波器稳定测距数据
*          3. 延时让小车执行完动作，避免指令被立即覆盖导致抖动
*******************/
void Sonic_Avoid(void)
{
	static uint32_t last_read_tick = 0;  // 记录上次读取时间
	uint32_t current_tick = xTaskGetTickCount();  // 获取当前系统时间（ms）
	float raw_distance, filtered_distance;

	// 强制60ms最小间隔（HC-SR04需要恢复时间）
	if(current_tick - last_read_tick < 60)
 {
		osDelay(60 - (current_tick - last_read_tick));
	}

	// 读取原始数据并滤波
	raw_distance = HC_SR04_Read();
	filtered_distance = FilterDistance(raw_distance);
	last_read_tick = xTaskGetTickCount();  // 更新读取时间

	// 更新全局缓存值，供OLED显示
	HC_SR04_ReadBuf = filtered_distance;

	// 避障逻辑
	if(filtered_distance > 25)  // 前方无障碍物
	{
		motorPidSetSpeed(1,1);  // 前进
		osDelay(100);
	}
	else  // 前方有障碍物
	{
		motorPidSetSpeed(1,-1);  // 右转（原地）
		osDelay(500);

		// 等待60ms后再次测量（遵守时序要求）
		osDelay(60);
		raw_distance = HC_SR04_Read();
		filtered_distance = FilterDistance(raw_distance);
		HC_SR04_ReadBuf = filtered_distance;  // 更新显示

		if(filtered_distance > 25)  // 右边无障碍物
		{
			motorPidSetSpeed(1,1);  // 前进
			osDelay(100);
		}
		else  // 右边有障碍物
		{
			motorPidSetSpeed(-1,1);  // 左转（原地）
			osDelay(1000);

			// 再次等待60ms后测量
			osDelay(60);
			raw_distance = HC_SR04_Read();
			filtered_distance = FilterDistance(raw_distance);
			HC_SR04_ReadBuf = filtered_distance;  // 更新显示

			if(filtered_distance > 25)  // 左边无障碍物
			{
				motorPidSetSpeed(1,1);  // 前进
				osDelay(100);
			}
			else  // 左边有障碍物
			{
				motorPidSetSpeed(-1,-1);  // 后退
				osDelay(1000);
				motorPidSetSpeed(1,-1);  // 右转
				osDelay(50);
			}
		}
	}
}

float HC_SR04_ReadBuf; // 存放超声测距距离
float Follow_PID_Out; // 定距离pid跟随的目标速度
extern tPid pidFollow;

/*******************
*  @brief  PID跟随功能（改进版：添加滤波和时序控制）
*  @param  无
*  @return 无
*  @note   1. 强制60ms最小读取间隔（原来10ms违反HC-SR04时序要求）
*          2. 使用滤波器稳定测距数据，避免小车乱走
*          3. 目标距离20cm，PID控制平滑跟随
*******************/
void Sonic_Follow(void)
{
	static uint32_t last_read_tick = 0;  // 记录上次读取时间
	uint32_t current_tick = xTaskGetTickCount();  // 获取当前系统时间（ms）
	float raw_distance, filtered_distance;

	// 强制60ms最小间隔（HC-SR04需要恢复时间）
	if(current_tick - last_read_tick < 60) {
		osDelay(60 - (current_tick - last_read_tick));
	}

	// 读取原始数据并滤波
	raw_distance = HC_SR04_Read();
	filtered_distance = FilterDistance(raw_distance);
	last_read_tick = xTaskGetTickCount();  // 更新读取时间

	// 使用滤波后的数据
	HC_SR04_ReadBuf = filtered_distance;

	// 如果前50cm有物体就启动跟随
	if(HC_SR04_ReadBuf < 50 && HC_SR04_ReadBuf > 0)
	{
		// PID计算输出目标速度（目标距离20cm）
		Follow_PID_Out = PID_realize(&pidFollow, HC_SR04_ReadBuf);

		// 对输出速度限幅
		if(Follow_PID_Out > 6) Follow_PID_Out = 6;
		if(Follow_PID_Out < -6) Follow_PID_Out = -6;

		// 速度作用于电机
		motorPidSetSpeed_AntiFlip(Follow_PID_Out, Follow_PID_Out);
	}
	else
	{
		// 如果前面50cm没有东西就停止
		motorPidSetSpeed(0, 0);
	}

	// 额外延时，确保总周期约70ms（60ms间隔+10ms处理时间）
	osDelay(10);
}
