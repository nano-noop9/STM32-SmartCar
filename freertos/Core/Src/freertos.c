/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"

#include "cJSON.h"
#include <string.h>
#include "Sonic.h"


#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t lastKeyPin = 0;  // 记录最近触发的按键
/* USER CODE END Variables */
osThreadId StopTaskHandle;
osThreadId InputTaskHandle;
osThreadId OLEDTaskHandle;
osThreadId MultiModeTaskHandle;
osMessageQId ModeQueueHandle;
osMessageQId UltrasonicQueueHandle;
osMutexId OLED_MutexHandle;
osSemaphoreId KeySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
  * @brief  清理当前运行模式的资源（停止电机、复位状态等）
  * @param  oldMode: 即将离开的模式值
  * @retval None
  */
void CleanupCurrentMode(int8_t oldMode)
{
	// 所有模式共有操作：停止电机
	motorPidSetSpeed(0, 0);
	osDelay(50);  // 使用软延迟，避免阻塞其他任务
}

/* USER CODE END FunctionPrototypes */

void StartStopTask(void const * argument);
void StartInputTask(void const * argument);
void StartOLEDTask(void const * argument);
void StartMultiModTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of OLED_Mutex */
  osMutexDef(OLED_Mutex);
  OLED_MutexHandle = osMutexCreate(osMutex(OLED_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of KeySem */
  osSemaphoreDef(KeySem);
  KeySemHandle = osSemaphoreCreate(osSemaphore(KeySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ModeQueue */
  osMessageQDef(ModeQueue, 16, int8_t);
  ModeQueueHandle = osMessageCreate(osMessageQ(ModeQueue), NULL);

  /* definition and creation of UltrasonicQueue */
  osMessageQDef(UltrasonicQueue, 16, uint32_t);
  UltrasonicQueueHandle = osMessageCreate(osMessageQ(UltrasonicQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StopTask */
  osThreadDef(StopTask, StartStopTask, osPriorityHigh, 0, 128);
  StopTaskHandle = osThreadCreate(osThread(StopTask), NULL);

  /* definition and creation of InputTask */
  osThreadDef(InputTask, StartInputTask, osPriorityAboveNormal, 0, 128);
  InputTaskHandle = osThreadCreate(osThread(InputTask), NULL);

  /* definition and creation of OLEDTask */
  osThreadDef(OLEDTask, StartOLEDTask, osPriorityBelowNormal, 0, 200);
  OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

  /* definition and creation of MultiModeTask */
  osThreadDef(MultiModeTask, StartMultiModTask, osPriorityNormal, 0, 156);
  MultiModeTaskHandle = osThreadCreate(osThread(MultiModeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartStopTask */
/**
  * @brief  Function implementing the StopTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const * argument)
{
  /* USER CODE BEGIN StartStopTask */
  
  /* Infinite loop */
  for(;;)
  {
    if(carmode == 0)
    {
      motorPidSetSpeed(0,0);  // 设置小车速度为0
    }
    
    osDelay(10);
  }
  /* USER CODE END StartStopTask */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the InputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void const * argument)
{
  /* USER CODE BEGIN StartInputTask */
  extern uint16_t lastKeyPin;
  
  osSemaphoreWait(KeySemHandle, 0);  // 先清空初值
  
  /* Infinite loop */
  for(;;)
  {
		UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);// 获取当前任务的栈高水位值
		printf("StartInputTask Mark: %u words\n", (unsigned int)stackHighWaterMark);
		size_t freeHeapSize = xPortGetFreeHeapSize();// 获取系统的可用堆空间
		printf("StartInputTask Free Heap Size: %u bytes\n", (unsigned int)freeHeapSize);
    
    // 阻塞等待按键中断释放的信号量
    if(osSemaphoreWait(KeySemHandle, osWaitForever) == osOK)
    {
      // 软防抖：用osDelay替代HAL_Delay
      osDelay(20);
      
      // 根据记录的按键类型处理
      if(lastKeyPin == KEY1_Pin)  // KEY1: 循环切换模式
      {
        // 模式循环：1→2→3→4→5→1（模式0是停止，不在循环中）
        carmode = (carmode % 5) + 1;
        //printf("InputTask: KEY1 pressed - Mode: %d\r\n", carmode);
      }
      else if(lastKeyPin == KEY2_Pin)  // KEY2: 停止
      {
        carmode = 0;
        //printf("InputTask: KEY2 pressed - Mode: 0 (STOP)\r\n");
      }
      
      // 防止连续触发：等待用户松开按键
      osDelay(200);
    }
  }
  /* USER CODE END StartInputTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* Infinite loop */
  for(;;)
  {
    // 获取OLED互斥锁，保护显示硬件不被多任务并发访问
    osMutexWait(OLED_MutexHandle, osWaitForever);

    // 所有OLED显示操作都在互斥锁保护下进行
    OLED_Show_base();

    sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_ReadBuf);
    OLED_ShowString(0,3,OledString,12);

    sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);
    OLED_ShowString(0,4,OledString,12);

    sprintf((char *)OledString,"y:%.2f  \r\n",yaw);
    OLED_ShowString(0,5,OledString,12);

    sprintf((char *)OledString,"carmode:%d  \r\n",carmode);
    OLED_ShowString(0,6,OledString,12);

    // 释放OLED互斥锁
    osMutexRelease(OLED_MutexHandle);

    // UART发送（恢复原有功能）
    sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_ReadBuf);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),100);

    sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),100);

    // 恢复100ms刷新频率
    osDelay(100);
  }
  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartMultiModTask */
/**
* @brief Function implementing the MultiModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMultiModTask */
void StartMultiModTask(void const * argument)
{
  /* USER CODE BEGIN StartMultiModTask */
  int8_t lastExecutedMode = -1;  // 记录上一次执行的模式
  uint8_t mpu_retry = 0;  // MPU6050重试计数

  /* Infinite loop */
  for(;;)
  {
    // 模式改变时执行清理逻辑
    if(carmode != lastExecutedMode && lastExecutedMode >= 0)
    {
      CleanupCurrentMode(lastExecutedMode);
    }
    lastExecutedMode = carmode;

    // 根据carmode执行对应的模式
    switch(carmode)
    {
      case 0:  // 模式0：停止模式
        // 获得6050数据（添加超时保护）
        mpu_retry = 0;
        while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0 && mpu_retry < 20) {
          osDelay(5);  // 让出CPU，避免饿死调度器
          mpu_retry++;
        }

        motorPidSetSpeed(0,0);  // 停止电机
        osDelay(100);
        break;

      case 1:  // 模式1：蓝牙遥控模式
        // 遥控命令由UART中断直接处理，此任务仅做数据输出
        // 获得6050数据（添加超时保护）
        mpu_retry = 0;
        while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0 && mpu_retry < 20) {
          osDelay(5);
          mpu_retry++;
        }

        osDelay(100);
        break;

      case 2:  // 模式2：PID循迹
        HW_PID_Tracking();
        break;

      case 3:  // 模式3：超声波避障
        Sonic_Avoid();
        break;

      case 4:  // 模式4：超声波跟随
        Sonic_Follow();
        break;

      case 5:  // 模式5：MPU6050走直线90°
        // 获得6050数据（添加超时保护）
        mpu_retry = 0;
        while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0 && mpu_retry < 20) {
          osDelay(5);
          mpu_retry++;
        }

        MPU6050_GoStraight();
        break;

      default:
        osDelay(10);
        break;
    }

    osDelay(1);  // 防止任务饿死
  }
  /* USER CODE END StartMultiModTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

