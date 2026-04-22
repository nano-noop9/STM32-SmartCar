/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdint.h>
#include <stdio.h>
#include "Motor.h"
#include "niming.h"
#include "PID.h"
#include "Sonic.h"
#include "Use_cJSON.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void OLED_Show_base(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
short Encoder1Count = 0;
short Encoder2Count = 0;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
int Motor1PWM = 0;
int Motor2PWM = 0;
float Mileage;
uint16_t TimerCount = 0;

uint8_t Usart1_ReadBuf[256];	//串口1 缓冲数组
uint8_t Usart1_ReadCount = 0;	//串口1 接收字节计数
uint8_t Usart3ReceiveData;  //保存串口三接收的数据

float p,i,d,a;

uint8_t OledString[50];
uint8_t Usart3String[50];

float pitch,roll,yaw; // 俯仰角 横滚角 航向角

int8_t carmode;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	PID_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();			
	OLED_Clear(); 
	
	HAL_TIM_Base_Start_IT(&htim1);                //开启定时器1 中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//开启串口1接收中断
	HAL_UART_Receive_IT(&huart3,&Usart3ReceiveData,1);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1 通道1 PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1 通道4 PWM输出
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4
	HAL_TIM_Base_Start_IT(&htim2);				//开启定时器2 中断
	HAL_TIM_Base_Start_IT(&htim4);        //开启定时器4 中断
	
  HAL_Delay(500);//延时0.5秒 6050上电稳定后初始化
  MPU_Init(); //初始化MPU6050
  while(MPU_Init()!=0);
  while(mpu_dmp_init()!=0);

	//motorPidSetSpeed(-1,-1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch (carmode)
    {
      case 0://OLED基本显示
      OLED_Show_base();

      sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
      OLED_ShowString(0,3,OledString,12);
      
      sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
      OLED_ShowString(0,4,OledString,12);
      
      sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
      OLED_ShowString(0,5,OledString,12);

      sprintf((char *)OledString,"carmode:%d  \r\n",carmode);//显示当前模式
      OLED_ShowString(0,6,OledString,12);

      //获得6050数据
		  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 
        
      //显示模式电机停转
      motorPidSetSpeed(0,0);
      break;

      case 1://串口蓝牙显示+遥控模式
      sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//显示两个电机转速 单位：转/秒
		//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
      
      sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//计算小车里程 单位cm
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
      
      sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//显示电池电压
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
      
      sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);
      
      sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);

      //获得6050数据
		  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 
      /*遥控功能在串口三接收中时执行*/
      break;

      case 2://pid循迹
      OLED_Show_base();
      HW_PID_Tracking(); 
      break;

      case 3://超声波避障
      OLED_Show_base();
  
      sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
      OLED_ShowString(0,3,OledString,12);

      Sonic_Avoid();
      break;

      case 4://超声波跟随
      OLED_Show_base();
      
      sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
      OLED_ShowString(0,3,OledString,12);

      Sonic_Follow(); 
      break;

      case 5://MPU6050走直线90°
      sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);

      //获得6050数据
		  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 

      MPU6050_GoStraight();//必须调用这个函数才能转弯
      break;

    }
		//HAL_Delay(5);


/***********************************************************************/		
		//电机速度等信息发送到上位机
		//注意上位机不支持浮点数，所以要乘100
		// ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);
		
		// PID_RW();
		// printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*******************
*  @brief  定时器回调函数
*  @param  
*  @return  
*
*******************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)//htim1 500HZ  2ms 中断一次
	{
		TimerCount++;
		if(TimerCount %5 == 0)//每10ms执行一次
		{
			Encoder1Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
			Encoder2Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);
			//清零计数值
			__HAL_TIM_SET_COUNTER(&htim2,0);
			__HAL_TIM_SET_COUNTER(&htim4,0);
			
			//Motor1Speed 最大值 ≈ 10.4（rps）
			Motor1Speed = (float)Encoder1Count*100/9.6/11/4;
			Motor2Speed = (float)Encoder2Count*100/9.6/11/4;
			
		}
		if(TimerCount %10 ==0)//每20ms一次
		{
		 /*里程数(cm) += 时间周期（s）*车轮转速(转/s)*车轮周长(cm)*/
		  Mileage += 0.02*Motor1Speed*22;

		  Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
		  TimerCount=0;
			
		}
	}
}

/*******************
*  @brief  OLED基本显示函数
*  @param  
*  @return  
*
*******************/
void OLED_Show_base(void)
{
  OLED_Clear();
  sprintf((char *)OledString,"V1:%.2fV2:%.2f",Motor1Speed,Motor2Speed);//显示两个电机的速度
  OLED_ShowString(0,0,OledString,12);
  
  sprintf((char *)OledString,"Mileage:%.2f   ",Mileage);//显示里程数
  OLED_ShowString(0,1,OledString,12);
  
  sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
  OLED_ShowString(0,2,OledString,12);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
