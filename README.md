# STM32 Smart Car

基于 STM32F103 的智能车控制系统，包含裸机版本与 FreeRTOS 版本。项目实现了 OLED 状态显示、蓝牙遥控、红外循迹、超声波避障、超声波跟随、MPU6050 姿态控制等功能，并使用 PID 算法完成循迹与跟随控制。

## 项目特点

- 基于 STM32 HAL 库完成底层外设开发
- 提供裸机版本与 FreeRTOS 多任务版本
- 实现电机 PWM 调速、方向控制与运动状态管理
- 使用 PID 算法实现红外循迹与超声波跟随
- 支持蓝牙遥控与串口通信指令解析
- 集成 OLED 显示模块，用于状态、参数和调试信息显示
- 集成 MPU6050，实现姿态数据读取与运动控制
- FreeRTOS 版本对传感器采集、控制逻辑、通信和显示任务进行模块化拆分

## 功能列表

| 功能 | 裸机版本 | FreeRTOS 版本 |
|---|---|---|
| OLED 显示 | 支持 | 支持 |
| 蓝牙遥控 | 支持 | 支持 |
| 红外循迹 | 支持 | 支持 |
| 红外循迹 PID | 支持 | 支持 |
| 超声波避障 | 支持 | 支持 |
| 超声波跟随 | 支持 | 支持 |
| 超声波跟随 PID | 支持 | 支持 |
| MPU6050 姿态控制 | 支持 | 支持 |
| 多任务调度 | 不支持 | 支持 |

## 硬件平台

- 主控芯片：STM32F103 系列
- 开发框架：STM32CubeMX + HAL
- 开发环境：Keil MDK-ARM
- RTOS：FreeRTOS
- 显示模块：OLED
- 通信模块：蓝牙串口模块
- 传感器：红外循迹模块、超声波模块、MPU6050
- 执行机构：直流电机 / 电机驱动模块

## 软件架构

```text
STM32-SmartCar
├── bare-metal              # 裸机版本
│   ├── Core                # CubeMX 生成的核心代码
│   ├── Drivers             # STM32 HAL / CMSIS 驱动
│   ├── HARDWARE            # OLED、MPU6050、电机、超声波等模块
│   ├── SYSTEM              # PID、协议解析等系统模块
│   └── MDK-ARM             # Keil 工程
│
└── freertos                # FreeRTOS 版本
    ├── Core
    ├── Drivers
    ├── Middlewares         # FreeRTOS 源码
    ├── HARDWARE
    ├── SYSTEM
    └── MDK-ARM
