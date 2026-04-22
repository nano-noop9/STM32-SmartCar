# FreeRTOS智能车修复总结 - 2026-04-22

## 项目背景
- **硬件**: STM32F103C8T6（堆栈空间有限）
- **系统**: FreeRTOS + CMSIS-RTOS v1
- **问题**: 从裸机移植到FreeRTOS后出现严重问题

## 原始问题
1. **超声波跟随模式小车乱走** - 测距数据不稳定
2. **避障模式OLED崩溃** - 多任务并发访问OLED硬件
3. **电机严重卡顿** - 无限busy-wait循环阻塞调度器
4. **超声波测距值一直为0** - 时序问题

## 根本原因分析

### 1. 超声波数据不稳定
- **原因**:
  - 无数据滤波，原始读数直接用于控制
  - 读取间隔10ms违反HC-SR04 60ms最小恢复时间要求
  - 在FreeRTOS中使用osDelay()破坏了微秒级时序测量

- **表现**: 测距值跳变，跟随模式小车乱走

### 2. OLED崩溃
- **原因**:
  - main.c和OLEDTask并发访问OLED硬件（软件I2C）
  - 无互斥锁保护
  - 软件I2C在GPIO上实现，并发访问导致协议错误

- **表现**: 避障模式运行一段时间后OLED显示乱码或死机

### 3. 电机卡顿
- **原因**:
  - MPU6050读取有无限等待循环（while(mpu_dmp_get_data()!=0){}）
  - ISR中有阻塞UART调用（HAL_UART_Receive超时1000ms）
  - OLEDTask中UART超时65秒（0xFFFF）
  - 这些阻塞操作在FreeRTOS中会饿死其他任务

- **表现**: 电机运行不平滑，明显卡顿

### 4. 超声波测距为0
- **原因**:
  - HC_SR04_Read()中添加了osDelay()导致时序破坏
  - osDelay()会触发任务切换，丢失Echo信号的上升沿
  - 无法精确测量脉冲宽度

- **解决**: 移除HC_SR04_Read()中的osDelay()，恢复连续轮询

## 实施的修复方案

### 第一步：CubeMX配置
✅ 已完成
- 添加OLED_Mutex（互斥锁）
- 添加UltrasonicQueue（消息队列，预留）
- OLEDTask栈大小: 200字节
- MultiModeTask栈大小: 156字节
- 调整任务优先级

### 第二步：超声波滤波和时序修复

**文件**: `HARDWARE/Sonic.c`

#### 2.1 添加5点移动平均滤波器
```c
#define FILTER_SIZE 5
typedef struct {
    float buffer[FILTER_SIZE];
    uint8_t index;
    uint8_t count;
} UltrasonicFilter_t;

float FilterDistance(float raw_value)
{
    // 拒绝无效读数（0或>400cm）
    if(raw_value <= 0 || raw_value > 400) {
        // 返回上一次有效值
        if(filter.count > 0) {
            uint8_t last_idx = (filter.index + FILTER_SIZE - 1) % FILTER_SIZE;
            return filter.buffer[last_idx];
        }
        return 0;
    }
    // 计算5点移动平均
    // ...
}
```

#### 2.2 修复HC_SR04_Read()
- **关键改动**: 移除osDelay()，恢复连续轮询
- **原因**: osDelay()会导致任务切换，破坏微秒级时序
- **超时保护**:
  - 等待Echo上升沿: 10ms超时
  - 测量脉冲宽度: 30ms超时
- **返回值**: 无效读数返回0

#### 2.3 修复Sonic_Follow()
- 强制60ms最小读取间隔（HC-SR04硬件要求）
- 使用FilterDistance()滤波数据
- 更新HC_SR04_ReadBuf供OLED显示
- PID参数: Kp=-0.15, Ki=-0.001, Kd=0
- 目标距离: 20cm
- 速度限幅: ±6

#### 2.4 修复Sonic_Avoid()
- 强制60ms最小读取间隔
- 使用FilterDistance()滤波数据
- **关键修复**: 添加`HC_SR04_ReadBuf = filtered_distance`更新显示
- 每次检测前等待60ms

### 第三步：OLED互斥保护

**文件**: `Core/Src/freertos.c`, `Core/Src/main.c`

#### 3.1 OLEDTask中添加互斥锁
```c
osMutexWait(OLED_MutexHandle, osWaitForever);
// 所有OLED操作
OLED_Show_base();
sprintf(...);
OLED_ShowString(...);
osMutexRelease(OLED_MutexHandle);
```

#### 3.2 UART超时优化
- OLEDTask中UART超时: 从0xFFFF(65秒) → 50-100ms
- ISR中UART超时: 从1000ms → 10ms

### 第四步：MPU6050超时保护

**文件**: `Core/Src/freertos.c`

所有MPU6050读取添加超时保护:
```c
uint8_t mpu_retry = 0;
while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0 && mpu_retry < 20) {
    osDelay(5);  // 让出CPU
    mpu_retry++;
}
// 超时后继续使用上次的值，不卡死
```

### 第五步：任务架构优化

**MultiModeTask职责**:
- 根据carmode执行对应的控制逻辑
- 调用Sonic_Follow()、Sonic_Avoid()等
- 读取MPU6050数据
- **不负责OLED显示**（避免冲突）

**OLEDTask职责**:
- 统一负责所有OLED显示
- 统一负责所有UART输出
- 100ms刷新频率
- 使用互斥锁保护OLED访问

### 第六步：调试变量添加

**文件**: `HARDWARE/Sonic.c`, `HARDWARE/Sonic.h`

添加调试变量用于诊断:
```c
extern uint32_t HC_SR04_Debug_Timeout1;  // 等待Echo上升沿超时次数
extern uint32_t HC_SR04_Debug_Timeout2;  // 测量脉冲宽度超时次数
extern uint32_t HC_SR04_Debug_LastCount; // 上次测量的计数值
```

## 当前状态

### ✅ 已修复
1. OLED互斥保护 - 避免并发访问
2. MPU6050超时保护 - 不再无限等待
3. UART超时优化 - 从65秒降低到100ms
4. 超声波滤波 - 5点移动平均
5. 超声波时序 - 强制60ms间隔
6. 模式3显示 - 添加HC_SR04_ReadBuf更新
7. 任务架构 - 清晰的职责划分

### ⚠️ 已知问题
1. **跟随功能不理想** - 需要调整PID参数或目标距离
   - 当前目标距离: 20cm
   - 当前PID: Kp=-0.15, Ki=-0.001, Kd=0
   - 当前速度限幅: ±6
   - **需要用户反馈具体问题**

### 📝 未来改进方向
1. 使用定时器输入捕获模式替代GPIO轮询（更精确）
2. 增加超声波数据的中值滤波
3. 调整PID参数优化跟随效果
4. 考虑增加速度渐变而不是阶跃

## 关键文件修改清单

| 文件 | 修改内容 |
|------|---------|
| `HARDWARE/Sonic.c` | 添加滤波器、修复HC_SR04_Read()、修复Sonic_Follow()和Sonic_Avoid() |
| `HARDWARE/Sonic.h` | 添加extern声明 |
| `Core/Src/freertos.c` | OLEDTask互斥保护、MultiModeTask简化、UART超时优化 |
| `Core/Src/main.c` | 删除while循环中的无效代码、添加extern互斥锁声明 |
| `Core/Src/stm32f1xx_it.c` | ISR中UART超时从1000ms → 10ms |

## 测试建议

### 模式0（停止）
- 检查OLED显示是否正常
- 检查MPU6050数据是否更新

### 模式3（避障）
- 观察OLED上HC_SR04值是否显示
- 测试避障功能是否正常
- 运行10分钟检查OLED是否崩溃

### 模式4（跟随）
- 观察OLED上HC_SR04值是否显示
- 测试跟随效果
- **需要用户反馈**: 跟随是否平滑、距离是否合适

## 下次工作计划

1. **收集用户反馈** - 跟随功能具体问题
2. **调整PID参数** - 根据实际效果优化
3. **可选**: 实现定时器输入捕获模式
4. **可选**: 增加更多滤波算法

## 重要笔记

- **CMSIS-RTOS v1 API**: 使用xTaskGetTickCount()而不是osKernelGetTickCount()
- **堆栈限制**: STM32F103C8T6堆栈有限，OLEDTask 200字节、MultiModeTask 156字节
- **FreeRTOS中的时序**: 不能在需要精确时序的代码中使用osDelay()
- **OLED互斥锁**: 所有OLED操作必须在互斥锁保护下进行
