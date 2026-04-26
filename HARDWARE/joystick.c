#include "joystick.h"
#include "Motor.h"
#include <stdlib.h>
#include <string.h>

#define MAX_SPEED         2.0f
#define JOYSTICK_STALE_MS 200U   // 摇杆数据超时阈值

int8_t Joystick_LH = 0;
int8_t Joystick_LV = 0;
int8_t Joystick_RH = 0;
int8_t Joystick_RV = 0;

/* 新增：最近一次收到有效摇杆包的时间 */
static volatile uint32_t s_joyLastTick = 0;

uint8_t RxPacket[100];
uint16_t RxPacketLen = 0;

void Joystick_Init(void)
{
    RxPacketLen = 0;
    Joystick_LH = 0;
    Joystick_LV = 0;
    Joystick_RH = 0;
    Joystick_RV = 0;
    s_joyLastTick = 0;
}

uint8_t Joystick_HasRecentData(void)
{
    uint32_t now = HAL_GetTick();
    if (s_joyLastTick == 0) return 0;
    return (now - s_joyLastTick) <= JOYSTICK_STALE_MS;
}

void Joystick_ParseData(uint8_t *data, uint16_t len)
{
    if (len == 0 || data == NULL) return;

    data[len] = '\0';

    char *tag = strtok((char *)data, ",");
    if (tag == NULL) return;
    if (strcmp(tag, "joystick") != 0) return;

    char *lh = strtok(NULL, ",");
    char *lv = strtok(NULL, ",");
    char *rh = strtok(NULL, ",");
    char *rv = strtok(NULL, ",");

    if (lh == NULL || lv == NULL || rh == NULL || rv == NULL) return;

    Joystick_LH = atoi(lh);
    Joystick_LV = atoi(lv);
    Joystick_RH = atoi(rh);
    Joystick_RV = atoi(rv);

    /* 新增：只有成功解析后才刷新时间戳 */
    s_joyLastTick = HAL_GetTick();
}

void Joystick_Control(void)
{
    /* 关键门控：没有新鲜摇杆数据，就不覆写电机 */
    if (!Joystick_HasRecentData())
    {
        return;
    }

    if (Joystick_LH == 0 && Joystick_LV == 0)
    {
        motorPidSetSpeed(0, 0);
        return;
    }

    float motor1Speed = 0.0f;
    float motor2Speed = 0.0f;
    float speed = (Joystick_LV / 100.0f) * MAX_SPEED;

    if (Joystick_LV != 0)
    {
        if (Joystick_LH < 0)
        {
            float turn = (-Joystick_LH / 100.0f) * MAX_SPEED;
            motor1Speed = speed;
            motor2Speed = speed + turn;
        }
        else if (Joystick_LH > 0)
        {
            float turn = (Joystick_LH / 100.0f) * MAX_SPEED;
            motor1Speed = speed + turn;
            motor2Speed = speed;
        }
        else
        {
            motor1Speed = speed;
            motor2Speed = speed;
        }
    }

    motorPidSetSpeed_AntiFlip(motor1Speed, motor2Speed);
}

//注：如果觉得响应延迟影响体感，或其他问题，可以给摇杆控制单独加一个模式
