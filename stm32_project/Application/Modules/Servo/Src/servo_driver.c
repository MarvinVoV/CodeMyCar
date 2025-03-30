/*
 * servo_driver.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include <stdlib.h>
#include "cmsis_os.h"
#include "servo_hal.h"
#include "servo_driver.h"

#define ANGLE_DEAD_ZONE 2 // 死区控制
#define DEFAULT_BASE_DELAY_MS 15

void ServoDriver_Init(ServoInstance* servo)
{
    if (servo == NULL) return;
    // 参数默认值
    if (servo->hw.min_pulse == 0) servo->hw.min_pulse = SERVO_MIN_PULSE;
    if (servo->hw.max_pulse == 0) servo->hw.max_pulse = SERVO_MAX_PULSE;

    // 初始化舵机硬件
    ServoHAL_init(&servo->hw);
    servo->currentAngle = 90;
    servo->targetAngle = 90;
    servo->status = SERVO_IDLE;
    servo->stepSize = 1;                        // 默认 1 度
    servo->baseDelayMs = DEFAULT_BASE_DELAY_MS; // 默认速度15ms/度
}

// 带运动控制的设置函数
void ServoDriver_setSmoothAngle(ServoInstance* servo, int angle, const uint16_t speedMs)
{
    if (!servo || servo->status == SERVO_ERROR) return;
    angle = CLAMP(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    // 死区检测
    if (abs(servo->currentAngle - angle) < ANGLE_DEAD_ZONE)
    {
        if (servo->status == SERVO_MOVING)
        {
            servo->status = SERVO_IDLE;
        }
        return;
    }


    // 配置运动参数
    servo->targetAngle = angle;
    servo->initialAngle = servo->currentAngle;
    servo->baseDelayMs = MAX(speedMs, 2); // 最小2ms保护
    servo->status = SERVO_MOVING;

    // 立即执行第一次更新
    ServoDriver_updateSmooth(servo);
}

void ServoDriver_updateSmooth(ServoInstance* servo)
{
    if (!servo || servo->status != SERVO_MOVING) return;

    const int delta = servo->targetAngle - servo->currentAngle;
    if (delta == 0)
    {
        servo->status = SERVO_IDLE;
        return;
    }

    // 动态计算步长（最后一步直接到位）
    const int step = (abs(delta) > servo->stepSize) ? servo->stepSize * SIGN(delta) : delta;

    servo->currentAngle += step;
    ServoHAL_setAngle(servo->currentAngle);

    // 到达目标后状态更新
    if (servo->currentAngle == servo->targetAngle)
    {
        servo->status = SERVO_IDLE;
    }
}

void ServoDriver_setRawAngle(ServoInstance* servo, int angle)
{
    if (!servo || servo->status == SERVO_ERROR) return;

    // 角度限幅保护
    angle = CLAMP(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    // 直接更新目标值和当前值
    servo->targetAngle = angle;
    servo->currentAngle = angle;

    // 立即设置硬件角度
    ServoHAL_setAngle(angle);

    // 状态更新为就绪（无移动过程）
    servo->status = SERVO_IDLE;
}
