/*
 * servo_driver.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_driver.h"

#include <stdlib.h>

#include "cmsis_os.h"
#include "servo_hal.h"

#define ANGLE_DEAD_ZONE 2 // 死区控制

void servo_driver_init(servo_instance_t* servo)
{
    if (servo == NULL) return;
    // 参数默认值
    if (servo->hw.min_pulse == 0) servo->hw.min_pulse = SERVO_MIN_PULSE;
    if (servo->hw.max_pulse == 0) servo->hw.max_pulse = SERVO_MAX_PULSE;

    // 初始化舵机硬件
    servo_hal_init(&servo->hw);
    servo->current_angle = 90;
    servo->target_angle = 90;
    servo->status = SERVO_IDLE;
    servo->move_speed = 180; // 默认180度/秒
    servo->max_accel = 360;  // 默认360度/秒²
}

// 带运动控制的设置函数
void servo_driver_set_angle(servo_instance_t* servo, int angle) // NOLINT(*-no-recursion)
{
    if (!servo || abs(servo->current_angle - angle) < ANGLE_DEAD_ZONE)
    {
        return; // 小角度不响应
    }

    angle = CLAMP(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    // 分级运动限制（防止突变）
    static int last_angle = 90;
    const int delta = abs(angle - last_angle);
    if (delta > 45)
    {
        // 大角度分步走
        const int step = (angle > last_angle) ? 30 : -30;
        servo_hal_set_angle(last_angle + step);
        last_angle += step;
        osDelay(100);                         // 简单延时
        servo_driver_set_angle(servo, angle); // 递归调用
        return;
    }

    if (servo_hal_set_angle(angle))
    {
        servo->current_angle = angle;
        last_angle = angle;
    }
}

void servo_driver_smooth_set_angle(servo_instance_t* servo, const int target_angle, const uint16_t duration_ms)
{
    const int start_angle = servo->current_angle;
    const int steps = duration_ms / 20; // 按20ms步进

    for (int i = 1; i <= steps; i++)
    {
        const int new_angle = start_angle + (target_angle - start_angle) * i / steps;
        servo_driver_set_angle(servo, new_angle);
        osDelay(20);
    }
}
