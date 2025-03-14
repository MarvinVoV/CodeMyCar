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
    servo->step_delay = 15; // 默认速度15ms/度
}

// 带运动控制的设置函数
void servo_driver_set_smooth_angle(servo_instance_t* servo, int angle, const uint16_t speed_ms)
{
    if (!servo || abs(servo->current_angle - angle) < ANGLE_DEAD_ZONE)
    {
        return; // 小角度不响应
    }

    angle = CLAMP(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    // 配置运动参数
    servo->target_angle = angle;
    servo->step_delay = speed_ms; // 每度移动时间
    servo->status = SERVO_MOVING;

    // 立即执行第一次更新
    servo_driver_update_smooth(servo);

}

// 非阻塞式平滑角度设置 (需在主循环中周期性调用)
void servo_driver_update_smooth(servo_instance_t* servo)
{
    if (!servo || servo->status != SERVO_MOVING) return;

    // 单步逼近
    const int delta = servo->target_angle - servo->current_angle;
    if (delta == 0) {
        servo->status = SERVO_IDLE;
        return;
    }

    const int step = (delta > 0) ? 1 : -1;

    if (abs(delta) > 0) {
        // 逐度逼近
        servo->current_angle += step;
        servo_hal_set_angle(servo->current_angle);

        // 到达目标后停止
        if (servo->current_angle == servo->target_angle) {
            servo->status = SERVO_IDLE;
        }
    }
}