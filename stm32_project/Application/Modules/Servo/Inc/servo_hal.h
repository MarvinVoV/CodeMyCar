/*
 * servo_hal.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_SERVO_INC_SERVO_HAL_H_
#define MODULES_SERVO_INC_SERVO_HAL_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

typedef struct
{
    TIM_HandleTypeDef* pwm_tim; // PWM定时器句柄
    uint32_t channel;           // PWM通道
    uint16_t min_pulse;         // 最小脉冲宽度
    uint16_t max_pulse;         // 最大脉冲宽度
    uint16_t safety_pulse;      // 新增安全位置
    int min_angle;              // 最小角度
    int max_angle;              // 最大角度
} servo_hw_config;

/**
 * 舵机HAL初始化
 * @param config config
 */
void servo_hal_init(servo_hw_config* config);


/**
 * 设置角度
 * @param angle angle
 * @return result
 */
bool servo_hal_set_angle(int angle);

/**
 * 安全停止接口
 * @param config config
 */
void servo_hal_emergency_stop();

#endif /* MODULES_SERVO_INC_SERVO_HAL_H_ */
