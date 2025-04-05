/*
 * servo_hal.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_SERVO_INC_SERVO_HAL_H_
#define MODULES_SERVO_INC_SERVO_HAL_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "sys_utils.h"


typedef struct
{
    TIM_HandleTypeDef* pwmTim; // PWM定时器句柄
    uint32_t channel;          // PWM通道
    uint16_t minPulse;         // 最小脉冲宽度
    uint16_t maxPulse;         // 最大脉冲宽度
    uint16_t safetyPulse;      // 安全位置
    int minAngle;              // 最小角度
    int maxAngle;              // 最大角度
} HAL_ServoConfig;

/**
 * 舵机HAL初始化
 * @param hardwareConfig config
 */
void ServoHAL_init(HAL_ServoConfig* hardwareConfig);


/**
 * 设置角度
 * @param angle angle
 * @return result
 */
bool ServoHAL_setAngle(const int angle);

/**
 * 安全停止接口
 */
void ServoHAL_emergencyStop();

#endif /* MODULES_SERVO_INC_SERVO_HAL_H_ */
