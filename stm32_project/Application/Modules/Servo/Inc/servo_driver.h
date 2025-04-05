/*
 * servo_driver.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_SERVO_INC_SERVO_DRIVER_H_
#define MODULES_SERVO_INC_SERVO_DRIVER_H_

#include "servo_hal.h"
#include "sys_utils.h"

// 定义舵机相关参数
#define SERVO_MIN_PULSE 500   // 最小脉冲宽度 (500us)
#define SERVO_MAX_PULSE 2500  // 最大脉冲宽度 (2500us)
#define SERVO_ANGLE_MIN 0     // 最小角度 (0°)
#define SERVO_ANGLE_MAX 180   // 最大角度 (180°)



typedef struct
{
    HAL_ServoConfig* hw; // 硬件配置
} ServoDriver;

/**
 * 初始化
 * @param driver driver
 */
void ServoDriver_Init(const ServoDriver* driver);
/**
 * @brief 直接设置舵机角度（立即生效，无平滑过程）
 * @param driver driver
 * @param angle 目标角度（SERVO_ANGLE_MIN~SERVO_ANGLE_MAX）
 */
void ServoDriver_setAngle(ServoDriver* driver, int angle);
void ServoDriver_emergencyStop();

#endif /* MODULES_SERVO_INC_SERVO_DRIVER_H_ */
