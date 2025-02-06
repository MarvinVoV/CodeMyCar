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

typedef struct {
  TIM_HandleTypeDef *pwm_tim;  // PWM定时器句柄
  uint32_t channel;            // PWM通道
  uint16_t min_pulse;          // 最小脉冲宽度
  uint16_t max_pulse;          // 最大脉冲宽度
  int min_angle;               // 最小角度
  int max_angle;               // 最大角度
} Servo_HWConfig;

void SERVO_HAL_Init(Servo_HWConfig *config);
bool SERVO_HAL_SetAngle(float angle);

#endif /* MODULES_SERVO_INC_SERVO_HAL_H_ */
