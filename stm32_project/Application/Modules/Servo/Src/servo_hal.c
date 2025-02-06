/*
 * servo_drive.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_hal.h"


static Servo_HWConfig *serv_config;

void SERVO_HAL_Init(Servo_HWConfig *config) {
  // 启动PWM输出
  HAL_TIM_PWM_Start(config->pwm_tim, config->channel);
  serv_config = config;
}

bool SERVO_HAL_SetAngle(float angle) {
  // valid serv_config
  if (serv_config == NULL) {
    // TODO log
    return false;
  }
  int min_angle = serv_config->min_angle;
  int max_angle = serv_config->max_angle;

  if (angle < min_angle) angle = min_angle;
  if (angle > max_angle) angle = max_angle;

  // 将角度转换为脉冲宽度 ,计算CCR值
  uint16_t pulse_width =
      serv_config->min_pulse +
      (uint16_t)((float)(serv_config->max_pulse - serv_config->min_pulse) *
                 angle / 180.0f);

  if (__HAL_TIM_SET_COMPARE(serv_config->pwm_tim, serv_config->channel,
                            pulse_width) != HAL_OK) {
    // TODO log
    return false;
  }
  return true;
}
