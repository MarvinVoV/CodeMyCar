/*
 * servo_driver.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_driver.h"

#include "cmsis_os.h"
#include "servo_hal.h"

void Servo_Init(Servo_Instance* servo) {
  // 初始化舵机硬件
  SERVO_HAL_Init(&servo->hw);
  servo->current_angle = 90.0f;
  servo->status = SERVO_OK;
  servo->last_update = HAL_GetTick();
}

void Servo_SetAngle(Servo_Instance* servo, float angle) {
  // 设置舵机角度
  if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
  if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

  if (SERVO_HAL_SetAngle(angle)) {
    servo->current_angle = angle;
    servo->last_update = HAL_GetTick();
  } else {
    servo->status = SERVO_COMM_ERROR;
  }
}

Servo_Status Servo_GetStatus(Servo_Instance* servo) {
  // 获取舵机状态
  return servo->status;
}
