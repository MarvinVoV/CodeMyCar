/*
 * servo_driver.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_SERVO_INC_SERVO_DRIVER_H_
#define MODULES_SERVO_INC_SERVO_DRIVER_H_


#include <servo_hal.h>

// 定义舵机相关参数
#define SERVO_MIN_PULSE 500   // 最小脉冲宽度 (500us)
#define SERVO_MAX_PULSE 2500  // 最大脉冲宽度 (2500us)
#define SERVO_ANGLE_MIN 0     // 最小角度 (0°)
#define SERVO_ANGLE_MAX 180   // 最大角度 (180°)

typedef enum {
  SERVO_OK,
  SERVO_OVERHEAT,
  SERVO_BLOCKED,
  SERVO_COMM_ERROR
} Servo_Status;

typedef struct {
  Servo_HWConfig hw;
  float current_angle;
  Servo_Status status;
  uint32_t last_update;
} Servo_Instance;

void Servo_Init(Servo_Instance* servo);
void Servo_SetAngle(Servo_Instance* servo, float angle);
Servo_Status Servo_GetStatus(Servo_Instance* servo);

#endif /* MODULES_SERVO_INC_SERVO_DRIVER_H_ */
