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

typedef enum
{
    SERVO_IDLE,
    SERVO_MOVING,
    SERVO_ERROR
} servo_status_t;


typedef struct
{
    servo_hw_config hw;
    int target_angle;
    int current_angle;
    servo_status_t status;
    uint32_t last_update;
    uint16_t move_speed; // 角度/秒
    uint16_t max_accel;  // 角度/秒²
} servo_instance_t;

/**
 * 初始化
 * @param servo servo
 */
void servo_driver_init(servo_instance_t* servo);
/**
 * 设置目标角度
 * @param servo servo
 * @param angle target angle
 */
void servo_driver_set_angle(servo_instance_t* servo, int angle);

/**
 * 平滑设置角度
 * @param servo servo
 * @param target_angle target angle
 * @param duration_ms interval
 */
void servo_driver_smooth_set_angle(servo_instance_t* servo, int target_angle, uint16_t duration_ms);

#endif /* MODULES_SERVO_INC_SERVO_DRIVER_H_ */
