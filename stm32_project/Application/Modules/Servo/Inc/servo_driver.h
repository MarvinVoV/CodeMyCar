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
    servo_hw_config hw;    // 硬件配置
    int target_angle;      // 目标角度
    int current_angle;     // 当前角度(运动过程中)
    int initial_angle;     // 记录运动起始角度
    servo_status_t status; // 状态
    int step_size;         // 步长 默认 1 度
    uint16_t base_delay;   // 基准延时ms (建议5-20ms)
    uint32_t last_update;  // 上次更新时间
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
 * @param speed_ms speed ms
 */
void servo_driver_set_smooth_angle(servo_instance_t* servo, int angle, uint16_t speed_ms);

void servo_driver_update_smooth(servo_instance_t* servo, int step_size);

#endif /* MODULES_SERVO_INC_SERVO_DRIVER_H_ */
