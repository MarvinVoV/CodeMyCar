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

#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef enum
{
    SERVO_IDLE,
    SERVO_MOVING,
    SERVO_ERROR
} ServoStatus;


typedef struct
{
    ServoHardwareConfig hw; // 硬件配置
    int targetAngle;        // 目标角度
    int currentAngle;       // 当前角度(运动过程中)
    int initialAngle;       // 记录运动起始角度
    ServoStatus status;     // 状态
    int stepSize;           // 步长 默认 1 度
    uint16_t baseDelayMs;   // 基准延时ms (建议5-20ms)
    uint32_t lastUpdate;    // 上次更新时间
} ServoInstance;

/**
 * 初始化
 * @param servo servo
 */
void ServoDriver_Init(ServoInstance* servo);
/**
 * 设置目标角度
 * @param servo servo
 * @param angle target angle
 * @param speedMs speed ms
 */
void ServoDriver_setSmoothAngle(ServoInstance* servo, int angle, uint16_t speedMs);

void ServoDriver_updateSmooth(ServoInstance* servo);

/**
 * @brief 直接设置舵机角度（立即生效，无平滑过程）
 * @param servo 舵机实例指针
 * @param angle 目标角度（SERVO_ANGLE_MIN~SERVO_ANGLE_MAX）
 */
void ServoDriver_setRawAngle(ServoInstance* servo, int angle);

#endif /* MODULES_SERVO_INC_SERVO_DRIVER_H_ */
