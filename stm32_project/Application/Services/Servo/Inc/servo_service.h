/*
 * servo_service.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SERVICES_SERVO_INC_SERVO_SERVICE_H_
#define SERVICES_SERVO_INC_SERVO_SERVICE_H_

#include "ctrl_protocol.h"
#include "servo_driver.h"
#include "sys_utils.h"

// 默认移动速度 200ms/度
#define SERVO_DEFAULT_SPEED_MS 200

typedef struct
{
    int minAngle;            // 最小角度
    int maxAngle;            // 最大角度
    uint16_t smoothStep;     // 平滑步长（每次更新的步长）
    uint16_t updateInterval; // 更新间隔(ms)
    uint16_t baseDelayMs;    // 更新间隔ms (建议5-20ms)
    uint16_t deadZone;       // 死区度数
} InstanceConfig;


typedef struct
{
    ServoDriver* driver;    // 硬件配置
    InstanceConfig* config; // 实例配置
    int targetAngle;        // 目标角度
    int currentAngle;       // 当前角度
    int initAngle;          // 运动起始角度
    int stepSize;           // 步长 默认 1 度
    uint32_t lastUpdate;    // 上次更新的时间
} ServoInstance;


/**
 * @brief 初始化舵机服务
 * @param instance instance
 */
void ServoService_init(ServoInstance* instance);

/**
 * @brief 设置目标角度（带平滑移动）
 * @param instance instance
 * @param angle 目标角度
 * @param speedMs 移动速度（ms/度）
 */
void ServoService_setAngleSmoothly(ServoInstance* instance, const int angle, uint16_t speedMs);

/**
 * @brief 立即设置角度（无平滑）
 * @param instance instance
 * @param angle 目标角度
 */
void ServoService_setAngleImmediate(ServoInstance* instance, const int angle);

/**
 * @brief 更新舵机状态（需周期调用）
 * @param instance 服务句柄
 * @return 当前角度
 */
int ServoService_update(ServoInstance* instance);

#endif /* SERVICES_SERVO_INC_SERVO_SERVICE_H_ */
