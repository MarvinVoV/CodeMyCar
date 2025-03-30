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

#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct
{
    uint16_t smooth_step;     // 平滑步长（度数）
    uint16_t update_interval; // 更新间隔(ms)
} ServoServiceConfig;

// 服务层控制句柄
typedef struct
{
    ServoInstance* driver;     // 驱动层实例
    ServoServiceConfig config; // 服务配置
    int target_angle;          // 服务层目标角度
    int current_angle;         // 服务层当前角度
    uint32_t last_update;      // 最后更新时间戳
} ServoService;

/**
 * @brief 初始化舵机服务
 * @param service 服务句柄
 * @param driver 驱动层实例
 */
void ServoService_init(ServoService* service, ServoInstance* driver);

/**
 * @brief 设置目标角度（带平滑移动）
 * @param service 服务句柄
 * @param angle 目标角度
 * @param speed_ms 移动速度（ms/度）
 */
void ServoService_setAngle(ServoService* service, int angle, uint16_t speed_ms);

/**
 * @brief 更新舵机状态（需周期调用）
 * @param service 服务句柄
 * @return 当前角度
 */
int ServoService_update(ServoService* service);

/**
 * @brief 立即设置角度（无平滑）
 * @param service 服务句柄
 * @param angle 目标角度
 */
void ServoService_setAngleImmediate(ServoService* service, int angle);

#endif /* SERVICES_SERVO_INC_SERVO_SERVICE_H_ */
