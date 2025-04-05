/*
 * servo_service.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "servo_service.h"

#include "cmsis_os.h"
#include "log_manager.h"
#include "servo_task.h"


void ServoService_init(ServoInstance* instance)
{
    if (instance == NULL || instance->driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid instance or driver");
        return;
    }

    if (instance->config == NULL)
    {
        if (instance->config == NULL)
        {
            static InstanceConfig defaultConfig = {
                .minAngle = 0,
                .maxAngle = 180,
                .smoothStep = 2,
                .updateInterval = 5,
                .baseDelayMs = 10,
                .deadZone = 2
            };
            instance->config = &defaultConfig;
        }
    }

    instance->driver->hw->minAngle = instance->config->minAngle;
    instance->driver->hw->maxAngle = instance->config->maxAngle;

    // 调用驱动层初始化函数
    ServoDriver_Init(instance->driver);

    // 创建任务
    ServoTask_init(instance);
}

void ServoService_setAngleSmoothly(ServoInstance* instance, const int angle, uint16_t speedMs)
{
    if (!instance || !instance->driver) return;
    // 角度限幅
    const int targetAngle = CLAMP(angle, instance->config->minAngle, instance->config->maxAngle);

    // 角度未发生变化，直接返回
    if (instance->currentAngle == targetAngle) return;

    // 死区检测
    const int delta = abs(targetAngle - instance->currentAngle);
    if (delta <= instance->config->deadZone)
    {
        if (instance->targetAngle != instance->currentAngle)
        {
            // 若之前处于移动状态，需要同步目标值
            instance->targetAngle = instance->currentAngle;
        }
        return;
    }
    if (speedMs > 0)
    {
        speedMs = SERVO_DEFAULT_SPEED_MS;
    }

    // 计算动态步长（速度越快步长越大）
    const int smoothStep = instance->config->smoothStep;
    const uint16_t updateInterval = instance->config->updateInterval;

    // 动态步长计算
    const uint32_t totalTime = delta * speedMs;
    const uint32_t cycleCount = totalTime / updateInterval;
    instance->stepSize = (cycleCount > 0) ? (int)ceil((double)delta / cycleCount) : smoothStep;

    // 步长保护:不超过smoothStep且不小于1
    instance->stepSize = CLAMP(instance->stepSize, 1, instance->config->smoothStep);


    // 更新目标状态
    instance->targetAngle = targetAngle;
    instance->initAngle = instance->currentAngle;

    // 立即更新一次
    ServoService_update(instance);
}

int ServoService_update(ServoInstance* instance)
{
    if (!instance || !instance->driver) return -1;

    const uint32_t now = HAL_GetTick();
    const uint32_t lastUpdate = instance->lastUpdate;
    const uint16_t updateInterval = instance->config->updateInterval;

    if ((now - lastUpdate) < updateInterval)
    {
        return instance->currentAngle;
    }
    instance->lastUpdate = now;

    // 到达目标
    if (instance->currentAngle == instance->targetAngle)
    {
        return instance->currentAngle;
    }

    // 计算移动方向和步长
    const int delta = instance->targetAngle - instance->currentAngle;
    const int direction = (delta > 0) ? 1 : -1;
    const int step = abs(delta) > instance->stepSize ? instance->stepSize * direction : delta;
    instance->currentAngle += step;

    ServoDriver_setAngle(instance->driver, instance->currentAngle);
    return instance->currentAngle;
}

void ServoService_setAngleImmediate(ServoInstance* instance, const int angle)
{
    if (!instance || !instance->driver) return;
    // 角度限幅
    const int targetAngle = CLAMP(angle, instance->config->minAngle, instance->config->maxAngle);
    // 直接更新所有状态
    instance->targetAngle = targetAngle;
    instance->currentAngle = targetAngle;
    instance->initAngle = targetAngle;

    ServoDriver_setAngle(instance->driver, instance->currentAngle);
}
