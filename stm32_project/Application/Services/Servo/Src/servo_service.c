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



void ServoService_Init(ServoService* service, ServoInstance* driver)
{
    const ServoServiceConfig config = {
        .smooth_step = 2,
        .update_interval = 5 // ms
    };
    service->driver = driver;
    service->config = config;
    service->target_angle = driver->currentAngle;
    service->current_angle = driver->currentAngle;
    service->last_update = HAL_GetTick();

    // 调用驱动层初始化函数
    ServoDriver_Init(driver);

    // 创建任务
    ServoTask_init(service);
}

void ServoService_SetAngle(ServoService* service, int angle, uint16_t speed_ms)
{
    // 角度限幅
    angle = MAX(MIN(angle, service->driver->hw.max_angle), service->driver->hw.min_angle);

    // 计算步进参数
    int step = service->config.smooth_step;
    if (speed_ms > 0)
    {
        step = (speed_ms * 1000) / service->config.update_interval; // 根据速度计算步长
    }

    // 更新目标状态
    service->target_angle = angle;
    service->driver->stepSize = step;
    service->driver->baseDelayMs = service->config.update_interval;
}

int ServoService_Update(ServoService* service)
{
    uint32_t now = HAL_GetTick();
    if (now - service->last_update < service->config.update_interval)
    {
        return service->current_angle;
    }

    // 计算角度差值
    int delta = service->target_angle - service->current_angle;

    if (delta != 0)
    {
        // 计算步进
        int step = service->driver->stepSize;
        step = (delta > 0) ? MIN(step, delta) : MAX(-step, delta);

        // 更新当前角度
        service->current_angle += step;

        // 调用驱动层实际设置角度
        ServoDriver_setRawAngle(service->driver, service->current_angle);
    }

    service->last_update = now;
    return service->current_angle;
}

void ServoService_SetAngleImmediate(ServoService* service, int angle)
{
    service->target_angle = angle;
    service->current_angle = angle;
    ServoDriver_setRawAngle(service->driver, angle); // 直接调用驱动层
}
