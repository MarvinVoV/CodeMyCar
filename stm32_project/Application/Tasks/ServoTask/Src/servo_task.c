/*
 * servo_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_task.h"
#include "cmsis_os.h"
#include "log_manager.h"
#include "servo_service.h"

static osThreadId_t servoTaskHandle = NULL;

void ServoTask_init(SteerInstance* instance)
{
    const osThreadAttr_t servoTaskAttributes = {
        .name = "ServoTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    servoTaskHandle = osThreadNew(ServoTask_doTask, instance, &servoTaskAttributes);
    if (servoTaskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create ServoTask");
        return;
    }
}

void ServoTask_doTask(void* pvParameters)
{
    SteerInstance* instance = (SteerInstance*)pvParameters;
    uint32_t lastWakeTime = osKernelGetTickCount(); // 获取初始基准时间

    while (1)
    {
        // 获取当前配置（线程安全）
        uint16_t intervalMs;
        osMutexAcquire(instance->mutex, osWaitForever);
        intervalMs = instance->config.updateIntervalMs;
        osMutexRelease(instance->mutex);


        /* 执行控制更新 */
        SteerService_update(instance);

        /* 计算下一次唤醒时间（绝对时间点） */
        const uint32_t nextWakeTime = lastWakeTime + pdMS_TO_TICKS(intervalMs);

        /* 精确延时到指定时间点（单参数调用） */
        osDelayUntil(nextWakeTime);

        /* 更新基准时间 */
        lastWakeTime = nextWakeTime;
    }
}
