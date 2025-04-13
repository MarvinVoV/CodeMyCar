/*
 * motor_task.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "motor_task.h"
#include "ctrl_protocol.h"
#include "log_manager.h"
#include "motor_service.h"

#define CONTROL_FREQ_ACTIVE  200   // 活动时控制频率 (Hz)
#define CONTROL_FREQ_IDLE    50    // 空闲时控制频率 (Hz)

static osThreadId_t taskHandle = NULL;

void MotorTask_init(MotorService* motorService)
{
    const osThreadAttr_t taskAttributes = {
        .name = "MotorTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    taskHandle = osThreadNew(MotorTask_doTask, motorService, &taskAttributes);
    if (taskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create MotorTask");
        return;
    }
}

void MotorTask_doTask(void* arg)
{
    MotorService* service = (MotorService*)arg;


    uint32_t lastWake = osKernelGetTickCount();
    uint32_t interval = osKernelGetTickFreq() / CONTROL_FREQ_IDLE;

    while (1)
    {
        // 动态检测状态（增加滤波防止抖动）
        static uint8_t activeCount = 0;
        bool isActive = false;
        for (int i = 0; i < MOTOR_MAX_NUM; i++)
        {
            // 5RPM死区
            if (fabsf(service->instances[i].state.velocity.rpm) > 5.0f)
            {
                isActive = true;
                break;
            }
        }
        // 滞回滤波（防频繁切换）
        if (isActive)
        {
            activeCount = (activeCount < 10) ? activeCount + 1 : 10;
        }
        else
        {
            activeCount = (activeCount > 0) ? activeCount - 1 : 0;
        }

        // 更新频率（滞回阈值）
        if (activeCount >= 8)
        {
            interval = osKernelGetTickFreq() / CONTROL_FREQ_ACTIVE;
        }
        else if (activeCount <= 2)
        {
            interval = osKernelGetTickFreq() / CONTROL_FREQ_IDLE;
        }

        // 执行状态更新
        MotorService_updateState(service);

        // 精确等待
        osDelayUntil(lastWake + interval);
        lastWake = osKernelGetTickCount();
    }
}
