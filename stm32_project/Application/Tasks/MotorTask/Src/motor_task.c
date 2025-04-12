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
    uint32_t interval = osKernelGetTickFreq() / service->controlFreqHz;

    while (1)
    {
        // 检查是否需要高频率运行
        bool isActive = false;
        for (int i = 0; i < MOTOR_MAX_NUM; i++)
        {
            if (service->instances[i].state.velocity.rpm != 0)
            {
                isActive = true;
                break;
            }
        }
        // 动态调整间隔
        if (isActive)
        {
            interval = osKernelGetTickFreq() / 100; // 高频率：100 Hz
        }
        else
        {
            interval = osKernelGetTickFreq() / 10; // 低频率：10 Hz
        }

        // 执行状态更新
        MotorService_updateState(service);

        // 精确等待
        osDelayUntil(lastWake + interval);
        lastWake = osKernelGetTickCount();
    }
}
