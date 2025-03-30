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
#include "queue_manager.h"

static osThreadId_t taskHandle = NULL;

void MotorTask_init(MotionController* motionCtrl)
{
    const osThreadAttr_t taskAttributes = {
        .name = "MotorTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    taskHandle = osThreadNew(MotorTask_doTask, motionCtrl, &taskAttributes);
    if (taskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create MotorTask");
        return;
    }
}

void MotorTask_doTask(void* arg)
{
    MotionController* ctrl = (MotionController*)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        MotionController_Update(ctrl, 0.01f); // 10ms周期
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
