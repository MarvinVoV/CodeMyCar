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

void ServoTask_init(ServoInstance* instance)
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
    ServoInstance* instance = (ServoInstance*)pvParameters;
    while (1)
    {
        ServoService_update(instance);
        vTaskDelay(pdMS_TO_TICKS(instance->config->updateInterval));
    }
}
