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

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MIN_UPDATE_INTERVAL   2  // 最小2ms更新间隔

static osThreadId_t servoTaskHandle = NULL;
uint16_t calculate_dynamic_delay(int remaining_steps, int total_steps, uint16_t base_delay);

void ServoTask_init(ServoService* service)
{
    const osThreadAttr_t servoTaskAttributes = {
        .name = "ServoTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    servoTaskHandle = osThreadNew(ServoTask_doTask, service, &servoTaskAttributes);
    if (servoTaskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create ServoTask");
        return;
    }
}

void ServoTask(void* pvParameters)
{
    ServoService* service = (ServoService*)pvParameters;
    // 任务主循环
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        ServoService_update(service);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(service->config.update_interval));
    }
}
