/*
 * servo_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_task.h"
#include "cmsis_os.h"
#include "ctrl_protocol.h"
#include "log_manager.h"
#include "queue_manager.h"
#include "servo_service.h"

static osThreadId_t servoTaskHandle = NULL;

void servo_task_init(servo_instance_t* servo)
{
    const osThreadAttr_t servoTaskAttributes = {
        .name = "ServoTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    servoTaskHandle = osThreadNew(servo_task, servo, &servoTaskAttributes);
    if (servoTaskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create ServoTask");
        return;
    }
}

// 任务函数
void servo_task(void* arg)
{
    servo_instance_t* servo = (servo_instance_t*)arg;
    control_cmd_t cmd;

    osMessageQueueId_t servoCommandQueue = QueueManager_GetQueueByType(QUEUE_TYPE_SERVO);
    if (servoCommandQueue == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to get ServoCommandQueue");
        osThreadTerminate(NULL);
    }

    while (1)
    {
        // 从队列中获取命令
        if (osMessageQueueGet(servoCommandQueue, &cmd, NULL, osWaitForever) == osOK)
        {
            servo_service_exec_cmd(servo, &cmd);
        }
    }
}
