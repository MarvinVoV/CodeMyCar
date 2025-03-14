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
        /*
         * 1. 优先处理队列中的新命令（非阻塞模式）
         * 2. 命令抢占: 新命令会立即更新 target_angle，当前运动自动转向新目标
         */

        // 动态超时：运动时等待步延时，空闲时等待10ms
        const uint32_t timeout = servo->status == SERVO_MOVING ? servo->step_delay : 10;
        if (osMessageQueueGet(servoCommandQueue, &cmd, NULL, timeout) == osOK)
        {
            servo_service_exec_cmd(servo, &cmd);
        }

        // 执行平滑运动更新
        if (servo->status == SERVO_MOVING)
        {
            servo_driver_update_smooth(servo);
        }
    }
}
