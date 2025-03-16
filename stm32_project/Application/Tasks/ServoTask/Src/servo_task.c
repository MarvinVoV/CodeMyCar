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

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

static osThreadId_t servoTaskHandle = NULL;
uint16_t calculate_dynamic_delay(int remaining_steps, int total_steps, uint16_t base_delay);

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
    control_cmd_t *cmd;

    osMessageQueueId_t commandQueue = QueueManager_GetQueueByType(QUEUE_TYPE_SERVO);
    if (commandQueue == NULL)
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
        if (osMessageQueueGet(commandQueue, &cmd, NULL, 0) == osOK)
        {
            // double check
            if (is_ctrl_field_set(cmd->ctrl_fields, CTRL_FIELD_SERVO))
            {
                continue;
            }
            servo_service_exec_cmd(servo, cmd);
            vPortFree(cmd);
        }

        // 执行平滑运动更新
        if (servo->status == SERVO_MOVING)
        {
            const int current_angle = servo->current_angle;
            const int target_angle = servo->target_angle;
            const int remaining_steps = abs(target_angle - current_angle);
            const int total_steps = abs(target_angle - servo->initial_angle);
            const int step = (remaining_steps > 3 * servo->step_size) ? servo->step_size : 1; // 接近目标时切回小步长

            const uint16_t delay = calculate_dynamic_delay(remaining_steps, total_steps, servo->base_delay);

            servo_driver_update_smooth(servo, step);
            osDelay(delay);
        }
        else
        {
            osDelay(10);
        }
    }
}

// 修改后的延时计算函数
uint16_t calculate_dynamic_delay(const int remaining_steps, const int total_steps, const uint16_t base_delay)
{
    if (total_steps == 0) return base_delay; // 防零除错误
    // 加速阶段（前30%步数）
    if (remaining_steps > total_steps * 0.7)
    {
        return MAX(base_delay / 2, 2);
    }

    // 减速阶段（后30%步数）
    if (remaining_steps < total_steps * 0.3)
    {
        return MIN(base_delay * 2, 10);
    }

    // 匀速阶段
    return base_delay;
}
