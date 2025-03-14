/*
 * servo_service.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "servo_service.h"

#include "cmsis_os.h"
#include "log_manager.h"
#include "queue_manager.h"
#include "servo_task.h"

#define SERVO_QUEUE_SIZE 10  // 定义队列大小

static osMessageQueueId_t servoCommandQueue = NULL;

// 初始化函数
void servo_service_init(servo_instance_t* servo)
{
    // 调用驱动层初始化函数
    servo_driver_init(servo);

    // 创建消息队列
    servoCommandQueue = osMessageQueueNew(SERVO_QUEUE_SIZE, sizeof(control_cmd_t), NULL);
    if (servoCommandQueue == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create servo command queue");
        return;
    }

    // 注册队列到queue_manager
    QueueManager_RegisterHandler(QUEUE_TYPE_SERVO, servoCommandQueue);

    // 创建任务
    servo_task_init(servo);
}

// 命令执行函数
void servo_service_exec_cmd(servo_instance_t* servo, const control_cmd_t* cmd)
{
    if (cmd->ctrl_type != CTRL_SERVO)
    {
        return;
    }

    // 提取舵机角度
    const uint8_t angle = cmd->data.servo.angle;

    // 调用舵机设置角度函数
    servo_driver_set_smooth_angle(servo, angle, 15);
}
