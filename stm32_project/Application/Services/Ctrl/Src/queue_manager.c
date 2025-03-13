/*
 * queue_manager.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "queue_manager.h"

#include "cmsis_os.h"

#define MAX_HANDLERS 8

static QueueHandler_t handlers[MAX_HANDLERS];
static uint32_t handler_count = 0;

void QueueManager_RegisterHandler(queue_type type, osMessageQueueId_t queue)
{
    if (handler_count < QUEUE_TYPE_MAX)
    {
        handlers[handler_count].type = type;
        handlers[handler_count].queue = queue;
        handler_count++;
    }
}

osMessageQueueId_t QueueManager_GetQueueByType(queue_type type)
{
    for (uint32_t i = 0; i < handler_count; ++i)
    {
        if (handlers[i].type == type)
        {
            return handlers[i].queue;
        }
    }
    return NULL; // 未找到对应的队列
}

osStatus_t QueueManager_DispatchCommand(const control_cmd_t* cmd)
{
    if (cmd == NULL)
    {
        return osErrorParameter;
    }
    queue_type queue_type;
    switch (cmd->ctrl_type)
    {
        case CTRL_MOTOR:
            queue_type = QUEUE_TYPE_MOTOR;
            break;
        case CTRL_SERVO:
            queue_type = QUEUE_TYPE_SERVO;
            break;
        default:
            return osErrorParameter;
    }

    osMessageQueueId_t mq_id = QueueManager_GetQueueByType(queue_type);
    if (mq_id == NULL)
    {
        return osErrorResource;
    }
    return osMessageQueuePut(
        mq_id,        // 消息队列句柄
        cmd,          // 消息内容
        0,            // 默认优先级
        osWaitForever // 如果消息队列已满，调用线程将无限期等待直到有空闲空间
    );
}
