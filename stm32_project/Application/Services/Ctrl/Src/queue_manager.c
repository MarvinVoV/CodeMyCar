/*
 * queue_manager.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "queue_manager.h"

#include <string.h>

#include "cmsis_os.h"

#define MAX_HANDLERS 8

static QueueHandler_t handlers[MAX_HANDLERS];
static uint32_t handler_count = 0;

static const struct
{
    ctrl_field_t field;
    queue_type_t queue_type;
} field_queue_map[] = {
    {CTRL_FIELD_MOTOR, QUEUE_TYPE_MOTOR},
    {CTRL_FIELD_SERVO, QUEUE_TYPE_SERVO}
};

void QueueManager_RegisterHandler(const queue_type_t type, osMessageQueueId_t queue)
{
    if (handler_count < QUEUE_TYPE_MAX)
    {
        handlers[handler_count].type = type;
        handlers[handler_count].queue = queue;
        handler_count++;
    }
}

osMessageQueueId_t QueueManager_GetQueueByType(const queue_type_t type)
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

    osStatus_t final_status = osErrorResource;
    const uint8_t fields = cmd->ctrl_fields;

    // 遍历所有可能的控制字段
    for (size_t i = 0; i < sizeof(field_queue_map) / sizeof(field_queue_map[0]); ++i)
    {
        if (!is_ctrl_field_set(fields, field_queue_map[i].field))
        {
            continue; // 跳过未设置的字段
        }

        // 获取目标队列
        osMessageQueueId_t mq_id = QueueManager_GetQueueByType(field_queue_map[i].queue_type);
        if (mq_id == NULL)
        {
            continue; // 记录错误日志但继续处理其他字段
        }

        // 计算完整数据长度（包含柔性数组）
        const size_t data_len = sizeof(control_cmd_t) + cmd->ping_text.len;

        // 动态复制数据（保证线程安全）
        control_cmd_t* cmd_copy = pvPortMalloc(data_len);
        if (cmd_copy == NULL)
        {
            final_status = osErrorNoMemory;
            continue;
        }
        memcpy(cmd_copy, cmd, data_len);

        // 发送到队列（接收方负责释放内存）
        const osStatus_t status = osMessageQueuePut(
            mq_id,
            &cmd_copy, // 传递指针的指针
            0,
            osWaitForever // 如果消息队列已满，调用线程将无限期等待直到有空闲空间
        );

        if (status == osOK)
        {
            final_status = osOK; // 至少有一个队列成功
        }
        else
        {
            vPortFree(cmd_copy); // 发送失败立即释放内存
        }
    }
    return final_status;
}
