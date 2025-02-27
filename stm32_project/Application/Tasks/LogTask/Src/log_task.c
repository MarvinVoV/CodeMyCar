/*
 * log_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "pkt_protocol.h"
#include "log_task.h"

#include "cmsis_os2.h"
#include "log_manager.h"
#include "queue_manager.h"
#include "uart_handle.h"

static void LogTask_Execute(void* argument);

/**
 * @brief  LogTask初始化
 */
void LogTask_Init(void)
{
    const osThreadAttr_t log_task_attributes = {
        .name = "LogTask",
        .stack_size = 1024,                // 根据需求调整
        .priority = osPriorityBelowNormal, // 低于关键任务
    };
    osThreadNew(LogTask_Execute, NULL, &log_task_attributes);
}

/**
 * @brief LogTask执行函数
 * @param argument args
 */
static void LogTask_Execute(void* argument)
{
    osMessageQueueId_t log_queue = QueueManager_GetQueueByType(QUEUE_TYPE_LOG);
    if (log_queue == NULL)
    {
        /*错误处理*/
        osThreadTerminate(NULL);
    }

    log_entry_t* entry;
    while (1)
    {
        // 等待队列中的日志（永久阻塞）
        if (osMessageQueueGet(log_queue, &entry, NULL, osWaitForever) == osOK)
        {
            // 日志处理
            UART_Send_Protocol_Log(entry);

            // 释放内存块
            osMemoryPoolFree(log_pool, entry);
        }
    }
}
