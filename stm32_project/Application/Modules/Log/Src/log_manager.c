#include "log_manager.h"

#include <stdio.h>

#include "cmsis_os2.h"
#include "log_task.h"
#include "queue_manager.h"

//

osMemoryPoolId_t log_pool = NULL;    // 内存池句柄
osMessageQueueId_t log_queue = NULL; // 消息队列句柄

static log_output_channel* channels[LOG_MAX_CHANNELS] = {NULL};


static void log_dispatch_entry(log_entry_t* entry);
static void sent_log_to_queue(const log_entry_t* entry);

void LogManager_AddOutputChannel(log_output_channel* channel)
{
    for (int i = 0; i < LOG_MAX_CHANNELS; i++)
    {
        if (channels[i] == NULL)
        {
            channels[i] = channel;
            return;
        }
    }
}

void LogManager_Init(void)
{
    // 创建内存池：5个块，每个块大小为日志结构体
    log_pool = osMemoryPoolNew(LOG_POOL_BLOCKS, sizeof(log_entry_t), NULL);
    if (log_pool == NULL)
    {
        /* 处理错误 */
    }

    // 创建消息队列：10个消息，每个消息大小为日志结构体
    log_queue = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(log_entry_t), NULL);
    if (log_queue == NULL)
    {
        /* 处理错误 */
    }

    // 注册消息队列到队列管理器
    QueueManager_RegisterHandler(QUEUE_TYPE_LOG, log_queue);

    // 初始化日志任务
    LogTask_Init();
}

void log_record(const log_level_t level, const log_module_t module, const char* format, ...)
{
    /* 参数有效性检查 */
    if (!format || (level & LOG_LEVEL_FILTER) == 0 ||
        (module & LOG_MODULE_FILTER) == 0)
    {
        return;
    }

    // 申请内存块（0表示不等待）
    log_entry_t* entry = osMemoryPoolAlloc(log_pool, 0);
    if (entry == NULL)
    {
        // 处理内存不足（如丢弃日志或等待）
        return;
    }
    memset(entry, 0, sizeof(log_entry_t));

    /* 填充基础信息 */
    entry->timestamp = osKernelGetTickCount();
    entry->module = module;
    entry->level = level;

    /* 格式化日志内容（带溢出保护） */
    if (strchr(format, '%') == NULL)
    {
        strncpy(entry->message, format, sizeof(entry->message) - 1);
        entry->message[sizeof(entry->message) - 1] = '\0';
    }
    else
    {
        va_list args;
        va_start(args, format);
        const int len = vsnprintf(entry->message,
                                  sizeof(entry->message) - 1, // 保留1字节给结束符
                                  format, args);
        va_end(args);
        /* 处理格式化结果 */
        if (len < 0)
        {
            // 格式化错误处理
            strcpy(entry->message, "[LOG FORMAT ERROR]");
        }
        else if (len >= sizeof(entry->message))
        {
            // 添加截断标记
            entry->message[sizeof(entry->message) - 2] = '~';
            entry->message[sizeof(entry->message) - 1] = '\0';
        }
        else
        {
            entry->message[len] = '\0'; // 确保终止符
        }
    }
    // Dispatch
    log_dispatch_entry(entry);
}

static void log_dispatch_entry(log_entry_t* entry)
{
    for (int i = 0; i < LOG_MAX_CHANNELS; i++)
    {
        if (channels[i] != NULL && (channels[i]->level_filter & entry->level) &&
            (channels[i]->module_filter & entry->module))
        {
            if (channels[i]->direct_output)
            {
            if (channels[i]->huart != NULL)
            {
                // 日志处理
                LOG_DEBUG_UART(channels[i]->huart, "%s\n", entry->message);
            }
                // 释放内存块
                osMemoryPoolFree(log_pool, entry);
            }
            else
            {
                sent_log_to_queue(entry);
            }
        }
    }
}

static void sent_log_to_queue(const log_entry_t* entry)
{
    const osStatus_t status = osMessageQueuePut(log_queue, entry, osPriorityNone, 0);
    // 处理队列满的情况（可选策略）
    if (status == osErrorResource)
    {
        /*错误日志处理*/
    }
}
