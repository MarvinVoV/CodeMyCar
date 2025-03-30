#include "log_manager.h"

#include <stdio.h>
#include <stdarg.h>

#include "cmsis_os2.h"
#include "log_task.h"
#include "queue_manager.h"


osMessageQueueId_t log_queue = NULL; // 消息队列句柄

static log_output_channel* channels[LOG_MAX_CHANNELS] = {NULL};


static void log_dispatch_entry(log_entry_t* entry);

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
    // 消息队列保存指针（每个元素大小 = 指针大小）
    log_queue = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(log_entry_t*), NULL);
    if (log_queue == NULL)
    {
        /* 处理错误 */
    }

    // 注册消息队列到队列管理器
    QueueManager_registerHandler(QUEUE_TYPE_LOG, log_queue);

    // 初始化日志任务
    LogTask_Init();
}

void log_record(const log_level_t level, const log_module_t module, const char* format, ...)
{
    /* 参数有效性检查 */
    if (!format || (level & LOG_LEVEL_FILTER) == 0 || (module & LOG_MODULE_FILTER) == 0)
    {
        return;
    }

    va_list args;
    int msg_len = 0;
    log_entry_t* entry = NULL;
    char* msg_buffer = NULL;

    // 1. 计算实际需要的消息长度
    va_start(args, format);
    /*
     * (1) NULL 和 0 参数组合：指示函数仅计算长度而不实际写入
     * (2) 返回值：返回格式化内容的理论长度（不含终止符）
     * (3) +1 操作：为字符串终止符 \0 预留空间
     */
    msg_len = vsnprintf(NULL, 0, format, args) + 1;
    va_end(args);
    // 添加长度限制
    if (msg_len > MAX_LOG_LEN) {
        msg_len = MAX_LOG_LEN;
    }

    // 1.1. 动态分配内存：结构体基础大小 + 实际消息长度
    const size_t entry_size = sizeof(log_entry_t) + msg_len;
    entry = (log_entry_t*)pvPortMalloc(entry_size);

    if (entry == NULL)
    {
        // vApplicationMallocFailedHook()
        return;
    }

    memset(entry, 0, sizeof(log_entry_t));
    /* 填充基础信息 */
    entry->timestamp = osKernelGetTickCount();
    entry->module = module;
    entry->level = level;
    /* 消息缓冲区定位（柔性数组成员） */
    msg_buffer = entry->message;


    // 2. 实际格式化内容
    va_start(args, format);
    const int actual_len = vsnprintf(msg_buffer, msg_len, format, args);
    va_end(args);
    /* 处理格式化结果 */
    if (actual_len < 0) {
        strcpy(msg_buffer, "[LOG FORMAT ERROR]");
    } else {
        msg_buffer[actual_len] = '\0'; // 确保终止符
    }


    // Dispatch
    log_dispatch_entry(entry);
}

static void log_dispatch_entry(log_entry_t* entry)
{
    bool entry_queued = false;
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
            }
            else
            {
                const osStatus_t status = osMessageQueuePut(log_queue, &entry, osPriorityNone, osWaitForever);
                // 处理队列满的情况（可选策略）
                if (status == osOK)
                {
                    entry_queued = true;
                }
                else
                {
                    /*错误日志处理*/
                }
            }
        }
    }
    // 若未入队（例如仅有直接通道），立即释放
    if (!entry_queued)
    {
        vPortFree(entry);
    }
}
