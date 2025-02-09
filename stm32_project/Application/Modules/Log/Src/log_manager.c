#include "log_manager.h"

#include <stdio.h>

#include "cmsis_os.h"
#include "log_task.h"
#include "queue_manager.h"
#include "stm32h7xx_hal.h"

static osMessageQueueId_t log_queue = NULL;

void log_manager_init(void) {
  log_queue = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(log_entry_t), NULL);
  configASSERT(log_queue != NULL);  // 确保队列创建成功

  QueueManager_RegisterHandler(QUEUE_TYPE_LOG, log_queue);

  log_task_init();
}

void log_record(log_level_t level, log_module_t module, const char* format,
                ...) {
  /* 参数有效性检查 */
  if (!format || (level & LOG_LEVEL_FILTER) == 0 ||
      (module & LOG_MODULE_FILTER) == 0) {
    return;
  }

  log_entry_t entry = {0};
  va_list args;

  /* 填充基础信息 */
  entry.timestamp = HAL_GetTick() - LOG_TIMESTAMP_BASE;
  entry.module = module;
  entry.level = level;

  /* 格式化日志内容（带溢出保护） */
  if (strchr(format, '%') == NULL) {
    strncpy(entry.message, format, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';
  } else {
    va_start(args, format);
    int len = vsnprintf(entry.message,
                        sizeof(entry.message) - 1,  // 保留1字节给结束符
                        format, args);
    va_end(args);
    /* 处理格式化结果 */
    if (len < 0) {
      // 格式化错误处理
      strcpy(entry.message, "[LOG FORMAT ERROR]");
    } else if (len >= sizeof(entry.message)) {
      // 添加截断标记
      entry.message[sizeof(entry.message) - 2] = '~';
      entry.message[sizeof(entry.message) - 1] = '\0';
    } else {
      entry.message[len] = '\0';  // 确保终止符
    }
  }

  // 使用RTOS非阻塞方式发送（等待时间0）
  osStatus_t status = osMessageQueuePut(log_queue, &entry,
                                        osPriorityNone,  // 无优先级提升
                                        0                // 立即返回
  );

  // 处理队列满的情况（可选策略）
  if (status == osErrorResource) {
    // 1. 丢弃新日志（当前策略）
    // 2. 覆盖旧日志：先取出一个再放入
    // log_entry_t temp;
    // osMessageQueueGet(log_queue, &temp, NULL, 0);
    // osMessageQueuePut(log_queue, &entry, osPriorityNone, 0);
  }
}