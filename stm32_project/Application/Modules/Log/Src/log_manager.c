/*
 * log_manager.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

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

void log_record(log_level_t level, log_module_t module, const char* file,
                uint16_t line, const char* format, ...) {
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

  /* 处理代码位置信息 */
  if (file && line) {
    entry.code_location = 1;
    entry.content.location.line = line;

    /* 智能文件名截取策略 */
    const char* base_name = strrchr(file, '/');
    if (!base_name) base_name = strrchr(file, '\\');
    base_name = base_name ? base_name + 1 : file;

    strncpy(entry.content.location.file, base_name,
            sizeof(entry.content.location.file) - 1);
    entry.content.location.file[sizeof(entry.content.location.file) - 1] = '\0';
  } else {
    entry.code_location = 0;
  }

  /* 格式化日志内容（带溢出保护） */
  va_start(args, format);
  int len = vsnprintf(entry.content.message,
                      sizeof(entry.content.message) - 1,  // 保留1字节给结束符
                      format, args);
  va_end(args);

  /* 处理格式化结果 */
  if (len < 0) {
    // 格式化错误处理
    strcpy(entry.content.message, "[LOG FORMAT ERROR]");
  } else if (len >= sizeof(entry.content.message)) {
    // 添加截断标记
    entry.content.message[sizeof(entry.content.message) - 2] = '~';
    entry.content.message[sizeof(entry.content.message) - 1] = '\0';
  } else {
    entry.content.message[len] = '\0';  // 确保终止符
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
