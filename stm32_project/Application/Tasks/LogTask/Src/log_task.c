/*
 * log_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "log_task.h"

#include "cmsis_os.h"
#include "uart_handle.h"
#include "uart_protocol.h"

// 创建日志任务（在系统初始化时调用）
void log_task_init(void) {
  const osThreadAttr_t log_task_attributes = {
      .name = "LogTask",
      .stack_size = 1024,                 // 根据需求调整
      .priority = osPriorityBelowNormal,  // 低于关键任务
  };
  osThreadNew(log_task, NULL, &log_task_attributes);
}

void log_task(void* argument) {
  log_entry_t entry;
  osMessageQueueId_t mq_id = QueueManager_GetQueueByType(QUEUE_TYPE_LOG);
  if (mq_id == NULL) {
    // TODO log
    osThreadTerminate(NULL);
  }
  // check log queue
  while (1) {
    osStatus_t status = osMessageQueueGet(mq_id, &entry, NULL, osWaitForever);
    if (status == osOK) {
      Send_Log(&entry);
    } else {
      // TODO log
    }
  }
}
