#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum { UART = 0, MQTT } QueueType_t;

typedef struct {
  QueueType_t type;  // 队列类型
  void* data;        // 数据指针
  size_t len;        // 数据长度
  void* extra_info;  // 扩展信息
} QueueMessage_t;

/**
 * @brief 初始化消息队列系统
 */
bool queue_manager_init(QueueType_t queue_type);

/**
 * @brief 销毁消息队列
 */
void queue_manager_destroy(QueueType_t queue_type);

/**
 * @brief 获取指定队列句柄
 * @param queue_type 队列类型（MQTT发送队列/UART发送队列）
 * @return QueueHandle_t 队列句柄
 */
QueueHandle_t queue_manager_get_queue(QueueType_t queue_type);
