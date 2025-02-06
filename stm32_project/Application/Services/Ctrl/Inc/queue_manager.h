/*
 * queue_manager.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SERVICES_CTRL_INC_QUEUE_MANAGER_H_
#define SERVICES_CTRL_INC_QUEUE_MANAGER_H_

#include "cmsis_os.h"
#include "ctrl_protocol.h"

typedef enum {
  QUEUE_TYPE_LOG = 0x01,  // 日志
  QUEUE_TYPE_SERVO,       // 舵机
  QUEUE_TYPE_MOTOR,       // 电机
  QUEUE_TYPE_SYS,         // 系统
  QUEUE_TYPE_MAX          // 仅判断
} queue_type;

typedef struct {
  queue_type type;
  osMessageQueueId_t queue;
} QueueHandler_t;

/**
 * @brief 注册队列管理器
 */
void QueueManager_RegisterHandler(queue_type type, osMessageQueueId_t queue);
/**
 * @brief 根据类型获取队列
 */
osMessageQueueId_t QueueManager_GetQueueByType(queue_type type);

/**
 * 指令分发
 */
osStatus_t QueueManager_DispatchCommand(const control_cmd_t* cmd);

#endif /* SERVICES_CTRL_INC_QUEUE_MANAGER_H_ */
