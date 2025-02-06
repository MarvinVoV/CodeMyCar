/*
 * log_task.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef TASKS_LOGTASK_INC_LOG_TASK_H_
#define TASKS_LOGTASK_INC_LOG_TASK_H_

#include "queue_manager.h"
/**
 * 创建日志任务
 */
void log_task_init(void);

/**
 * 日志任务
 */
void log_task(void* argument);

#endif /* TASKS_LOGTASK_INC_LOG_TASK_H_ */
