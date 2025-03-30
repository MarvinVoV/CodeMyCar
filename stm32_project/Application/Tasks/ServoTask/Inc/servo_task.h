/*
 * servo_task.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef TASKS_SERVOTASK_SERVO_TASK_H_
#define TASKS_SERVOTASK_SERVO_TASK_H_

#include "servo_service.h"

void ServoTask_init(ServoService* service);
void ServoTask_doTask(void* arg);

#endif /* TASKS_SERVOTASK_SERVO_TASK_H_ */
