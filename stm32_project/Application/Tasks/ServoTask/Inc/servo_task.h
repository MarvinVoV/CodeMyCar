/*
 * servo_task.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef TASKS_SERVOTASK_SERVO_TASK_H_
#define TASKS_SERVOTASK_SERVO_TASK_H_

#include "queue_manager.h"
#include "servo_service.h"

void servo_task_init(Servo_Instance* servo);
void Servo_Task(void* arg);

#endif /* TASKS_SERVOTASK_SERVO_TASK_H_ */
