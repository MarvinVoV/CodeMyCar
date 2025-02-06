/*
 * servo_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_task.h"
#include "cmsis_os.h"
#include "ctrl_protocol.h"
#include "queue_manager.h"
#include "servo_service.h"

static osThreadId_t servoTaskHandle = NULL;

void servo_task_init(Servo_Instance* servo) {
  osThreadAttr_t servoTaskAttributes = {
      .name = "ServoTask",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  servoTaskHandle = osThreadNew(Servo_Task, servo, &servoTaskAttributes);
  if (servoTaskHandle == NULL) {
    // 处理任务创建失败的情况
    // 例如：记录日志或断言
    return;
  }
}
// 任务函数
void Servo_Task(void* arg) {
  Servo_Instance* servo = (Servo_Instance*)arg;
  control_cmd_t cmd;

  osMessageQueueId_t servoCommandQueue =
      QueueManager_GetQueueByType(QUEUE_TYPE_SERVO);
  if (servoCommandQueue == NULL) {
    // TODO LOG
    osThreadTerminate(NULL);
  }

  while (1) {
    // 从队列中获取命令
    osStatus_t status =
        osMessageQueueGet(servoCommandQueue, &cmd, NULL, osWaitForever);
    if (status == osOK) {
      // 执行命令
      Servo_ExecuteCommand(servo, &cmd);
    }
  }
}
