/*
 * servo_service.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "servo_service.h"

#include "cmsis_os.h"
#include "queue_manager.h"
#include "servo_task.h"

#define SERVO_QUEUE_SIZE 10  // 定义队列大小

static osMessageQueueId_t servoCommandQueue = NULL;

// 初始化函数
void Servo_Service_Init(Servo_Instance* servo) {
  // 创建消息队列
  osMessageQueueAttr_t servoQueueAttributes = {.name = "ServoCommandQueue",
                                               .attr_bits = 0,
                                               .cb_mem = NULL,
                                               .cb_size = 0,
                                               .mq_mem = NULL,
                                               .mq_size = 0};
  servoCommandQueue = osMessageQueueNew(SERVO_QUEUE_SIZE, sizeof(control_cmd_t),
                                        &servoQueueAttributes);
  if (servoCommandQueue == NULL) {
    // 处理队列创建失败的情况
    // 例如：记录日志或断言
    // TODO
    return;
  }

  // 注册队列到queue_manager
  QueueManager_RegisterHandler(QUEUE_TYPE_SERVO, servoCommandQueue);

  // 创建任务
  servo_task_init(servo);
}

// 命令执行函数
void Servo_ExecuteCommand(Servo_Instance* servo, control_cmd_t* cmd) {
  // 检查控制类型是否为舵机
  if (cmd->ctrl_type == CTRL_SERVO) {
    // 提取舵机角度
    uint8_t angle = cmd->data.servo.angle;

    // 调用舵机设置角度函数
    Servo_SetAngle(servo, angle);
  } else {
    // 处理其他控制类型或未知命令类型
    // 例如：记录日志或忽略
  }
}