/*
 * cmd_process.h
 *
 *  Created on: Mar 30, 2025
 *      Author: marvin
 */

#ifndef SERVICES_CTRL_INC_CMD_PROCESS_H_
#define SERVICES_CTRL_INC_CMD_PROCESS_H_
#include "ctrl_protocol.h"
#include "motor_service.h"
#include "servo_driver.h"

// Q格式转换宏
#define Q15_TO_FLOAT(x) ((float)(x) / 32768.0f)
#define Q7_TO_FLOAT(x)  ((float)(x) / 128.0f)

typedef struct
{
    MotionController* motionCtrl; // 运动控制器实例
    ServoInstance* steerServo;    // 转向舵机实例
} CmdProcessorContext;

void CmdProcessor_Init(CmdProcessorContext* commandContext);

void CmdProcessor_processCommand(const ControlCmd* cmd);
#endif /* SERVICES_CTRL_INC_CMD_PROCESS_H_ */
