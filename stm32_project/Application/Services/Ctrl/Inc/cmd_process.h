/*
 * cmd_process.h
 *
 *  Created on: Mar 30, 2025
 *      Author: marvin
 */

#ifndef SERVICES_CTRL_INC_CMD_PROCESS_H_
#define SERVICES_CTRL_INC_CMD_PROCESS_H_

#include "ctrl_protocol.h"
#include "motion_service.h"

// Q格式转换宏
#define Q15_TO_FLOAT(x) ((float)(x) / 32768.0f)
#define Q7_TO_FLOAT(x)  ((float)(x) / 128.0f)

typedef struct
{
    MotionContext* motionContext;
} CmdProcessorContext;

void CmdProcessor_Init(CmdProcessorContext* commandContext);

void CmdProcessor_processCommand(const ControlCmd* cmd);
#endif /* SERVICES_CTRL_INC_CMD_PROCESS_H_ */
