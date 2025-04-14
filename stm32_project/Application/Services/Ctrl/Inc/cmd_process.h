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

typedef struct
{
    MotionContext* motionContext;
} CmdProcessorContext;

void CmdProcessor_Init(CmdProcessorContext* commandContext);

void CmdProcessor_processCommand(ControlCmd* cmd);
#endif /* SERVICES_CTRL_INC_CMD_PROCESS_H_ */
