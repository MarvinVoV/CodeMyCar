/*
 * servo_service.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SERVICES_SERVO_INC_SERVO_SERVICE_H_
#define SERVICES_SERVO_INC_SERVO_SERVICE_H_

#include "ctrl_protocol.h"
#include "servo_driver.h"

void Servo_Service_Init(Servo_Instance* servo);

void Servo_ExecuteCommand(Servo_Instance* servo, control_cmd_t* cmd);

#endif /* SERVICES_SERVO_INC_SERVO_SERVICE_H_ */
