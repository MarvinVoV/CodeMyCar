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

void servo_service_init(servo_instance_t* servo);

void servo_service_exec_cmd(servo_instance_t* servo, const control_cmd_t* cmd);

#endif /* SERVICES_SERVO_INC_SERVO_SERVICE_H_ */
