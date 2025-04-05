/*
 * servo_driver.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include <stdlib.h>
#include "cmsis_os.h"
#include "servo_hal.h"
#include "servo_driver.h"

#include "log_manager.h"

void ServoDriver_Init(const ServoDriver* driver)
{
    if (driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_Init: driver is NULL");
        return;
    }
    // 参数默认值
    if (driver->hw->minPulse == 0) driver->hw->minPulse = SERVO_MIN_PULSE;
    if (driver->hw->maxPulse == 0) driver->hw->maxPulse = SERVO_MAX_PULSE;
    // 初始化舵机硬件
    ServoHAL_init(driver->hw);
}

void ServoDriver_setAngle(ServoDriver* driver, int angle)
{
    if (driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_setAngle: driver is NULL");
        return;
    }

    // 角度限幅保护
    angle = CLAMP(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    // 立即设置硬件角度
    ServoHAL_setAngle(angle);
}

void ServoDriver_emergencyStop()
{
    ServoHAL_emergencyStop();
}
