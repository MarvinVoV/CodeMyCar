/*
 * motor_service.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "motor_service.h"
#include "error_code.h"
#include "log_manager.h"
#include "motor_task.h"
#include "pid_controller.h"


#define LOCK_TIMEOUT_MS (50)

static float calculateOdometer(float revolutions, float wheelRadiusMM);

int MotorService_init(MotorService* service, MotorDriver* leftDriver, MotorDriver* rightDriver)
{
    if (!service || !leftDriver || !rightDriver)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_init: invalid instance or hardware driver!");
        return ERR_MOTOR_SVR_INVALID_PARAM;
    }

    memset(service, 0, sizeof(MotorService));
    // 初始化左轮
    service->instances[MOTOR_LEFT_WHEEL].driver = leftDriver;
    service->instances[MOTOR_LEFT_WHEEL].mutex = osMutexNew(NULL);
    if (service->instances[MOTOR_LEFT_WHEEL].mutex == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to create left motor mutex!");
        return ERR_MOTOR_SVR_INIT_FAIL;
    }

    // 初始化右轮
    service->instances[MOTOR_RIGHT_WHEEL].driver = rightDriver;
    service->instances[MOTOR_RIGHT_WHEEL].mutex = osMutexNew(NULL);
    if (service->instances[MOTOR_RIGHT_WHEEL].mutex == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to create right motor mutex!");
        return ERR_MOTOR_SVR_INIT_FAIL;
    }

    // 初始化硬件驱动
    MotorDriver_Init(leftDriver, NULL);
    MotorDriver_Init(rightDriver, NULL);

    // 创建任务
    MotorTask_init(service);
    return ERR_SUCCESS;
}

int MotorService_setTargetRPM(MotorService* service, MotorID motorId, float rpm)
{
    if (!service || motorId >= MOTOR_MAX_NUM)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_setTargetRPM: invalid instance or motor id!");
        return -1;
    }

    MotorInstance* instance = &service->instances[motorId];
    if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
    {
        return -2;
    }

    MotorDriver* driver = instance->driver;

    // 转速限幅
    const float targetRpm = CLAMP(rpm, -1 * driver->spec->maxRPM, driver->spec->maxRPM);

    // 设置闭环模式并更新目标
    MotorDriver_SetMode(driver, MOTOR_DRIVER_MODE_CLOSED_LOOP);
    MotorDriver_SetTargetRPM(driver, targetRpm);

    osMutexRelease(instance->mutex);
    return 0;
}

int MotorService_setOpenLoopDutyCycle(MotorService* service, MotorID motorId, float dutyCycle)
{
    if (!service || motorId >= MOTOR_MAX_NUM)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Invalid instance or motor id!");
        return -1;
    }

    MotorInstance* instance = &service->instances[motorId];
    if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
    {
        return -2;
    }

    MotorDriver* driver = instance->driver;

    // 参数限幅
    const float clampedDuty = CLAMP(dutyCycle, -100.0f, 100.0f);

    // 设置开环模式并更新占空比
    MotorDriver_SetMode(driver, MOTOR_DRIVER_MODE_OPEN_LOOP);
    MotorDriver_SetDutyCycle(driver, clampedDuty);

    // 更新状态
    // instance->state.velocity.rpm = 0; // 开环模式下转速未知
    // instance->state.velocity.mps = 0;
    instance->state.mode = MOTOR_DRIVER_MODE_OPEN_LOOP;
    instance->state.openLoopDuty = clampedDuty;

    osMutexRelease(instance->mutex);
    return 0;
}

int MotorService_emergencyStop(MotorService* service, MotorID motorId)
{
    if (!service || motorId >= MOTOR_MAX_NUM)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_emergencyStop: invalid instance or motor id!");
        return -1;
    }

    MotorInstance* instance = &service->instances[motorId];
    if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
    {
        return -2;
    }

    MotorDriver_EmergencyStop(instance->driver);

    // 状态更新
    instance->state.errorCode = MOTOR_ERR_EMERGENCY_STOP;
    instance->state.velocity.rpm = 0;
    instance->state.velocity.mps = 0;

    osMutexRelease(instance->mutex);
    return 0;
}

void MotorService_updateState(MotorService* service)
{
    if (!service)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_updateState: invalid instance");
        return;
    }

    for (int i = 0; i < MOTOR_MAX_NUM; ++i)
    {
        MotorInstance* instance = &service->instances[i];
        if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
        {
            continue;
        }
        // 驱动状态更新
        MotorDriver_Update(instance->driver);
        const MotorDriverState driverState = MotorDriver_getState(instance->driver);
        // 更新速度
        instance->state.velocity.rpm = driverState.velocity.rpm;
        instance->state.velocity.mps = driverState.velocity.mps;
        // 更新累计值
        instance->state.position.revolutions = driverState.position.revolutions;
        instance->state.position.odometer = calculateOdometer(
            instance->state.position.revolutions,
            instance->driver->spec->wheelRadiusMM
        );
        // 更新控制模式
        instance->state.mode = instance->driver->control.mode;
        // 开环模式下记录占空比
        if (instance->driver->control.mode == MOTOR_DRIVER_MODE_OPEN_LOOP)
        {
            instance->state.openLoopDuty = driverState.openLoopDuty;
        }

        // 更新时间戳
        instance->state.lastUpdate = HAL_GetTick();

        osMutexRelease(instance->mutex);
    }
}

MotorState MotorService_getState(MotorService* service, MotorID motorId)
{
    const MotorState emptyState = {0};
    if (!service || motorId >= MOTOR_MAX_NUM)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_getState: invalid instance or motor id!");
        return emptyState;
    }

    MotorInstance* instance = &service->instances[motorId];
    if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
    {
        return emptyState;
    }
    MotorState stateCopy = {0};
    memcpy(&stateCopy, &instance->state, sizeof(MotorState));

    osMutexRelease(instance->mutex);
    return stateCopy;
}

int MotorService_configurePID(MotorService* service, MotorID motorId, const PID_Params* pidParams)
{
    if (!service || motorId >= MOTOR_MAX_NUM)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "MotorService_configurePID: invalid instance or motor id!");
        return -1;
    }
    if (!pidParams)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "PID params is NULL!");
        return -1;
    }

    MotorInstance* instance = &service->instances[motorId];
    if (osMutexAcquire(instance->mutex, LOCK_TIMEOUT_MS) != osOK)
    {
        return -2;
    }
    // 参数校验
    if (pidParams->kp < 0 || pidParams->ki < 0 || pidParams->kd < 0)
    {
        instance->state.errorCode |= MOTOR_ERR_INVALID_PARAM;

        osMutexRelease(instance->mutex);
        return -1;
    }

    // 应用新参数
    MotorDriver_ConfigurePID(instance->driver, pidParams);

    osMutexRelease(instance->mutex);
    return 0;
}

static float calculateOdometer(const float revolutions, const float wheelRadiusMM)
{
    return revolutions * (2.0f * (float)M_PI) * (wheelRadiusMM / 1000.0f);
}
