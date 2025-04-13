/*
 * servo_service.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "servo_service.h"

#include <error_code.h>

#include "cmsis_os2.h"
#include "log_manager.h"
#include "servo_task.h"

static bool checkPositionReached(const SteerInstance* inst);
static void handleCalibratingState(SteerInstance* inst);
static int handleMovingState(SteerInstance* inst);

int SteerService_init(SteerInstance* steerInstance, SteerConfig* config, ServoDriver* driver)
{
    if (!steerInstance || !config || !driver)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid config or driver");
        return ERR_SRV_SERVO_INVALID_ARG;
    }
    // 初始化配置
    memcpy(&steerInstance->config, config, sizeof(SteerConfig));
    steerInstance->driver = driver;

    // 初始化运行时状态
    steerInstance->state.actualAngleDeg = 0.0f;
    steerInstance->state.targetAngleDeg = 0.0f;
    steerInstance->state.lastUpdateTick = 0;
    steerInstance->state.motionState = STEER_STATE_IDLE;
    steerInstance->state.errorCode = 0;

    // 初始化平滑控制
    steerInstance->smoothCtrl.startAngleDeg = 0.0f;
    steerInstance->smoothCtrl.remainingDeg = 0.0f;
    steerInstance->smoothCtrl.stepDeg = 0.0f;

    steerInstance->calibCtrl.calibPhase = 0;
    steerInstance->calibCtrl.calibStartAngle = 0.0f;

    // 创建互斥锁
    steerInstance->mutex = osMutexNew(NULL);
    if (!steerInstance->mutex)
    {
        return ERR_SRV_SERVO_INIT_FAIL;
    }

    // 调用驱动层初始化函数
    if (ServoDriver_Init(steerInstance->driver) != ERR_SUCCESS)
    {
        return ERR_SRV_SERVO_INIT_FAIL;
    }

    // 创建任务
    ServoTask_init(steerInstance);
    return ERR_SUCCESS;
}

int SteerService_setAngleSmoothly(SteerInstance* instance, float targetAngle, uint16_t speedMs)
{
    if (!instance || !instance->driver)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid instance or driver");
        return ERR_INVALID_PARAM;
    }
    osMutexAcquire(instance->mutex, osWaitForever);

    /*--------------------- 参数预处理 ---------------------*/
    // 角度限幅（浮点版本）
    const float clampedAngle = fmaxf(instance->config.minAngleDeg,
                                     fminf(targetAngle, instance->config.maxAngleDeg));

    // 计算角度差（浮点）
    const float delta = clampedAngle - instance->state.actualAngleDeg;

    // 死区判断（浮点容差）
    if (fabsf(delta) <= instance->config.deadZoneDeg)
    {
        osMutexRelease(instance->mutex);
        return ERR_SUCCESS;
    }

    /*--------------------- 动态步长计算 ---------------------*/
    // 计算总耗时（毫秒）
    const float effectiveSpeed = (speedMs == 0) ? (float)SERVO_DEFAULT_SPEED_MS : (float)speedMs;
    const float totalDurationMs = fabsf(delta) * effectiveSpeed;

    // 计算周期数（ 显式转换为浮点数）
    const float intervalMs = (float)instance->config.updateIntervalMs;
    const float cycleCount = ceilf(totalDurationMs / intervalMs);

    // 计算步长（保留浮点精度）
    float dynamicStep = delta / cycleCount;

    // 步长限幅（支持亚角度级步长）
    const float maxStep = instance->config.defaultStepDeg;
    dynamicStep = copysign(fminf(fabsf(dynamicStep), maxStep), delta);

    /*--------------------- 更新控制上下文 ---------------------*/
    instance->state.targetAngleDeg = clampedAngle;

    // 平滑控制上下文（全浮点）
    instance->smoothCtrl = (SmoothControlContext){
        .startAngleDeg = instance->state.actualAngleDeg,
        .remainingDeg = delta, // 剩余总角度（非步数）
        .stepDeg = dynamicStep // 单步角度（可小于1度）
    };

    // 状态机更新
    instance->state.motionState = STEER_STATE_MOVING;
    instance->state.lastUpdateTick = HAL_GetTick();

    // 立即触发首次更新
    SteerService_update(instance);

    osMutexRelease(instance->mutex);
    return ERR_SUCCESS;
}

int SteerService_update(SteerInstance* instance)
{
    if (!instance || !instance->driver)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid instance or driver");
        return ERR_INVALID_PARAM;
    }
    const uint32_t now = HAL_GetTick();
    if ((now - instance->state.lastUpdateTick) < instance->config.updateIntervalMs)
    {
        return ERR_STEER_TOO_FREQUENT;
    }
    osMutexAcquire(instance->mutex, osWaitForever);

    if (instance->state.motionState != STEER_STATE_MOVING)
    {
        osMutexRelease(instance->mutex);
        return ERR_SUCCESS;
    }

    // 错误状态优先处理
    if (instance->state.errorCode != 0)
    {
        osMutexRelease(instance->mutex);
        return ERR_SUCCESS;
    }
    /*--------- 状态机处理 ---------*/
    switch (instance->state.motionState)
    {
        case STEER_STATE_MOVING:
            handleMovingState(instance);
            break;

        case STEER_STATE_CALIBRATING:
            handleCalibratingState(instance);
            break;

        case STEER_STATE_IDLE:
        default:
            // 无操作
            break;
    }

    /*--------- 更新最后活动时间 ---------*/
    instance->state.lastUpdateTick = HAL_GetTick();

    osMutexRelease(instance->mutex);
    return ERR_SUCCESS;
}

int SteerService_setAngleImmediate(SteerInstance* instance, const float angle)
{
    if (!instance || !instance->driver)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid instance or driver");
        return ERR_INVALID_PARAM;
    }
    osMutexAcquire(instance->mutex, osWaitForever);
    /*--------------------- 角度处理 ---------------------*/
    // 浮点角度限幅（兼容原整型配置）
    const float minAngle = (float)instance->config.minAngleDeg;
    const float maxAngle = (float)instance->config.maxAngleDeg;
    const float clampedAngle = fmaxf(fminf(angle, maxAngle), minAngle);

    // 更新状态（全浮点模型）
    instance->state.targetAngleDeg = clampedAngle;
    instance->state.actualAngleDeg = clampedAngle;

    // 立即驱动硬件
    const int err = ServoDriver_setAngle(instance->driver, clampedAngle);
    if (err != ERR_SUCCESS)
    {
        osMutexRelease(instance->mutex);
        LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_setAngle failed");
        return err;
    }

    // 清除平滑控制上下文
    memset(&instance->smoothCtrl, 0, sizeof(SmoothControlContext));

    // 更新状态
    instance->state.motionState = STEER_STATE_IDLE;
    instance->state.lastUpdateTick = HAL_GetTick();

    osMutexRelease(instance->mutex);
    return ERR_SUCCESS;
}

// 紧急停止
void SteerService_forceStop(SteerInstance* inst)
{
    if (!inst) return;

    osMutexAcquire(inst->mutex, osWaitForever);

    // 立即停止运动
    inst->smoothCtrl.remainingDeg = 0;
    inst->smoothCtrl.stepDeg = 0;
    inst->smoothCtrl.startAngleDeg = 0;
    inst->state.motionState = STEER_STATE_IDLE;

    // 同步硬件角度
    const int err = ServoDriver_emergencyStop(inst->driver);
    if (err != ERR_SUCCESS)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_emergencyStop failed");
    }

    osMutexRelease(inst->mutex);
}

// 获取当前状态快照（线程安全）
SteerState SteerService_getState(const SteerInstance* inst)
{
    SteerState state = {0};
    if (!inst) return state;

    osMutexAcquire(inst->mutex, osWaitForever);
    memcpy(&state, &inst->state, sizeof(SteerState));
    osMutexRelease(inst->mutex);

    return state;
}

/*--------------------- 运动状态处理函数 ---------------------*/
static int handleMovingState(SteerInstance* inst)
{
    SmoothControlContext* ctrl = &inst->smoothCtrl;
    const float delta = inst->state.targetAngleDeg - ctrl->startAngleDeg;
    const float totalDistance = fabsf(delta);
    const float direction = (delta >= 0) ? 1.0f : -1.0f;
    // 计算经过的时间
    uint32_t elapsed = HAL_GetTick() - inst->state.lastUpdateTick;
    const uint32_t maxElapse = inst->config.updateIntervalMs;
    if (elapsed > maxElapse)
    {
        elapsed = maxElapse; // 防止时间累积过大
    }

    // 计算移动速度
    const float speedDegPerMs = (float)ctrl->stepDeg / (float)inst->config.updateIntervalMs;
    const float movedDeg = speedDegPerMs * (float)elapsed;

    // 更新剩余距离
    float remainingDeg = totalDistance - movedDeg;
    if (remainingDeg < 0.0f) remainingDeg = 0.0f;
    ctrl->remainingDeg = remainingDeg;
    // 计算实际角度
    inst->state.actualAngleDeg = ctrl->startAngleDeg + direction * (totalDistance - remainingDeg);

    // 硬件驱动
    int err = ServoDriver_setAngle(inst->driver, inst->state.actualAngleDeg);
    if (err != ERR_SUCCESS)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_setAngle failed");
        return err;
    }

    /*--------------------- 终点判断 ---------------------*/
    if (remainingDeg <= inst->config.deadZoneDeg)
    {
        // 强制对齐目标角度（消除累积误差）
        inst->state.actualAngleDeg = inst->state.targetAngleDeg;
        inst->state.motionState = STEER_STATE_IDLE;

        // 最终位置同步
        err = ServoDriver_setAngle(inst->driver, inst->state.actualAngleDeg);
        if (err != ERR_SUCCESS)
        {
            LOG_ERROR(LOG_MODULE_SERVO, "ServoDriver_setAngle failed");
            return err;
        }
    }

    return ERR_SUCCESS;
}

static void handleCalibratingState(SteerInstance* inst)
{
    switch (inst->calibCtrl.calibPhase)
    {
        case 0: // 移动至最大角度
            ServoDriver_setAngle(inst->driver, inst->config.maxAngleDeg);
            if (checkPositionReached(inst))
            {
                inst->calibCtrl.calibPhase++;
            }
            break;

        case 1: // 记录最大值
            inst->config.maxAngleDeg = inst->state.actualAngleDeg;
            inst->calibCtrl.calibPhase++;
            break;

        case 2: // 移动至最小角度
            ServoDriver_setAngle(inst->driver, inst->config.minAngleDeg);
            if (checkPositionReached(inst))
            {
                inst->calibCtrl.calibPhase++;
            }
            break;

        case 3: // 记录最小值
            inst->config.minAngleDeg = inst->state.actualAngleDeg;
            inst->state.motionState = STEER_STATE_IDLE;
            inst->calibCtrl.calibPhase = 0;
            break;
    }
}

/*--------------------- 辅助函数 ---------------------*/
static bool checkPositionReached(const SteerInstance* inst)
{
    // 实际角度与目标角度偏差在死区范围内视为到达
    const float delta = fabsf(inst->state.targetAngleDeg - inst->state.actualAngleDeg);
    return (delta <= inst->config.deadZoneDeg);
}
