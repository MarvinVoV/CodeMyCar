/*
 * motion_service.c
 *
 *  Created on: Apr 6, 2025
 *      Author: marvin
 */
#include "motion_service.h"
#include "log_manager.h"
#include "error_code.h"
#include <math.h>

#define LOCK_TIMEOUT_MS 50

static void handleEmergencyStop(const MotionContext* ctx);
static void handleDirectControl(MotionContext* ctx, const MotionTarget* target);
static void handleDifferentialControl(MotionContext* ctx, const MotionTarget* target);
static void handleAckermannSteering(MotionContext* ctx, const MotionTarget* target);
static void handleSpinInPlace(MotionContext* ctx, const MotionTarget* target);
static void handleMixedSteering(MotionContext* ctx, const MotionTarget* target);
static void updateRuntimeState(MotionContext* ctx);


static float mmToMeters(float mm);
static float radiansToDegrees(float rad);
static float sign(float val);
static float clamp(float value, float min, float max);

static float calculateRpmFromVelocity(float velocity, float wheelRadiusM);

static void calculateDifferentialRPM(const ChassisConfig* config, float linearVel, float angularVel,
                                     float* leftRpm, float* rightRpm);
static float calculateAckermannAngle(float wheelbaseM, float turnRadiusM, float maxSteerAngleDeg);
static float clampAckermannSteerAngle(const ChassisConfig* config, float steerAngleDeg, float minTurnRadiusM);

#define MOTION_MODE_MAX 6
// 模式转换合法性检查表
static const uint8_t modeTransitionTable[MOTION_MODE_MAX][MOTION_MODE_MAX] = {
    /* CurrentMode / TargetMode → EMERGENCY | DIRECT | DIFFERENTIAL | STEER | SPIN | MIXED */
    /* EMERGENCY_STOP   */ {1, 0, 0, 0, 0, 0},
    /* DIRECT_CONTROL   */ {1, 1, 0, 0, 0, 0},
    /* DIFFERENTIAL     */ {1, 0, 1, 1, 1, 1},
    /* STEER_ONLY       */ {1, 0, 1, 1, 0, 1},
    /* SPIN_IN_PLACE    */ {1, 0, 1, 0, 1, 0},
    /* MIXED_STEER      */ {1, 0, 1, 1, 0, 1}
};

// 互斥锁包装宏
#define LOCK(ctx)   osMutexAcquire((ctx)->targetMutex, osWaitForever)
#define UNLOCK(ctx) osMutexRelease((ctx)->targetMutex)

int MotionContext_Init(MotionContext* ctx, MotorService* motor, SteerInstance* steer, const ChassisConfig* config)
{
    if (!ctx || !motor || !steer || !config)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionContext_Init: Invalid parameter!");
        return -1;
    }
    // 清零整个结构体（关键！避免union残留值）
    memset(ctx, 0, sizeof(MotionContext));

    // 初始化硬件服务实例
    ctx->motorService = motor;
    ctx->steerServo = steer;

    // 深拷贝底盘配置
    memcpy(&ctx->chassisConfig, config, sizeof(ChassisConfig));

    // 初始化目标参数
    ctx->motionTarget.mode = MOTION_EMERGENCY_STOP;
    memset(&ctx->motionTarget.params, 0, sizeof(ctx->motionTarget.params));

    // 初始化状态
    ctx->state = (MotionState){
        .currentMode = MOTION_EMERGENCY_STOP,
        .timestamp = HAL_GetTick(), // 获取系统启动时间
        .drive = {
            .leftRpm = 0,
            .rightRpm = 0,
            .leftErrorCode = ERR_SUCCESS,
            .rightErrorCode = ERR_SUCCESS
        },
        .steering = {
            .errorCode = ERR_SUCCESS,
            .targetAngle = 0,
            .actualAngle = 0
        },
        .velocity = {
            .linear = 0.0f,
            .angular = 0.0f,
            .acceleration = 0.0f
        },
        .pose = {
            .x = 0.0f,
            .y = 0.0f,
            .theta = 0.0f,
            .leftOdometer = 0.0f,
            .rightOdometer = 0.0f
        },
        .lastError = ERR_SUCCESS
    };

    // 创建互斥锁
    ctx->targetMutex = osMutexNew(NULL);
    if (!ctx->targetMutex)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "Mutex create failed!");
        return -2;
    }
    return 0;
}

int MotionService_SetControlMode(MotionContext* ctx, MotionMode newMode)
{
    if (!ctx || newMode >= MOTION_MODE_MAX)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetControlMode: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    const MotionMode currentMode = ctx->motionTarget.mode;

    // 检查模式转换合法性
    if (!modeTransitionTable[currentMode][newMode])
    {
        ctx->state.lastError = MOTION_ERR_MODE_TRANSITION;
        UNLOCK(ctx);
        return MOTION_ERR_MODE_TRANSITION;
    }

    // 特殊条件检查（例如急停未解除）
    if (currentMode == MOTION_EMERGENCY_STOP && newMode != MOTION_EMERGENCY_STOP)
    {
        if (ctx->state.lastError != ERR_SUCCESS)
        {
            UNLOCK(ctx);
            return MOTION_ERR_EMERGENCY_ACTIVE;
        }
    }

    if (ctx->motionTarget.mode != newMode)
    {
        // 模式切换时重置旧参数
        memset(&ctx->motionTarget.params, 0, sizeof(ctx->motionTarget.params));
        // 执行模式切换
        ctx->motionTarget.mode = newMode;
        if (newMode == MOTION_DIFFERENTIAL)
        {
            ctx->motionTarget.params.velocityCtrl = (typeof(ctx->motionTarget.params.velocityCtrl)){0};
        }
        else if (newMode == MOTION_MIXED_STEER)
        {
            ctx->motionTarget.params.mixedCtrl = (typeof(ctx->motionTarget.params.mixedCtrl)){0};
        }
        else if (newMode == MOTION_SPIN_IN_PLACE)
        {
            ctx->motionTarget.params.velocityCtrl.linearVelocity = 0.0f;
        }
    }

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

/**
 * @brief 获取当前运动控制模式（线程安全）
 * @param ctx 运动控制上下文指针
 * @return MotionMode 当前控制模式
 */
MotionMode MotionService_GetCurrentMode(const MotionContext* ctx)
{
    if (!ctx)
    {
        return MOTION_EMERGENCY_STOP; // 安全默认值
    }

    // 加锁读取（即使基本类型也保证原子性）
    // LOCK(ctx);
    const MotionMode currentMode = ctx->motionTarget.mode;
    // UNLOCK(ctx);

    return currentMode;
}

// 差速模式
int MotionService_SetVelocity(MotionContext* ctx, float linear, float angular)
{
    if (!ctx || !isfinite(linear) || !isfinite(angular))
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetVelocity: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 检查当前模式兼容性
    if (ctx->motionTarget.mode != MOTION_DIFFERENTIAL && ctx->motionTarget.mode != MOTION_MIXED_STEER)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        UNLOCK(ctx);
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetVelocity: Invalid mode!");
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 参数范围校验
    const float maxLinear = ctx->chassisConfig.maxLinearVelocityMPS;
    if (fabsf(linear) > maxLinear)
    {
        linear = copysignf(maxLinear, linear);
        ctx->state.lastError = MOTION_ERR_PARAM_CLAMPED;
    }

    // 写入目标参数
    if (ctx->motionTarget.mode == MOTION_DIFFERENTIAL)
    {
        ctx->motionTarget.params.velocityCtrl.linearVelocity = linear;
        ctx->motionTarget.params.velocityCtrl.angularVelocity = angular;
        ctx->motionTarget.params.velocityCtrl.clockwise = (angular >= 0);
    }
    else
    {
        // MIXED_STEER
        ctx->motionTarget.params.mixedCtrl.linearVelocity = linear;
        ctx->motionTarget.params.mixedCtrl.angularVelocity = angular;
    }

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

int MotionService_SetDirectControl(MotionContext* ctx, float leftRpm, float rightRpm, float steerAngle)
{
    if (!ctx || !isfinite(leftRpm) || !isfinite(rightRpm))
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetDirectControl: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    if (ctx->motionTarget.mode != MOTION_DIRECT_CONTROL)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        UNLOCK(ctx);
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetDirectControl: Invalid mode!");
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 写入直接控制参数
    ctx->motionTarget.params.directCtrl.leftRpm = leftRpm;
    ctx->motionTarget.params.directCtrl.rightRpm = rightRpm;
    ctx->motionTarget.params.directCtrl.steerAngle = steerAngle;

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

// 阿克曼转向模式
int MotionService_SetAckermannParams(MotionContext* ctx, float linearVel, float steerAngle)
{
    if (!ctx || !isfinite(linearVel) || !isfinite(steerAngle))
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetAckermannParams: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 模式兼容性检查
    if (ctx->motionTarget.mode != MOTION_STEER_ONLY && ctx->motionTarget.mode != MOTION_MIXED_STEER)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        UNLOCK(ctx);
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetAckermannParams: Invalid mode!");
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 参数范围校验
    const float maxSteer = ctx->chassisConfig.maxSteerAngleDeg;
    const float maxVel = ctx->chassisConfig.maxLinearVelocityMPS;

    // 钳位处理
    float clampedVel = CLAMP(linearVel, -maxVel, maxVel);
    float clampedAngle = CLAMP(steerAngle, -maxSteer, maxSteer);

    // 设置参数
    if (ctx->motionTarget.mode == MOTION_STEER_ONLY)
    {
        ctx->motionTarget.params.ackermannCtrl.linearVelocity = clampedVel;
        ctx->motionTarget.params.ackermannCtrl.steerAngle = clampedAngle;
    }
    else
    {
        // MIXED_STEER
        ctx->motionTarget.params.mixedCtrl.linearVelocity = clampedVel;
        ctx->motionTarget.params.mixedCtrl.steerAngle = clampedAngle;
    }

    // 记录参数调整事件
    if (fabsf(clampedVel - linearVel) > 0.001f || fabsf(clampedAngle - steerAngle) > 0.1f)
    {
        ctx->state.lastError = MOTION_ERR_PARAM_CLAMPED;
    }

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

int MotionService_SetHybridParams(MotionContext* ctx, float linear, float angular, float steerAngle,
                                  float steeringBlend)
{
    // 参数有效性检查
    if (!ctx || !isfinite(linear) || !isfinite(angular) ||
        !isfinite(steerAngle) || !isfinite(steeringBlend))
    {
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 模式兼容性检查
    if (ctx->motionTarget.mode != MOTION_MIXED_STEER)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        UNLOCK(ctx);
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 获取配置参数
    const ChassisConfig* cfg = &ctx->chassisConfig;
    uint8_t clampFlag = 0;

    // 参数钳位处理
    const float clampedLinear = CLAMP(linear, -cfg->maxLinearVelocityMPS, cfg->maxLinearVelocityMPS);
    const float clampedAngular = CLAMP(angular, -cfg->maxAngularVelRad, cfg->maxAngularVelRad);
    const float clampedSteer = CLAMP(steerAngle, -cfg->maxSteerAngleDeg, cfg->maxSteerAngleDeg);
    const float clampedBlend = CLAMP(steeringBlend, 0.0f, 1.0f);

    // 记录参数调整事件
    if (clampedLinear != linear) clampFlag |= 0x01;
    if (clampedAngular != angular) clampFlag |= 0x02;
    if (clampedSteer != steerAngle) clampFlag |= 0x04;
    if (clampedBlend != steeringBlend) clampFlag |= 0x08;
    if (clampFlag) ctx->state.lastError = MOTION_ERR_PARAM_CLAMPED;

    // 写入混合控制参数
    ctx->motionTarget.params.mixedCtrl.linearVelocity = clampedLinear;
    ctx->motionTarget.params.mixedCtrl.angularVelocity = clampedAngular;
    ctx->motionTarget.params.mixedCtrl.steerAngle = clampedSteer;
    ctx->motionTarget.params.mixedCtrl.differentialGain = clampedBlend;

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

void MotionService_GetState(const MotionContext* ctx, MotionState* outState)
{
    if (!ctx || !outState)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_GetState: Invalid parameter!");
        return;
    }

    LOCK(ctx);
    memcpy(outState, &ctx->state, sizeof(MotionState));
    UNLOCK(ctx);
}

/**
 * @brief 设置转向角度（线程安全） （STEER_ONLY/MIXED模式）
 * @param ctx 运动控制上下文指针
 * @param angleDeg 转向角度（-max_angle ~ +max_angle）
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 无效参数
 *         MOTION_ERR_MODE_MISMATCH - 当前模式不支持转向控制
 */
int MotionService_SetSteerAngle(MotionContext* ctx, float angleDeg)
{
    if (!ctx || !isfinite(angleDeg))
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetSteerAngle: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 检查模式兼容性
    const MotionMode mode = ctx->motionTarget.mode;
    if (mode != MOTION_STEER_ONLY && mode != MOTION_MIXED_STEER)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        UNLOCK(ctx);
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 获取配置参数
    const float maxAngle = ctx->chassisConfig.maxSteerAngleDeg;
    const float clampedAngle = CLAMP(angleDeg, -maxAngle, maxAngle);

    // 记录参数调整事件
    if (fabsf(clampedAngle - angleDeg) > 0.1f)
    {
        ctx->state.lastError = MOTION_ERR_PARAM_CLAMPED;
    }

    // 写入目标参数
    if (mode == MOTION_STEER_ONLY)
    {
        ctx->motionTarget.params.ackermannCtrl.steerAngle = clampedAngle;
    }
    else
    {
        // MIXED_STEER
        ctx->motionTarget.params.mixedCtrl.steerAngle = clampedAngle;
    }

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

int MotionService_SetSpinParams(MotionContext* ctx, float angularVel)
{
    // 参数有效性检查
    if (!ctx || !isfinite(angularVel))
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_SetSpinParams: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    osMutexAcquire(ctx->targetMutex, osWaitForever);

    // 模式兼容性检查
    if (ctx->motionTarget.mode != MOTION_SPIN_IN_PLACE)
    {
        ctx->state.lastError = MOTION_ERR_MODE_MISMATCH;
        osMutexRelease(ctx->targetMutex);
        return MOTION_ERR_MODE_MISMATCH;
    }

    // 获取配置参数
    const ChassisConfig* cfg = &ctx->chassisConfig;
    const float maxAngular = cfg->maxAngularVelRad;
    const float clampedAngular = CLAMP(angularVel, -maxAngular, maxAngular);

    // 记录参数调整事件
    if (clampedAngular != angularVel)
    {
        ctx->state.lastError = MOTION_ERR_PARAM_CLAMPED;
    }

    // 写入旋转参数（线速度强制置零）
    ctx->motionTarget.params.velocityCtrl.linearVelocity = 0.0f;
    ctx->motionTarget.params.velocityCtrl.angularVelocity = clampedAngular;
    ctx->motionTarget.params.velocityCtrl.clockwise = (clampedAngular >= 0);

    osMutexRelease(ctx->targetMutex);
    return ERR_SUCCESS;
}

int MotionService_EmergencyStop(MotionContext* ctx)
{
    if (!ctx)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_EmergencyStop: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 强制切换模式
    ctx->motionTarget.mode = MOTION_EMERGENCY_STOP;
    memset(&ctx->motionTarget.params, 0, sizeof(ctx->motionTarget.params));

    // 执行急停
    MotorService_emergencyStop(ctx->motorService, MOTOR_LEFT_WHEEL);
    MotorService_emergencyStop(ctx->motorService, MOTOR_RIGHT_WHEEL);

    // 更新状态
    ctx->state.lastError = MOTION_ERR_EMERGENCY_ACTIVE;
    ctx->state.currentMode = MOTION_EMERGENCY_STOP;

    UNLOCK(ctx);
    return ERR_SUCCESS;
}

/**
 * @brief 清除故障状态（线程安全）
 * @param ctx 运动控制上下文指针
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_CLEAR_FAILED - 仍有未恢复故障
 */
int MotionService_ClearFaults(MotionContext* ctx)
{
    if (!ctx)
    {
        LOG_ERROR(LOG_MODULE_MOTION, "MotionService_ClearFaults: Invalid parameter!");
        return MOTION_ERR_INVALID_PARAM;
    }

    LOCK(ctx);

    // 仅在急停模式允许清除故障
    if (ctx->motionTarget.mode != MOTION_EMERGENCY_STOP)
    {
        UNLOCK(ctx);
        return MOTION_ERR_CLEAR_FAILED;
    }

    // 检查所有故障是否已恢复
    const bool isFaultClear = (ctx->motorService->instances[MOTOR_LEFT_WHEEL].state.errorCode == 0 &&
        ctx->motorService-> instances[MOTOR_RIGHT_WHEEL].state.errorCode == 0);

    if (isFaultClear)
    {
        // 重置错误状态
        ctx->state.lastError = ERR_SUCCESS;

        // 恢复至安全模式（默认为空闲）
        ctx->motionTarget.mode = MOTION_DIFFERENTIAL;
        memset(&ctx->motionTarget.params, 0, sizeof(ctx->motionTarget.params));
    }
    else
    {
        ctx->state.lastError = MOTION_ERR_CLEAR_FAILED;
    }

    UNLOCK(ctx);
    return isFaultClear ? ERR_SUCCESS : MOTION_ERR_CLEAR_FAILED;
}


void MotionService_Run(MotionContext* ctx)
{
    // 参数安全检查
    if (ctx == NULL || ctx->motorService == NULL || ctx->steerServo == NULL)
    {
        return;
    }

    // 互斥锁获取目标参数
    MotionTarget currentTarget;
    if (osMutexAcquire(ctx->targetMutex, LOCK_TIMEOUT_MS) == osOK)
    {
        // 拷贝整个结构体（确保内存布局一致）
        memcpy(&currentTarget, &ctx->motionTarget, sizeof(MotionTarget));
        osMutexRelease(ctx->targetMutex);
    }
    else
    {
        ctx->state.lastError = ERR_MUTEX_TIMEOUT;
        return;
    }

    // 根据模式执行控制
    switch (currentTarget.mode)
    {
        case MOTION_EMERGENCY_STOP:
            handleEmergencyStop(ctx);
            break;

        case MOTION_DIRECT_CONTROL:
            handleDirectControl(ctx, &currentTarget);
            break;

        case MOTION_DIFFERENTIAL:
            handleDifferentialControl(ctx, &currentTarget);
            break;

        case MOTION_STEER_ONLY:
            handleAckermannSteering(ctx, &currentTarget);
            break;

        case MOTION_SPIN_IN_PLACE:
            handleSpinInPlace(ctx, &currentTarget);
            break;

        case MOTION_MIXED_STEER:
            handleMixedSteering(ctx, &currentTarget);
            break;
    }

    // 更新运行时状态
    updateRuntimeState(ctx);
}

/*--------------------- 模式处理子函数 ---------------------*/
static void handleEmergencyStop(const MotionContext* ctx)
{
    // 停止所有电机
    MotorService_emergencyStop(ctx->motorService, MOTOR_LEFT_WHEEL);
    MotorService_emergencyStop(ctx->motorService, MOTOR_RIGHT_WHEEL);
    // 快速归中
    SteerService_setAngleImmediate(ctx->steerServo, 90);
}

static void handleDirectControl(MotionContext* ctx, const MotionTarget* target)
{
    // 设置直接转速
    MotorService_setTargetRPM(ctx->motorService, MOTOR_LEFT_WHEEL,
                              target->params.directCtrl.leftRpm);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_RIGHT_WHEEL,
                              target->params.directCtrl.rightRpm);

    // 舵机角度
    SteerService_setAngleSmoothly(ctx->steerServo, (float)target->params.directCtrl.steerAngle, 200);
}

static void handleDifferentialControl(MotionContext* ctx, const MotionTarget* target)
{
    const ChassisConfig* config = &ctx->chassisConfig;

    // 差速模型计算
    const float linearVel = target->params.velocityCtrl.linearVelocity;
    const float angularVel = target->params.velocityCtrl.angularVelocity;

    // 计算差速RPM
    float leftRpm, rightRpm;
    calculateDifferentialRPM(config, linearVel, angularVel, &leftRpm, &rightRpm);


    // 执行控制
    MotorService_setTargetRPM(ctx->motorService, MOTOR_LEFT_WHEEL, leftRpm);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_RIGHT_WHEEL, rightRpm);
    // 舵机归中
    SteerService_setAngleImmediate(ctx->steerServo, 90);
}

static void handleAckermannSteering(MotionContext* ctx, const MotionTarget* target)
{
    const ChassisConfig* config = &ctx->chassisConfig;
    const float linearVel = target->params.ackermannCtrl.linearVelocity;
    const float steerAngleDeg = target->params.ackermannCtrl.steerAngle;

    // 计算限幅后的转向角
    const float clampedAngle = clampAckermannSteerAngle(config, steerAngleDeg, 0.5f); // 假设最小半径 0.5m

    // 计算基准转速
    const float wheelRadiusM = mmToMeters(config->wheelRadiusMM);
    const float baseRpm = calculateRpmFromVelocity(linearVel, wheelRadiusM);

    // 执行控制
    SteerService_setAngleSmoothly(ctx->steerServo, clampedAngle, 200);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_LEFT_WHEEL, baseRpm);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_RIGHT_WHEEL, baseRpm);
}

static void handleSpinInPlace(MotionContext* ctx, const MotionTarget* target)
{
    const ChassisConfig* config = &ctx->chassisConfig;
    const float trackWidthM = (float)((double)config->trackWidthMM / 1000.0f);
    const float wheelRadiusM = (float)((double)config->wheelRadiusMM / 1000.0f);

    // 计算差速转速
    const float direction = target->params.velocityCtrl.clockwise ? 1.0f : -1.0f;
    const float rpm = (target->params.velocityCtrl.angularVelocity * trackWidthM / 2.0f) * 60.0f
        / (2.0f * (float)M_PI * wheelRadiusM);

    // 执行差速控制
    MotorService_setTargetRPM(ctx->motorService, MOTOR_LEFT_WHEEL, -direction * rpm);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_RIGHT_WHEEL, direction * rpm);
    // 舵机归中
    SteerService_setAngleImmediate(ctx->steerServo, 90);
}

static void handleMixedSteering(MotionContext* ctx, const MotionTarget* target)
{
    const ChassisConfig* config = &ctx->chassisConfig;

    const float wheelbaseM = mmToMeters(config->wheelbaseMM); // 轴距（米）
    const float linearVel = target->params.velocityCtrl.linearVelocity;
    const float angularVel = clamp(target->params.velocityCtrl.angularVelocity,
                                   -config->maxAngularVelRad, config->maxAngularVelRad);
    const float blendFactor = clamp(target->params.mixedCtrl.differentialGain, 0.0f, 1.0f);
    const float manualSteer = (float)target->params.directCtrl.steerAngle;

    // 1.  阿克曼转向角计算
    float ackermannAngle = 0.0f;
    if (fabsf(linearVel) > 0.001f && fabsf(angularVel) > 0.001f)
    {
        const float turnRadius = linearVel / angularVel;
        ackermannAngle = calculateAckermannAngle(wheelbaseM, turnRadius, config->maxSteerAngleDeg);
    }
    // 2.  混合转向处理
    const float blendedSteer = ackermannAngle * (1.0f - blendFactor) + manualSteer * blendFactor;
    const float finalSteer = clamp(blendedSteer, -config->maxSteerAngleDeg, config->maxSteerAngleDeg);

    // 3.  差速计算
    float leftRpm, rightRpm;
    calculateDifferentialRPM(config, linearVel, angularVel * blendFactor, &leftRpm, &rightRpm);

    // 4. 执行控制
    SteerService_setAngleSmoothly(ctx->steerServo, finalSteer, 200);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_LEFT_WHEEL, leftRpm);
    MotorService_setTargetRPM(ctx->motorService, MOTOR_RIGHT_WHEEL, rightRpm);

    // 5. 状态更新

    // 驱动系统状态
    ctx->state.drive.leftRpm = leftRpm;
    ctx->state.drive.rightRpm = rightRpm;

    // 转向系统状态
    ctx->state.steering.targetAngle = finalSteer;

    // 运动学状态
    ctx->state.velocity.linear = linearVel;
    ctx->state.velocity.angular = angularVel;

    // 时间戳
    ctx->state.timestamp = HAL_GetTick();
}

/*--------------------- 状态更新函数 ---------------------*/

static void updateRuntimeState(MotionContext* ctx)
{
    // 获取电机实例
    MotorInstance* leftMotor = &ctx->motorService->instances[MOTOR_LEFT_WHEEL];
    MotorInstance* rightMotor = &ctx->motorService->instances[MOTOR_RIGHT_WHEEL];

    SteerState steerState = SteerService_getState(ctx->steerServo);

    /*========== 驱动系统状态更新 ==========*/
    // 左电机状态获取（带锁）
    MotorState leftState;
    osMutexAcquire(leftMotor->mutex, osWaitForever);
    memcpy(&leftState, &leftMotor->state, sizeof(MotorState));
    osMutexRelease(leftMotor->mutex);

    // 右电机状态获取（带锁）
    MotorState rightState;
    osMutexAcquire(rightMotor->mutex, osWaitForever);
    memcpy(&rightState, &rightMotor->state, sizeof(MotorState));
    osMutexRelease(rightMotor->mutex);

    // 写入驱动系统状态
    ctx->state.drive.leftRpm = leftState.velocity.rpm;
    ctx->state.drive.rightRpm = rightState.velocity.rpm;
    ctx->state.drive.leftErrorCode = leftState.errorCode;
    ctx->state.drive.rightErrorCode = rightState.errorCode;

    /*========== 运动学状态计算 ==========*/
    const float trackWidthM = mmToMeters(ctx->chassisConfig.trackWidthMM);
    const float wheelRadiusM = mmToMeters(ctx->chassisConfig.wheelRadiusMM);

    // 实际线速度 = 平均轮速
    ctx->state.velocity.linear =
        (leftState.velocity.mps + rightState.velocity.mps) / 2.0f;

    // 实际角速度 = (右轮速 - 左轮速) / 轮距
    if (trackWidthM > 0.001f)
    {
        // 避免除零
        ctx->state.velocity.angular =
            (rightState.velocity.mps - leftState.velocity.mps) / trackWidthM;
    }

    /*========== 定位状态更新 ==========*/
    // 计算轮周长
    const float wheelCircumference = 2.0f * M_PI * wheelRadiusM;

    // 计算本次采样周期内的转数变化量
    static float lastLeftRevs = 0, lastRightRevs = 0; // 需持久化
    const float deltaLeftRevs = leftState.position.revolutions - lastLeftRevs;
    const float deltaRightRevs = rightState.position.revolutions - lastRightRevs;
    lastLeftRevs = leftState.position.revolutions;
    lastRightRevs = rightState.position.revolutions;

    // 转换为实际行程（米）
    const float deltaLeftDist = deltaLeftRevs * wheelCircumference;
    const float deltaRightDist = deltaRightRevs * wheelCircumference;

    // 更新累积里程
    ctx->state.pose.leftOdometer += deltaLeftDist;
    ctx->state.pose.rightOdometer += deltaRightDist;

    // 航向角变化量 = (右轮行程 - 左轮行程) / 轮距
    if (trackWidthM > 0.001f)
    {
        const float deltaTheta =
            (deltaRightDist - deltaLeftDist) / trackWidthM;
        ctx->state.pose.theta += deltaTheta;
    }

    // 坐标更新（航迹推算）
    const float avgDelta = (deltaLeftDist + deltaRightDist) / 2.0f;
    ctx->state.pose.x += avgDelta * cos(ctx->state.pose.theta);
    ctx->state.pose.y += avgDelta * sin(ctx->state.pose.theta);

    /*========== 转向系统状态 ==========*/
    ctx->state.steering.actualAngle = steerState.actualAngleDeg;
    ctx->state.steering.targetAngle = steerState.targetAngleDeg;
    ctx->state.steering.errorCode = steerState.errorCode;

    /*========== 错误状态更新 ==========*/
    // todo

    /*========== 时间戳更新 ==========*/
    ctx->state.timestamp = HAL_GetTick();
}


// 毫米 → 米
static float mmToMeters(const float mm)
{
    return (float)((double)mm / 1000.0); // 双精度中间计算
}

// 弧度 → 度
static float radiansToDegrees(const float rad)
{
    return rad * (180.0f / (float)M_PI);
}

/* 速度转RPM计算 */
static float calculateRpmFromVelocity(const float velocity, const float wheelRadiusM)
{
    return (velocity * 60.0f) / (2.0f * (float)M_PI * wheelRadiusM);
}

// 差速模型核心计算（单位：m/s → RPM）
static void calculateDifferentialRPM(const ChassisConfig* config, const float linearVel, const float angularVel,
                                     float* leftRpm, float* rightRpm)
{
    const float trackWidthM = mmToMeters(config->trackWidthMM);
    const float wheelRadiusM = mmToMeters(config->wheelRadiusMM);
    const float factor = 60.0f / (2.0f * (float)M_PI * wheelRadiusM);
    *leftRpm = (linearVel - angularVel * trackWidthM / 2.0f) * factor;
    *rightRpm = (linearVel + angularVel * trackWidthM / 2.0f) * factor;
}

// 转向角限幅（基于最小转弯半径）
static float clampAckermannSteerAngle(const ChassisConfig* config, const float steerAngleDeg,
                                      const float minTurnRadiusM)
{
    const float maxSteerDeg = radiansToDegrees(
        atanf(mmToMeters(config->wheelbaseMM) / minTurnRadiusM)
    );
    return fmaxf(fminf(steerAngleDeg, maxSteerDeg), -maxSteerDeg);
}

/**
 * @brief 计算阿克曼几何模型的标准转向角（带安全约束）
 * @param wheelbaseM 轴距（米）
 * @param turnRadiusM 目标转弯半径（米）
 * @param maxSteerAngleDeg 机械允许最大转向角（度）
 * @return 有效转向角度数（-max ~ +max）
 *
 * @note 当turnRadiusM趋近0时返回最大转向角
 *       输入异常时返回NAN
 */
float calculateAckermannAngle(const float wheelbaseM, const float turnRadiusM, const float maxSteerAngleDeg)
{
    // 参数有效性检查
    if (wheelbaseM <= 0.0f || isnan(turnRadiusM) || maxSteerAngleDeg <= 0.0f)
    {
        return NAN;
    }

    // 处理直行和原地转向情况
    if (fabsf(turnRadiusM) < 0.001f)
    {
        return copysignf(maxSteerAngleDeg, turnRadiusM);
    }

    // 计算理论转向角（使用双精度提升计算精度）
    const double radiusRatio = (double)wheelbaseM / fabs((double)turnRadiusM);
    const double steerAngleRad = atan(radiusRatio);

    // 转换为角度并保留符号
    const float angleDeg = (float)(steerAngleRad * 180.0 / M_PI) * sign(turnRadiusM);

    // 应用机械约束
    return clamp(angleDeg, -maxSteerAngleDeg, maxSteerAngleDeg);
}

// 符号函数
static float sign(const float val)
{
    return (val >= 0.0f) ? 1.0f : -1.0f;
}

// 安全限幅函数
static float clamp(const float value, const float min, const float max)
{
    return fmaxf(fminf(value, max), min);
}
