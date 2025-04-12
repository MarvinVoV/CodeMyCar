/*
 * motor_driver.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include <math.h>

#include "motor_driver.h"
#include "log_manager.h"

static float rpmToRadps(float rpm);
static float radpsToRpm(float radps);
static float radpsToMps(float radps, float wheelRadiusMM);
static float pulseToRad(int32_t pulses, float encoderPPR, float gearRatio);
static void updateRevolutions(MotorDriverState* state, int32_t deltaPulses, float encoderPPR, float gearRatio);
static float applySpeedFilter(MotorDriver* driver, float rawSpeed);

/**
 * @brief 将转速从转每分钟（RPM）转换为弧度每秒（rad/s）
 *
 * @param rpm 电机输出轴转速值，单位：转/分钟（Revolutions Per Minute）
 *           物理意义：电机输出轴（减速后）的每分钟转数
 * @return float 电机输出轴角速度值，单位：弧度/秒（rad/s）
 *
 * @note 转换公式：
 * rad/s = (RPM × 2π) / 60
 */
static float rpmToRadps(const float rpm)
{
    return (rpm * 2.0f * (float)M_PI) / 60.0f;
}

/**
 * @brief 将输出轴的角速度（rad/s）转换为输出轴的转速（RPM）
 *
 * @param radps 输出轴角速度，单位：弧度/秒（rad/s）
 * @return float 输出轴转速，单位：转/分钟（RPM）
 *
 * @note 公式：
 * RPM = (radps × 60) / (2π)
 * 示例：当 radps = 10.472 rad/s（≈ 100 RPM）时：
 * (10.472 × 60) / 6.283 ≈ 100 RPM
 */
static float radpsToRpm(const float radps)
{
    return (radps * 60.0f) / (2.0f * (float)M_PI);
}

/**
 * @brief 将角速度转换为线速度
 *
 * @param radps 角速度值，单位：弧度/秒（rad/s）
 * @param wheelRadiusMM 轮子半径，单位：毫米（mm）
 * @return float 线速度值，单位：米/秒（m/s）
 *
 * @note 转换公式：
 * mps = radps × (wheelRadiusMM / 1000.0f)
 * 物理意义：轮缘线速度 = 角速度 × 轮半径（米制）
 *
 * 示例 当radps=10 rad/s，wheelRadiusMM=65 mm时：
 * 10 × (65 / 1000) = 0.65 m/s
 */
static float radpsToMps(const float radps, const float wheelRadiusMM)
{
    return radps * (wheelRadiusMM / 1000.0f);
}

/**
 * @brief 将编码器脉冲数转换为旋转弧度值
 *
 * @param pulses 编码器脉冲数，单位：个（count）
 *              物理意义：四倍频后的累计脉冲数（包含方向信息）
 * @param encoderPPR 编码器每转脉冲数，单位：脉冲/转（Pulses Per Revolution）
 *                  物理意义：编码器未倍频时每转输出的脉冲数
 * @param gearRatio 减速比，单位：无因次量（如30:1减速比则传入30.0）
 * @return float 旋转角度值，单位：弧度（rad）
 *
 * @note 转换公式：
 * radians = (pulses × 2π) / (4 × PPR × gearRatio)
 * 四倍频说明：编码器每个脉冲周期产生4个计数（上升沿+下降沿×2通道）
 * 示例：encoderPPR=13, gearRatio=30, pulses=1560时：
 * (1560 × 6.2832) / (4 × 13 × 30) = 6.2832 rad（正好1转）
 */
static float pulseToRad(const int32_t pulses, const float encoderPPR, const float gearRatio)
{
    return ((float)pulses * (float)M_PI * 2.0f) / (4.0f * encoderPPR * gearRatio);
}

/**
 * @brief 更新累计转数值
 *
 * @param state 电机状态对象指针
 * @param deltaPulses 增量脉冲数，单位：个（count）
 * @param encoderPPR 编码器每转脉冲数，单位：脉冲/转（Pulses Per Revolution）
 * @param gearRatio 减速比，单位：无因次量
 *
 * @note 转换过程：
 * 1. 脉冲数转弧度：pulseToRad()
 * 2. 弧度转转数：radians / 2π
 * 公式：
 * revolutions += (deltaPulses × π × 2) / (4 × PPR × gearRatio) / (2π)
 * 简化为：
 * revolutions += deltaPulses / (4 × PPR × gearRatio)
 */
static void updateRevolutions(MotorDriverState* state, const int32_t deltaPulses, const float encoderPPR,
                              const float gearRatio)
{
    state->position.revolutions += (float)deltaPulses / (4.0f * encoderPPR * gearRatio);
}

static float applySpeedFilter(MotorDriver* driver, float rawSpeed)
{
    if (driver->filter.speedFilterDepth == 0)
        return rawSpeed;

    // 更新滤波缓冲区
    driver->filter.speedFilterBuf[driver->filter.filterIndex] = rawSpeed;
    driver->filter.filterIndex = (driver->filter.filterIndex + 1) % driver->filter.speedFilterDepth;

    // 计算移动平均
    float sum = 0.0f;
    for (uint8_t i = 0; i < driver->filter.speedFilterDepth; ++i)
    {
        sum += driver->filter.speedFilterBuf[i];
    }
    return sum / driver->filter.speedFilterDepth;
}

void MotorDriver_Init(MotorDriver* driver, const PID_Params* initialPid)
{
    // 参数检查
    if (!driver || !driver->hal_cfg || !driver->spec || !driver->control.pidControl)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver initialization failed");
        return;
    }
    if (driver->spec->encoderPPR == 0 || driver->spec->gearRatio == 0)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Invalid encoderPPR or gearRatio");
        return;
    }

    // 控制参数初始化
    driver->control.mode = MOTOR_DRIVER_MODE_STOP;

    // PID控制器初始化
    const PID_Params defaultPid = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.05f,
        .integral_max = 100.0f,
        .output_max = 95.0f
    };
    PIDController_Init(driver->control.pidControl, initialPid ? initialPid : &defaultPid);

    // 状态初始化
    memset(&driver->state, 0, sizeof(MotorDriverState));
    driver->state.velocity.radps = 0.0f;
    driver->state.velocity.mps = 0.0f;
    driver->state.lastUpdate = HAL_GetTick();

    /* 滤波初始化 */
    if (driver->filter.speedFilterDepth > 0 && driver->filter.speedFilterBuf != NULL)
    {
        // 清零滤波缓冲区
        memset(driver->filter.speedFilterBuf, 0, sizeof(float) * driver->filter.speedFilterDepth);
        driver->filter.filterIndex = 0;
    }
    else
    {
        // 非法配置时禁用滤波
        driver->filter.speedFilterDepth = 0;
        driver->filter.speedFilterBuf = NULL;
    }
    // 初始化HAL层
    HAL_Motor_Init(driver->hal_cfg);
}

void MotorDriver_SetMode(MotorDriver* driver, const MotorDriverMode mode)
{
    if (!driver)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    /* 模式切换处理 */
    if (driver->control.mode != mode)
    {
        /* 退出前模式清理 */
        switch (driver->control.mode)
        {
            case MOTOR_DRIVER_MODE_CLOSED_LOOP:
                PID_Reset(driver->control.pidControl); // 重置积分项和误差历史
                break;
            case MOTOR_DRIVER_MODE_OPEN_LOOP:
            case MOTOR_DRIVER_MODE_STOP:
                HAL_Motor_SetDuty(driver->hal_cfg, 0); // 释放PWM
                break;
        }
        /* 进入新模式初始化 */
        driver->control.mode = mode;
        driver->state.errorCode = 0;
    }
}

void MotorDriver_SetDutyCycle(MotorDriver* driver, const float dutyCycle)
{
    // 1. 有效性检查：确保驱动对象存在且处于开环模式
    if (!driver || driver->control.mode != MOTOR_DRIVER_MODE_OPEN_LOOP)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is not in open loop mode");
        return;
    }

    // 2. 占空比限幅
    const float targetDuty = fmaxf(fminf(dutyCycle, 100.0f), -100.0f);

    // 3. 计算并设置占空比（包含了方向）
    HAL_Motor_SetDuty(driver->hal_cfg, targetDuty);

    // 4. 更新状态
    driver->state.openLoopDuty = targetDuty;
}

void MotorDriver_SetTargetRPM(MotorDriver* driver, const float rpm)
{
    if (!driver || driver->control.mode != MOTOR_DRIVER_MODE_CLOSED_LOOP)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is not in closed loop mode");
        return;
    }

    // 转速限速保护(支持双向控制)
    const float targetRPM = fmaxf(fminf(rpm, driver->spec->maxRPM), driver->spec->maxRPM * -1.0f);

    /* 更新PID控制器的设定值 */
    driver->control.pidControl->setpoint = rpmToRadps(targetRPM);

    driver->control.targetRPM = targetRPM;
}

void MotorDriver_Update(MotorDriver* driver)
{
    if (!driver)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    const uint32_t lastUpdate = driver->state.lastUpdate;
    const uint32_t now = HAL_GetTick();
    const float deltaTime = ((float)(now - lastUpdate)) / 1000.0f; // 转换为秒


    if (deltaTime < 0.001f) return; // 最小时间间隔1ms

    /******************** 状态更新 ********************/
    // 1. 读取编码器脉冲总数（累计）
    const int32_t newPulses = HAL_Motor_ReadEncoder(driver->hal_cfg);
    const int32_t deltaPulses = newPulses - driver->state.position.totalPulses;

    // 2. 计算角速度
    const float radps = pulseToRad(deltaPulses, driver->spec->encoderPPR, driver->spec->gearRatio) / deltaTime;

    // 3. 滤波处理
    driver->state.velocity.radps = applySpeedFilter(driver, radps);

    // 4. 更新状态
    driver->state.velocity.rpm = radpsToRpm(driver->state.velocity.radps);
    driver->state.velocity.mps = radpsToMps(driver->state.velocity.radps, driver->spec->wheelRadiusMM);
    driver->state.position.totalPulses = newPulses;
    driver->state.position.revolutions += pulseToRad(deltaPulses, driver->spec->encoderPPR, driver->spec->gearRatio);
    updateRevolutions(&driver->state, deltaPulses, driver->spec->encoderPPR, driver->spec->gearRatio);
    driver->state.lastUpdate = now;


    /******************** 控制逻辑 ********************/
    switch (driver->control.mode)
    {
        case MOTOR_DRIVER_MODE_CLOSED_LOOP:
            {
                // 转换目标转速为rad/s
                const float targetRadps = rpmToRadps(driver->control.targetRPM);

                // 设置 PID 控制器的目标值(关键)
                driver->control.pidControl->setpoint = targetRadps;

                // 获取当前实际角速度测量值
                const float actualRadps = driver->state.velocity.radps;

                // 执行PID计算
                const float output = PID_Calculate(driver->control.pidControl, actualRadps, deltaTime);

                // 应用输出
                const float duty = CLAMP(output, -100.0f, 100.0f);
                HAL_Motor_SetDuty(driver->hal_cfg, duty);
                break;
            }

        case MOTOR_DRIVER_MODE_OPEN_LOOP:
            /* 开环模式无需额外处理，PWM由用户直接设置 */
            break;

        case MOTOR_DRIVER_MODE_STOP:
        default:
            HAL_Motor_SetDuty(driver->hal_cfg, 0.0f);
            break;
    }
}


void MotorDriver_EmergencyStop(MotorDriver* driver)
{
    if (!driver)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    HAL_Motor_EmergencyStop(driver->hal_cfg);
    driver->control.mode = MOTOR_DRIVER_MODE_STOP;
    // 重置 PID 积分项
    PID_Reset(driver->control.pidControl);
    driver->state.errorCode |= 0x01;
}


void MotorDriver_ConfigurePID(MotorDriver* driver, const PID_Params* params)
{
    if (!driver || !params)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver or PID params is NULL");
        return;
    }

    // 拷贝PID参数到控制器
    memcpy(&driver->control.pidControl->params, params, sizeof(PID_Params));
    // 重置积分项和微分历史状态
    PID_Reset(driver->control.pidControl);
}

MotorDriverState MotorDriver_getState(MotorDriver* driver)
{
    const MotorDriverState emptyState = {0};
    if (!driver)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return emptyState;
    }
    MotorDriverState copyState = {0};
    memcpy(&copyState, &driver->state, sizeof(MotorDriverState));
    return copyState;
}
