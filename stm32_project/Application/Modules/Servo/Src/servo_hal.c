/*
 * servo_drive.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_hal.h"
#include "log_manager.h"


static HAL_ServoConfig config;

void ServoHAL_init(HAL_ServoConfig* hardwareConfig)
{
    if (hardwareConfig == NULL || hardwareConfig->pwmTim == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid servo config");
        return;
    }
    // 验证定时器配置
    if (hardwareConfig->minPulse >= hardwareConfig->maxPulse)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid pulse range");
        return;
    }

    // 安全脉冲配置
    if (hardwareConfig->safetyPulse == 0)
    {
        hardwareConfig->safetyPulse = (hardwareConfig->minPulse + hardwareConfig->maxPulse) / 2;
    }


    // 启动PWM并验证
    if (HAL_TIM_PWM_Start(hardwareConfig->pwmTim, hardwareConfig->channel) != HAL_OK)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "PWM start failed");
        return;
    }
    memcpy(&config, hardwareConfig, sizeof(HAL_ServoConfig));
}

bool ServoHAL_setAngle(const int angle)
{
    const int32_t pulseRange = config.maxPulse - config.minPulse;
    const int targetAngle = CLAMP(angle, config.minAngle, config.maxAngle);

    // 预计算缩放因子避免重复计算
    const int32_t angleScale = config.maxAngle - config.minAngle;
    if (angleScale == 0)
    {
        return false;
    }

    // 将角度转换为脉冲宽度 ,计算CCR值
    uint16_t pulseWidth = config.minPulse +
        ((pulseRange * (targetAngle - config.minAngle) + angleScale / 2) / angleScale);

    // LOG_INFO(LOG_MODULE_SERVO, "Setting angle=%d, pulse=%uus", angle, pulse_width);

    // 验证脉冲范围
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(config.pwmTim);
    if (pulseWidth > arr)
    {
        LOG_WARN(LOG_MODULE_SERVO, "Pulse exceeds ARR");
        pulseWidth = arr;
    }

    // 原子操作设置比较值
    __HAL_TIM_SET_COMPARE(config.pwmTim, config.channel, pulseWidth);
    return true;
}

void ServoHAL_emergencyStop()
{
    __HAL_TIM_SET_COMPARE(config.pwmTim, config.channel, config.safetyPulse);
}
