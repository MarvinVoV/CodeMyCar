/*
 * servo_drive.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#include "servo_hal.h"
#include "log_manager.h"


static servo_hw_config serv_config; // 静态存储配置副本

void servo_hal_init(servo_hw_config* config)
{
    if (config == NULL || config->pwm_tim == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid servo config");
        return;
    }
    // 验证定时器配置
    if (config->min_pulse >= config->max_pulse)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Invalid pulse range");
        return;
    }

    // 安全脉冲配置
    if (config->safety_pulse == 0)
    {
        config->safety_pulse = (config->min_pulse + config->max_pulse) / 2;
    }


    // 启动PWM并验证
    if (HAL_TIM_PWM_Start(config->pwm_tim, config->channel) != HAL_OK)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "PWM start failed");
        return;
    }
    memcpy(&serv_config, config, sizeof(servo_hw_config)); // 拷贝配置
}

bool servo_hal_set_angle(int angle)
{
    const int32_t pulse_range = serv_config.max_pulse - serv_config.min_pulse;
    angle = CLAMP(angle, serv_config.min_angle, serv_config.max_angle);

    // 预计算缩放因子避免重复计算
    const int32_t angle_scale = serv_config.max_angle - serv_config.min_angle;
    if (angle_scale == 0)
    {
        return false;
    }

    // 将角度转换为脉冲宽度 ,计算CCR值
    uint16_t pulse_width = serv_config.min_pulse +
        ((pulse_range * (angle - serv_config.min_angle) + angle_scale / 2) / angle_scale);

    // 验证脉冲范围
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(serv_config.pwm_tim);
    if (pulse_width > arr)
    {
        LOG_WARN(LOG_MODULE_SERVO, "Pulse exceeds ARR");
        pulse_width = arr;
    }

    // 原子操作设置比较值
    __HAL_TIM_SET_COMPARE(serv_config.pwm_tim, serv_config.channel, pulse_width);
    return true;
}

void servo_hal_emergency_stop()
{
    __HAL_TIM_SET_COMPARE(serv_config.pwm_tim, serv_config.channel, serv_config.safety_pulse);
}
