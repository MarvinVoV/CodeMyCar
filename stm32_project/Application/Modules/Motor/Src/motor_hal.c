/*
 * motor_hal.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include <string.h>
#include "motor_hal.h"

#include "cmd_process.h"
#include "log_manager.h"


/**
 * 相关配置已经通过 CubeMX 完成配置，这里只需要实现启动逻辑即可
 */
void HAL_Motor_Init(HAL_MotorConfig* config)
{
    // 参数校验
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return;
    }
    // 初始化PWM
    HAL_TIM_PWM_Start(config->pwm.tim, config->pwm.ch);

    // 初始化编码器
    HAL_TIM_Encoder_Start(config->encoder.tim, TIM_CHANNEL_ALL);

    // 初始化GPIO默认状态
    HAL_GPIO_WritePin(config->gpio.in1Port, config->gpio.in1Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->gpio.in2Port, config->gpio.in2Pin, GPIO_PIN_RESET);

    // 初始化状态
    memset(&config->state, 0, sizeof(HAL_MotorState));
}

/**
 * 初始化编码器定时器
 * 注: 相关配置已经通过 CubeMX 完成配置，这里只需要实现启动逻辑即可
 * @param htim htim
 */
void HAL_Motor_InitEncoder(TIM_HandleTypeDef* htim)
{
    // 启动编码器接口
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0); // 初始计数器归零
}

void HAL_Motor_SetDirection(HAL_MotorConfig* config, const MotorDirection direction)
{
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return;
    }

    // 更新方向状态
    config->state.direction = direction;
    switch (direction)
    {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(config->gpio.in1Port, config->gpio.in1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->gpio.in2Port, config->gpio.in2Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_DIR_BACKWARD:
            HAL_GPIO_WritePin(config->gpio.in1Port, config->gpio.in1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->gpio.in2Port, config->gpio.in2Pin, GPIO_PIN_SET);
            break;
        case MOTOR_COAST_STOP:
            HAL_GPIO_WritePin(config->gpio.in1Port, config->gpio.in1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->gpio.in2Port, config->gpio.in2Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BRAKE_STOP:
            HAL_GPIO_WritePin(config->gpio.in1Port, config->gpio.in1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->gpio.in2Port, config->gpio.in2Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

void HAL_Motor_EmergencyStop(HAL_MotorConfig* config)
{
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return;
    }

    // 立即关闭PWM输出
    HAL_Motor_SetDuty(config, 0);

    // 激活硬件刹车模式
    HAL_Motor_SetDirection(config, MOTOR_BRAKE_STOP);

    // 复位编码器
    __HAL_TIM_SET_COUNTER(config->encoder.tim, 0);

    // 设置急停标志位
    config->state.errorFlags |= 0x01;
}

void HAL_Motor_SetDuty(HAL_MotorConfig* config, const float duty)
{
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return;
    }

    // 占空比限幅
    const int32_t targetDuty = CLAMP(duty, -100, 100);

    // 获取定时器的 ARR（PWM 周期）
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(config->pwm.tim);

    // 计算 CCR 值（四舍五入优化）
    const uint32_t ccr = (uint32_t)(fabs(targetDuty) * arr + 50.0) / 100;

    // 确保 CCR 不超过 ARR
    const uint16_t safeCCR = (ccr > arr) ? arr : ccr;

    // 设置方向
    MotorDirection direction;
    if (targetDuty == 0)
    {
        direction = MOTOR_COAST_STOP;
    }
    else if (targetDuty > 0)
    {
        direction = MOTOR_DIR_FORWARD;
    }
    else
    {
        direction = MOTOR_DIR_BACKWARD;
    }
    HAL_Motor_SetDirection(config, direction);

    // 更新 PWM 比较值
    __HAL_TIM_SET_COMPARE(config->pwm.tim, config->pwm.ch, safeCCR);

    // 更新状态
    config->state.currentDuty = targetDuty;
}

int32_t HAL_Motor_ReadEncoder(HAL_MotorConfig* config)
{
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return 0;
    }

    const uint32_t lastTicks = config->state.encoderTicks;
    const uint32_t currentTicks = __HAL_TIM_GET_COUNTER(config->encoder.tim);
    int32_t delta = (int32_t)(currentTicks - lastTicks);
    // if (currentTicks < lastTicks && delta < -0x8000) // 16位溢出
    // {
    //     {
    //         delta = (int32_t)currentTicks + (0x10000 - (int32_t)lastTicks);
    //     }
    //
    //     // 更新编码器累积值
    //     config->state.encoderTicks += delta;
    // }
    config->state.encoderTicks += delta;
    return config->state.encoderTicks;
}

HAL_MotorState HAL_Motor_GetStatus(const HAL_MotorConfig* config)
{
    if (!config)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL MotorConfig is NULL");
        return (HAL_MotorState){0};
    }
    return config->state; // 返回状态结构体副本 (值返回机制：结构体值拷贝而非指针)
}
