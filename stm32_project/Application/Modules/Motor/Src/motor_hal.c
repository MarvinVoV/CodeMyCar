/*
 * motor_hal.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include "motor_hal.h"

/**
 * 初始化PWM定时器；
 * 注: 相关配置已经通过 CubeMX 完成配置，这里只需要实现启动逻辑即可
 * @param htim htim
 */
void HAL_Motor_InitPWM(TIM_HandleTypeDef* htim)
{
    // 启动PWM输出
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
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
}

void HAL_Motor_SetDirection(const HAL_MotorConfig* config, const int8_t dir)
{
    switch (dir)
    {
        case 1: // 正转
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
            break;

        case -1: // 反转
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_SET);
            break;

        default: // 刹车/停止 (滑行停止)
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
        // 同时拉高实现急停
        // HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_SET);
            break;
    }
}

void HAL_Motor_EmergencyStop(const HAL_MotorConfig* config)
{
    // 立即关闭PWM输出
    __HAL_TIM_SET_COMPARE(config->pwm_tim, config->pwm_ch, 0);

    // 激活硬件刹车模式
    HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_SET);
}

int32_t HAL_Motor_ReadEncoder(const TIM_HandleTypeDef* htim)
{
    return __HAL_TIM_GET_COUNTER(htim);
}
