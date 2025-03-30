/*
 * motor_hal.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include <string.h>
#include "motor_hal.h"


/**
 * 相关配置已经通过 CubeMX 完成配置，这里只需要实现启动逻辑即可
 */
void HAL_Motor_Init(HAL_MotorConfig* config)
{
    // 初始化PWM
    HAL_TIM_PWM_Start(config->pwm.tim, config->pwm.ch);

    // 初始化编码器
    HAL_TIM_Encoder_Start(config->encoder.tim, TIM_CHANNEL_ALL);

    // 初始化GPIO默认状态
    HAL_GPIO_WritePin(config->gpio.in1_port, config->gpio.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->gpio.in2_port, config->gpio.in2_pin, GPIO_PIN_RESET);

    // 初始化状态
    memset(&config->state.data, 0, sizeof(HAL_MotorStatus));
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
    // 更新方向状态
    config->state.data.direction = direction;
    switch (direction)
    {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(config->gpio.in1_port, config->gpio.in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->gpio.in2_port, config->gpio.in2_pin, GPIO_PIN_RESET);
            break;

        case MOTOR_DIR_BACKWARD:
            HAL_GPIO_WritePin(config->gpio.in1_port, config->gpio.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->gpio.in2_port, config->gpio.in2_pin, GPIO_PIN_SET);
            break;
        case MOTOR_COAST_STOP:
            HAL_GPIO_WritePin(config->gpio.in1_port, config->gpio.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->gpio.in2_port, config->gpio.in2_pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BRAKE_STOP:
            HAL_GPIO_WritePin(config->gpio.in1_port, config->gpio.in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->gpio.in2_port, config->gpio.in2_pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

void HAL_Motor_EmergencyStop(HAL_MotorConfig* config)
{
    // 立即关闭PWM输出
    HAL_Motor_SetPWM(config, 0);

    // 激活硬件刹车模式
    HAL_Motor_SetDirection(config, MOTOR_BRAKE_STOP);

    // 复位编码器
    __HAL_TIM_SET_COUNTER(config->encoder.tim, 0);

    // 设置急停标志位
    config->state.data.error_flags |= 0x01;
}

void HAL_Motor_SetPWM(HAL_MotorConfig* config, uint16_t duty)
{
    // 限制占空比在ARR范围内
    const uint16_t arr = __HAL_TIM_GET_AUTORELOAD(config->pwm.tim);
    duty = (duty > arr) ? arr : duty;

    __HAL_TIM_SET_COMPARE(config->pwm.tim, config->pwm.ch, duty);

    // 更新状态中的占空比
    config->state.data.error_flags |= 0x01; // 设置急停标志位
}

int32_t HAL_Motor_ReadEncoder(HAL_MotorConfig* config)
{
    static uint32_t last_cnt = 0;
    static int32_t total_ticks = 0;

    const uint32_t current_cnt = __HAL_TIM_GET_COUNTER(config->encoder.tim);
    /*
     * 自动识别溢出，例如:
     * 当计数器从65535溢出到0时，delta = 0 - 65535 = -65535（实际应为+1），但由于int16_t强制类型转换，会修正为+1。
     */
    const int16_t delta = (int16_t)(current_cnt - last_cnt);
    total_ticks += delta;
    last_cnt = current_cnt;

    // 更新编码器累积值
    config->state.data.encoder_ticks += delta;

    return total_ticks;
}

HAL_MotorStatus HAL_Motor_GetStatus(const HAL_MotorConfig* config)
{
    return config->state.data; // 返回状态结构体副本 (值返回机制：结构体值拷贝而非指针)
}
