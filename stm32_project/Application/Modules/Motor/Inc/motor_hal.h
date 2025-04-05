/*
 * motor_hal.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#ifndef MODULES_MOTOR_INC_MOTOR_HAL_H_
#define MODULES_MOTOR_INC_MOTOR_HAL_H_

#include "stm32h7xx_hal.h"

typedef enum
{
    MOTOR_DIR_FORWARD = 0x1, // 正转
    MOTOR_DIR_BACKWARD,      // 反转
    MOTOR_COAST_STOP,        // 滑行停止（释放H桥）
    MOTOR_BRAKE_STOP         // 刹车停止（短接线圈）
} MotorDirection;


// 电机状态结构体
typedef struct
{
    uint16_t currentDuty;     // 当前PWM占空比
    MotorDirection direction; // 当前方向状态
    int32_t encoderTicks;     // 编码器累计脉冲数
    uint8_t errorFlags;       // 错误标志位（过流、过热等）
} HAL_MotorState;


typedef struct
{
    /* PWM控制 */
    struct
    {
        TIM_HandleTypeDef* tim; // PWM定时器句柄（如TIM3）
        uint32_t ch;            // PWM输出通道（0-15，如TIM3_CH1
    } pwm;

    /* 编码器接口 */
    struct
    {
        TIM_HandleTypeDef* tim; // 编码器定时器句柄（如TIM4
    } encoder;

    /* 驱动控制引脚 */
    struct
    {
        GPIO_TypeDef* in1Port; // IN1控制引脚GPIO端口（如GPIOA)
        uint16_t in1Pin;       // IN1控制引脚编号（如GPIO_PIN_4
        GPIO_TypeDef* in2Port; // IN2控制引脚GPIO端口（如GPIOB)
        uint16_t in2Pin;       // IN2控制引脚编号（如GPIO_PIN_5）
    } gpio;

    HAL_MotorState state; // 电机状态结构体
} HAL_MotorConfig;

/**
 * 初始化
 * @param config config
 */
void HAL_Motor_Init(HAL_MotorConfig* config);

/**
 * 设置占空比
 * @param config config
 * @param duty duty
 */
void HAL_Motor_SetDuty(HAL_MotorConfig* config, float duty);

/**
 * 电机方向控制
 * @param config config
 * @param direction direction
 */
void HAL_Motor_SetDirection(HAL_MotorConfig* config, MotorDirection direction);

/**
 * 立即停止
 * @param config config
 */
void HAL_Motor_EmergencyStop(HAL_MotorConfig* config);

/**
 * 读取电机编码器的累积脉冲值，用于获取电机的转动位置和速度信息
 * @param config config
 * @return count
 */
int32_t HAL_Motor_ReadEncoder(HAL_MotorConfig* config);

/**
 * 获取电机状态
 * @param config config
 * @return state
 */
HAL_MotorState HAL_Motor_GetStatus(const HAL_MotorConfig* config);

#endif /* MODULES_MOTOR_INC_MOTOR_HAL_H_ */
