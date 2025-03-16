/*
 * motor_hal.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#ifndef MODULES_MOTOR_INC_MOTOR_HAL_H_
#define MODULES_MOTOR_INC_MOTOR_HAL_H_

#include "stm32h7xx_hal.h"


typedef struct
{
    /*!< PWM控制配置 */
    TIM_HandleTypeDef* pwm_tim; /*!< PWM定时器句柄（如TIM3）*/
    uint32_t pwm_ch;            /*!< PWM输出通道（0-15，如TIM3_CH1）*/

    /*!< 编码器输入配置 */
    TIM_HandleTypeDef* encode_tim; /*!< 编码器定时器句柄（如TIM4）*/
    uint32_t encode_ch_a;          /*!< 编码器A相信号输入捕获通道 */
    uint32_t encode_ch_b;          /*!< 编码器B相信号输入捕获通道 */

    /*!< 电机功率控制引脚 */
    GPIO_TypeDef* in1_port; /*!< IN1控制引脚GPIO端口（如GPIOA）*/
    uint16_t in1_pin;       /*!< IN1控制引脚编号（如GPIO_PIN_4）*/
    GPIO_TypeDef* in2_port; /*!< IN2控制引脚GPIO端口（如GPIOB）*/
    uint16_t in2_pin;       /*!< IN2控制引脚编号（如GPIO_PIN_5）*/
} HAL_MotorConfig;

/**
 * PWM定时器初始化
 * @param htim htim (TIM3)
 */
void HAL_Motor_InitPWM(TIM_HandleTypeDef* htim);
/**
 * 编码器接口初始化
 * @param htim htim (TIM4)
 */
void HAL_Motor_InitEncoder(TIM_HandleTypeDef* htim);
/**
 * 电机方向控制
 * @param config config
 * @param dir 方向（1:正转，-1:反转，0:刹车）
 */
void HAL_Motor_SetDirection(const HAL_MotorConfig* config, int8_t dir);

/**
 * 立即停止
 * @param config config
 */
void HAL_Motor_EmergencyStop(const HAL_MotorConfig* config);


/**
 * 编码器读数
 * @param htim htim
 * @return count
 */
int32_t HAL_Motor_ReadEncoder(const TIM_HandleTypeDef* htim);
#endif /* MODULES_MOTOR_INC_MOTOR_HAL_H_ */
