/*
 * motor_driver.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#ifndef MODULES_MOTOR_INC_MOTOR_DRIVER_H_
#define MODULES_MOTOR_INC_MOTOR_DRIVER_H_
#include "motor_hal.h"

typedef struct
{
    HAL_MotorConfig hal;            // 硬件抽象层配置（PWM和GPIO控制）
    TIM_HandleTypeDef* encoder_tim; // 编码器定时器句柄（用于读取位置）
    TIM_HandleTypeDef* pwm_tim;     // PWM定时器句柄（用于控制占空比）

    // 编码器参数
    int32_t encoder_res; // 编码器分辨率/编码器脉冲数（电机轴每转的脉冲数，已考虑4倍频）
    double gear_ratio;    // 减速比（电机轴到输出轴的减速比，如30:1则填30）

    // 运行时状态
    double rpm;            // 实时转速（转/分钟，输出轴转速）
    int32_t total_pulses; // 累计脉冲数（用于计算绝对位置）
} Motor;

void Motor_Init(Motor* motor);
/**
 * 设置电机转速（占空比控制）
 * @param motor motor
 * @param duty 占空比（-100%~+100%）
*/
void Motor_SetSpeed(const Motor* motor, double duty);
/**
 * 更新电机状态（需周期性调用）
 * @param motor motor
 */
void Motor_UpdateState(Motor* motor);

#endif /* MODULES_MOTOR_INC_MOTOR_DRIVER_H_ */
