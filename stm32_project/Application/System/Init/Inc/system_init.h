/*
 * system_init.h
 *
 *  Created on: Apr 12, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_INIT_SYSTEM_INIT_H_
#define SYSTEM_INIT_SYSTEM_INIT_H_

#include "cmd_process.h"
#include "motion_service.h"

// 舵机 PWM TIM
extern TIM_HandleTypeDef htim2;
// 电机 PWM TIM
extern TIM_HandleTypeDef htim3;
// 左轮电机编码器 TIM
extern TIM_HandleTypeDef htim4;
// 右轮电机编码器 TIM
extern TIM_HandleTypeDef htim5;

/*--------------------- 系统上下文 ---------------------*/
typedef struct
{
    MotorService* motorService;
    SteerInstance* steerInstance;
    MotionContext* motionContext;
    CmdProcessorContext* cmdContext;
} SystemContext;

/**
 * @brief 系统全局初始化
 * @return 初始化状态 0=成功，其他=错误码
 */
int System_Initialize();

/**
 * @brief 获取系统上下文
 */
SystemContext* System_GetContext(void);

#endif /* SYSTEM_INIT_SYSTEM_INIT_H_ */
