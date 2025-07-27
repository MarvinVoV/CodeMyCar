/*
 * pic_controller.h
 *
 *  Created on: Mar 16, 2025
 *      Author: marvin
 */

#ifndef SERVICES_MOTOR_INC_PID_CONTROLLER_H_
#define SERVICES_MOTOR_INC_PID_CONTROLLER_H_
#ifndef CLAMP
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#include <stdbool.h>
#endif

#define ZERO_TARGET_THRESHOLD  (0.1f)

/**
 * @brief PID控制器参数结构
 */
typedef struct
{
    float kp;                        ///< 比例系数（建议范围：0.1-10.0）
    float ki;                        ///< 积分系数（建议范围：0.01-2.0）
    float kd;                        ///< 微分系数（建议范围：0.0-1.0）
    float integral_max;              ///< 积分限幅（防饱和，建议为output_max的1.2-2倍）
    float output_max;                ///< 输出限幅（绝对值不超过该值）
    bool  anti_windup;               ///< 抗饱和使能 (true=启用, false=禁用)
    float integral_threshold;        ///< 积分激活阈值（误差小于此值时积分）
    float dead_zone;                 ///< 死区阈值（误差小于此值时输出为0）
    float setpoint_change_threshold; ///< 设定点变化阈值（单位：rad/s）
} PID_Params;

/**
 * @brief PID控制器状态结构
 */
typedef struct
{
    float setpoint;         ///< 当前目标值
    float integral;         ///< 积分项累积值
    float prev_measurement; ///< 前次测量值（用于微分计算）
    float output;           ///< 上一次输出值
} PID_State;

/**
 * @brief PID控制器主结构
 */
typedef struct
{
    PID_Params params; ///< PID参数
    PID_State  state;  ///< PID状态
} PID_Controller;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器实例指针
 * @param params PID参数指针
 */
void PIDController_Init(PID_Controller* pid, const PID_Params* params);

/**
 * @brief 设置PID控制器参数
 * @param pid PID控制器实例指针
 * @param params 新的参数值
 */
void PID_SetParameters(PID_Controller* pid, const PID_Params* params);

/**
 * @brief 执行PID计算
 * @param pid PID控制器实例指针
 * @param setpoint 目标设定值
 * @param measurement 当前测量值
 * @param delta_time 距离上次计算的时间间隔（秒）
 * @return 计算得到的控制输出值
 */
float PID_Calculate(PID_Controller* pid, float setpoint, float measurement, float delta_time);

/**
 * @brief 重置控制器内部状态
 * @param pid 要重置的PID控制器指针
 */
void PID_Reset(PID_Controller* pid);

#endif /* SERVICES_MOTOR_INC_PID_CONTROLLER_H_ */
