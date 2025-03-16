/*
 * pic_controller.h
 *
 *  Created on: Mar 16, 2025
 *      Author: marvin
 */

#ifndef SERVICES_MOTOR_INC_PIC_CONTROLLER_H_
#define SERVICES_MOTOR_INC_PIC_CONTROLLER_H_

typedef struct
{
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    float integral_limit; // 积分限幅
    float output_limit;   // 输出限幅 ±100%
} PIDParams;

/**
 * PID控制器结构体
 *
 * 用于实现比例-积分-微分控制算法，通过对系统误差进行比例、积分、微分运算
 * 计算出控制输出量，适用于需要闭环控制的场景（如电机速度控制、温度控制等）
 */
typedef struct
{
    /**
     * 目标设定值
     *
     * 表示控制系统期望达到的目标值，单位与测量值一致
     * 示例：当控制电机转速时，设置为1000表示目标转速为1000 RPM
     */
    float setpoint;

    /**
     * 积分项累积值
     *
     * 用于存储误差的积分值，主要作用：
     * 1. 消除系统稳态误差
     * 2. 提高系统对持续偏差的响应能力
     *
     * 注意：实际使用中需要设置积分限幅(params.integral_limit)防止积分饱和
     */
    float integral;

    /**
     * 前次误差值
     *
     * 存储上一次控制周期的误差值（setpoint - measurement），用于：
     * 1. 计算微分项的误差变化率
     * 2. 实现微分先行（避免设定值突变导致的微分冲击）
     *
     * 初始化时应设为0，重置控制器时需要清除此值
     */
    float prev_error;

    /**
     * PID控制参数
     *
     * 包含控制器调节参数：
     * - kp: 比例系数，决定系统响应速度
     * - ki: 积分系数，消除稳态误差
     * - kd: 微分系数，抑制超调/振荡
     * - integral_limit: 积分限幅，防止积分项过大
     * - output_limit: 输出限幅，确保控制量在合理范围
     *
     * 建议通过Ziegler-Nichols方法等调参方法进行参数整定
     */
    PIDParams params;
} PIDController;

/**
 * @brief 初始化PID控制器
 * @param pid 要初始化的PID控制器指针，必须指向有效内存
 * @param params PID参数指针，必须指向有效参数结构体
 *
 * @details
 * - 执行控制器内部状态重置（积分项清零，前次误差归零）
 * - 复制参数结构体内容到控制器实例
 * - 典型调用场景：控制器首次使用前或参数修改后
 *
 * @warning
 * - 参数指针必须有效，否则会导致未定义行为
 * - 初始化后建议设置合理的setpoint值
 */
void PIDController_Init(PIDController* pid, const PIDParams* params);

/**
 * @brief 执行PID控制计算
 * @param pid PID控制器实例指针
 * @param measurement 当前测量值（反馈值），单位与setpoint一致
 * @param delta_time 距离上次计算的时间间隔（单位：秒），必须为正数
 * @return 计算得到的控制量输出，范围由params.output_limit限定
 *
 * @details
 * - 实现离散PID计算公式：
 *   output = Kp*e + Ki*Σe*dt + Kd*(e - e_prev)/dt
 * - 积分项采用抗饱和处理，限制范围由params.integral_limit决定
 * - 微分项基于测量值变化（微分先行），避免设定值突变引起的微分冲击
 *
 * @note
 * - 建议以固定周期调用（如每10ms调用一次）
 * - 当dt接近0时（<1us），返回0输出防止计算异常
 * - 典型调用流程：
 *   1. 读取传感器获得measurement
 *   2. 调用本函数计算控制量
 *   3. 将输出作用于执行器
 */
float PIDController_Update(PIDController* pid, float measurement, float delta_time);

/**
 * @brief 重置控制器内部状态
 * @param pid 要重置的PID控制器指针
 *
 * @details
 * - 清零积分项（integral = 0）
 * - 重置前次误差（prev_error = 0）
 * - 保持参数设置(setpoint和params)不变
 *
 * @warning
 * - 必须在控制器停止工作时调用（如急停后/模式切换时）
 * - 误用会导致控制不连续，可能引发系统不稳定
 *
 * @example
 * // 当系统从手动模式切换到自动控制时
 * PIDController_Reset(&pid);
 * pid.setpoint = target_value;  // 设置新目标值
 * enable_control();             // 启用控制器
 */
void PIDController_Reset(PIDController* pid);

#endif /* SERVICES_MOTOR_INC_PIC_CONTROLLER_H_ */
