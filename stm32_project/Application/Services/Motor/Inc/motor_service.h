/*
 * motor_service.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#ifndef SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#define SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#include <stdint.h>

#include "motor_driver.h"
#include "pid_controller.h"

/**
 * @brief 运动控制器配置参数
 */
typedef struct
{
    // 运动学参数
    float wheel_radius; // 轮半径 (m)
    float wheel_base;   // 轮距 (m)

    // 动态限制
    float max_linear_speed;  // 最大线速度 (m/s)
    float max_angular_speed; // 最大角速度 (rad/s)

    PID_Params linear_pid;  // 线速度控制参数
    PID_Params angular_pid; // 角速度控制参数
} MotionConfig;


/**
 * @brief 运动控制器状态结构体
 */
typedef struct
{
    // ----------- 目标值 -----------
    struct
    {
        float linear_speed;  // 目标线速度 (m/s)
        float angular_speed; // 目标角速度 (rad/s)
    } target;

    // ----------- 实际值 -----------
    struct
    {
        float linear_speed;  // 实际线速度 (m/s)
        float angular_speed; // 实际角速度 (rad/s)
        float odometer;      // 累计行驶里程 (m)
    } actual;

    /* PID控制器状态 */
    struct
    {
        // 线速度PID状态
        struct
        {
            float integral;   // 积分项累计
            float last_error; // 上一次误差
        } linear;

        // 角速度PID状态
        struct
        {
            float integral;   // 积分项累计
            float last_error; // 上一次误差
        } angular;
    } pid_state;

    // ----------- 系统状态 -----------
    struct
    {
        uint32_t error_code; // 错误码（整车级）
        float voltage;       // 系统电压 (V)
        float temperature;   // 控制器温度 (°C)
    } system;
} MotionStatus;

/**
 * @brief 运动控制器核心结构体
 */
typedef struct
{
    MotorDriver* left;          // 左电机驱动实例
    MotorDriver* right;         // 左电机驱动实例
    MotionConfig config;        // 控制器配置参数
    MotionStatus status;        // 当前状态信息
    PID_Controller linear_pid;  // 线速度PID控制器
    PID_Controller angular_pid; // 角速度PID控制器
} MotionController;

/**
 * @brief 初始化运动控制器
 * @param ctrl 控制器实例指针
 * @param left 左电机驱动实例
 * @param right 右电机驱动实例
 * @param config 初始化配置参数
 *
 * @note 该函数不会分配内存，需确保传入的结构体已正确初始化
 */
void MotionController_init(MotionController* ctrl, MotorDriver* left, MotorDriver* right, const MotionConfig* config);

/**
 * @brief 设置目标运动速度
 * @param ctrl 控制器实例指针
 * @param linear 目标线速度 (m/s)
 * @param angular 目标角速度 (rad/s)
 *
 * @note 实际设置值会被限制在配置的 max_linear_speed 和 max_angular_speed 范围内
 */
void MotionController_setVelocity(MotionController* ctrl, float linear, float angular);
/**
 * @brief 立即执行紧急停止
 * @param ctrl 控制器实例指针
 *
 * @warning 该操作会切断电机输出并锁定控制器状态，需手动调用恢复函数
 */
void MotionController_emergencyStop(MotionController* ctrl);

/**
 * @brief 更新控制器状态（需周期性调用）
 * @param ctrl 控制器实例指针
 * @param delta_time 距离上次更新的时间间隔 (秒)
 *
 * @note 典型调用周期为5-20ms，时间间隔错误可能导致控制异常
 */
void MotionController_Update(MotionController* ctrl, float delta_time);

/**
 * @brief 获取当前状态信息（只读）
 * @param ctrl 控制器实例指针
 * @return 指向当前状态结构体的常量指针
 */
MotionStatus* MotionController_getStatus(MotionController* ctrl);


#endif /* SERVICES_MOTOR_INC_MOTOR_SERVICE_H_ */
