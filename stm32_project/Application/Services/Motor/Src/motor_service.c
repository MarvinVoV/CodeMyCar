/*
 * motor_service.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "motor_service.h"
#include "pid_controller.h"


#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 私有函数声明
/* 私有函数声明 */
static void CalculateWheelSpeeds(MotionController* ctrl);
static void UpdateActualStates(MotionController* ctrl, float delta_time);
static void ApplyControlLogic(MotionController* ctrl, float delta_time);
static void CheckSystemHealth(MotionController* ctrl);

/* 初始化与配置 */
void MotionController_init(MotionController* ctrl, MotorDriver* left, MotorDriver* right, const MotionConfig* config)
{
    if (!ctrl || !left || !right || !config) return;

    // 绑定硬件驱动
    ctrl->left = left;
    ctrl->right = right;

    // 复制配置参数
    ctrl->config = *config;

    // 初始化状态
    memset(&ctrl->status, 0, sizeof(MotionStatus));
    ctrl->status.system.voltage = 12.0f; // 初始电压


    // 初始化线速度PID
    const PID_Params linear_pid_params = {
        .kp = config->linear_pid.kp,
        .ki = config->linear_pid.ki,
        .kd = config->linear_pid.kd,
        .integral_max = config->linear_pid.integral_max,
        .output_max = config->linear_pid.output_max
    };
    PIDController_Init(&ctrl->linear_pid, &linear_pid_params);
    ctrl->linear_pid.setpoint = 0.0f; // 初始目标为0

    // 初始化角速度PID
    const PID_Params angular_pid_params = {
        .kp = config->angular_pid.kp,
        .ki = config->angular_pid.ki,
        .kd = config->angular_pid.kd,
        .integral_max = config->angular_pid.integral_max,
        .output_max = config->angular_pid.output_max
    };
    PIDController_Init(&ctrl->angular_pid, &angular_pid_params);
    ctrl->angular_pid.setpoint = 0.0f; // 初始目标为0
}

/* 设置目标速度 */
void MotionController_setVelocity(MotionController* ctrl, const float linear, const float angular)
{
    if (!ctrl || ctrl->status.system.error_code) return;

    // 参数限幅
    ctrl->status.target.linear_speed = CLAMP(linear,
                                             -ctrl->config.max_linear_speed,
                                             ctrl->config.max_linear_speed);

    ctrl->status.target.angular_speed = CLAMP(angular,
                                              -ctrl->config.max_angular_speed,
                                              ctrl->config.max_angular_speed);

    // 更新PID目标值
    ctrl->linear_pid.setpoint = ctrl->status.target.linear_speed;
    ctrl->angular_pid.setpoint = ctrl->status.target.angular_speed;

    // 计算电机目标转速
    CalculateWheelSpeeds(ctrl);
}

/* 紧急停止 */
void MotionController_emergencyStop(MotionController* ctrl)
{
    if (!ctrl) return;

    PID_Reset(&ctrl->linear_pid);
    PID_Reset(&ctrl->angular_pid);

    // 设置错误标志位
    ctrl->status.system.error_code |= 0x80000000; // 最高位表示急停

    // 停止电机输出
    Motor_EmergencyStop(ctrl->left);
    Motor_EmergencyStop(ctrl->right);
}

/* 状态更新 */
void MotionController_Update(MotionController* ctrl, const float delta_time)
{
    if (!ctrl || delta_time <= 0.0f) return;

    // 1. 安全检测
    CheckSystemHealth(ctrl);

    // 2. 更新底层驱动状态
    Motor_Update(ctrl->left);
    Motor_Update(ctrl->right);

    // 3. 计算实际运动状态
    UpdateActualStates(ctrl, delta_time);

    // 4. 闭环控制逻辑
    if (!(ctrl->status.system.error_code & 0x80000000))
    {
        ApplyControlLogic(ctrl, delta_time);
    }
}

/* 获取状态 */
MotionStatus* MotionController_getStatus(MotionController* ctrl)
{
    return ctrl ? &ctrl->status : NULL;
}

/******************** 静态函数实现 ********************/

/* 计算轮速目标值 */
static void CalculateWheelSpeeds(MotionController* ctrl)
{
    const float L = ctrl->config.wheel_base;
    const float R = ctrl->config.wheel_radius;

    // 差速计算
    const float v_left = ctrl->status.target.linear_speed - (L / 2) * ctrl->status.target.angular_speed;
    const float v_right = ctrl->status.target.linear_speed + (L / 2) * ctrl->status.target.angular_speed;

    // 转换为RPM
    const float left_rpm = (float)(v_left / (2 * M_PI * R)) * 60.0f;
    const float right_rpm = (float)(v_right / (2 * M_PI * R)) * 60.0f;

    // 设置电机目标
    Motor_SetTargetRPM(ctrl->left, left_rpm);
    Motor_SetTargetRPM(ctrl->right, right_rpm);
}

/* 更新实际状态 */
static void UpdateActualStates(MotionController* ctrl, const float delta_time)
{
    // 获取电机实际转速
    const float left_rpm = ctrl->left->state.rpm;
    const float right_rpm = ctrl->right->state.rpm;
    const float R = ctrl->config.wheel_radius;
    const float L = ctrl->config.wheel_base;

    // 计算实际线速度
    ctrl->status.actual.linear_speed =
        (left_rpm + right_rpm) * (2.0f * (float)M_PI * R) / 120.0f;

    // 计算实际角速度
    ctrl->status.actual.angular_speed =
        (right_rpm - left_rpm) * (2.0f * (float)M_PI * R) / (L * 60.0f);

    // 更新累计里程
    ctrl->status.actual.odometer += fabsf(ctrl->status.actual.linear_speed) * delta_time;

    // 同步系统状态
    // ctrl->status.system.voltage = SafetyMonitor_GetVoltage();
}

/* 应用控制算法 */
static void ApplyControlLogic(MotionController* ctrl, const float delta_time)
{
    // ================ 1. 计算线速度PID输出 ================
    const float linear_output = PID_Calculate(
        &ctrl->linear_pid,
        ctrl->status.actual.linear_speed, // 当前实际线速度作为测量值
        delta_time
    );

    // ================ 2. 计算角速度PID输出 ================
    const float angular_output = PID_Calculate(
        &ctrl->angular_pid,
        ctrl->status.actual.angular_speed, // 当前实际角速度作为测量值
        delta_time
    );

    // ================ 3. 合成新的目标速度 ================
    const float new_linear = ctrl->status.target.linear_speed + linear_output;
    const float new_angular = ctrl->status.target.angular_speed + angular_output;

    // ================ 4. 目标速度限幅 ================
    ctrl->status.target.linear_speed = CLAMP(new_linear,
        -ctrl->config.max_linear_speed,
        ctrl->config.max_linear_speed
    );
    ctrl->status.target.angular_speed = CLAMP(new_angular,
        -ctrl->config.max_angular_speed,
        ctrl->config.max_angular_speed
    );

    // ================ 5. 转换为轮速目标 ================
    CalculateWheelSpeeds(ctrl);
}

/* 系统健康检查 */
static void CheckSystemHealth(MotionController* ctrl)
{
    uint32_t errors = 0;

    // 聚合电机错误
    if (ctrl->left->state.error_code) errors |= 0x01;
    if (ctrl->right->state.error_code) errors |= 0x02;

    // 电压检测
    if (ctrl->status.system.voltage < 10.5f) errors |= 0x04;
    if (ctrl->status.system.voltage > 14.5f) errors |= 0x08;

    // 温度检测
    if (ctrl->status.system.temperature > 85.0f) errors |= 0x10;

    // 更新错误码（保留最高位为急停标志）
    ctrl->status.system.error_code = (ctrl->status.system.error_code & 0x80000000) | errors;

    // 触发急停条件
    if (errors != 0)
    {
        MotionController_emergencyStop(ctrl);
    }
}
