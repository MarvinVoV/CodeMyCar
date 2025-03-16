/*
 * motor_service.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#include "motor_service.h"

#include <string.h>
#include <math.h>
#include "pic_controller.h"

#define PI 3.14159265358979323846f

static void calculate_target_rpm(MotionController* motion_controller, float linear, float angular);

void MotionService_Init(MotionController* motion_controller)
{
    // 初始化电机驱动层
    Motor_Init(motion_controller->left_motor);
    Motor_Init(motion_controller->right_motor);

    // 控制模式初始化
    motion_controller->target_rpm = 0;
    motion_controller->target_omega = 0;

    // PID初始化
    const PIDParams default_left_pid = {
        .Kp = 0.5f, .Ki = 0.1f, .Kd = 0.02f,
        .integral_limit = 100.0f,
        .output_limit = 100.0f
    };
    const PIDParams default_right_pid = {
        .Kp = 0.5f, .Ki = 0.1f, .Kd = 0.02f,
        .integral_limit = 100.0f,
        .output_limit = 100.0f
    };

    PIDController_Init(&motion_controller->left_pid, &default_left_pid);
    PIDController_Init(&motion_controller->right_pid, &default_right_pid);
    // 运动参数初始化
    motion_controller->kinematic_params = (KinematicParams){
        .wheel_base = 0.167f,
        .wheel_radius = 0.0325f,
        .max_linear_speed = 1.2f,  // 按额定转速计算值
        .max_angular_speed = 3.14f // 按π rad/s（180°/s）安全值
    };
    motion_controller->emergency_stop = false;
    memset(motion_controller->status, 0, sizeof(motion_controller->status));
}


void MotionService_SetVelocity(MotionController* motion_controller, float linear, float angular)
{
    if (motion_controller->emergency_stop) return;

    // 速度限幅
    linear = fmaxf(-motion_controller->kinematic_params.max_linear_speed,
                   fminf(linear, motion_controller->kinematic_params.max_linear_speed));
    angular = fmaxf(-motion_controller->kinematic_params.max_angular_speed,
                    fminf(angular, motion_controller->kinematic_params.max_angular_speed));

    // 计算目标转速
    calculate_target_rpm(motion_controller, linear, angular);
}

void MotionService_EmergencyStop(MotionController* motion_controller)
{
    motion_controller->emergency_stop = true;

    // 立即停止电机
    Motor_SetSpeed(motion_controller->left_motor, 0);
    Motor_SetSpeed(motion_controller->right_motor, 0);

    // 重置PID状态
    PIDController_Reset(&motion_controller->left_pid);
    PIDController_Reset(&motion_controller->right_pid);
}

const MotorStatus* MotionService_GetStatus(MotionController* motion_controller, int motor_id)
{
    if (motor_id < 0 || motor_id > 1) return NULL;
    // 电流检测需要硬件支持，此处留空
    return &motion_controller->status[motor_id];
}


static void calculate_target_rpm(MotionController* motion_controller, const float linear, const float angular)
{
    // 差速运动学计算
    const float linear_left = linear - angular * motion_controller->kinematic_params.wheel_base / 2;
    const float linear_right = linear + angular * motion_controller->kinematic_params.wheel_base / 2;

    // 转换为RPM：RPM = (线速度 m/s * 60) / (2π * 轮半径)
    const float rpm_left = (linear_left * 60) /
        (2 * PI * motion_controller->kinematic_params.wheel_radius);
    const float rpm_right = (linear_right * 60) /
        (2 * PI * motion_controller->kinematic_params.wheel_radius);

    // 设置目标转速（注意符号方向）
    motion_controller->target_rpm = rpm_left; // 根据实际电机转向调整
    motion_controller->status[MOTOR_LEFT].current_rpm = rpm_left;
    motion_controller->status[MOTOR_RIGHT].current_rpm = rpm_right;
}

// 私有函数实现
void MotionService_UpdateMotorStatu(const Motor* motor, MotorStatus* status)
{
    status->current_rpm = (float)motor->rpm;
    status->total_pulses = motor->total_pulses;
    // 电流检测需要硬件支持，此处留空
}
