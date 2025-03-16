/*
 * motor_service.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#ifndef SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#define SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"
#include "pic_controller.h"

#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

// 运动学参数配置
typedef struct
{
    float wheel_radius;      // 轮子半径（米） 65mm/2
    float wheel_base;        // 轮距（米） 167mm
    float max_linear_speed;  // 最大线速度（米/秒）
    float max_angular_speed; // 最大角速度（弧度/秒）
} KinematicParams;


// 电机状态监控
typedef struct
{
    float current_rpm;         // 当前转速（RPM）
    float current_consumption; // 电流（安培）
    int32_t total_pulses;      // 累计编码器脉冲数
} MotorStatus;

// 服务层控制句柄
typedef struct
{
    // 硬件相关
    Motor* left_motor;
    Motor* right_motor;
    // 控制相关
    PIDController left_pid;
    PIDController right_pid;
    KinematicParams kinematic_params;
    MotorStatus status[2]; // 0:左电机, 1:右电机
    // 目标速度
    float target_rpm;   // 当前目标转速
    float current_rpm;  // 当前转速（来自编码器）
    float target_omega; // 当前目标角速度
    // 急停
    bool emergency_stop;
} MotionController;

// ================ 初始化函数 ================
void MotionService_Init(MotionController* motion_controller);

// ================ 核心控制接口 ================
void MotionService_SetVelocity(MotionController* ctrl,
                               float linear, float angular);

void MotionService_EmergencyStop(MotionController* ctrl);

// ================ 状态监控接口 ================
const MotorStatus* MotionService_GetStatus(MotionController* ctrl, int motor_id);

// ================ 任务函数（需在RTOS任务中调用） ================
void MotionService_UpdateTask(void* arg);

void MotionService_UpdateMotorStatu(const Motor* motor, MotorStatus* status);


#endif /* SERVICES_MOTOR_INC_MOTOR_SERVICE_H_ */
