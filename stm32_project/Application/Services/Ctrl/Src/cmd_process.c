/*
 * cmd_process.c
 *
 *  Created on: Mar 30, 2025
 *      Author: marvin
 */
#include "cmd_process.h"
#include "log_manager.h"
#include <string.h>

#include "ctrl_protocol.h"
#include "servo_service.h"

static CmdProcessorContext* ctx;

void CmdProcessor_Init(CmdProcessorContext* commandContext)
{
    ctx = commandContext;
}

void CmdProcessor_processCommand(const ControlCmd* cmd)
{
    // 1. 安全校验
    if (!ctx || !cmd || !ctx->motionCtrl || !ctx->steerServo) return;

    // 2. 处理运动指令
    if (cmd->fields & CTRL_FIELD_MOTION)
    {
        const MotionCmd* motion = &cmd->motion;

        switch (motion->mode)
        {
            case MOTION_EMERGENCY_STOP:
                {
                    MotionController_emergencyStop(ctx->motionCtrl);
                    // 快速回中
                    ServoService_setAngleImmediate(ctx->steerServo, 90);
                    break;
                }

            case MOTION_DIRECT_CONTROL:
                {
                    // 直接控制模式：独立设置舵机和轮速
                    const DirectCtrlParam* param = &motion->params.direct;

                    // 设置舵机角度
                    const int angle = param->steer_angle;
                    ServoService_setAngleSmoothly(ctx->steerServo, angle, 200);

                    // 转换为差速控制
                    const float left_speed = Q15_TO_FLOAT(param->left_speed);
                    const float right_speed = Q15_TO_FLOAT(param->right_speed);

                    // 计算合成线速度和角速度
                    const float linear = (left_speed + right_speed) / 2.0f;
                    const float angular = (right_speed - left_speed) / ctx->motionCtrl->config.wheel_base;

                    MotionController_setVelocity(ctx->motionCtrl, linear, angular);
                    break;
                }

            case MOTION_DIFFERENTIAL:
                {
                    // 纯差速模式：舵机回中，通过角速度控制
                    ServoService_setAngleSmoothly(ctx->steerServo, 90, 500); // 慢速回中

                    // 从运动学参数获取指令
                    const KinematicParam* param = &motion->params.kinematic;
                    const float linear = Q15_TO_FLOAT(param->linear_vel);
                    const float angular = Q15_TO_FLOAT(param->angular_vel);

                    MotionController_setVelocity(ctx->motionCtrl, linear, angular);
                    break;
                }

            case MOTION_STEER_ONLY:
                {
                    // 前轮转向模式：保持当前线速度，只控制转向
                    const KinematicParam* param = &motion->params.kinematic;

                    // 1. 设置前轮转角
                    const float steer_angle = Q7_TO_FLOAT(param->steer_angle);
                    ServoService_setAngleSmoothly(ctx->steerServo, (int)steer_angle, 200);

                    // 2. 根据转向几何计算等效角速度
                    const float linear = Q15_TO_FLOAT(param->linear_vel);
                    const float angular = linear * tanf(steer_angle * M_PI / 180.0f) /
                        ctx->motionCtrl->config.wheel_base;
                    MotionController_setVelocity(ctx->motionCtrl, linear, angular);
                    break;
                }

            case MOTION_SPIN_IN_PLACE:
                {
                    // 原地旋转：线速度为0，角速度由参数决定
                    const SpinParam* param = &motion->params.spin;
                    const float angular = param->radius == 0 ? param->angular_vel : 0;
                    MotionController_setVelocity(ctx->motionCtrl, 0, angular);
                    break;
                }

            case MOTION_MIXED_STEER:
                {
                    // 混合模式：同时控制转向和差速
                    const KinematicParam* param = &motion->params.kinematic;

                    // 1. 设置前轮转角
                    const float steer_angle = Q7_TO_FLOAT(param->steer_angle);
                    ServoService_setAngleSmoothly(ctx->steerServo, (int)steer_angle, 200);

                    // 2. 综合两种转向方式
                    const float linear = Q15_TO_FLOAT(param->linear_vel);
                    const float angular_steer = linear * tanf(steer_angle * M_PI / 180.0f) /
                        ctx->motionCtrl->config.wheel_base;
                    const float angular_diff = Q15_TO_FLOAT(param->angular_vel);

                    MotionController_setVelocity(ctx->motionCtrl, linear, angular_steer + angular_diff);
                    break;
                }

            default:
                break;
        }
    }
    else if (cmd->fields & CTRL_FIELD_TEXT)
    {
        LOG_INFO(LOG_MODULE_SYSTEM, "Receive debug %.*s\n", cmd->pingText.len, cmd->pingText.msg);
    }
}
