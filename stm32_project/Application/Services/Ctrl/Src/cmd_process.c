/*
 * cmd_process.c
 *
 *  Created on: Mar 30, 2025
 *      Author: marvin
 */
#include <string.h>
#include "cmd_process.h"
#include "error_code.h"
#include "log_manager.h"
#include "ctrl_protocol.h"


static float decodeLinearVel(uint16_t value);
static float decodeSteerAngle(uint16_t deg);
static float decodeAngularVel(uint16_t angularVel);


static CmdProcessorContext* ctx;

void CmdProcessor_Init(CmdProcessorContext* commandContext)
{
    if (!commandContext->motionContext)
    {
        return;
    }
    ctx = commandContext;
}

void CmdProcessor_processCommand(ControlCmd* cmd)
{
    // 1. 安全校验
    if (!ctx) return;

    // 2. 处理运动指令
    if (cmd->fields & CTRL_FIELD_MOTION)
    {
        MotionCmd* motion = &cmd->motion;
        MotionContext* motionCtx = ctx->motionContext;

        int ret = ERR_SUCCESS;
        // 模式切换前检查
        if (motion->mode != MOTION_EMERGENCY_STOP)
        {
            ret = MotionService_SetControlMode(motionCtx, motion->mode);
            if (ret != ERR_SUCCESS)
            {
                LOG_ERROR(LOG_MODULE_CMD, "Mode switch failed: 0x%02X", motion->mode);
                return;
            }
        }

        // 模式分派处理
        switch (motion->mode)
        {
            case MOTION_EMERGENCY_STOP:
                MotionService_EmergencyStop(motionCtx);
                break;

            case MOTION_DIFFERENTIAL:
                {
                    const float linear = decodeLinearVel(motion->params.diffCtrl.linearVel);
                    const float angular = decodeAngularVel(motion->params.diffCtrl.angularVel);
                    ret = MotionService_SetVelocity(motionCtx, linear, angular);
                    break;
                }

            case MOTION_STEER_ONLY:
                {
                    const float linear = decodeLinearVel(motion->params.kinematicCtrl.linearVel);
                    const float steer = decodeAngularVel(motion->params.kinematicCtrl.steerAngle);
                    ret = MotionService_SetAckermannParams(motionCtx, linear, steer);
                    break;
                }

            case MOTION_DIRECT_CONTROL:
                {
                    const float leftRpm = motion->params.directCtrl.leftRpm;
                    const float rightRpm = motion->params.directCtrl.rightRpm;
                    const float steer = decodeSteerAngle(motion->params.directCtrl.steerAngle);
                    ret = MotionService_SetDirectControl(motionCtx, leftRpm, rightRpm, steer);
                    break;
                }

            case MOTION_MIXED_STEER:
                {
                    const DiffCtrlParam* base = &motion->params.mixedCtrl.base;
                    const float linear = decodeLinearVel(base->linearVel);
                    const float angular = decodeAngularVel(base->angularVel);
                    const float steer = decodeSteerAngle(motion->params.mixedCtrl.steerAngle);
                    const float gain = motion->params.mixedCtrl.differentialGain;
                    ret = MotionService_SetHybridParams(motionCtx, linear, angular, steer, gain);
                    break;
                }

            case MOTION_SPIN_IN_PLACE:
                {
                    const DiffCtrlParam* p = &motion->params.diffCtrl;
                    const float angular = decodeAngularVel(p->angularVel);
                    ret = MotionService_SetSpinParams(motionCtx, angular);
                    break;
                }

            default:
                LOG_ERROR(LOG_MODULE_CMD, "Unknown motion mode: 0x%02X", motion->mode);
                ret = ERR_INVALID_PARAM;
                break;
        }
        // 处理执行结果
        if (ret != ERR_SUCCESS)
        {
            LOG_ERROR(LOG_MODULE_CMD, "Command execute failed: 0x%04X", ret);
        }
    }
}

static float decodeLinearVel(const uint16_t value)
{
    return (float)value * 0.001f; // 0.01 m/s分辨率
}

static float decodeSteerAngle(const uint16_t deg)
{
    return (float)deg * 0.1f; // 0.1度分辨率
}

static float decodeAngularVel(const uint16_t angularVel)
{
    return (float)angularVel * 0.0644f; // 0.0644rad/s分辨率
}
