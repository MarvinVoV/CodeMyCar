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

static inline float Q16_ToFloat(int32_t fixed);
static inline float Q8_7_ToFloat(int16_t fixed);
static inline float Gain8bit_ToFloat(uint8_t gain);

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
                    const float linear = Q16_ToFloat(motion->params.diffCtrl.linearVel);
                    const float angular = Q16_ToFloat(motion->params.diffCtrl.angularVel);
                    ret = MotionService_SetVelocity(motionCtx, linear, angular);
                    break;
                }

            case MOTION_STEER_ONLY:
                {
                    const float linear = Q16_ToFloat(motion->params.kinematicCtrl.linearVel);
                    const float steer = Q8_7_ToFloat(motion->params.kinematicCtrl.steerAngle);
                    ret = MotionService_SetAckermannParams(motionCtx, linear, steer);
                    break;
                }

            case MOTION_DIRECT_CONTROL:
                {
                    const float leftRpm = Q16_ToFloat(motion->params.directCtrl.leftRpm);
                    const float rightRpm = Q16_ToFloat(motion->params.directCtrl.rightRpm);
                    const float steer = Q8_7_ToFloat(motion->params.directCtrl.steerAngle);
                    ret = MotionService_SetDirectControl(motionCtx, leftRpm, rightRpm, steer);
                    break;
                }

            case MOTION_MIXED_STEER:
                {
                    DiffCtrlParam* base = &motion->params.mixedCtrl.base;
                    float linear = Q16_ToFloat(base->linearVel);
                    float angular = Q16_ToFloat(base->angularVel);
                    float steer = Q8_7_ToFloat(motion->params.mixedCtrl.steerAngle);
                    float gain = Gain8bit_ToFloat(motion->params.mixedCtrl.differentialGain);
                    ret = MotionService_SetHybridParams(motionCtx, linear, angular, steer, gain);
                    break;
                }

            case MOTION_SPIN_IN_PLACE:
                {
                    DiffCtrlParam* p = &motion->params.diffCtrl;
                    const float angular = Q16_ToFloat(p->angularVel);
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
    else if (cmd->fields & CTRL_FIELD_TEXT)
    {
        LOG_INFO(LOG_MODULE_SYSTEM, "Receive debug %.*s\n", cmd->pingText.len, cmd->pingText.msg);
    }
}

// Q16.16定点数转换（带范围检查）
static inline float Q16_ToFloat(int32_t fixed)
{
    const float MAX_VALUE = 32767.99998f; // 0x7FFFFFFF / 65536.0
    const float MIN_VALUE = -32768.0f;
    return fmaxf(fminf((float)fixed / 65536.0f, MAX_VALUE), MIN_VALUE);
}

// Q8.7定点数转换（角度专用）
static inline float Q8_7_ToFloat(int16_t fixed)
{
    const float MAX_ANGLE = 90.0f; // 协议限制最大角度
    const float raw = (float)fixed / 128.0f;
    return fmaxf(fminf(raw, MAX_ANGLE), -MAX_ANGLE);
}

/**
 * @brief 将8位增益值转换为浮点数
 * @param gain 输入参数（0-255）
 * @return 映射到 0.0~1.0 的浮点值
 *
 * @note 边界处理：
 *   - gain=0   → 0.0
 *   - gain=255 → 1.0
 *   - 其他值按线性映射
 */
static inline float Gain8bit_ToFloat(uint8_t gain)
{
    return (float)gain / 255.0f;
}
