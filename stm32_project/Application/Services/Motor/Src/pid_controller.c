/*
 * pic_controller.c
 *
 *  Created on: Mar 16, 2025
 *      Author: marvin
 */


#include <math.h>
#include <pid_controller.h>
#include "log_manager.h"

void PIDController_Init(PID_Controller* pid, const PID_Params* params)
{
    if (!pid || !params) return;
    // 安全初始化参数
    PID_Params safe_params = *params;
    safe_params.kp         = fmaxf(safe_params.kp, 0.0f);
    safe_params.ki         = fmaxf(safe_params.ki, 0.0f);
    safe_params.kd         = fmaxf(safe_params.kd, 0.0f);

    pid->params = safe_params;
    // 初始化状态
    memset(&pid->state, 0, sizeof(PID_State));
}

void PID_SetParameters(PID_Controller* pid, const PID_Params* params)
{
    if (!pid || !params) return;
    // 安全更新参数
    PID_Params safe_params = *params;
    safe_params.kp         = fmaxf(safe_params.kp, 0.0f);
    safe_params.ki         = fmaxf(safe_params.ki, 0.0f);
    safe_params.kd         = fmaxf(safe_params.kd, 0.0f);
    // 保留当前积分值（带限幅）
    const float prev_integral = pid->state.integral;
    pid->params               = safe_params;
    pid->state.integral       = CLAMP(prev_integral, -pid->params.integral_max, pid->params.integral_max);
}


float PID_Calculate(PID_Controller* pid, float setpoint, float measurement, float delta_time)
{
    // 参数检查
    if (!pid || delta_time <= 0.0f) return 0.0f;

    // 更新目标值
    const float prev_setpoint = pid->state.setpoint;
    pid->state.setpoint       = setpoint;

    // 计算当前误差
    const float error = setpoint - measurement;

    // 死区处理
    if (fabsf(error) <= pid->params.dead_zone)
    {
        pid->state.prev_measurement = measurement;
        pid->state.output           = 0.0f;
        return 0.0f;
    }

    // 零目标处理（仅重置积分）
    if (fabsf(setpoint) < ZERO_TARGET_THRESHOLD)
    {
        pid->state.integral = 0.0f;
        pid->state.output   = 0.0f;
        return 0.0f;
    }

    // 目标值突变检测
    const float setpoint_change = fabsf(setpoint - prev_setpoint);
    if (setpoint_change > pid->params.setpoint_change_threshold)
    {
        pid->state.integral = 0.0f; // 重置积分
    }

    // 比例项
    const float p_term = pid->params.kp * error;

    // 积分项（带条件激活和速率限制）
    if (fabsf(error) < pid->params.integral_threshold)
    {
        const float integral_increment = error * delta_time;
        pid->state.integral += integral_increment;
    }

    // 积分限幅
    pid->state.integral = CLAMP(pid->state.integral, -pid->params.integral_max, pid->params.integral_max);
    const float i_term  = pid->params.ki * pid->state.integral;

    // 微分项（基于误差变化率）
    float d_term = 0.0f;
    if (delta_time > 1e-6f && pid->params.kd > 0.0f)
    {
        // 计算前次误差
        const float prev_error = prev_setpoint - pid->state.prev_measurement;

        // 计算误差变化率
        const float error_derivative = (error - prev_error) / delta_time;
        d_term                       = pid->params.kd * error_derivative;
    }
    pid->state.prev_measurement = measurement;

    // 10. 合成输出
    float output = p_term + i_term + d_term;

    // 输出限幅和抗饱和
    const float max_output = pid->params.output_max;
    const float min_output = -pid->params.output_max;
    if (pid->params.anti_windup)
    {
        // 临时钳位输出
        const float clamped_output = CLAMP(output, min_output, max_output);

        // 计算输出超出量
        const float excess = (output > max_output)
                                 ? (output - max_output)
                                 : (output < min_output)
                                 ? (output - min_output)
                                 : 0.0f;

        // 反向积分修正
        if (fabsf(excess) > 1e-6f && pid->params.ki > 1e-6f)
        {
            pid->state.integral -= excess / pid->params.ki;
        }

        output = clamped_output;
    }
    else
    {
        output = CLAMP(output, min_output, max_output);
    }

    pid->state.output = output;
    return output;
}

void PID_Reset(PID_Controller* pid)
{
    if (!pid) return;
    pid->state.integral = 0.0f;
    pid->state.output   = 0.0f;
    // 保留prev_measurement和setpoint!
}
