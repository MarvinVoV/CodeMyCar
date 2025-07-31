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
    pid->state.prev_measurement = 0.0f;
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

    // 计算当前误差
    const float error = setpoint - measurement;

    // 比例项
    const float p_term = pid->params.kp * error;

    // 积分项 - 添加积分分离
    if (fabsf(error) > pid->params.integral_threshold) {
        // 大误差时不累积积分
        pid->state.integral += 0;
    } else {
        // 小误差时正常积分
        pid->state.integral += error * delta_time;
    }

    // 积分限幅
    pid->state.integral = CLAMP(pid->state.integral, -pid->params.integral_max, pid->params.integral_max);
    const float i_term  = pid->params.ki * pid->state.integral;

    // 微分项（基于测量值变化）
    float d_term = 0.0f;
    if (delta_time > 1e-6f && pid->params.kd > 0.0f)
    {
        const float derivative = (pid->state.prev_measurement - measurement) / delta_time;
        d_term                 = pid->params.kd * derivative;
    }
    pid->state.prev_measurement = measurement;

    // 合成输出
    float output = p_term + i_term + d_term;

    // 输出限幅
    output = CLAMP(output, -pid->params.output_max, pid->params.output_max);

    return output;
}

void PID_Reset(PID_Controller* pid)
{
    if (!pid) return;
    pid->state.integral   = 0.0f;
    pid->state.prev_error = 0.0f;
}


