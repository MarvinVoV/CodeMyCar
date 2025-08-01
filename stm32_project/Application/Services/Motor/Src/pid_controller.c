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

    // 参数安全初始化
    pid->params.kp = fmaxf(params->kp, 0.0f);
    pid->params.ki = fmaxf(params->ki, 0.0f);
    pid->params.kd = fmaxf(params->kd, 0.0f);
    pid->params.output_max = fmaxf(params->output_max, 0.0f);
    pid->params.dead_zone = fmaxf(params->dead_zone, 0.0f);

    // 初始化状态
    memset(&pid->state, 0, sizeof(PID_State));
    pid->state.first_run = true;
}


float PID_Calculate(PID_Controller* pid, float setpoint, float measurement, float delta_time)
{
    // 参数检查
    if (!pid || delta_time <= 0.0f) return 0.0f;

    // 计算当前误差
    float error           = setpoint - measurement;

    // 死区处理
    if (fabsf(error) < pid->params.dead_zone)
    {
        error = 0.0f;
    }

    // 首次运行初始化
    if (pid->state.first_run) {
        pid->state.prev_error = error;
        pid->state.prev_measurement = measurement;
        pid->state.prev_output = 0;
        pid->state.first_run = false;
        return 0.0f; // 首次运行返回0
    }

    // 1. 比例项增量：Kp * (e[k] - e[k-1])
    const float p_term = pid->params.kp * (error - pid->state.prev_error);

    // 2. 积分项增量：Ki * e[k] * Δt
    const float i_term = pid->params.ki * error * delta_time;

    // 微分项：Kd * (测量值变化率)
    float d_term = 0.0f;
    if (pid->params.kd > 0.0f && delta_time > 1e-6f) {
        const float derivative = (pid->state.prev_measurement - measurement) / delta_time;
        d_term = pid->params.kd * derivative;
    }

    // 5. 计算输出增量
    const float delta_output = p_term + i_term + d_term;
    // 6. 计算新输出
    float output = pid->state.prev_output + delta_output;

    // 7. 输出限幅
    output = CLAMP(output, -pid->params.output_max, pid->params.output_max);
    // 8. 更新状态
    pid->state.prev_error = error;
    pid->state.prev_measurement = measurement;
    pid->state.prev_output = output;


    return output;
}

void PID_Reset(PID_Controller* pid)
{
    if (!pid) return;
    memset(&pid->state, 0, sizeof(PID_State));
    pid->state.first_run = true;
}
