/*
 * pic_controller.c
 *
 *  Created on: Mar 16, 2025
 *      Author: marvin
 */


#include "pic_controller.h"
#include <math.h>


void PIDController_Init(PIDController* pid, const PIDParams* params)
{
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->params = *params;
}

float PIDController_Update(PIDController* pid, const float measurement, const float delta_time)
{
    if (delta_time < 1e-6f) return 0.0f; // 防止除零

    const float error = pid->setpoint - measurement;

    // 积分项（带抗饱和）
    pid->integral += error * delta_time;
    pid->integral = fmaxf(fminf(pid->integral, pid->params.integral_limit), -pid->params.integral_limit);

    // 微分项（测量值微分）
    const float deriv = (error - pid->prev_error) / delta_time;
    pid->prev_error = error;

    // 计算输出
    const float output = pid->params.Kp * error
        + pid->params.Ki * pid->integral
        + pid->params.Kd * deriv;

    return fmaxf(fminf(output, pid->params.output_limit), -pid->params.output_limit);
}

void PIDController_Reset(PIDController* pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
