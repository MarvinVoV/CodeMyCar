/*
 * pic_controller.c
 *
 *  Created on: Mar 16, 2025
 *      Author: marvin
 */


#include <pid_controller.h>


void PIDController_Init(PID_Controller* pid, const PID_Params* params)
{
    if (!pid || !params) return;

    pid->params = *params;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
void PID_SetParameters(PID_Controller* pid, const PID_Params* params)
{
    if (!pid || !params) return;
    pid->params = *params;
}


float PID_Calculate(PID_Controller* pid, float measurement, float delta_time)
{
    if (!pid || delta_time <= 0.0f) return 0.0f;

    // 计算误差
    const float error = pid->setpoint - measurement;

    // ======= 比例项 =======
    const float p_term = pid->params.kp * error;

    // ======= 积分项 =======
    pid->integral += error * delta_time;
    // 积分限幅
    pid->integral = CLAMP(pid->integral, -pid->params.integral_max, pid->params.integral_max);
    const float i_term = pid->params.ki * pid->integral;

    // ======= 微分项 =======
    float d_term = 0.0f;
    if (delta_time > 1e-6f) { // 防止除以0
        const float derivative = (error - pid->prev_error) / delta_time;
        d_term = pid->params.kd * derivative;
    }
    pid->prev_error = error; // 保存当前误差

    // ======= 合成输出 =======
    float output = p_term + i_term + d_term;
    return CLAMP(output, -pid->params.output_max, pid->params.output_max);
}

void PID_Reset(PID_Controller* pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
