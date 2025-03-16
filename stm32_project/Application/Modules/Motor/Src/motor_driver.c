/*
 * motor_driver.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include "motor_driver.h"
#include <math.h>

#include "log_manager.h"

// 私有函数声明
static double calculate_rpm(int32_t delta_pulses, float delta_time,
                            int32_t encoder_ppr, float gear_ratio);
static uint32_t calculate_pulse(double duty, const TIM_HandleTypeDef* pwm_tim);

/**
 * 初始化电机
 * @param motor motor
 */
void Motor_Init(Motor* motor)
{
    // 参数配置
    motor->encoder_res = motor->encoder_res * 4; // 4倍频正交解码
    // 状态初始化
    motor->rpm = 0.0f;
    motor->total_pulses = 0;

    // 初始化HAL层
    HAL_Motor_InitPWM(motor->pwm_tim);
    // 启动编码器接口
    HAL_Motor_InitEncoder(motor->encoder_tim);
}

void Motor_SetSpeed(const Motor* motor, double duty)
{
    // 限幅保护
    duty = fmax(fmin(duty, 100.0), -100.0);

    // 设置方向
    const int8_t dir = (duty >= 0) ? 1 : -1;
    HAL_Motor_SetDirection(&motor->hal, dir);

    /*
     * 计算PWM占空比对应的实际脉冲宽度值,拆解逻辑如下：
     * 1. fabsf(duty) 获取占空比的绝对值，将负值转换为正值
     * 2. 将占空比百分比转换为系数（0.0~1.0范围）
     * 3. 获取定时器的自动重装载值（Autoreload Register），即PWM周期对应的计数值
     * 4. 将占空比系数与ARR相乘，得到PWM比较寄存器（CCR）的设定值，即：脉冲宽度 = 占空比系数 × PWM周期
     */
    const uint32_t pulse = calculate_pulse(duty, motor->pwm_tim);

    // TODO print pulse and duty
    LOG_INFO(LOG_MODULE_MOTOR, "Duty: %.1lf%%, Calculated Pulse: %lu, ARR: %lu\n",
             duty, pulse, motor->hal.pwm_tim->Instance->ARR);

    // 更新PWM占空比
    __HAL_TIM_SET_COMPARE(motor->hal.pwm_tim, motor->hal.pwm_ch, pulse);
}


void Motor_UpdateState(Motor* motor)
{
    static uint32_t last_tick = 0;
    static int32_t last_cnt = 0;

    // 获取当前计数器和时间
    const int32_t curr_cnt = __HAL_TIM_GET_COUNTER(motor->encoder_tim);
    const uint32_t curr_tick = HAL_GetTick();

    // 计算增量
    const float delta_time = (float)((int32_t)(curr_tick - last_tick)) / 1000.0f; // 转为秒
    int32_t delta_pulses = curr_cnt - last_cnt;

    // 处理16位计数器溢出
    if (delta_pulses > 0x7FFF)
    {
        delta_pulses -= 0xFFFF;
    }
    else if (delta_pulses < -0x7FFF)
    {
        delta_pulses += 0xFFFF;
    }

    // 计算转速
    motor->rpm = (float)calculate_rpm(delta_pulses, delta_time,
                                      motor->encoder_res, motor->gear_ratio);

    // 更新累计位置
    motor->total_pulses += delta_pulses;

    // 保存当前状态
    last_cnt = curr_cnt;
    last_tick = curr_tick;
}

/**
 * 转速计算核心算法
 * @param delta_pulses 脉冲变化量
 * @param delta_time 时间变化量
 * @param encoder_ppr 每转脉冲数
 * @param gear_ratio 减速比
 * @return
 */
static double calculate_rpm(const int32_t delta_pulses, const float delta_time,
                            const int32_t encoder_ppr, const float gear_ratio)
{
    if (delta_time < 0.001f) return 0.0f; // 防止除零

    // 计算公式：
    // RPM = (脉冲变化量 / 每转脉冲数) / 减速比 / 时间(分钟)
    return (delta_pulses / (double)encoder_ppr) // 电机轴转数
        / gear_ratio                            // 输出轴转数
        * (60.0f / delta_time);                 // 转为每分钟转速
}


// 安全计算PWM脉冲值（带防御性编程）
static uint32_t calculate_pulse(const double duty, const TIM_HandleTypeDef* pwm_tim)
{
    if (pwm_tim == NULL || pwm_tim->Instance == NULL) return 0;

    // 获取ARR值并转换为双精度
    const double arr = pwm_tim->Instance->ARR;

    // 计算并四舍五入
    const double ratio = fabs(duty) / 100.0;
    const double raw_pulse = ratio * arr;

    // 边界保护
    const double clamped_pulse = fmax(0.0, fmin(raw_pulse, arr));

    return (uint32_t)(round(clamped_pulse));
}
