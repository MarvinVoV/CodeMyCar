/*
 * motor_driver.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include <math.h>

#include "motor_driver.h"
#include "log_manager.h"

static float calculate_rpm(const MotorDriver* driver, int32_t delta_pulses, float delta_time);


static uint32_t calculate_pulse(double duty, const TIM_HandleTypeDef* pwm_tim);
static void update_speed_filter(MotorDriver* driver, float new_speed);
static float apply_pid_control(MotorDriver* driver, float current_rpm, float delta_time);

void Motor_Init(MotorDriver* driver, HAL_MotorConfig* hal_cfg, const MotorSpec* spec)
{
    // 参数检查
    if (!driver || !hal_cfg || !spec) return;

    // 硬件配置
    driver->hal_cfg = hal_cfg;

    // 规格参数
    memcpy(&driver->spec, spec, sizeof(MotorSpec));
    driver->spec.encoder_ppr *= 4; // 正交编码4倍频

    // 控制参数初始化
    driver->ctrl.mode = MOTOR_MODE_STOP;
    driver->ctrl.target_rpm = 0.0f;
    driver->ctrl.last_error = 0.0f;
    driver->ctrl.integral = 0.0f;

    // 状态初始化
    memset(&driver->state, 0, sizeof(MotorState));

    // 初始化HAL层
    HAL_Motor_Init(hal_cfg);
}

void Motor_SetMode(MotorDriver* driver, const MotorMode mode)
{
    if (!driver) return;

    driver->ctrl.mode = mode;

    // 模式切换时重置积分项
    driver->ctrl.integral = 0.0f;
    driver->ctrl.last_error = 0.0f;

    if (mode == MOTOR_MODE_STOP)
    {
        HAL_Motor_SetPWM(driver->hal_cfg, 0);
    }
}

void Motor_Update(MotorDriver* driver)
{
    if (!driver || driver->ctrl.mode == MOTOR_MODE_STOP)
        return;

    static uint32_t last_update = 0;
    const uint32_t current_tick = HAL_GetTick();
    // 单位: 秒
    const float delta_time = (float)(current_tick - last_update) / 1000.0f;

    if (delta_time < 0.001f)
        return; // 最小时间间隔1ms

    // 1. 读取编码器
    const int32_t current_pulses = HAL_Motor_ReadEncoder(driver->hal_cfg);
    const int32_t delta_pulses = current_pulses - driver->state.total_pulses;

    // 2. 计算转速
    const float rpm = calculate_rpm(driver, delta_pulses, delta_time);
    driver->state.total_pulses = current_pulses;

    // 3. 滤波处理
    update_speed_filter(driver, rpm);

    // 4. 闭环控制
    if (driver->ctrl.mode == MOTOR_MODE_CLOSED_LOOP)
    {
        const float output = apply_pid_control(driver, driver->state.rpm, delta_time);
        const float duty = (output / driver->spec.max_rpm) * 100.0f; // 转换为占空比
        Motor_SetSpeed(driver, duty);
    }

    last_update = current_tick;
}

void Motor_SetSpeed(const MotorDriver* driver, float duty)
{
    // 1. 有效性检查：确保驱动对象存在且处于开环模式
    if (!driver || driver->ctrl.mode != MOTOR_MODE_OPEN_LOOP) return;

    // 2. 占空比限幅：限制在[-100%, 100%]范围内
    duty = fmaxf(fminf(duty, 100.0f), -100.0f);

    // 3. 方向控制：根据占空比正负设置电机转向
    const MotorDirection dir = (duty >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    HAL_Motor_SetDirection(driver->hal_cfg, dir);


    // 4. 计算并设置PWM脉冲值
    const uint32_t pulse = calculate_pulse(duty, driver->hal_cfg->pwm.tim);
    HAL_Motor_SetPWM(driver->hal_cfg, pulse);
}

void Motor_SetTargetRPM(MotorDriver* driver, const float rpm)
{
    if (!driver) return;

    // 限幅保护
    driver->ctrl.target_rpm = fmaxf(fminf(rpm, driver->spec.max_rpm), -driver->spec.max_rpm);
}


void Motor_EmergencyStop(MotorDriver* driver)
{
    if (!driver) return;

    HAL_Motor_EmergencyStop(driver->hal_cfg);
    driver->ctrl.mode = MOTOR_MODE_STOP;
    driver->state.error_code |= 0x01;
}


void Motor_ConfigurePID(MotorDriver* driver, const PID_Params* params)
{
    // 参数有效性检查
    if (driver == NULL || params == NULL)
    {
        return;
    }



    // 拷贝PID参数到控制器
    memcpy(&driver->ctrl.pid_params, params, sizeof(PID_Params));

    // 重置积分项和微分历史状态
    driver->ctrl.integral = 0.0f;
    driver->ctrl.last_error = 0.0f;

}

/**
 * 转速计算核心算法
 * @param driver driver
 * @param delta_pulses 脉冲变化量
 * @param delta_time 时间变化量 单位：秒
 * @return
 */
static float calculate_rpm(const MotorDriver* driver, const int32_t delta_pulses, const float delta_time)
{
    if (delta_time < 0.001f) return 0.0f; // 防止除零; delta_time 单位: 秒
    // 输出轴每转一圈对应的总编码器脉冲数
    const float pulses_per_rev = (float)driver->spec.encoder_ppr * driver->spec.gear_ratio;
    // 转/分钟
    return ((float)delta_pulses / pulses_per_rev) * (60.0f / delta_time);
}

/**
 * @brief PID控制器计算函数
 * @param driver       电机驱动对象指针，包含PID参数和状态
 * @param current_rpm 当前实际转速（单位：RPM）
 * @param delta_time  距离上次计算的时间间隔（单位：秒）
 * @return float      经过PID计算后的控制输出（范围：[-output_max, output_max]）
 *
 * @note 控制器实现特点：
 * 1. 标准位置式PID算法
 * 2. 积分抗饱和处理
 * 3. 输出限幅保护
 * 4. 微分项采用误差差分法（避免设定值突变导致的微分冲击）
 */
static float apply_pid_control(MotorDriver* driver, const float current_rpm, const float delta_time)
{
    // 1. 计算转速偏差（目标值 - 实际值）
    const float error = driver->ctrl.target_rpm - current_rpm;

    // 2. 比例项计算：P = Kp × e(t)
    const float p_term = driver->ctrl.pid_params.kp * error;

    // 3. 积分项计算（带抗饱和）：
    //    a. 累计误差积分：I = ∫e(t)dt ≈ Σe(t)×Δt
    //    b. 积分限幅防止windup
    driver->ctrl.integral += error * delta_time;
    driver->ctrl.integral = fmaxf(fminf(driver->ctrl.integral, driver->ctrl.pid_params.integral_max),
                                  -driver->ctrl.pid_params.integral_max);
    const float i_term = driver->ctrl.pid_params.ki * driver->ctrl.integral;

    // 4. 微分项计算：D = Kd × de(t)/dt ≈ Kd × (e(t)-e(t-1))/Δt
    //    使用误差差分而非测量值差分，避免设定值突变引起的微分冲击
    const float d_term = driver->ctrl.pid_params.kd * (error - driver->ctrl.last_error) / delta_time;
    driver->ctrl.last_error = error;

    // 5. 合成输出：Output = P + I + D
    const float output = p_term + i_term + d_term;

    // 6. 输出限幅保护，确保输出在安全范围内
    return fmaxf(fminf(output, driver->ctrl.pid_params.output_max),
                 -driver->ctrl.pid_params.output_max);
}

/**
 * @brief 更新电机速度滤波并计算线速度
 * @param driver     电机驱动对象指针
 * @param new_speed 未经滤波的原始转速值（单位：RPM）
 *
 * @功能说明：
 * 1. 执行移动平均滤波：当滤波深度>1时，将新速度存入环形缓冲区，计算窗口平均值
 * 2. 更新状态转速值：存储滤波后的转速值到driver->state.rpm
 * 3. 计算线速度：根据公式 v = ωr = (2π*rpm/60)*r 转换角速度为线速度
 *
 * @实现细节：
 * - 使用环形缓冲区实现滑动窗口，缓冲区索引自动回绕
 * - 滤波深度通过driver->filter.speed_filter_depth配置
 * - 当滤波深度<=1时直接透传原始速度值
 * - 线速度单位：米/秒（取决于wheel_radius的单位）
 */
static void update_speed_filter(MotorDriver* driver, const float new_speed)
{
    if (driver->filter.speed_filter_depth > 1)
    {
        // 移动平均滤波
        driver->filter.speed_filter_buf[driver->filter.filter_index] = new_speed;
        driver->filter.filter_index = (driver->filter.filter_index + 1) % driver->filter.speed_filter_depth;

        float sum = 0.0f;
        for (int i = 0; i < driver->filter.speed_filter_depth; i++)
        {
            sum += driver->filter.speed_filter_buf[i];
        }
        driver->state.rpm = sum / (float)driver->filter.speed_filter_depth;
    }
    else
    {
        driver->state.rpm = new_speed;
    }

    // 计算线速度：v = ω * r = (2π * rpm / 60) * r  单位：转/秒
    driver->state.velocity = (2.0f * (float)M_PI * driver->state.rpm / 60.0f) * driver->spec.wheel_radius;
}


static uint32_t calculate_pulse(const double duty, const TIM_HandleTypeDef* pwm_tim)
{
    if (pwm_tim == NULL || pwm_tim->Instance == NULL) return 0;

    // 动态获取ARR值
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(pwm_tim);

    // 计算并四舍五入
    const double ratio = fabs(duty) / 100.0;
    const double raw_pulse = ratio * arr;

    // 边界保护
    const double clamped_pulse = fmax(0.0, fmin(raw_pulse, arr));

    return (uint32_t)(round(clamped_pulse));
}
