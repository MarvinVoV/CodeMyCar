/*
 * motor_driver.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */


#include "motor_driver.h"
#include "log_manager.h"

/// 最小有效时间间隔 (1ms)
#define MIN_DELTA_TIME_S 0.001f

/// 默认速度滤波窗口大小
#ifndef DEFAULT_SPEED_FILTER_WINDOW
#define DEFAULT_SPEED_FILTER_WINDOW 5
#endif


//------------------------------------------------------------------------------
// 静态函数声明
//------------------------------------------------------------------------------
/**
 * @brief 将转速从RPM转换为rad/s
 * @param rpm 转速 [RPM]
 * @return float 角速度 [rad/s]
 */
static inline float rpm_to_radps(float rpm);
/**
 * @brief 将角速度从rad/s转换为RPM
 * @param radps 角速度 [rad/s]
 * @return float 转速 [RPM]
 */
static inline float radps_to_rpm(float radps);

/**
 * @brief 将角速度转换为线速度
 * @param radps 角速度 [rad/s]
 * @param wheel_radius_mm 轮半径 [mm]
 * @return float 线速度 [m/s]
 */
static inline float radps_to_mps(float radps, float wheel_radius_mm);


/**
 * @brief 将编码器脉冲数转换为旋转弧度值
 *
 * @param pulses 编码器脉冲数，单位：个（count）
 *              物理意义：四倍频后的累计脉冲数（包含方向信息）
 * @param encoder_ppr 编码器每转脉冲数，单位：脉冲/转（Pulses Per Revolution）
 *                  物理意义：编码器未倍频时每转输出的脉冲数
 * @param gear_ratio 减速比，单位：无因次量（如30:1减速比则传入30.0）
 * @return float 旋转角度值，单位：弧度（rad）
 *
 * @note 转换公式：
 * radians = (pulses × 2π) / (4 × PPR × gearRatio)
 * 四倍频说明：编码器每个脉冲周期产生4个计数（上升沿+下降沿×2通道），所以，4 * PPR 表示电机转一圈产生的总脉冲数（四倍频后）
 * 示例：encoderPPR=13, gearRatio=30, pulses=1560时：
 * (1560 × 6.2832) / (4 × 13 × 30) = 6.2832 rad（正好1转）
 */
static inline float pulses_to_rad(int32_t pulses, uint16_t encoder_ppr, uint16_t gear_ratio);

/**
 * @brief 更新累计转数
 * @param state 状态结构体
 * @param delta_pulses 脉冲增量
 * @param encoder_ppr 编码器每转脉冲数
 * @param gear_ratio 减速比
 */
static void update_revolutions(MotorDriverState* state,
                               int32_t           delta_pulses,
                               uint16_t          encoder_ppr,
                               uint16_t          gear_ratio);

/**
 * @brief 应用速度滤波
 * @param driver 驱动控制器
 * @param raw_speed 原始速度值
 * @return float 滤波后速度
 */
static float apply_speed_filter(MotorDriver* driver, float raw_speed);

/**
 *  计算脉冲增量
 * @param driver  驱动控制器
 * @param current_raw 当前原始脉冲数
 * @return int32_t 脉冲增量
 */
static int32_t calculate_delta_pulses(const MotorDriver* driver, uint32_t current_raw);

//------------------------------------------------------------------------------
// 公共API实现
//------------------------------------------------------------------------------

bool MotorDriver_Init(MotorDriver* driver, const PID_Params* initialPid)
{
    // 参数检查
    if (driver == NULL || driver->halCfg == NULL || driver->spec == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver initialization failed");
        return false;
    }

    if (driver->spec->encoderPPR == 0 || driver->spec->gearRatio == 0)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Invalid encoderPPR or gearRatio");
        return false;
    }

    PIDController_Init(&driver->control.pidCtrl, initialPid);

    // 初始化滤波系统
    driver->filter.windowSize   = DEFAULT_SPEED_FILTER_WINDOW;
    driver->filter.filterBuffer = (float*)pvPortMalloc(driver->filter.windowSize * sizeof(float));
    if (driver->filter.filterBuffer == NULL)
    {
        LOG_WARN(LOG_MODULE_MOTOR, "Speed filter buffer allocation failed");
        driver->filter.windowSize = 0;
    }
    else
    {
        // 初始化缓冲区
        memset(driver->filter.filterBuffer, 0, driver->filter.windowSize * sizeof(float));
        driver->filter.bufferIndex = 0;
    }

    // 状态初始化
    memset(&driver->state, 0, sizeof(MotorDriverState));
    driver->state.mode           = MOTOR_DRIVER_MODE_STOP;
    driver->state.lastUpdateTick = HAL_GetTick();

    // 初始化HAL层
    if (HAL_Motor_Init(driver->halCfg) != HAL_OK)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "HAL motor initialization failed");
        return false;
    }

    return true;
}

void MotorDriver_SetMode(MotorDriver* driver, const MotorDriverMode mode)
{
    if (driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    // 模式切换处理
    if (driver->control.mode != mode)
    {
        switch (mode)
        {
            case MOTOR_DRIVER_MODE_CLOSED_LOOP:
                // 重置PID状态
                PID_Reset(&driver->control.pidCtrl);
                break;
            case MOTOR_DRIVER_MODE_OPEN_LOOP:
                // 保留当前占空比
                break;
            case MOTOR_DRIVER_MODE_STOP:
                HAL_Motor_EmergencyStop(driver->halCfg);
                break;
        }
        driver->control.mode         = mode;
        driver->state.lastUpdateTick = HAL_GetTick();
    }
}

void MotorDriver_SetDutyCycle(MotorDriver* driver, const float duty)
{
    if (driver == NULL || driver->state.mode != MOTOR_DRIVER_MODE_OPEN_LOOP)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Invalid call to SetDutyCycle");
        return;
    }

    // 限制占空比范围
    const float clamped_duty = (duty > 100.0f) ? 100.0f : (duty < -100.0f) ? -100.0f : duty;

    // 设置HAL层占空比
    HAL_Motor_SetDuty(driver->halCfg, clamped_duty);

    // 更新状态
    driver->state.openLoopDuty = clamped_duty;
}

void MotorDriver_SetTargetRPM(MotorDriver* driver, const float rpm)
{
    if (driver == NULL)
    {
        return;
    }
    // 限制在规格范围内
    const float max_rpm       = driver->spec->maxRPM;
    driver->control.targetRPM = (rpm > max_rpm) ? max_rpm : (rpm < -max_rpm) ? -max_rpm : rpm;
}

void MotorDriver_Update(MotorDriver* driver)
{
    if (driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    // 更新模式
    driver->state.mode = driver->control.mode;

    // 获取时间增量（毫秒）
    const uint32_t current_tick = HAL_GetTick();
    const uint32_t last_tick    = driver->state.lastUpdateTick;

    // 计算时间增量（秒）
    float delta_time_s = 0.0f;
    if (current_tick > last_tick)
    {
        delta_time_s = (float)(current_tick - last_tick) * 0.001f;
    }

    // 更新时间太短则跳过
    if (delta_time_s < MIN_DELTA_TIME_S)
    {
        return;
    }

    // 获取当前编码器脉冲值
    const uint32_t current_raw = HAL_Motor_ReadRawEncoder(driver->halCfg);

    // 计算脉冲增量（处理计数器溢出）
    const int32_t delta_pulses = calculate_delta_pulses(driver, current_raw);

    // 更新原始值存储
    driver->state.position.lastRawPulses = current_raw;

    // 计算实际脉冲率（脉冲/秒）
    float pulses_per_sec = (float)delta_pulses / delta_time_s;

    // 更新位置信息（必须在速度计算后）
    driver->state.position.totalPulses += delta_pulses;
    update_revolutions(&driver->state, delta_pulses, driver->spec->encoderPPR, driver->spec->gearRatio);

    // 计算实际转速（RPM）计算公式: RPM = (脉冲率 × 60) / (PPR × 减速比)
    const float actual_rpm = (pulses_per_sec * 60.0f) /
                             ((float)driver->spec->encoderPPR * (float)driver->spec->gearRatio);

    // 更新速度状态
    driver->state.velocity.rpm   = actual_rpm;
    driver->state.velocity.radps = rpm_to_radps(actual_rpm);
    driver->state.velocity.mps   = radps_to_mps(driver->state.velocity.radps, driver->spec->wheelRadiusMM);

    // 更新时间戳
    driver->state.lastUpdateTick = current_tick;


    // 闭环控制处理
    if (driver->state.mode == MOTOR_DRIVER_MODE_CLOSED_LOOP)
    {
        // 计算最大脉冲率（用于标准化）
        const float max_pulses_rate = (driver->spec->maxRPM *
                                       (float)driver->spec->encoderPPR *
                                       (float)driver->spec->gearRatio) / 60.0f;

        // 计算目标脉冲率（脉冲/秒）
        const float target_pulses_per_sec = (driver->control.targetRPM *
                                             (float)driver->spec->gearRatio *
                                             (float)driver->spec->encoderPPR) / 60.0f;

        // 标准化输入（-1到1范围）
        const float normalized_target = target_pulses_per_sec / max_pulses_rate;
        const float normalized_actual = pulses_per_sec / max_pulses_rate;

        // 执行PID计算
        const float output = PID_Calculate(&driver->control.pidCtrl, normalized_target,
                                           normalized_actual, delta_time_s);

        // 应用输出
        const float clamped_output = (output > 1.0f) ? 1.0f : (output < -1.0f) ? -1.0f : output;

        HAL_Motor_SetDuty(driver->halCfg, clamped_output * 100.0f);

        // PID debug
        static uint32_t last_print = 0;
        if (HAL_GetTick() - last_print >= 20) { // 50Hz采样率
            last_print = HAL_GetTick();
            // 格式：目标值,实际值,输出值\n
            LOG_INFO(LOG_MODULE_SYSTEM, "%.2f,%.2f,%.2f\n",normalized_target,normalized_actual, clamped_output);
        }
    }
}


void MotorDriver_EmergencyStop(MotorDriver* driver)
{
    if (driver == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver is NULL");
        return;
    }

    HAL_Motor_EmergencyStop(driver->halCfg);
    driver->control.mode = MOTOR_DRIVER_MODE_STOP;
    PID_Reset(&driver->control.pidCtrl);
}

MotorDriverState MotorDriver_getState(const MotorDriver* driver)
{
    MotorDriverState state = {0};
    if (driver != NULL)
    {
        // 创建状态副本
        memcpy(&state, &driver->state, sizeof(MotorDriverState));
    }
    return state;
}

void MotorDriver_ConfigurePID(MotorDriver* driver, const PID_Params* params)
{
    if (!driver || !params)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor driver or PID params is NULL");
        return;
    }

    // 拷贝PID参数到控制器
    memcpy(&driver->control.pidCtrl.params, params, sizeof(PID_Params));
    // 重置积分项和微分历史状态
    PID_Reset(&driver->control.pidCtrl);
}

//------------------------------------------------------------------------------
// 静态函数实现
//------------------------------------------------------------------------------

/**
 * @brief 将转速从转每分钟（RPM）转换为弧度每秒（rad/s）
 *
 * @param rpm 电机输出轴转速值，单位：转/分钟（Revolutions Per Minute）
 *           物理意义：电机输出轴（减速后）的每分钟转数
 * @return float 电机输出轴角速度值，单位：弧度/秒（rad/s）
 *
 * @note 转换公式：
 * rad/s = (RPM × 2π) / 60
 */
static inline float rpm_to_radps(const float rpm)
{
    return rpm * (2.0f * (float)M_PI) / 60.0f;
}


/**
 * @brief 将输出轴的角速度（rad/s）转换为输出轴的转速（RPM）
 *
 * @param radps 输出轴角速度，单位：弧度/秒（rad/s）
 * @return float 输出轴转速，单位：转/分钟（RPM）
 *
 * @note 公式：
 * RPM = (radps × 60) / (2π)
 * 示例：当 radps = 10.472 rad/s（≈ 100 RPM）时：
 * (10.472 × 60) / 6.283 ≈ 100 RPM
 */
static inline float radps_to_rpm(const float radps)
{
    return radps * 60.0f / (2.0f * (float)M_PI);
}


/**
 * @brief 将角速度转换为线速度
 *
 * @param radps 角速度值，单位：弧度/秒（rad/s）
 * @param wheel_radius_mm 轮子半径，单位：毫米（mm）
 * @return float 线速度值，单位：米/秒（m/s）
 *
 * @note 转换公式：
 * mps = radps × (wheelRadiusMM / 1000.0f)
 * 物理意义：轮缘线速度 = 角速度 × 轮半径（米制）
 *
 * 示例 当radps=10 rad/s，wheelRadiusMM=65 mm时：
 * 10 × (65 / 1000) = 0.65 m/s
 */
static inline float radps_to_mps(const float radps, const float wheel_radius_mm)
{
    return radps * (wheel_radius_mm / 1000.0f);
}


/**
 * @brief 将编码器脉冲数转换为旋转弧度值
 *
 * @param pulses 编码器脉冲数，单位：个（count）
 *              物理意义：四倍频后的累计脉冲数（包含方向信息）
 * @param encoder_ppr 编码器每转脉冲数，单位：脉冲/转（Pulses Per Revolution）
 *                  物理意义：编码器未倍频时每转输出的脉冲数
 * @param gear_ratio 减速比，单位：无因次量（如30:1减速比则传入30.0）
 * @return float 旋转角度值，单位：弧度（rad）
 *
 * @note 转换公式：
 * radians = (pulses × 2π) / (4 × PPR × gearRatio)
 * 四倍频说明：编码器每个脉冲周期产生4个计数（上升沿+下降沿×2通道）
 * 示例：encoderPPR=13, gearRatio=30, pulses=1560时：
 * (1560 × 6.2832) / (4 × 13 × 30) = 6.2832 rad（正好1转）
 */
static inline float pulses_to_rad(const int32_t pulses, const uint16_t encoder_ppr, const uint16_t gear_ratio)
{
    return ((float)pulses * (float)M_PI * 2.0f) / (4.0f * (float)encoder_ppr * (float)gear_ratio);
}


/**
 * @brief 更新累计转数值
 *
 * @param state 电机状态对象指针
 * @param delta_pulses 增量脉冲数，单位：个（count）
 * @param encoder_ppr 编码器每转脉冲数，单位：脉冲/转（Pulses Per Revolution）
 * @param gear_ratio 减速比，单位：无因次量
 *
 * @note 转换过程：
 * 1. 脉冲数转弧度：pulseToRad()
 * 2. 弧度转转数：radians / 2π
 * 公式：
 * revolutions += (deltaPulses × π × 2) / (4 × PPR × gearRatio) / (2π)
 * 简化为：
 * revolutions += deltaPulses / (4 × PPR × gearRatio)
 */
static void update_revolutions(MotorDriverState* state, const int32_t        delta_pulses,
                               const uint16_t    encoder_ppr, const uint16_t gear_ratio)
{
    state->position.revolutions += (float)delta_pulses / (4.0f * (float)encoder_ppr * (float)gear_ratio);
}

static float apply_speed_filter(MotorDriver* driver, float raw_speed)
{
    // 未启用滤波
    if (driver->filter.windowSize == 0 || driver->filter.filterBuffer == NULL)
    {
        return raw_speed;
    }

    // 添加新采样值
    driver->filter.filterBuffer[driver->filter.bufferIndex] = raw_speed;

    // 移动平均滤波
    float sum = 0.0f;
    for (uint8_t i = 0; i < driver->filter.windowSize; i++)
    {
        sum += driver->filter.filterBuffer[i];
    }

    // 更新索引
    driver->filter.bufferIndex = (driver->filter.bufferIndex + 1) % driver->filter.windowSize;

    return sum / (float)driver->filter.windowSize;
}

//  计算脉冲增量
int32_t calculate_delta_pulses(const MotorDriver* driver, const uint32_t current_raw)
{
    const uint32_t last_raw = driver->state.position.lastRawPulses;
    const uint32_t maxValue = HAL_Motor_GetEncoderMaxValue(driver->halCfg);
    const uint32_t halfMax  = maxValue / 2;

    int32_t delta = (int32_t)current_raw - (int32_t)last_raw;

    if (delta > halfMax)
    {
        delta -= (int32_t)(maxValue + 1);
    }
    else if (delta < -(int32_t)halfMax)
    {
        delta += (int32_t)(maxValue + 1);
    }
    return delta;
}
