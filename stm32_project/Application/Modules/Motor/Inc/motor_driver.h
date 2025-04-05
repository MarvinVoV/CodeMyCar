/*
 * motor_driver.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#ifndef MODULES_MOTOR_INC_MOTOR_DRIVER_H_
#define MODULES_MOTOR_INC_MOTOR_DRIVER_H_

#include "pid_controller.h"
#include "motor_hal.h"

// 电机运行模式
typedef enum
{
    MOTOR_MODE_STOP,       // 停止模式
    MOTOR_MODE_OPEN_LOOP,  // 开环控制
    MOTOR_MODE_CLOSED_LOOP // 闭环控制
} MotorMode;


// 电机物理参数
typedef struct
{
    uint16_t encoderPPR; // [count/rev] 编码器每转脉冲数（未计算倍频）
    uint16_t gearRatio;  // [x:1] 减速比 （如30:1存储为30）
    float wheelRadiusMM; // [mm] 轮半径（毫米级精度）
    float maxRPM;        // [rpm] 最大转速
} MotorSpec;

// 电机实时状态
typedef struct
{
    struct
    {
        float radps; // [rad/s] 当前角速度，电机输出轴（即经过减速后的最终输出轴）的角速度
        float mps;   // [m/s] 当前线速度，轮子边缘的线速度
        float rpm;   // [RPM] 当前转速，电机输出轴（即经过减速后的最终输出轴）的转速
    } velocity;

    struct
    {
        int32_t totalPulses; // 累计脉冲数（原始数据）
        float revolutions;   // 累计转数
    } position;

    uint32_t lastUpdate; // 上次更新时间
    int32_t errorCode;   // 错误码
} MotorState;

// 电机驱动控制器
typedef struct
{
    // 硬件抽象层配置
    HAL_MotorConfig* hal_cfg;

    // 规格参数
    MotorSpec* spec;

    // 控制参数
    struct
    {
        MotorMode mode;             // 当前控制模式
        float targetRPM;            // 目标转速 单位是转每分钟，表示电机输出轴（即经过减速后的最终输出轴）的转速
        PID_Controller* pidControl; // PID参数
    } control;

    // 实时状态
    MotorState* state;

    // 滤波参数
    struct
    {
        uint8_t speedFilterDepth; // 速度滤波窗口大小
        float* speedFilterBuf;    // 滤波缓冲区
        uint8_t filterIndex;      // 滤波缓冲区索引
    } filter;
} MotorDriver;

/**
 * 初始化电机驱动
 * @param driver driver
 * @param initialPid 初始化PID参数
 */
void Motor_Init(MotorDriver* driver, const PID_Params* initialPid);

/**
 * @brief 设置电机控制模式
 *
 * @param driver 电机驱动对象指针
 * @param mode 控制模式枚举值：
 *             - MOTOR_MODE_STOP: 立即停止（释放PWM输出）
 *             - MOTOR_MODE_OPEN_LOOP: 开环速度控制
 *             - MOTOR_MODE_CLOSED_LOOP: 闭环PID速度控制
 *
 * @details 模式切换行为：
 * - 切换到STOP模式时会立即停止PWM输出
 * - 模式切换会重置积分项和微分历史状态
 * - 从闭环切到开环时会保留最后输出的占空比
 *
 * @warning 重要限制：
 * - 必须通过此函数切换模式，不可直接修改ctrl.mode字段
 * - 闭环模式需先配置有效PID参数，否则可能输出异常
 *
 * @example
 * // 启动闭环控制
 * Motor_SetMode(&motor, MOTOR_MODE_CLOSED_LOOP);
 *
 * // 紧急停机
 * Motor_SetMode(&motor, MOTOR_MODE_STOP);
 */
void Motor_SetMode(MotorDriver* driver, MotorMode mode);

/**
 * @brief 设置电机目标转速（RPM） (闭环模式)
 * @param driver 电机驱动对象指针，指向要控制的电机实例
 * @param rpm 电机输出轴目标转速值，单位为转/分钟（Revolutions Per Minute）
 *            正值为正转方向，负值为反转方向
 *
 * @note 此函数会自动将目标转速限制在电机规格允许的范围内：
 *       - 最大正向转速：+spec.max_rpm
 *       - 最大反向转速：-spec.max_rpm
 *       例如：若 spec.max_rpm = 300，则允许的 rpm 范围为 [-300, 300]
 */
void Motor_SetTargetRPM(MotorDriver* driver, float rpm);


/**
 * @brief 设置电机开环控制模式下的PWM占空比
 *
 * @param driver 电机驱动对象指针，指向已初始化的MotorDriver实例
 * @param duty PWM占空比（百分比），有效范围[-100.0, 100.0]
 *            - 正值表示正转方向，绝对值越大输出功率越高
 *            - 负值表示反转方向，绝对值越大输出功率越高
 *            - 0表示停止（具体行为取决于HAL_Motor_SetDirection的实现）
 *
 * @note 使用限制和注意事项：
 * 1. 仅在开环控制模式（MOTOR_MODE_OPEN_LOOP）下生效
 * 2. 函数内部会自动进行输入限幅保护，确保参数在安全范围内
 * 3. 会同步更新方向控制信号和PWM输出
 * 4. 底层最终会通过HAL_Motor_SetPWM接口设置硬件寄存器
 *
 * @code
 * // 典型使用示例：
 * Motor_SetMode(&motor, MOTOR_MODE_OPEN_LOOP);  // 先切换为开环模式
 * Motor_SetSpeed(&motor, 75.0f);  // 正转75%功率
 * Motor_SetSpeed(&motor, -30.5f); // 反转30.5%功率
 * @endcode
 *
 * @see Motor_SetTargetRPM 闭环转速控制函数
 * @see HAL_Motor_SetDirection 底层方向控制接口
 * @see HAL_Motor_SetPWM 底层PWM设置接口
 */
void Motor_SetSpeed(MotorDriver* driver, const float duty);


/**
 * @brief 电机控制主循环更新函数
 *
 * @param driver 电机驱动对象指针，指向已初始化的MotorDriver实例
 *
 * @details 功能描述：
 * 1. 计算距离上次调用的时间间隔（delta_time）
 * 2. 读取编码器脉冲增量值（delta_pulses）
 * 3. 计算实时转速（RPM）并进行滤波处理
 * 4. 在闭环模式下执行PID控制算法
 * 5. 根据计算结果更新PWM输出
 *
 * @note 注意事项：
 * - 必须周期性调用（建议控制周期5-20ms）
 * - 依赖HAL_GetTick()系统时钟，需确保时钟基准正确
 * - 在非闭环模式下仅更新状态数据，不改变PWM输出
 *
 * @code
 * // 典型用法（放在1kHz定时器中断中）
 * void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
 *     if (htim == &htim6) { // 1ms定时器
 *         static uint32_t cnt = 0;
 *         if (++cnt >= 10) { // 10ms控制周期
 *             Motor_Update(&motor);
 *             cnt = 0;
 *         }
 *     }
 * }
 * @endcode
 */
void Motor_Update(MotorDriver* driver);

/**
 * @brief 获取电机当前状态信息
 *
 * @param driver 电机驱动对象指针
 * @return MotorState 包含以下字段的状态结构体：
 *         - rpm: 滤波后的转速值（转/分）
 *         - velocity: 换算后的线速度（米/秒）
 *         - total_pulses: 累计编码器脉冲数
 *         - error_code: 错误标志位组合（见错误码定义）
 *
 * @details 数据特性：
 * - rpm经过移动平均滤波处理（滤波深度由filter.speed_filter_depth定义）
 * - velocity = (2π * rpm * wheel_radius) / 60
 * - total_pulses为绝对值累积，可用于里程计算
 *
 * @note 安全访问建议：
 * - 在RTOS环境中建议使用Motor_GetStateSafe()函数
 * - 读取时禁用中断保证数据原子性
 *
 * @example
 * // 打印当前转速
 * MotorState s = Motor_GetState(&motor);
 * printf("当前转速: %.2f RPM\n", s.rpm);
 */
MotorState Motor_GetState(MotorDriver* driver);

/**
 * @brief 执行电机紧急停止操作
 *
 * @param driver 电机驱动对象指针
 *
 * @details 执行动作：
 * 1. 立即切断PWM输出（HAL_Motor_SetPWM(0)）
 * 2. 激活硬件刹车模式（HAL_Motor_SetDirection刹车状态）
 * 3. 强制切换控制模式到STOP
 * 4. 设置错误标志位（ERROR_EMERGENCY_STOP）
 *
 * @warning 重要特性：
 * - 不可逆操作，需调用Motor_SetMode()恢复控制
 * - 优先级高于所有其他控制指令
 * - 会触发HAL层的硬件刹车逻辑（如有）
 *
 * @code
 * // 急停按钮中断回调
 * void EXTI15_10_IRQHandler(void) {
 *     if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13)) {
 *         Motor_EmergencyStop(&motor);
 *         __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
 *     }
 * }
 * @endcode
 */
void Motor_EmergencyStop(MotorDriver* driver);


/**
 * @brief 配置PID控制参数
 *
 * @param driver 电机驱动对象指针
 * @param params PID参数结构体指针，包含：
 *              - kp: 比例系数（建议范围0.1-10.0）
 *              - ki: 积分系数（建议范围0.01-2.0）
 *              - kd: 微分系数（建议范围0.0-1.0）
 *              - integral_max: 积分限幅（防饱和）
 *              - output_max: 输出限幅（百分比）
 *
 * @note 设计特性：
 * - 参数修改立即生效，建议在停止状态下调整参数
 * - 自动重置积分项和微分项，避免参数突变引起的震荡
 * - 线程安全（在RTOS环境中使用临界区保护）
 *
 * @warning 参数校验：
 * - 所有增益参数必须为非负数
 * - output_max范围[1.0, 100.0]
 * - integral_max建议设置为output_max的1.2-2倍
 *
 * @code
 * // 配置PID参数示例
 * PID_Params pid = {
 *     .kp = 1.2f,
 *     .ki = 0.3f,
 *     .kd = 0.05f,
 *     .integral_max = 80.0f,
 *     .output_max = 75.0f
 * };
 * Motor_ConfigurePID(&motor, &pid);
 * @endcode
 */
void Motor_ConfigurePID(MotorDriver* driver, const PID_Params* params);


#endif /* MODULES_MOTOR_INC_MOTOR_DRIVER_H_ */
