/*
 * motion_service.h
 *
 *  Created on: Apr 6, 2025
 *      Author: marvin
 */

#ifndef SERVICES_CTRL_INC_MOTION_SERVICE_H_
#define SERVICES_CTRL_INC_MOTION_SERVICE_H_
#include "ctrl_protocol.h"
#include "motor_service.h"
#include "servo_service.h"

typedef struct
{
    float trackWidthMM;         // 轮距
    float wheelbaseMM;          // 轴距（阿克曼模型）
    float wheelRadiusMM;        // 车轮半径
    float minTurnRadiusMM;      // 最小转弯半径（毫米） 阿克曼转弯约束
    float maxSteerAngleDeg;     // 最大转向角度 deg
    float maxAngularVelRad;     // 最大角速度rad/s
    float maxLinearVelocityMPS; // 最大线速度 m/s
} ChassisConfig;

//
// typedef enum
// {
//     MOTION_EMERGENCY_STOP = 0x00, // 紧急停止
//     MOTION_DIRECT_CONTROL = 0x01, // 直接控制模式
//     MOTION_DIFFERENTIAL = 0x02,   // 纯差速控制（舵机归中）
//     MOTION_STEER_ONLY = 0x03,     // 前轮转向模式（阿克曼转向几何模型）
//     MOTION_SPIN_IN_PLACE = 0x04,  // 原地旋转模式
//     MOTION_MIXED_STEER = 0x05     // 混合转向模式(舵机+差速)
// } MotionCtrlMode;

typedef struct
{
    MotionMode mode;

    union
    {
        // 开环控制参数
        struct
        {
            float leftDutyCycle;  // 左轮占空比 (-100.0 ~ 100.0)
            float rightDutyCycle; // 右轮占空比
        } openLoop;

        // 速度控制参数（差速模型）
        struct
        {
            float linearVelocity;  // 目标线速度 (m/s)
            float angularVelocity; // 目标角速度 (rad/s)
            bool clockwise;        // 旋转方向标识
        } velocityCtrl;

        // 阿克曼转向参数
        struct
        {
            float linearVelocity; // 目标线速度 (m/s)
            float steerAngle;     // 前轮转向角 (deg)
        } ackermannCtrl;

        // 直接转速控制
        struct
        {
            float leftRpm;    // 左轮目标转速
            float rightRpm;   // 右轮目标转速
            float steerAngle; // 前轮转向角 (deg)
        } directCtrl;

        // 混合模式扩展参数
        struct
        {
            float differentialGain; // 差速补偿系数 (0.0~1.0)
            float linearVelocity;   // 目标线速度 (m/s)
            float angularVelocity;  // 目标角速度 (rad/s)
            float steerAngle;       // 前轮转向角 (deg)
        } mixedCtrl;
    } params;
} MotionTarget;

// 新增运行时状态监控
typedef struct
{
    // --- 基础状态 ---
    MotionMode currentMode; // 当前控制模式
    uint32_t timestamp;     // 状态更新时间戳 (ms)

    // --- 执行器状态 ---
    struct
    {
        float leftRpm;           // 左轮实际转速
        float rightRpm;          // 右轮实际转速
        uint32_t leftErrorCode;  // 左电机错误码
        uint32_t rightErrorCode; // 右电机错误码
    } drive;

    struct
    {
        float actualAngle;  // 实际转向角 (deg)
        float targetAngle;  // 目标转向角 (deg)
        uint32_t errorCode; // 转向系统错误码
    } steering;

    // --- 运动学状态 ---
    struct
    {
        float linear;       // 整车线速度 (m/s)
        float angular;      // 整车角速度 (rad/s)
        float acceleration; // 当前加速度 (m/s²)
    } velocity;

    // --- 位置状态 ---
    struct
    {
        float x;             // X轴坐标 (m)
        float y;             // Y轴坐标 (m)
        float theta;         // 航向角 (rad)
        float leftOdometer;  // 左轮累计行程 (m)
        float rightOdometer; // 右轮累计行程 (m)
    } pose;

    // --- 错误管理 ---
    uint32_t lastError; // 最新错误码
} MotionState;

typedef struct
{
    MotorService* motorService;  // 电机控制实例
    SteerInstance* steerServo;   // 转向舵机实例
    ChassisConfig chassisConfig; // 底盘物理参数配置
    MotionTarget motionTarget;   // 运动目标参数
    MotionState state;           // 运动状态
    osMutexId_t targetMutex;     // 目标参数互斥锁
} MotionContext;


/**
 * @brief 初始化运动控制上下文
 * @param ctx 上下文指针
 * @param motor 电机服务实例
 * @param steer 转向服务实例
 * @param config 底盘配置参数
 * @return int 0=成功，负数=错误码
 */
int MotionContext_Init(MotionContext* ctx, MotorService* motor, SteerInstance* steer, const ChassisConfig* config);

//=============================== 控制模式 ================================
/**
 * @brief 设置运动控制模式（线程安全）
 * @param ctx 上下文指针
 * @param mode 目标控制模式
 * @return int 0成功，错误码见备注
 * @note 错误码：-1=无效模式，-2=模式切换被拒绝（如急停未解除）
 */
int MotionService_SetControlMode(MotionContext* ctx, MotionMode mode);

/**
 * @brief 获取当前运动控制模式（线程安全）
 * @param ctx 运动控制上下文指针
 * @return MotionMode 当前控制模式
 */
MotionMode MotionService_GetCurrentMode(const MotionContext* ctx);


//=============================== 运动模式 ================================
/**
 * @brief 设置差速/混合模式运动参数（线程安全）
 * @param ctx 上下文指针
 * @param linear 目标线速度 (m/s)，范围[-maxLinear, +maxLinear]
 * @param angular 目标角速度 (rad/s)，范围[-maxAngular, +maxAngular]
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 输入含NaN/INF
 *         MOTION_ERR_MODE_MISMATCH - 当前非差速/混合模式
 * @note 实际参数将被钳位至安全范围，触发MOTION_ERR_PARAM_CLAMPED警告
 */
int MotionService_SetVelocity(MotionContext* ctx, float linear, float angular);

/**
 * @brief 设置直接控制参数（线程安全）
 * @param ctx 上下文指针
 * @param leftRpm 左轮目标转速 (RPM)，范围[±motorMaxRpm]
 * @param rightRpm 右轮目标转速 (RPM)
 * @param steerAngle 转向角 (deg)，范围[-maxSteer, +maxSteer]
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_OVER_CURRENT - 任一电机过流
 *         MOTION_ERR_MODE_MISMATCH - 当前非直接控制模式
 * @warning 直接绕过运动学模型，需谨慎使用
 */
int MotionService_SetDirectControl(MotionContext* ctx, float leftRpm, float rightRpm, float steerAngle);

/**
 * @brief 设置混合转向控制参数（线程安全）
 * @param ctx 上下文指针
 * @param linear 目标线速度 (m/s)
 * @param angular 目标角速度 (rad/s)
 * @param steerAngle 目标转向角 (deg)，范围[-maxSteer, +maxSteer]
 * @param steeringBlend 转向混合比例 [0.0, 1.0]
 *                      - 0.0: 纯差速控制（忽略转向角）
 *                      - 1.0: 纯转向控制（忽略角速度）
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 无效参数
 *         MOTION_ERR_MODE_MISMATCH - 当前非混合模式
 */
int MotionService_SetHybridParams(MotionContext* ctx, float linear, float angular, float steerAngle,
                                  float steeringBlend);
/**
 * @brief 设置阿克曼转向模式参数（线程安全）(MOTION_STEER_ONLY/MIXED模式)
 * @param ctx 运动控制上下文指针
 * @param linearVel 目标线速度（m/s）
 * @param steerAngle 目标转向角度（度）
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 无效参数
 *         MOTION_ERR_MODE_MISMATCH - 当前模式不兼容
 */
int MotionService_SetAckermannParams(MotionContext* ctx, float linearVel, float steerAngle);
/**
 * @brief 设置原地旋转参数（线程安全）
 * @param ctx 上下文指针
 * @param angularVel 目标角速度 (rad/s)
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 无效参数
 *         MOTION_ERR_MODE_MISMATCH - 当前非原地旋转模式
 */
int MotionService_SetSpinParams(MotionContext* ctx, float angularVel);

/**
 * @brief 触发紧急停止（线程安全）
 * @param ctx 上下文指针
 */
int MotionService_EmergencyStop(MotionContext* ctx);

/**
 * @brief 设置转向角度（线程安全） （STEER_ONLY/MIXED模式）
 * @param ctx 运动控制上下文指针
 * @param angleDeg 转向角度（-max_angle ~ +max_angle）
 * @return 错误码：
 *         MOTION_ERR_NONE - 成功
 *         MOTION_ERR_INVALID_PARAM - 无效参数
 *         MOTION_ERR_MODE_MISMATCH - 当前模式不支持转向控制
 */
int MotionService_SetSteerAngle(MotionContext* ctx, float angleDeg);

/**
 * @brief 获取运动状态快照（原子操作）
 * @param ctx 上下文指针
 * @param[out] outState 状态结构体指针
 */
void MotionService_GetState(const MotionContext* ctx, MotionState* outState);

/**
 * @brief 清除错误状态（退出急停前必须调用）
 * @param ctx 上下文指针
 * @return int 0=成功，-1=仍有未恢复故障
 */
int MotionService_ClearFaults(MotionContext* ctx);


/**
 * @brief 运行运动控制服务 task中调用
 * @param ctx context
 */
void MotionService_Run(MotionContext* ctx);


#endif /* SERVICES_CTRL_INC_MOTION_SERVICE_H_ */
