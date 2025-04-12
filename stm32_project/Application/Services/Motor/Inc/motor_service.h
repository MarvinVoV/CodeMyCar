/*
 * motor_service.h
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#ifndef SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#define SERVICES_MOTOR_INC_MOTOR_SERVICE_H_
#include <stdint.h>

#include "cmsis_os2.h"
#include "motor_driver.h"
#include "pid_controller.h"

typedef enum
{
    MOTOR_ERR_EMERGENCY_STOP = 0x01,
    MOTOR_ERR_INVALID_PARAM,
    MOTOR_ERR_OPEN_LOOP_UNSUPPORTED, // 驱动不支持开环模式
    MOTOR_ERR_DUTY_OUT_OF_RANGE      // 占空比超限
} MotorErrorCode;

typedef enum
{
    MOTOR_LEFT_WHEEL = 0, // 左轮电机
    MOTOR_RIGHT_WHEEL,    // 右轮电机
    MOTOR_MAX_NUM
} MotorID;

/**
 * @brief 电机状态
 */
typedef struct
{
    struct
    {
        float mps; // 当前线速度 (m/s)
        float rpm; // 当前转速角，电机输出轴转速
    } velocity;

    struct
    {
        float revolutions; // 累计转数
        float odometer;    // 累计里程 (m)
    } position;

    // 开环控制状态字段
    float openLoopDuty;   // 当前占空比（0.0~1.0）
    MotorDriverMode mode; // 当前控制模式

    uint32_t lastUpdate; // 上次更新时间
    uint32_t errorCode;  // 错误码
} MotorState;

/**
 * @brief 电机实例
 */
typedef struct
{
    MotorDriver* driver; // 电机驱动实例
    MotorState state;    // 状态信息
    osMutexId_t mutex;   // 实例级互斥锁
} MotorInstance;

typedef struct
{
    MotorInstance instances[MOTOR_MAX_NUM]; // 电机实例数组
    uint32_t controlFreqHz;                 // 控制频率（Hz）
} MotorService;

/**
 * @brief 初始化电机实例
 * @param service 电机服务层实例
 * @param leftDriver 左轮驱动实例
 * @param rightDriver 右轮驱动实例
 *
 * @note 该函数不会分配内存，需确保传入的结构体已正确初始化
 */
void MotorService_init(MotorService* service, MotorDriver* leftDriver, MotorDriver* rightDriver);

/**
 * @brief 设置电机实例转速（线程安全）
 * @param service 电机服务层实例
 * @param motorId  电机ID
 * @param rpm 目标转速(转/分钟)
 *
 */
int MotorService_setTargetRPM(MotorService* service, MotorID motorId, float rpm);
/**
 * @brief 设置电机开环占空比
 * @param service 电机服务实例
 * @param motorId 电机ID
 * @param dutyCycle 占空比（范围：0.0 ~ 1.0）
 * @return 0成功，负数错误码
 */
int MotorService_setOpenLoopDutyCycle(MotorService* service, MotorID motorId, float dutyCycle);

/**
 * @brief 立即执行紧急停止
 * @param service 电机服务层实例
 * @param motorId  电机ID
 *
 * @warning 该操作会切断电机输出并锁定控制器状态，需手动调用恢复函数
 */
int MotorService_emergencyStop(MotorService* service, MotorID motorId);

void MotorService_updateState(MotorService* service);

/**
 * @brief 获取当前状态信息（只读）
 * @param service 电机服务层实例
 * @param motorId  电机ID
 * @return 状态副本
 */
MotorState MotorService_getState(MotorService* service, MotorID motorId);

/**
 * 动态调整PID参数（通过消息队列）
 * @param service 电机服务层实例
 * @param motorId  电机ID
 * @param pidParams 新的PID参数
 */
int MotorService_configurePID(MotorService* service, MotorID motorId, const PID_Params* pidParams);


#endif /* SERVICES_MOTOR_INC_MOTOR_SERVICE_H_ */
