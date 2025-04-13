/*
 * ctrl_protocol.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_
#define SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#pragma pack(push, 1)

/*---------------------- 控制掩码定义 ----------------------*/
typedef enum
{
    CTRL_FIELD_MOTION = (1 << 0), // 运动控制有效
    CTRL_FIELD_TEXT = (1 << 7)    // 调试信息有效
} CtrlField;


/*---------------------- 高层运动控制定义 ----------------------*/
typedef enum
{
    MOTION_EMERGENCY_STOP = 0x00, // 紧急停止
    MOTION_DIRECT_CONTROL = 0x01, // 直接控制模式
    MOTION_DIFFERENTIAL = 0x02,   // 纯差速控制（舵机归中）
    MOTION_STEER_ONLY = 0x03,     // 前轮转向模式（阿克曼转向几何模型）
    MOTION_SPIN_IN_PLACE = 0x04,  // 原地旋转模式
    MOTION_MIXED_STEER = 0x05     // 混合转向模式(舵机+差速)
} MotionCtrlMode;

// 差速控制参数（兼容混合模式）
typedef struct
{
    int32_t linearVel;  // Q16.16 (m/s)
    int32_t angularVel; // Q16.16 (rad/s)
} DiffCtrlParam;

// 直接控制参数
typedef struct
{
    int32_t leftRpm;    // Q16.16 (RPM)
    int32_t rightRpm;   // Q16.16 (RPM)
    int16_t steerAngle; // Q8.7 (deg)
} DirectCtrlParam;

// 阿克曼转向参数
typedef struct
{
    int32_t linearVel;  // Q16.16 (m/s)
    int16_t steerAngle; // Q8.7 (deg)
} AckermannParam;

// 混合控制参数
typedef struct
{
    DiffCtrlParam base;       // 基础差速参数
    int16_t steerAngle;       // Q8.7 (deg)
    uint8_t differentialGain; // 0-255(映射0.0-1.0)
} MixedCtrlParam;

// 运动控制主指令
typedef struct
{
    MotionCtrlMode mode; // 运动模式
    union
    {
        AckermannParam kinematicCtrl;
        DirectCtrlParam directCtrl;
        MixedCtrlParam mixedCtrl;
        DiffCtrlParam diffCtrl;
    } params;

    uint16_t duration; // 执行持续时间（ms）
} MotionCmd;


/*---------------------- 独立设备控制指令 ----------------------*/

typedef struct
{
    uint8_t len;   // 文本长度
    uint8_t msg[]; // 柔性数组（C99特性）
} PingText;


/*---------------------- 主控制指令 ----------------------*/
typedef struct
{
    uint8_t ctrlId;    // 控制器ID
    uint8_t fields;    // 控制字段（位掩码组合）
    MotionCmd motion;  // 运动控制参数
    PingText pingText; // 调试信息 debug
} ControlCmd;
#pragma pack(pop)


// 错误码定义
typedef enum
{
    CMD_ERR_NONE = 0,           // 无错误
    CMD_ERR_INVALID_LENGTH,     // 数据长度不合法
    CMD_ERR_RESERVED_FIELD,     // 保留字段被设置
    CMD_ERR_INVALID_MOTOR_MODE, // 电机模式不合法
    CMD_ERR_MOTOR_PARAM,        // 电机参数越界
    CMD_ERR_SERVO_PARAM,        // 舵机参数越界
    CMD_ERR_TEXT_LENGTH,        // 文本长度不合法
    CMD_ERR_FIELD_CONFLICT      // 字段冲突
} ControlCmdError;


/**
 * @brief 检查是否包含指定控制字段
 * @param fields 当前控制字段掩码
 * @param field 要检查的字段掩码
 * @return 包含返回true，否则false
 */
bool isCtrlFieldSet(uint8_t fields, CtrlField field);

/**
 * @brief 检查是否同时包含多个控制字段
 * @param fields 当前控制字段掩码
 * @param checkFields 要检查的字段掩码组合
 * @return 全部包含返回true，否则false
 */
bool isCtrlFieldsSet(uint8_t fields, uint8_t checkFields);


/**
 * @brief 校验控制指令的完整性
 * @param cmdPayload 指令指针
 * @param totalLen 接收到的数据总长度
 * @param error 输出详细错误码
 * @return 是否合法
 */
bool validateControlCmd(const uint8_t* cmdPayload, uint16_t totalLen, ControlCmdError* error);

#endif /* SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_ */
