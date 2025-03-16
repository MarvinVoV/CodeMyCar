/*
 * ctrl_protocol.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_
#define SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#pragma pack(push, 1)

// ================ 控制字段位掩码定义 ================
typedef enum
{
    CTRL_FIELD_MOTOR = (1 << 0), // 电机控制有效
    CTRL_FIELD_SERVO = (1 << 1), // 舵机控制有效
    CTRL_FIELD_TEXT = (1 << 7)   // 调试信息有效
} ctrl_field_t;

// ========= Motor 控制指令相关结构体 ========
// 控制模
typedef enum
{
    CTRL_MODE_DIFFERENTIAL = 0x01, // 差速模式（高层控制）
    CTRL_MODE_DIRECT = 0x02,       // 直接控制模式（底层控制）
    CTRL_MODE_EMERGENCY = 0x80     // 紧急模式（最高优先级）
} motor_ctrl_mode_t;

// 差速控制参数
typedef struct
{
    int16_t linear_vel;  // 线速度（-3000~3000 mm/s）
    int16_t angular_vel; // 角速度（-1800~1800 0.1°/s）
    uint8_t accel;       // 加速度（0-100%）
} motor_diff_ctrl_t;

// 直接控制参数
typedef struct
{
    int16_t left_speed;  // 左电机速度（-1000~1000）
    int16_t right_speed; // 右电机速度（-1000~1000）
    uint8_t left_accel;  // 左加速度（0-100%）
    uint8_t right_accel; // 右加速度（0-100%）
} motor_direct_ctrl_t;

// 控制参数联合体
typedef union
{
    motor_diff_ctrl_t diff;     // 差速模式参数
    motor_direct_ctrl_t direct; // 直接控制参数
    // 紧急模式无需payload
} motor_motion_t;

// 控制指令结构体
typedef struct
{
    motor_ctrl_mode_t mode; // 控制模式 @see motor_ctrl_mode_t
    motor_motion_t motion;  // 控制参数
} motor_ctrl_t;

// ========= Servo 控制指令相关结构体 ========
typedef struct
{
    uint8_t angle; // 目标角度（0~180）
    uint8_t speed; // 转向速度（0-100%）
} servo_ctrl_t;


// ========= Debug相关结构体 ========
typedef struct
{
    uint8_t len;   // 文本长度
    uint8_t msg[]; // 柔性数组（C99特性）
} ping_text_t;

typedef struct
{
    uint8_t ctrl_id;       // 控制器ID
    uint8_t ctrl_fields;   // 控制字段（位掩码组合） @see ctrl_field_t
    servo_ctrl_t servo;    // 舵机控制参数
    motor_ctrl_t motor;    // 电机控制参数
    ping_text_t ping_text; // 调试信息
} control_cmd_t;
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
} cmd_error_t;


/**
 * @brief 检查是否包含指定控制字段
 * @param fields 当前控制字段掩码
 * @param field 要检查的字段掩码
 * @return 包含返回true，否则false
 */
bool is_ctrl_field_set(uint8_t fields, ctrl_field_t field);

/**
 * @brief 检查是否同时包含多个控制字段
 * @param fields 当前控制字段掩码
 * @param check_fields 要检查的字段掩码组合
 * @return 全部包含返回true，否则false
 */
bool are_ctrl_fields_set(uint8_t fields, uint8_t check_fields);

/**
 * @brief 将控制字段转换为可读字符串
 * @param fields 控制字段掩码
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 生成的字符串
 */
const char* ctrl_fields_to_str(uint8_t fields, char* buf, size_t buf_size);

/**
 * @brief 校验控制指令的完整性
 * @param cmd_payload 指令指针
 * @param total_len 接收到的数据总长度
 * @param out_err 输出详细错误码
 * @return 是否合法
 */
bool validate_control_cmd(const uint8_t* cmd_payload, uint16_t total_len, cmd_error_t* out_err);

#endif /* SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_ */
