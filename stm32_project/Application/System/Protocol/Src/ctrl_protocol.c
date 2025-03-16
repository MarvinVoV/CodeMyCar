/*
 * ctrl_protocol.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "ctrl_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

/**
 * @brief 检查是否包含指定控制字段
 * @param fields 当前控制字段掩码
 * @param field 要检查的字段掩码
 * @return 包含返回true，否则false
 */
bool is_ctrl_field_set(const uint8_t fields, const ctrl_field_t field)
{
    return (fields & field) == field;
}


/**
 * @brief 检查是否同时包含多个控制字段
 * @param fields 当前控制字段掩码
 * @param check_fields 要检查的字段掩码组合
 * @return 全部包含返回true，否则false
 */
bool are_ctrl_fields_set(const uint8_t fields, const uint8_t check_fields)
{
    return (fields & check_fields) == check_fields;
}


/**
 * @brief 验证控制字段组合是否有效
 * @param fields 待验证的字段掩码
 * @param allowed_combinations 允许的字段组合掩码数组
 * @param num_combinations 数组长度
 * @return 有效返回true，否则false
 */
bool validate_ctrl_fields(const uint8_t fields, const uint8_t allowed_combinations[], const size_t num_combinations)
{
    for (size_t i = 0; i < num_combinations; i++)
    {
        if (fields == allowed_combinations[i])
        {
            return true;
        }
    }
    return false;
}


/**
 * @brief 检查是否存在冲突的字段组合
 * @param fields 当前字段掩码
 * @param conflict_masks 冲突掩码数组（每个元素表示互斥的字段组合）
 * @param num_conflicts 数组长度
 * @return 存在冲突返回true，否则false
 */
bool has_conflict_fields(const uint8_t fields, const uint8_t conflict_masks[], const size_t num_conflicts)
{
    for (size_t i = 0; i < num_conflicts; i++)
    {
        if ((fields & conflict_masks[i]) == conflict_masks[i])
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查保留位是否被错误使用
 * @param fields 当前字段掩码
 * @return 保留位被设置返回true，否则false
 */
bool has_reserved_fields(const uint8_t fields)
{
    const uint8_t reserved_mask = 0x7C; // 保留位掩码（BIT2~BIT6）
    return (fields & reserved_mask) != 0;
}

/**
 * @brief 将控制字段转换为可读字符串
 * @param fields 控制字段掩码
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 生成的字符串
 */
const char* ctrl_fields_to_str(const uint8_t fields, char* buf, const size_t buf_size)
{
    static const char* field_names[] = {
        "MOTOR", "SERVO",
        "RESV2", "RESV3", "RESV4", "RESV5", "RESV6",
        "TEXT"
    };

    char* p = buf;
    size_t remaining = buf_size;

    for (int i = 0; i < 8; i++)
    {
        if (fields & (1 << i))
        {
            const int written = snprintf(p, remaining, "%s|", field_names[i]);
            if (written >= remaining) break;
            p += written;
            remaining -= written;
        }
    }

    if (p > buf) *(p - 1) = '\0'; // 移除末尾的'|'
    else *buf = '\0';

    return buf;
}

bool validate_control_cmd(const uint8_t* cmd_payload, const uint16_t total_len, cmd_error_t* out_err)
{
    // ===== 基础长度校验 =====
    const uint8_t base_len = 2; // ctrl_id(1) + ctrl_fields(1)
    if (total_len <= base_len)
    {
        *out_err = CMD_ERR_INVALID_LENGTH;
        return false;
    }
    const control_cmd_t* cmd = (const control_cmd_t*)cmd_payload;

    // ===== 控制字段校验 =====
    // 检查保留位（BIT2~BIT6为保留位）0b01111100
    const uint8_t reserved_mask = 0x7C;
    if (cmd->ctrl_fields & reserved_mask)
    {
        *out_err = CMD_ERR_RESERVED_FIELD;
        return false;
    }

    // ===== 舵机控制校验 =====
    if (cmd->ctrl_fields & CTRL_FIELD_SERVO)
    {
        if (cmd->servo.angle > 180)
        {
            *out_err = CMD_ERR_SERVO_PARAM;
            return false;
        }
        if (cmd->servo.speed > 100)
        {
            *out_err = CMD_ERR_SERVO_PARAM;
            return false;
        }
    }

    // ===== 电机控制校验 =====
    if (cmd->ctrl_fields & CTRL_FIELD_MOTOR)
    {
        // 模式校验
        switch (cmd->motor.mode)
        {
            case CTRL_MODE_DIFFERENTIAL:
                // 差速模式参数校验
                if (cmd->motor.motion.diff.accel > 100)
                {
                    *out_err = CMD_ERR_MOTOR_PARAM;
                    return false;
                }
                break;

            case CTRL_MODE_DIRECT:
                // 直接模式参数校验
                if (cmd->motor.motion.direct.left_accel > 100 ||
                    cmd->motor.motion.direct.right_accel > 100)
                {
                    *out_err = CMD_ERR_MOTOR_PARAM;
                    return false;
                }
                break;

            case CTRL_MODE_EMERGENCY:
                // 紧急模式无需参数校验
                break;

            default:
                *out_err = CMD_ERR_INVALID_MOTOR_MODE;
                return false;
        }
    }
    return true;
}
