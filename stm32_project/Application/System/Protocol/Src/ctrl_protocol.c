/*
 * ctrl_protocol.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "ctrl_protocol.h"

#include <stdbool.h>

bool validate_control_cmd(const uint8_t* payload, const uint16_t frame_len)
{
    // 基础长度校验（必须包含控制头）
    const uint8_t base_len = 2; // ctrl_id(1) + ctrl_type(1)
    if (frame_len <= base_len)
    {
        return false;
    }

    const control_cmd_t* cmd = (const control_cmd_t*)payload;

    // 控制类型范围校验
    if (cmd->ctrl_type >= CTRL_MAX || cmd->ctrl_type < CTRL_MOTOR)
    {
        return false;
    }
    return true;
}
