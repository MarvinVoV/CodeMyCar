/*
 * ctrl_protocol.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_
#define SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_

#include <stdint.h>

#pragma pack(push, 1)
typedef enum
{
    CTRL_MOTOR = 0x01, // 电机
    CTRL_SERVO,        // 舵机
    CTRL_DEBUG,        // 调试
    CTRL_MAX           // 结束值 - 仅用于判断
} ctrl_type_t;

typedef enum
{
    MOTOR_CMD_STOP = 0x00,      // 停止
    MOTOR_CMD_FORWARD = 0x01,   // 前进
    MOTOR_CMD_BACKWARD = 0x02,  // 后退
    MOTOR_CMD_TURN_LEFT = 0x03, // 左转
    MOTOR_CMD_TURN_RIGHT = 0x04 // 右转
} motor_command_t;

typedef struct
{
    uint8_t left_speed;   // 速度 (0-255)
    uint8_t right_speed;  // 速度 (0-255)
    uint8_t acceleration; // 加速度 0-100
} motor_data_t;

typedef struct
{
    uint8_t ctrl_id;   // 控制器ID
    uint8_t ctrl_type; // 控制类型 @see ctrl_type_t
    union
    {
        struct
        {
            // 电机控制
            motor_command_t cmd;
            motor_data_t data;
        } motor;

        struct
        {
            // 舵机控制
            uint8_t angle;
        } servo;

        struct
        {
            uint8_t msg[0];
        } debug_t;

        // ...其他外设结构
    } data;
} control_cmd_t;
#pragma pack(pop)

#endif /* SYSTEM_PROTOCOL_INC_CTRL_PROTOCOL_H_ */
