/*
 * log_protocol.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_PROTOCOL_INC_LOG_PROTOCOL_H_
#define SYSTEM_PROTOCOL_INC_LOG_PROTOCOL_H_

#include <stdint.h>
#include <string.h>

#include "pkt_protocol.h"

// 日志级别定义（按bit位设计，支持组合过滤）
typedef enum
{
    LOG_LEVEL_DEBUG = 0x01,    // 调试信息
    LOG_LEVEL_INFO = 0x02,     // 常规信息
    LOG_LEVEL_WARNING = 0x04,  // 警告信息
    LOG_LEVEL_ERROR = 0x08,    // 错误信息
    LOG_LEVEL_CRITICAL = 0x10, // 严重错误
    LOG_LEVEL_ALL = 0xFF       // 所有级别
} log_level_t;

// 模块标识定义（按bit位设计，支持多模块组合）
typedef enum
{
    LOG_MODULE_SYSTEM = 0x0001,  // 系统模块
    LOG_MODULE_MOTOR = 0x0002,   // 电机控制
    LOG_MODULE_SENSOR = 0x0004,  // 传感器
    LOG_MODULE_NETWORK = 0x0008, // 网络通信
    LOG_MODULE_DRIVER = 0x0010,  // 设备驱动
    LOG_MODULE_ALL = 0xFFFF      // 所有模块
} log_module_t;

#pragma pack(push, 1)
typedef struct
{
    uint32_t timestamp;                  // 时间戳（单位ms）
    uint16_t module;                     // 模块标识（log_module_t）
    uint8_t level;                       // 日志级别（log_level_t）
    char message[PROTOCOL_MAX_DATA_LEN]; // 日志正文
} log_entry_t;
#pragma pack(pop)

// 日志协议控制参数
#define LOG_MAX_LENGTH PROTOCOL_MAX_DATA_LEN
#define LOG_QUEUE_SIZE 16                 // 日志队列缓冲数量
#define LOG_TIMESTAMP_BASE HAL_GetTick()  // 时间戳基准

#endif /* SYSTEM_PROTOCOL_INC_LOG_PROTOCOL_H_ */
