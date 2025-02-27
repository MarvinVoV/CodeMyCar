/*
 * log_manager.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_LOG_INC_LOG_MANAGER_H_
#define MODULES_LOG_INC_LOG_MANAGER_H_

#include <stdbool.h>

#include "cmsis_os2.h"
#include "log_protocol.h"
#include "stm32h7xx_hal.h"

#ifndef LOG_LEVEL_FILTER
#define LOG_LEVEL_FILTER \
  (LOG_LEVEL_INFO | LOG_LEVEL_WARNING | LOG_LEVEL_ERROR | LOG_LEVEL_CRITICAL)
#endif

#ifndef LOG_MODULE_FILTER
#define LOG_MODULE_FILTER (LOG_MODULE_ALL)
#endif

#define LOG_MAX_CHANNELS 2
// 定义内存池和队列的容量
#define LOG_POOL_BLOCKS  5
extern osMemoryPoolId_t log_pool;           // 内存池句柄

/**
 * @brief 日志记录
 * @param level
 * @param module
 * @param format
 * @param ...
 */
void log_record(log_level_t level, log_module_t module, const char* format,
                ...);

#define LOG(level, module, ...)                                       \
  do {                                                                \
    if ((level & LOG_LEVEL_FILTER) && (module & LOG_MODULE_FILTER)) { \
      log_record(level, module, __VA_ARGS__);                         \
    }                                                                 \
  } while (0)

// 调试专用宏（立即输出到指定UART）
#define LOG_DEBUG_UART(huart, ...) \
  do { \
    if (huart != NULL) { \
      char _dbg_buf[128]; \
      snprintf(_dbg_buf, sizeof(_dbg_buf), __VA_ARGS__); \
      HAL_UART_Transmit(huart, (uint8_t*)_dbg_buf, strlen(_dbg_buf), 10); \
    } \
  } while(0)

// 预定义快捷日志宏
#define LOG_DEBUG(module, ...) LOG(LOG_LEVEL_DEBUG, module, __VA_ARGS__)
#define LOG_INFO(module, ...) LOG(LOG_LEVEL_INFO, module, __VA_ARGS__)
#define LOG_WARN(module, ...) LOG(LOG_LEVEL_WARNING, module, __VA_ARGS__)
#define LOG_ERROR(module, ...) LOG(LOG_LEVEL_ERROR, module, __VA_ARGS__)
#define LOG_CRITICAL(module, ...) LOG(LOG_LEVEL_CRITICAL, module, __VA_ARGS__)

typedef struct
{
    UART_HandleTypeDef* huart;  // 物理接口
    log_level_t level_filter;   // 通道级别过滤
    log_module_t module_filter; // 通道模块过滤
    bool direct_output;         // 是否直接输出（不经过队列）
} log_output_channel;

/**
 * @brief 注册日志输出通道
 * @param channel log channel
 */
void LogManager_AddOutputChannel(log_output_channel* channel);


/**
 * 初始化日志管理器
 */
void LogManager_Init(void);

#endif /* MODULES_LOG_INC_LOG_MANAGER_H_ */
