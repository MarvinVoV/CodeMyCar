/*
 * log_manager.h
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */

#ifndef MODULES_LOG_INC_LOG_MANAGER_H_
#define MODULES_LOG_INC_LOG_MANAGER_H_

#include "log_protocol.h"

// 日志过滤配置（编译时可配置）
#ifndef LOG_LEVEL_FILTER
#define LOG_LEVEL_FILTER \
  (LOG_LEVEL_INFO | LOG_LEVEL_WARNING | LOG_LEVEL_ERROR | LOG_LEVEL_CRITICAL)
#endif

#ifndef LOG_MODULE_FILTER
#define LOG_MODULE_FILTER (LOG_MODULE_ALL)
#endif

// 核心日志记录函数（带格式化支持）
void log_record(log_level_t level, log_module_t module, const char* file,
                uint16_t line, const char* format, ...);

// 带代码位置的日志宏
#define LOG(level, module, ...)                                       \
  do {                                                                \
    if ((level & LOG_LEVEL_FILTER) && (module & LOG_MODULE_FILTER)) { \
      log_record(level, module, __FILE__, __LINE__, __VA_ARGS__);     \
    }                                                                 \
  } while (0)

// 预定义快捷日志宏
#define LOG_DEBUG(module, ...) LOG(LOG_LEVEL_DEBUG, module, __VA_ARGS__)
#define LOG_INFO(module, ...) LOG(LOG_LEVEL_INFO, module, __VA_ARGS__)
#define LOG_WARN(module, ...) LOG(LOG_LEVEL_WARNING, module, __VA_ARGS__)
#define LOG_ERROR(module, ...) LOG(LOG_LEVEL_ERROR, module, __VA_ARGS__)
#define LOG_CRITICAL(module, ...) LOG(LOG_LEVEL_CRITICAL, module, __VA_ARGS__)

/**
 * 初始化日志管理器
 */
void log_manager_init(void);
#endif /* MODULES_LOG_INC_LOG_MANAGER_H_ */
