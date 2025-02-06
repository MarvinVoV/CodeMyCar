/*
 * error_handle.h
 *
 *  Created on: Jan 19, 2025
 *      Author: marvin
 */

#ifndef INC_ERROR_HANDLE_H_
#define INC_ERROR_HANDLE_H_


#include <stdint.h>

// 错误类型枚举
typedef enum {
    ERROR_NONE = 0,
    ERROR_UART,
    ERROR_DMA,
    ERROR_MEMORY,
    ERROR_UNKNOWN
} ErrorType;

// 初始化异常处理模块
void ErrorHandler_Init(void);

// 通用错误处理函数
void ErrorHandler_Handle(ErrorType errorType, const char *message);

// 获取最后一个错误
ErrorType ErrorHandler_GetLastError(void);

// 用户实现的系统复位
void ErrorHandler_Reset(void);


#endif /* INC_ERROR_HANDLE_H_ */
