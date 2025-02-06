/*
 * error_handle.c
 *
 *  Created on: Jan 19, 2025
 *      Author: marvin
 */


#include "string.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "uart_handle.h"
#include "error_handle.h"

static ErrorType lastError = ERROR_NONE; // 保存最近一次错误类型

void ErrorHandler_Init(void) {
    lastError = ERROR_NONE; // 初始化错误状态
}

void ErrorHandler_Handle(ErrorType errorType, const char *message) {
    // 记录错误类型
    lastError = errorType;

    // 用户可以根据需要添加错误处理逻辑
    // 例如，记录错误日志，点亮错误指示灯，或通过 UART 输出
    if (message) {
    	char data[128]; // 根据需要调整缓冲区大小
    	snprintf(data, sizeof(data), "Error: %s\n", message);

    	HAL_UART_Transmit(uart4Channel.huart, (uint8_t*) data, strlen(data), 1000);
    }

    // 禁用中断，进入死循环
    __disable_irq();
    while (1) {
        // 示例：点亮指示灯（需配置 LED 引脚）
        // HAL_GPIO_TogglePin(GPIOx, GPIO_PIN_x);
//        HAL_Delay(500);
    }
}

ErrorType ErrorHandler_GetLastError(void) {
    return lastError;
}

void ErrorHandler_Reset(void) {
    // 执行系统复位
    NVIC_SystemReset();
}
