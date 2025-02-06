#pragma once
#include <stdint.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// 环形缓冲区大小
#define RING_BUFFER_SIZE (512)
// TX RX 缓冲区大小
#define UART_BUF_SIZE (RING_BUFFER_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化UART模块
 * @param config UART配置参数
 */
void uart_init();

/**
 * @brief 向UART发送数据
 * @param data 数据指针
 * @param len 数据长度
 * @return 是否发送成功
 */
bool uart_send_data(const uint8_t* data, size_t len);


#ifdef __cplusplus
}
#endif