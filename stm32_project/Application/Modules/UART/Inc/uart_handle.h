/*
 * uart_handle.h
 *
 *  Created on: Feb 4, 2025
 *      Author: marvin
 */

#ifndef INC_UART_HANDLE_H_
#define INC_UART_HANDLE_H_

#include <pkt_protocol.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "log_protocol.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

// 定义UART缓冲区大小
#define UART_RX_BUFFER_SIZE (PROTOCOL_MAX_DATA_LEN * 4)

// extern __ALIGN_BEGIN uint8_t rxBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;
// extern __ALIGN_BEGIN uint8_t txBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;

typedef struct
{
    const uint8_t* data;
    uint16_t len;
} uart_msg_t;


typedef struct
{
    uint8_t buffer[2][UART_RX_BUFFER_SIZE] __ALIGNED(32);
    volatile uint8_t current; // 当前接收缓冲区索引
    volatile uint16_t length; // 有效数据长度
    SemaphoreHandle_t semaphore; // 二值信号量
} uart_dma_buffer_t;

#define QUEUE_ITEM_SIZE sizeof(uart_msg_t*) // 存储数据指针

extern uart_dma_buffer_t uart_dma_buffer;
void init_uart_dma(void);

/**
  * @brief  处理接收到的 UART 数据
  * @param  data 指向接收到数据的缓冲区
  * @param  len  接收到的数据字节数
  * @retval None
  */
void process_uart_data(const uint8_t* data, uint16_t len);


/**
 * 特指UART4，负责和ESP32通信
 */
bool UART_Send_To_Esp(uint8_t* data, uint16_t length, protocol_type_t type);

/**
 * 发送日志
 */
bool Send_Log(log_entry_t* log_entry);


#endif /* INC_UART_HANDLE_H_ */
