/*
 * uart_handle.h
 *
 *  Created on: Feb 4, 2025
 *      Author: marvin
 */

#ifndef INC_UART_HANDLE_H_
#define INC_UART_HANDLE_H_

#include <stdint.h>
#include <stdbool.h>

#include "log_protocol.h"
#include "stm32h7xx_hal.h"
#include "uart_protocol.h"

// 定义UART缓冲区大小
#define BUF_SIZE (PROTOCOL_MAX_DATA_LEN + PROTOCOL_HEADER_SIZE + 1)

// 声明接收和发送缓冲区
extern uint8_t rx_buffer[BUF_SIZE];
extern uint8_t tx_buffer[BUF_SIZE];

// 公共接口
void UART_Init(void);

/**
 * 特指UART4，负责和ESP32通信
 */
bool UART_Send_To_Esp(uint8_t *data, uint16_t length, protocol_type_t type);

/**
 * 发送日志
 */
bool Send_Log(log_entry_t *log_entry);


#endif /* INC_UART_HANDLE_H_ */
