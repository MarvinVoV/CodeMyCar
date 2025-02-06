/*
 * uart_manager.h
 *
 *  Created on: Jan 18, 2025
 *      Author: marvin
 */

#ifndef INC_UART_MANAGER_H_
#define INC_UART_MANAGER_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "ring_buffer.h"




typedef struct {
    UART_HandleTypeDef *huart;      // UART 句柄
    RingBuffer_t txBuffer;       // 发送缓冲区
    RingBuffer_t rxBuffer;       // 接收缓冲区
} UART_Channel;

bool BufferManager_Init(UART_Channel *channel, UART_HandleTypeDef *huart, uint16_t txSize, uint16_t rxSize);

void BufferManager_Free(UART_Channel *channel);

void UART_Send(UART_Channel *channel, const uint8_t *data, uint16_t length);




#endif /* INC_UART_MANAGER_H_ */
