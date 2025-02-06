/*
 * uart_manager.c
 *
 *  Created on: Jan 18, 2025
 *      Author: marvin
 */

#include <buffer_manager.h>

bool BufferManager_Init(UART_Channel *channel, UART_HandleTypeDef *huart,
		uint16_t txSize, uint16_t rxSize) {
	if (!channel || !huart)
		return false;

	channel->huart = huart;

	// 初始化发送缓冲区
	if (!RingBuffer_Init(&channel->txBuffer, txSize)) {
		return false;
	}

	// 初始化接收缓冲区
	if (!RingBuffer_Init(&channel->rxBuffer, rxSize)) {
		RingBuffer_Free(&channel->txBuffer);
		return false;
	}

	return true;
}

void BufferManager_Free(UART_Channel *channel) {
	if (!channel)
		return;

	RingBuffer_Free(&channel->txBuffer);
	RingBuffer_Free(&channel->rxBuffer);
}


