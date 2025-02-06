/*
 * uart_handle.c
 *
 *  Created on: Jan 18, 2025
 *      Author: marvin
 */

#include <string.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "error_handle.h"
#include "uart_handle.h"

// UART 通道实例
UART_Channel uart4Channel;

// 接收缓冲区大小
//#define RX_BUFFER_SIZE 512
//#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 5
#define TX_BUFFER_SIZE 5
// DMA 接收缓冲区大小
#define DMA_RX_BUFFER_SIZE 128

// DMA 接收临时缓冲区
uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE];

// UART 初始化
void UART_Handle_Init(UART_HandleTypeDef *huart) {
	// 初始化 UART4 的缓冲区管理
	if (!BufferManager_Init(&uart4Channel, huart, TX_BUFFER_SIZE, RX_BUFFER_SIZE)) {
		// 错误处理：初始化失败
		ErrorHandler_Handle(ERROR_UART, "BufferManager_Init error occurred");
	}
	// 启用 UART 空闲中断
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	// 开启 DMA 接收模式
	HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBuffer, DMA_RX_BUFFER_SIZE);
	if (status != HAL_OK) {
		// 错误处理：DMA 启动失败
		ErrorHandler_Handle(ERROR_UART,
				"HAL_UARTEx_ReceiveToIdle_DMA error occurred");
	}
}

/**
 * UART 空闲中断回调函数
 * NOTE: 放在单独的文件中，必须引入对应的头文件，使得编译器能够识别要覆盖的弱函数;
 * 这里需要引入的头文件如下：
 * 1. stm32h7xx_hal.h - 前置依赖
 * 2. stm32h7xx_hal_uart.h  - 定义了回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == uart4Channel.huart) {
		// 通过 DMA 计数器获取实际接收数据长度
		uint16_t receivedDataLength = DMA_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(huart->hdmarx);

		// 将 DMA 缓冲区数据批量写入 RingBuffer
		if (!RingBuffer_Write(&uart4Channel.rxBuffer, dmaRxBuffer, receivedDataLength)) {
			// 如果写入失败（缓冲区空间不足），可进行错误处理或丢弃数据
			ErrorHandler_Handle(ERROR_UART, "RingBuffer_Write error occurred");
		}

		// 重新启动 DMA 接收
		if (HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBuffer, DMA_RX_BUFFER_SIZE)
				!= HAL_OK) {
			// 错误处理：DMA 启动失败
			ErrorHandler_Handle(ERROR_UART, "Restart HAL_UARTEx_ReceiveToIdle_DMA error occurred");
		}
	}
}

void UART_Process_Data() {
	uint8_t tempBuffer[128];
	uint16_t dataLength;

	char digestBuffer[128];
	char dataBuffer[128];

	while (!RingBuffer_IsEmpty(&uart4Channel.rxBuffer)) {
		RingBuffer_digest(&uart4Channel.rxBuffer, digestBuffer, sizeof(digestBuffer), dataBuffer, sizeof(dataBuffer));
		UART_Send(&uart4Channel, (uint8_t *)digestBuffer, strlen(digestBuffer));


		dataLength = RingBuffer_Read(&uart4Channel.rxBuffer, tempBuffer,
				sizeof(tempBuffer));
		if (dataLength > 0) {
			UART_Send(&uart4Channel, tempBuffer, dataLength);
		}
	}
}

void UART_Send(UART_Channel *channel, const uint8_t *data, uint16_t length) {
	HAL_UART_Transmit(channel->huart, (uint8_t*) data, length, 1000);
//    HAL_UART_Transmit_IT(channel->huart, (uint8_t *)data, length);
}
