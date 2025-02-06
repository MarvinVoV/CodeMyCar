/*
 * uart_handle.h
 *
 *  Created on: Jan 19, 2025
 *      Author: marvin
 */

#ifndef INC_UART_HANDLE_H_
#define INC_UART_HANDLE_H_

#include "buffer_manager.h"

// 接收缓冲区大小
#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 128
// DMA 接收缓冲区大小
#define DMA_RX_BUFFER_SIZE 128


// UART 通道实例
extern UART_Channel uart4Channel;


// DMA 接收临时缓冲区
extern uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE];


// 初始化 UART
void UART_Handle_Init(UART_HandleTypeDef *huart);



// 数据处理
void UART_Process_Data(void);

void UART_Send(UART_Channel *channel, const uint8_t *data, uint16_t length);

#endif /* INC_UART_HANDLE_H_ */
