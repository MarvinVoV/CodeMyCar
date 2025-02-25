/*
 * uart_handle.c
 *
 *  Created on: Feb 4, 2025
 *      Author: marvin
 */
#include "uart_handle.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "ctrl_protocol.h"
#include "queue.h"
#include "queue_manager.h"
#include "stm32h7xx_hal_uart.h"

#include "uart_config.h"


// __ALIGN_BEGIN uint8_t rxBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;
// __ALIGN_BEGIN uint8_t txBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;

#define RAW_DATA_QUEUE_SIZE 16  // 定义队列大小

uart_dma_buffer_t uart_dma_buffer;


void start_reception()
{
    // 先切换缓冲区
    const uint8_t next_buffer = uart_dma_buffer.current ^ 1;
    uart_dma_buffer.current = next_buffer;

    // 启动DMA接收，交替使用缓冲区
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart_dma_buffer.buffer[next_buffer],
                                                            UART_RX_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        // error
    }

}

void init_uart_dma(void)
{
    uart_dma_buffer.current = 0;
    uart_dma_buffer.length = 0;
    uart_dma_buffer.semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uart_dma_buffer.semaphore); // 初始释放

    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    start_reception();
}

void process_uart_data(const uint8_t* data, const uint16_t len)
{
    HAL_UART_Transmit(&huart4, data, len, HAL_MAX_DELAY);
}

//
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart4)
    {
        // 获取DMA剩余计数
        const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        // 计算实际接收数据长度
        const uint16_t actual_len = UART_RX_BUFFER_SIZE - remaining;
        uart_dma_buffer.length  = actual_len;
        start_reception(actual_len);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart_dma_buffer.semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart4)
    {
        char* s = "errorcallback";
        process_uart_data((uint8_t*)s, strlen(s));
    }
}


bool UART_Send_To_Esp(uint8_t* data, uint16_t length, protocol_type_t type)
{
    if (data == NULL || length == 0)
    {
        return false;
    }
    if (type <= PROTOCOL_TYPE_MIN || type >= PROTOCOL_TYPE_MAX)
    {
        return false;
    }

    // 打包协议帧
    uint16_t frame_len = 0;
    uint8_t* packed_frame = protocol_pack_frame(type, data, length, &frame_len);

    if (packed_frame != NULL && frame_len > 0)
    {
        HAL_StatusTypeDef status =
            HAL_UART_Transmit_DMA(&huart4, packed_frame, frame_len);
        if (status == HAL_OK)
        {
            free(packed_frame);
            return true;
        }
        else
        {
            // TODO log
        }
    }
    if (packed_frame != NULL)
    {
        free(packed_frame);
    }
    return false;
}

bool Send_Log(log_entry_t* log_entry)
{
    if (log_entry == NULL)
    {
        return false;
    }
    return UART_Send_To_Esp((uint8_t*)log_entry, sizeof(log_entry_t),
                            PROTOCOL_TYPE_LOG);
}
