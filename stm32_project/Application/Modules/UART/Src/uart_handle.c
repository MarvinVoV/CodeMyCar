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


__ALIGN_BEGIN uint8_t rxBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN uint8_t txBuffer[UART_RX_BUFFER_SIZE] __ALIGN_END;

#define RAW_DATA_QUEUE_SIZE 16  // 定义队列大小

// 存储待解析的数据队列
static osMessageQueueId_t rawDataQueue = NULL;

void init_uart_dma(void)
{
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxBuffer, UART_RX_BUFFER_SIZE);

    // 创建消息队列
    const osMessageQueueAttr_t rawDataQueueAttrs = {
        .name = "uartRawDataQueue",
        .attr_bits = 0,
        .cb_mem = NULL,
        .cb_size = 0,
        .mq_mem = NULL,
        .mq_size = 0
    };
    rawDataQueue = osMessageQueueNew(RAW_DATA_QUEUE_SIZE, sizeof(uart_msg_t),
                                     &rawDataQueueAttrs);
    if (rawDataQueue == NULL)
    {
        // 处理队列创建失败的情况
        // 例如：记录日志或断言
        // TODO
        return;
    }

    // 注册队列到queue_manager
    QueueManager_RegisterHandler(QUEUE_TYPE_UART_RAW_DATA, rawDataQueue);
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

        if (actual_len > 0)
        {
            // 动态分配临时缓冲区（或使用静态全局缓冲区）
            uint8_t* temp_buf = (uint8_t*) malloc(actual_len);
            if (temp_buf != NULL)
            {
                // 拷贝数据到临时缓冲区
                memcpy(temp_buf, rxBuffer, actual_len);
                // // 创建消息对象（栈分配）
                const uart_msg_t msg = {
                    .data = temp_buf,
                    .len = actual_len
                };
                // xQueueSendFromISR()
                if (osMessageQueuePut(rawDataQueue, &msg, 0, 0) != osOK)
                {
                    // vPortFree(temp_buf); // 队列满时立即释放内存
                    free(temp_buf);
                }
            }
        }
        // 立即重启DMA（使用原缓冲区）
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, UART_RX_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart4)
    {
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
