/*
 * ctrl_task.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "ctrl_task.h"

#include "pkt_protocol_buf.h"
#include "queue_manager.h"
#include "uart_handle.h"

static osThreadId_t rawDataReadTaskHandle = NULL;
// 累积缓冲区大小
#define ACCUMULATE_BUF_SIZE ( 3 * 512)

// 协议解析处理器
static protocol_receiver receiver;

static void dispatcher(uint8_t type, const uint8_t* frame_data, uint16_t frame_len);


void uart_task_init()
{
    const osThreadAttr_t rawDataReadTaskAttrs = {
        .name = "RawDataReadTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal
    };
    rawDataReadTaskHandle = osThreadNew(uart_task, NULL, &rawDataReadTaskAttrs);
    if (rawDataReadTaskHandle == NULL)
    {
        // todo
    }

    // protocol_receiver_init(&receiver, ACCUMULATE_BUF_SIZE, dispatcher);
    // if (receiver.buffer == NULL)
    // {
    //     // todo
    // }
}

void uart_task()
{
    while (1)
    {
        if (xSemaphoreTake(uart_dma_buffer.semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            if (uart_dma_buffer.length > 0)
            {
                // 获取非活动缓冲区的索引和长度
                const uint8_t inactive_buffer = uart_dma_buffer.current ^ 1;
                uint8_t* data = uart_dma_buffer.buffer[inactive_buffer];
                const uint16_t len = uart_dma_buffer.length;

                // 无效化 Cache（H7 的 Cache 行大小为 32 字节）
                SCB_InvalidateDCache_by_Addr(data, len);
                process_uart_data(data, len);
            }
        }
    }
}

void dispatcher(uint8_t type, const uint8_t* frame_data, uint16_t frame_len)
{
    // todo
}
