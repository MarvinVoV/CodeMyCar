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
    osMessageQueueId_t rawDataQueue = QueueManager_GetQueueByType(QUEUE_TYPE_UART_RAW_DATA);
    if (rawDataQueue == NULL)
    {
        // todo
        osThreadTerminate(NULL);
    }
    uart_msg_t received_msg;
    while (1)
    {
        // 阻塞式等待队列消息
        if (osMessageQueueGet(rawDataQueue, &received_msg, NULL, osWaitForever) == osOK)
        {
            // 通过累积缓冲区接收并解析数据
            // protocol_receiver_append(&receiver, msg.data, msg.len);
            process_uart_data(received_msg.data, received_msg.len);

            // 处理完成后释放内存
            if (received_msg.data != NULL)
            {
                // vPortFree((void*)received_msg.data);
                free((uint8_t*)received_msg.data);
            }
        }
        else
        {
            // 添加错误处理逻辑 todo
            osDelay(10); // 防止错误时CPU占用过高
        }
    }
}

void dispatcher(uint8_t type, const uint8_t* frame_data, uint16_t frame_len)
{
    // todo
}
