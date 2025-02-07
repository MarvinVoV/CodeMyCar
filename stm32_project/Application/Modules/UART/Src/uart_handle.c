/*
 * uart_handle.c
 *
 *  Created on: Feb 4, 2025
 *      Author: marvin
 */
#include "uart_handle.h"

#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "ctrl_protocol.h"
#include "queue_manager.h"
#include "stm32h7xx_hal_uart.h"
#include "uart_config.h"

uint8_t rx_buffer[BUF_SIZE];  // 接收缓冲区
uint8_t tx_buffer[BUF_SIZE];  // 发送缓冲区

void UART_Init(void) {
  // 启用空闲中断
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  // 启动UART4的空闲中断和DMA接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, BUF_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart4) {
    // 获取DMA剩余计数
    uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    // 计算实际接收数据长度
    uint16_t actual_len = BUF_SIZE - remaining;

    if (actual_len > 0) {
      // 解析接收到的数据
      protocol_header_t *parsed_frame = NULL;
      bool parse_success = parse(rx_buffer, actual_len, &parsed_frame);

      if (parse_success && parsed_frame != NULL) {
        // 提取数据内容
        uint8_t *data = parsed_frame->data;
        uint16_t data_len = parsed_frame->len;

        if (parsed_frame->type == PROTOCOL_TYPE_CONTROL) {
          if (QueueManager_DispatchCommand((control_cmd_t *)data) != osOK) {
            // TODO log
          }
        } else {
          // TODO log
        }
      }

      // 释放解析后的协议帧
      if (parsed_frame != NULL) {
        free(parsed_frame);
      }
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, BUF_SIZE);
  }
}

bool UART_Send_To_Esp(uint8_t *data, uint16_t length, protocol_type_t type) {
  if (data == NULL || length == 0) {
    return false;
  }
  if (type < PROTOCOL_TYPE_MIN || type >= PROTOCOL_TYPE_MAX) {
    return false;
  }

  // 打包协议帧
  uint8_t frame_len = 0;
  uint8_t *packed_frame = pack(type, data, length, &frame_len);

  if (packed_frame != NULL && frame_len > 0) {
    HAL_StatusTypeDef status =
        HAL_UART_Transmit_DMA(&huart4, packed_frame, frame_len);
    if (status == HAL_OK) {
      free(packed_frame);
      return true;
    } else {
      // TODO log
    }
  }
  if (packed_frame != NULL) {
    free(packed_frame);
  }
  return false;
}

bool Send_Log(log_entry_t *log_entry) {
  if (log_entry == NULL) {
    return false;
  }
  return UART_Send_To_Esp((uint8_t *)log_entry, sizeof(log_entry_t),
                          PROTOCOL_TYPE_LOG);
}
