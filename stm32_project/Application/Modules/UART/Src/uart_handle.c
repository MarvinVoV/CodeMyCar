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

#include "cmsis_os.h"
#include "ctrl_protocol.h"
#include "queue_manager.h"
#include "stm32h7xx_hal_uart.h"
#include "uart_config.h"

size_t buffer_to_hex(const uint8_t* src_buffer, size_t src_len, char* dst_buffer, size_t dst_size);

uint8_t rx_buffer[BUF_SIZE];  // 接收缓冲区
uint8_t tx_buffer[BUF_SIZE];  // 发送缓冲区

protocol_parser_t parser;

void UART_Init(void) {
  // 启用空闲中断
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  // 启动UART4的空闲中断和DMA接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, BUF_SIZE);
  protocol_parser_init(&parser);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart4) {
    // 获取DMA剩余计数
    uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    // 计算实际接收数据长度
    uint16_t actual_len = BUF_SIZE - remaining;

    if (actual_len > 0) {
      char tmp[100] = {0};
      buffer_to_hex(rx_buffer, sizeof(rx_buffer), tmp, sizeof(tmp));
      

      // 解析接收到的数据
      for (uint16_t i = 0; i < actual_len; i++) {
        if (protocol_parse_byte(&parser, rx_buffer[i])) {
          // 解析成功
        	//TODO 考虑累积缓冲区应对拆数据包的情况
          // if (parser.frame.type == PROTOCOL_TYPE_CONTROL) {
            
            // if (QueueManager_DispatchCommand((control_cmd_t *)parser.frame.data) != osOK) {
            //   // TODO log
            // }
          // }
          // 实现回显 即通过构造回显帧 发送到串口
          UART_Send_To_Esp(parser.frame.data, parser.frame.len, parser.frame.type);



          free(parser.frame.data);
          protocol_parser_init(&parser);
        }
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
  uint16_t frame_len = 0;
  uint8_t *packed_frame = protocol_pack_frame(type, data, length, &frame_len);

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

/**
 * @brief 将缓冲区内容以十六进制格式打印到目标缓冲区
 * 
 * @param src_buffer 源缓冲区
 * @param src_len 源缓冲区长度
 * @param dst_buffer 目标缓冲区
 * @param dst_size 目标缓冲区大小
 * @return size_t 返回写入目标缓冲区的字符数（不包括终止符'\0'）
 */
size_t buffer_to_hex(const uint8_t* src_buffer, size_t src_len, char* dst_buffer, size_t dst_size) {
  if (src_buffer == NULL || dst_buffer == NULL || dst_size == 0) {
      return 0; // 参数无效
  }

  size_t written = 0; // 已写入目标缓冲区的字符数
  for (size_t i = 0; i < src_len; i++) {
      // 格式化为两位十六进制数，并添加空格
      int bytes_needed = snprintf(dst_buffer + written, dst_size - written, "%02X ", src_buffer[i]);
      if (bytes_needed < 0 || written + bytes_needed >= dst_size) {
          // 如果目标缓冲区空间不足，截断并返回已写入的字符数
          break;
      }
      written += bytes_needed;
  }

  // 移除最后一个多余的空格（如果有的话）
  if (written > 0 && dst_buffer[written - 1] == ' ') {
      dst_buffer[written - 1] = '\0';
      written--;
  } else {
      dst_buffer[written] = '\0'; // 确保字符串以'\0'结尾
  }

  return written;
}
