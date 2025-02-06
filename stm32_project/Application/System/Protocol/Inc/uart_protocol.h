/*
 * uart_protocol.h
 *
 *  Created on: Feb 4, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_PROTOCOL_INC_UART_PROTOCOL_H_
#define SYSTEM_PROTOCOL_INC_UART_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  PROTOCOL_TYPE_MIN = 0x00,     // 起始值 - 仅用于 判断
  PROTOCOL_TYPE_SENSOR = 0x01,  // 传感器数据-上行
  PROTOCOL_TYPE_CONTROL,        // 控制指令-下行
  PROTOCOL_TYPE_LOG,            // 系统日志-上行
  PROTOCOL_TYPE_MAX             // 结束值 - 仅用于 判断
} protocol_type_t;

// 柔性数组结构体（用于解析） 使用1byte对齐
#pragma pack(push, 1)
typedef struct {
  uint8_t header;   // 帧头 (固定值 0xAA)
  uint8_t type;     // 协议类型
  uint16_t len;     // 数据长度 最大支持65535
  uint8_t data[0];  // 柔性数组，用于动态数据
} protocol_header_t;
#pragma pack(pop)

#define PROTOCOL_HEADER_SIZE sizeof(protocol_header_t)
#define PROTOCOL_MAX_DATA_LEN 512
#define PROTOCOL_FRAME_END 0x55

// 完整帧结构体（用于打包）
typedef struct {
  uint8_t header;    // 帧头 (固定值 0xAA)
  uint8_t type;      // 协议类型
  uint16_t len;      // 数据长度 最大支持65535
  uint8_t* data;     // 数据内容 (动态分配)
  uint8_t checksum;  // 校验值
  uint8_t end;       // 帧尾 (固定值 0x55)
} protocol_frame_t;

/**
 * @brief 打包协议帧
 *
 * @param type 协议类型
 * @param data 数据内容
 * @param data_len 数据长度
 * @param frame_len 协议帧数据长度
 */
uint8_t* pack(protocol_type_t type, const uint8_t* data, uint16_t data_len,
              uint8_t* frame_len);

/**
 * @brief 解析协议帧
 * @param buffer 协议帧数据
 * @param buffer_len 协议帧数据长度
 * @param frame 解析后的协议帧结构体
 */
bool parse(const uint8_t* buffer, uint16_t buffer_len,
           protocol_header_t** frame);

#endif /* SYSTEM_PROTOCOL_INC_UART_PROTOCOL_H_ */
