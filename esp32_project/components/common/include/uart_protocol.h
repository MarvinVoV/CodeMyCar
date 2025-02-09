#pragma once
#include <stdint.h>


#define FRAME_HEADER 0xAA55
#define FRAME_HEADER_HIGH 0xAA
#define FRAME_HEADER_LOW 0x55
#define FRAME_TAIL 0x55AA
#define PROTOCOL_MAX_DATA_LEN 512

typedef enum {
  PROTOCOL_TYPE_MIN = 0x00,     // 起始值 - 仅用于 判断
  PROTOCOL_TYPE_SENSOR = 0x01,  // 传感器数据
  PROTOCOL_TYPE_CONTROL,        // 控制指令
  PROTOCOL_TYPE_LOG,            // 系统日志
  PROTOCOL_TYPE_MAX             // 结束值 - 仅用于 判断
} protocol_type_t;

// 柔性数组结构体（用于解析） 使用1byte对齐
#pragma pack(push, 1)
typedef struct {
  uint16_t header;  // 帧头 (固定值 0xAA55)
  uint8_t type;     // 协议类型
  uint16_t len;     // 数据长度 最大支持65535
  uint8_t data[0];  // 柔性数组，用于动态数据
} protocol_header_t;
#pragma pack(pop)

#define PROTOCOL_HEADER_SIZE sizeof(protocol_header_t)

// 完整帧结构体（用于打包）
typedef struct {
  uint16_t header;  // 帧头 (固定值 0xAA55)
  uint8_t type;     // 协议类型
  uint16_t len;     // 数据长度 最大支持65535
  uint8_t* data;    // 数据内容 (动态分配)
  uint16_t crc;     // CRC16 校验值
  uint16_t tail;    // 帧尾 (固定值 0x55AA)
} protocol_frame_t;

// 解析状态机
typedef enum {
  STATE_WAIT_HEADER_1,
  STATE_WAIT_HEADER_2,
  STATE_WAIT_TYPE,
  STATE_WAIT_LENGTH_1,
  STATE_WAIT_LENGTH_2,
  STATE_WAIT_DATA,
  STATE_WAIT_CRC_1,
  STATE_WAIT_CRC_2,
  STATE_WAIT_TAIL_1,
  STATE_WAIT_TAIL_2
} parse_state_t;

// 解析器结构
typedef struct {
  parse_state_t state;     // 协议解析状态
  protocol_frame_t frame;  // 解析出来的协议头
  uint16_t data_index;     // 解析出来的数据
} protocol_parser_t;

/**
 * 初始化协议解析器
 * @param parser 协议解析器
 */
void protocol_parser_init(protocol_parser_t* parser);

/**
 * 解析一个字节
 * @param parser 协议解析器
 * @param byte   待解析的字节
 * @return 解析结果，1-解析完成，0-解析未完成
 */
int protocol_parse_byte(protocol_parser_t* parser, uint8_t byte);

/**
 * @brief 打包协议帧
 *
 * @param type 协议类型
 * @param data 数据内容
 * @param data_len 数据长度
 * @param frame_len 协议帧数据长度
 */
uint8_t* protocol_pack_frame(protocol_type_t type, const uint8_t* data,
                             uint16_t data_len, uint16_t* frame_len);

/**
 * CRC16-CCITT 校验
 * @param data   待校验数据 Frame Header + Data
 * @param length  数据长度
 */
uint16_t crc16_ccitt(const uint8_t* data, uint16_t length);
