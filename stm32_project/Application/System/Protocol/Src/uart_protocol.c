#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <uart_protocol.h>

static uint16_t crc16_ccitt_continue(uint16_t crc, const uint8_t* data,
                                     uint16_t length);
// 封装协议数据帧
uint8_t* protocol_pack_frame(protocol_type_t type, const uint8_t* data,
                             uint16_t data_len, uint16_t* frame_len) {
  if (data_len > PROTOCOL_MAX_DATA_LEN) {
    return NULL;
  }

  // 计算总长度: Header(5) + Data(data_len) + CRC(2) + End(2)
  *frame_len = sizeof(protocol_header_t) + data_len + sizeof(uint16_t) +
               sizeof(uint16_t);
  uint8_t* frame = (uint8_t*)malloc(*frame_len);
  if (!frame) {
    return NULL;
  }

  protocol_header_t* header = (protocol_header_t*)frame;
  header->header = FRAME_HEADER;
  header->type = type;
  header->len = data_len;
  memcpy(header->data, data, data_len);

  const uint16_t crc = crc16_ccitt(frame, sizeof(protocol_header_t) + data_len);
  memcpy(frame + sizeof(protocol_header_t) + data_len, &crc, sizeof(crc));

  const uint16_t tail = FRAME_TAIL;
  memcpy(frame + sizeof(protocol_header_t) + data_len + sizeof(crc), &tail,
         sizeof(tail));

  return frame;
}

// 解析器初始化
void protocol_parser_init(protocol_parser_t* parser) {
  parser->state = STATE_WAIT_HEADER_1;
  parser->data_index = 0;
}

// 解析协议数据流
int protocol_parse_byte(protocol_parser_t* parser, uint8_t byte) {
  switch (parser->state) {
    case STATE_WAIT_HEADER_1:
      if (byte == (FRAME_HEADER & 0xFF)) {
        parser->state = STATE_WAIT_HEADER_2;
      }
      break;
    case STATE_WAIT_HEADER_2:
      // 注意 小端
      if (byte == (FRAME_HEADER >> 8)) {
        parser->frame.header = FRAME_HEADER;
        parser->state = STATE_WAIT_TYPE;
      } else {
        parser->state = STATE_WAIT_HEADER_1;
      }
      break;
    case STATE_WAIT_TYPE:
      parser->frame.type = byte;
      parser->state = STATE_WAIT_LENGTH_1;
      break;
    case STATE_WAIT_LENGTH_1:
      parser->frame.len = byte;
      parser->state = STATE_WAIT_LENGTH_2;
      break;
    case STATE_WAIT_LENGTH_2:
      parser->frame.len |= (byte << 8);
      parser->frame.data = (uint8_t*)malloc(parser->frame.len);
      parser->data_index = 0;
      parser->state = STATE_WAIT_DATA;
      break;
    case STATE_WAIT_DATA:
      parser->frame.data[parser->data_index++] = byte;
      if (parser->data_index >= parser->frame.len) {
        parser->state = STATE_WAIT_CRC_1;
      }
      break;
    case STATE_WAIT_CRC_1:
      parser->frame.crc = byte;
      parser->state = STATE_WAIT_CRC_2;
      break;
    case STATE_WAIT_CRC_2:
      parser->frame.crc |= (byte << 8);
      parser->state = STATE_WAIT_TAIL_1;
      break;
    case STATE_WAIT_TAIL_1:
      if (byte == (FRAME_TAIL & 0xFF)) {
        parser->state = STATE_WAIT_TAIL_2;
      } else {
        free(parser->frame.data);
        parser->state = STATE_WAIT_HEADER_1;
      }
      break;
    case STATE_WAIT_TAIL_2:
      if (byte == (FRAME_TAIL >> 8)) {
        // 构造协议头和数据部分的字节流
        uint8_t header_part[5];
        header_part[0] =
            (uint8_t)(parser->frame.header & 0xFF);  // Header低字节
        header_part[1] =
            (uint8_t)((parser->frame.header >> 8) & 0xFF);     // Header高字节
        header_part[2] = parser->frame.type;                   // 类型
        header_part[3] = (uint8_t)(parser->frame.len & 0xFF);  // 长度低字节
        header_part[4] =
            (uint8_t)((parser->frame.len >> 8) & 0xFF);  // 长度高字节

        // 计算CRC：协议头 + 数据
        uint16_t crc = crc16_ccitt(header_part, sizeof(header_part));
        if (parser->frame.len > 0) {
          crc =
              crc16_ccitt_continue(crc, parser->frame.data, parser->frame.len);
        }

        if (parser->frame.crc == crc) {
          return 1;  // 解析成功
        }
      }
      free(parser->frame.data);
      parser->state = STATE_WAIT_HEADER_1;
      break;
  }
  return 0;
}

/**
 * 计算CRC16校验码
 */
uint16_t crc16_ccitt(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static uint16_t crc16_ccitt_continue(uint16_t crc, const uint8_t* data,
                                     uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}