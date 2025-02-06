#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

static const char* TAG = "Protocol";
// 构建数据帧
uint8_t* pack(protocol_type_t type, const uint8_t* data, uint16_t data_len,
              uint8_t* frame_len) {
  if (data_len > PROTOCOL_MAX_DATA_LEN) {
    ESP_LOGE("pack", "data length too long");
    return NULL;  // 数据长度超出限制
  }
  if (data_len > 0 && data == NULL) {
    ESP_LOGE(TAG, "pack: data is NULL with non-zero length");
    return NULL;
  }
  // 计算总长度: Header(4) + Data(data_len) + Checksum(1) + End(1)
  *frame_len =
      PROTOCOL_HEADER_SIZE + data_len + 2;  // Header + Data + Checksum + End
  uint8_t* frame = malloc(*frame_len);
  if (!frame) {
    ESP_LOGE(TAG, "pack: malloc failed");
    return NULL;
  }
  // 填充协议头
  protocol_header_t* header = (protocol_header_t*)frame;
  header->header = 0xAA;
  header->type = (uint8_t)type;
  header->len = data_len;  // 直接赋值，自动处理小端

  // 填充数据内容
  if (data_len > 0) {
    memcpy(frame + PROTOCOL_HEADER_SIZE, data, data_len);
  }

  // 计算校验值 (异或所有字节)
  uint8_t checksum = 0;
  for (int i = 0; i < PROTOCOL_HEADER_SIZE + data_len; i++) {
    checksum ^= frame[i];
  }
  frame[PROTOCOL_HEADER_SIZE + data_len] = checksum;

  // 填充帧尾
  frame[PROTOCOL_HEADER_SIZE + data_len + 1] = PROTOCOL_FRAME_END;

  return frame;
}

// 解析数据帧
bool parse(const uint8_t* buffer, uint16_t buffer_len,
           protocol_header_t** frame) {
  if (!buffer || !frame) {
    ESP_LOGE(TAG, "parse: invalid input parameters");
    return false;
  }
  // 最小帧长度为 Header + DataLen + Checksum + End
  if (buffer_len < PROTOCOL_HEADER_SIZE + 2) {
    ESP_LOGE(TAG, "parse: buffer too short");
    return false;
  }

  // 验证帧头
  if (buffer[0] != 0xAA) {
    ESP_LOGE(TAG, "parse: invalid header");
    return false;
  }

  // 验证帧尾
  if (buffer[buffer_len - 1] != PROTOCOL_FRAME_END) {
    ESP_LOGE(TAG, "parse: invalid end");
    return false;
  }

  // 提取数据长度（小端模式）
  uint16_t data_len;
  memcpy(&data_len, &buffer[2], sizeof(uint16_t));  // 直接内存拷贝处理小端
  // 验证帧长度是否匹配: Header(4) + Data(data_len) + Checksum(1) + End(1)
  if (buffer_len != PROTOCOL_HEADER_SIZE + data_len + 2) {
    ESP_LOGE(TAG, "parse: invalid length");
    return false;
  }

  // 计算校验值
  uint8_t checksum = 0;
  for (int i = 0; i < buffer_len - 2; i++) {  // 排除 Checksum 和 End
    checksum ^= buffer[i];
  }
  if (checksum != buffer[buffer_len - 2]) {  // 验证校验值
    ESP_LOGE(TAG, "parse: invalid checksum (calc:0x%02X, actual:0x%02X)",
             checksum, buffer[buffer_len - 2]);
    return false;
  }
  // 检查命令类型有效性
  protocol_type_t type = (protocol_type_t)buffer[1];
  if (type <= PROTOCOL_TYPE_MIN || type >= PROTOCOL_TYPE_MAX) {
    ESP_LOGE(TAG, "parse: invalid type (0x%02X)", type);
    return false;
  }
  // 分配内存（包含协议头和数据部分）
  *frame = malloc(PROTOCOL_HEADER_SIZE + data_len);
  if (!(*frame)) {
    ESP_LOGE(TAG, "parse: malloc failed");
    return false;
  }

  memcpy(*frame, buffer, PROTOCOL_HEADER_SIZE + data_len);
  return true;
}