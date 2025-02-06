## 协议设计

### UART 数据协议设计

BINARY 格式

```text
[Header][TYPE][Len][Data][Checksum][End]
- Header: 0xAA (1 Byte)
- TYPE: 协议类型 (1 Byte)
- Len: Data长度 (2 Byte)
- Data: 参数 (Len Bytes)
- Checksum: 异或校验 (从Header到Data全部字节异或)
- End: 0x55 (1 Byte)
```



## 协议使用示例代码

### 发送端 (ESP32 发送传感器数据)

```c
#include "protocol.h"
#include "driver/uart.h"

void send_sensor_data() {
  uint8_t sensor_data[] = {0x01, 0x02, 0x03};
  uint8_t data_len = sizeof(sensor_data);
  uint8_t *frame;
  uint8_t frame_len;

  frame = pack(CMD_TYPE_SENSOR, sensor_data, data_len, &frame_len);
  if (frame) {
    // 通过UART发送数据
    uart_write_bytes(UART_NUM_1, (const char*)frame, frame_len);
    free(frame); // 释放内存
  }
}
```

### 接收端 (STM32 解析数据)

```c
#include "protocol.h"

void uart_rx_handler(uint8_t* buffer, uint8_t buffer_len) {
  protocol_header_t *frame = NULL;
  if (parse(buffer, buffer_len, &frame)) {
    cmd_type_t cmd = (cmd_type_t)frame->cmd;
    uint8_t data_len = frame->len;
    uint8_t *data = frame->data;

    switch (cmd) {
      case CMD_TYPE_SENSOR:
        // 处理传感器数据
        break;
      case CMD_TYPE_CONTROL:
        // 处理控制指令
        break;
      case CMD_TYPE_LOG:
        // 处理系统日志
        break;
      default:
        break;
    }

    free(frame); // 释放内存
  }
}
```
