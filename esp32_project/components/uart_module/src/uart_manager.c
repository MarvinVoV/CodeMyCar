#include "uart_manager.h"

#include <string.h>

#include "app_events.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "queue_manager.h"
#include "task_priorities.h"
#include "uart_protocol.h"
#include "mqtt_manager.h"

static const char* TAG = "UARTManager";
// 波特率
#define BAUD_RATE (115200)
// UART事件队列大小
#define UART_EVENT_QUEUE_SIZE (10)
#define UART_RX_PIN GPIO_NUM_5
#define UART_TX_PIN GPIO_NUM_4
#define UART_PORT UART_NUM_1

#define UART_EVENT_TASK_STACK_SIZE (4096)
#define RING_BUFFER_TASK_STACK_SIZE (4096)
#define MQTT_MESSAGE_TASK_STACK_SIZE (4096)

// UART事件队列
static QueueHandle_t uart_event_queue;

// 环形缓冲区句柄
static RingbufHandle_t rb_handle;

static void handle_uart_event_task(void* pvParameters);
static void handle_ring_buffer_task(void* pvParameters);
static void handle_mqtt_message_task(void* pvParameters);

size_t buffer_to_hex(const uint8_t* src_buffer, size_t src_len, char* dst_buffer, size_t dst_size) ;
/**
 * @brief 初始化UART模块
 * @param config UART配置参数
 */
void uart_init() {
  const uart_config_t uart_config = {.baud_rate = BAUD_RATE,
                                     .data_bits = UART_DATA_8_BITS,
                                     .parity = UART_PARITY_DISABLE,
                                     .stop_bits = UART_STOP_BITS_1,
                                     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  // 安装UART驱动，启用DMA和IRAM中断
  esp_err_t err =
      uart_driver_install(UART_PORT,              // UART端口号
                          UART_BUF_SIZE,          // RX缓冲区大小
                          UART_BUF_SIZE,          // TX缓冲区大小
                          UART_EVENT_QUEUE_SIZE,  // 事件队列的大小
                          &uart_event_queue,      // 事件队列的句柄
                          ESP_INTR_FLAG_IRAM  // 中断分配标志,ESP_INTR_FLAG_IRAM
                                              // 表示将 ISR 放置在 IRAM 中
      );
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
    return;
  }

  // 初始化 UART 参数
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  // 设置 UART 引脚
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // 创建环形缓冲区
  rb_handle = xRingbufferCreate(RING_BUFFER_SIZE,       // 缓冲区大小
                                RINGBUF_TYPE_BYTEBUF);  // 类型
  if (rb_handle == NULL) {
    ESP_LOGE(TAG, "Failed to create UART RX ring buffer");
    uart_driver_delete(UART_PORT);
    return;
  }

  // 创建接收UART事件的任务
  if (xTaskCreate(handle_uart_event_task, "handle_uart_event_task",
                  UART_EVENT_TASK_STACK_SIZE, NULL, TASK_PRIORITY_UART_RECEIVE,
                  NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create UART event task");
    uart_driver_delete(UART_PORT);
    vRingbufferDelete(rb_handle);
    return;
  }

  // 创建发送UART消息的任务
  if (xTaskCreate(handle_ring_buffer_task, "handle_ring_buffer_task",
                  RING_BUFFER_TASK_STACK_SIZE, NULL,
                  TASK_PRIORITY_UART_BUFFER_PROCESS, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create handle_ring_buffer_task");
    uart_driver_delete(UART_PORT);
    vRingbufferDelete(rb_handle);
    return;
  }

  // 创建UART发送数据队列
  if (!queue_manager_init(UART)) {
    ESP_LOGE(TAG, "Failed to create UART send queue");
    uart_driver_delete(UART_PORT);
    vRingbufferDelete(rb_handle);
    return;
  }

  // 创建接收MQTT指令消息队列的任务
  if (xTaskCreate(handle_mqtt_message_task, "handle_mqtt_message_task",
                  MQTT_MESSAGE_TASK_STACK_SIZE, NULL,
                  TASK_PRIORITY_UART_HANDLE_MQTT_MSG, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create handle_mqtt_message_task");
  }

  app_events_set_bits(UART_INIT_COMPLETE);
}

// 接收数据任务
static void handle_uart_event_task(void* pvParameters) {
  uart_event_t event;
  // NOTE: 内存分配在静态存储区,并且程序启动时元素就会被初始化为0
  static uint8_t accumulated_buf[RING_BUFFER_SIZE * 2];
  static size_t accumulated_len = 0;
  while (1) {
    // 等待UART事件
    if (xQueueReceive(uart_event_queue, (void*)&event,
                      (TickType_t)portMAX_DELAY)) {
      switch (event.type) {
        case UART_DATA:
          ESP_LOGD(TAG, "[UART DATA]: %d", event.size);

          // 读取新数据到临时缓冲区
          uint8_t temp_buf[RING_BUFFER_SIZE];
          size_t read_size =
              (event.size > sizeof(temp_buf)) ? sizeof(temp_buf) : event.size;
          int len =
              uart_read_bytes(UART_PORT, temp_buf, read_size, portMAX_DELAY);

          // 检查缓冲区剩余空间
          if (accumulated_len + len > sizeof(accumulated_buf)) {
            ESP_LOGE(TAG, "Accumulated buffer overflow, resetting");
            accumulated_len = 0;
          }

          // 将新数据追加到累积缓冲区
          memcpy(accumulated_buf + accumulated_len, temp_buf, len);
          accumulated_len += len;

          // 协议解析状态机
          size_t processed = 0;
          while (accumulated_len - processed >= PROTOCOL_HEADER_SIZE) {
            protocol_header_t* header =
                (protocol_header_t*)(accumulated_buf + processed);
            // 检查帧头有效性
            if (accumulated_buf[processed] != FRAME_HEADER_LOW ||
                accumulated_buf[processed + 1] != FRAME_HEADER_HIGH) {
              //   ESP_LOGW(TAG, "Invalid frame header 0x%04X", header->header);
              //   processed++

              // 优化：优化帧头搜索逻辑 #define FRAME_HEADER 0xAA55
              size_t next_header = accumulated_len;  // 初始化为末尾，表示未找到
              for (size_t i = processed; i <= accumulated_len - 2; i++) {
                if (accumulated_buf[i] == FRAME_HEADER_LOW &&
                    accumulated_buf[i + 1] == FRAME_HEADER_HIGH) {
                  next_header = i;
                  break;
                }
              }
              if (next_header == accumulated_len) {
                processed = accumulated_len;  // 未找到，清空缓冲区
              } else {
                processed = next_header;
              }
              continue;
            }

            // 计算完整帧长度 (头 + 数据 + CRC + 尾)
            // NOTE 计算FRAME_TAIL长度不能使用 sizeof(FRAME_TAIL)
            // FRAME_TAIL是宏定义的整型常量，sizeof(FRAME_TAIL)在32位系统中返回4字节
            uint16_t total_len = PROTOCOL_HEADER_SIZE + header->len +
                                 sizeof(uint16_t) + sizeof(uint16_t);
            // 检查是否收到完整帧
            if (accumulated_len - processed < total_len) {
              break;  // 数据不完整，等待下次接收
            }
            // 校验数据长度合法性
            // if (header->len > PROTOCOL_MAX_DATA_LEN) {
            // }

            // 检查帧尾有效性
            uint16_t* tail_ptr = (uint16_t*)(accumulated_buf + processed +
                                             total_len - sizeof(uint16_t));
            if (*tail_ptr != FRAME_TAIL) {
              ESP_LOGW(TAG, "Invalid frame tail 0x%04X", *tail_ptr);
              // processed += total_len;
              processed++;  // 仅跳过帧头第一个字节，重新同步，防止因错误的total_len导致越界
              continue;
            }
            // 计算CRC
            uint16_t received_crc =
                *((uint16_t*)(accumulated_buf + processed +
                              PROTOCOL_HEADER_SIZE + header->len));
            uint16_t calculated_crc =
                crc16_ccitt(accumulated_buf + processed,
                            PROTOCOL_HEADER_SIZE + header->len);
            if (received_crc != calculated_crc) {
              ESP_LOGW(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X",
                       received_crc, calculated_crc);
              processed += total_len;
              continue;
            }

            // 将完整数据包存入环形缓冲区
            BaseType_t xStatus =
                xRingbufferSend(rb_handle, accumulated_buf + processed,
                                total_len, portMAX_DELAY);
            if (xStatus != pdPASS) {
              ESP_LOGE(TAG, "Failed to send data to ring buffer");
            }

            processed += total_len;
            ESP_LOGI(TAG, "Processed complete frame: %d bytes", total_len);
          }

          // 移动剩余未处理数据到缓冲区头部
          if (processed > 0) {
            size_t remaining = accumulated_len - processed;
            memmove(accumulated_buf, accumulated_buf + processed, remaining);
            accumulated_len = remaining;
          }

          // 处理溢出情况
          if (event.size > read_size) {
            uart_flush_input(UART_PORT);
            ESP_LOGE(TAG, "Data overflow, flushed %d bytes",
                     event.size - read_size);
          }
          break;
        case UART_FIFO_OVF:
          ESP_LOGI(TAG, "hw fifo overflow");
          // If fifo overflow happened, you should consider adding flow control
          // for your application. The ISR has already reset the rx FIFO, As an
          // example, we directly flush the rx buffer here in order to read more
          // data.
          uart_flush_input(UART_PORT);
          // xQueueReset(uart_event_queue);
          break;
        case UART_BUFFER_FULL:
          // Event of UART ring buffer full
          ESP_LOGI(TAG, "ring buffer full");
          // If buffer full happened, you should consider increasing your buffer
          // size As an example, we directly flush the rx buffer here in order
          // to read more data.
          uart_flush_input(UART_PORT);
          // xQueueReset(uart_event_queue);
          break;
        case UART_BREAK:
          // Event of UART RX break detected
          ESP_LOGI(TAG, "uart rx break");
          uart_flush_input(UART_PORT);
          // xQueueReset(uart_event_queue);
          break;
        case UART_PARITY_ERR:
          ESP_LOGI(TAG, "uart parity error");
          break;
        case UART_FRAME_ERR:
          ESP_LOGI(TAG, "uart frame error");
          uart_flush_input(UART_PORT);
          break;
        case UART_PATTERN_DET:
          ESP_LOGI(TAG, "uart pattern detected");
          break;
        default:
          ESP_LOGI(TAG, "uart event type: %d", event.type);
          break;
      }
    }
  }
  vTaskDelete(NULL);
}

// 处理环形缓冲区中的数据，并发送到消息队列
static void handle_ring_buffer_task(void* pvParameters) {
  while (1) {
    size_t item_size;
    uint8_t* data =
        (uint8_t*)xRingbufferReceive(rb_handle, &item_size, portMAX_DELAY);
    if (data == NULL) {
      ESP_LOGE(TAG, "Ringbuffer receive returned NULL");
      continue;
    }
    if (item_size == 0) {
      ESP_LOGE(TAG, "Invalid zero-length data");
      vRingbufferReturnItem(rb_handle, data);
      continue;
    }
    if (item_size > RING_BUFFER_SIZE) {
      ESP_LOGE(TAG, "Data too large (%d bytes), truncating", item_size);
      item_size = RING_BUFFER_SIZE;
    }

    QueueMessage_t uart_send_data = {
        .type = UART,
        .data = malloc(item_size),
        .len = item_size,
    };
    if (uart_send_data.data == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for UART send queue data");
      vRingbufferReturnItem(rb_handle, data);
      continue;
    }
    memcpy(uart_send_data.data, data, item_size);
    // 已经深拷贝过了，立即返回项给环形缓冲区
    vRingbufferReturnItem(rb_handle, data);  // 返回项给环形缓冲区

    QueueHandle_t uart_send_queue = queue_manager_get_queue(UART);
    if (uart_send_queue == NULL) {
      ESP_LOGE(TAG, "UART send queue has not been initialized");
      free(uart_send_data.data);
    }
    if (xQueueSend(uart_send_queue, &uart_send_data, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Failed to send message to queue");
      free(uart_send_data.data);
    }
  }
  // 理论上执行不到这里
  vTaskDelete(NULL);
}

static void handle_mqtt_message_task(void* pvParameters) {
  QueueHandle_t mqtt_send_queue = NULL;

  // Wait up to 5 seconds for queue initialization
  const TickType_t max_queue_wait = pdMS_TO_TICKS(5000);
  const int max_retries = 5;
  int retry_count = 0;

  while ((mqtt_send_queue = queue_manager_get_queue(MQTT)) == NULL) {
    if (retry_count++ >= max_retries) {
      ESP_LOGE(TAG, "Failed to get MQTT queue after %d attempts", max_retries);
      vTaskDelete(NULL);
      return;
    }
    ESP_LOGW(TAG, "MQTT send queue not available, retrying (%d/%d)",
             retry_count, max_retries);
    vTaskDelay(max_queue_wait / max_retries);
  }

  ESP_LOGD(TAG, "MQTT send queue available");

  const TickType_t queue_timeout = pdMS_TO_TICKS(1000);  // 1s超时
  while (1) {
    // TODO 可增加检查系统关闭等事件 删除任务
    QueueMessage_t mqtt_send_data = {0};
    BaseType_t receive_status =
        xQueueReceive(mqtt_send_queue, &mqtt_send_data, queue_timeout);

    if (receive_status != pdPASS) {
      if (receive_status == errQUEUE_EMPTY) {
        ESP_LOGD(TAG, "MQTT queue empty after 1 second timeout");
      } else {
        ESP_LOGE(TAG, "Failed to receive message from MQTT queue (error: %d)",
                 receive_status);
      }
      continue;
    }

    // 验证数据有效性
    if (mqtt_send_data.data == NULL || mqtt_send_data.len == 0) {
      ESP_LOGE(TAG, "Invalid MQTT message (null data or zero length)");
      free(mqtt_send_data.data);  // 确保释放可能分配的内存
      free(mqtt_send_data.extra_info);
      continue;
    }

    // TODO 暂时只有指令控制类型
    // 构造数据帧
    uint8_t* frame = NULL;
    uint16_t frame_len;
    frame = protocol_pack_frame(PROTOCOL_TYPE_CONTROL, mqtt_send_data.data,
                                mqtt_send_data.len, &frame_len);
    // 1. 以hex方式打印data 2. 打印len 3. 以hex方式打印frame
    ESP_LOGI(TAG, "Ready send data len=%u", mqtt_send_data.len);
    // ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_send_data.data, mqtt_send_data.len,
    //                        ESP_LOG_INFO);
    if (!frame) {
      ESP_LOGE(TAG, "Failed to pack MQTT message");
    } else {
      ESP_LOG_BUFFER_HEXDUMP(TAG, frame, frame_len, ESP_LOG_INFO);
      // log temp TODO
      char hex_data[100] = {0};
      buffer_to_hex(frame, frame_len, hex_data, sizeof(hex_data));
      mqtt_publish("device/sensor", hex_data, strlen(hex_data),0);


      // 通过UART发送数据
      if (!uart_send_data(frame, frame_len)) {
        ESP_LOGE(TAG, "Failed to send MQTT data via UART");
      }
      // 释放数据帧内存
      free(frame);
    }

    // 释放队列消息分配的内存
    free(mqtt_send_data.data);
    free(mqtt_send_data.extra_info);
    mqtt_send_data.data = NULL;
    mqtt_send_data.extra_info = NULL;
    mqtt_send_data.len = 0;
  }
}
/**
 * @brief 向UART发送数据
 * @param data 数据指针
 * @param len 数据长度
 * @return 是否发送成功
 */
bool uart_send_data(const uint8_t* data, size_t len) {
  int bytes_written = uart_write_bytes(UART_PORT, data, len);
  if (bytes_written < 0) {
    ESP_LOGE(TAG, "UART write error: %s", esp_err_to_name(bytes_written));
    return false;
  }
  return bytes_written == len;
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