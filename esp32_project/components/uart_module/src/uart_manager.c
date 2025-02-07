#include "uart_manager.h"

#include <string.h>

#include "app_events.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "protocol.h"
#include "queue_manager.h"
#include "task_priorities.h"

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
  static uint8_t dtmp[RING_BUFFER_SIZE];
  while (1) {
    // 等待UART事件
    if (xQueueReceive(uart_event_queue, (void*)&event,
                      (TickType_t)portMAX_DELAY)) {
      // 清空缓冲区，避免残留数据
      bzero(dtmp, RING_BUFFER_SIZE);
      switch (event.type) {
        case UART_DATA:
          ESP_LOGD(TAG, "[UART DATA]: %d", event.size);
          // 限制读取长度不超过缓冲区大小
          size_t read_size =
              (event.size > RING_BUFFER_SIZE) ? RING_BUFFER_SIZE : event.size;
          int len = uart_read_bytes(UART_PORT, dtmp, read_size, portMAX_DELAY);
          // 将接收到的数据放入环形缓冲区
          BaseType_t xStatus =
              xRingbufferSend(rb_handle, dtmp, len, portMAX_DELAY);
          if (xStatus != pdPASS) {
            ESP_LOGE(TAG, "Failed to send data to ring buffer");
          }
          // 若实际数据长度超过缓冲区，丢弃剩余数据
          if (event.size > RING_BUFFER_SIZE) {
            uart_flush_input(UART_PORT);
            ESP_LOGE(TAG, "Data overflow, flushed %d bytes",
                     event.size - RING_BUFFER_SIZE);
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
    uint8_t frame_len;
    frame = pack(PROTOCOL_TYPE_CONTROL, mqtt_send_data.data, mqtt_send_data.len,
                 &frame_len);
    // 1. 以hex方式打印data 2. 打印len 3. 以hex方式打印frame
    ESP_LOGI(TAG, "Ready send data len=%d", mqtt_send_data.len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_send_data.data, mqtt_send_data.len,
                           ESP_LOG_INFO);
    if (!frame) {
      ESP_LOGE(TAG, "Failed to pack MQTT message");
    } else {
      ESP_LOG_BUFFER_HEXDUMP(TAG, frame, frame_len, ESP_LOG_INFO);

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
