#include "queue_manager.h"

#include "esp_log.h"

static const char* TAG = "QueueManager";

// 队列配置参数
#define MQTT_QUEUE_LENGTH 10
#define UART_QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(QueueMessage_t)

static QueueHandle_t mqtt_send_queue = NULL;
static QueueHandle_t uart_send_queue = NULL;

bool queue_manager_init(QueueType_t queue_type) {
  bool res = true;
  switch (queue_type) {
    case MQTT:
      if (mqtt_send_queue != NULL) {
        return mqtt_send_queue;
      }
      // 创建MQTT发送队列
      mqtt_send_queue = xQueueCreate(MQTT_QUEUE_LENGTH, QUEUE_ITEM_SIZE);
      if (!mqtt_send_queue) {
        ESP_LOGE(TAG, "Failed to create MQTT send queue");
        res = false;
      }
      break;
    case UART:
      if (uart_send_queue != NULL) {
        return uart_send_queue;
      }
      // 创建UART发送队列
      uart_send_queue = xQueueCreate(UART_QUEUE_LENGTH, QUEUE_ITEM_SIZE);
      if (!uart_send_queue) {
        ESP_LOGE(TAG, "Failed to create UART send queue");
        res = false;
      }
      break;
    default:
      ESP_LOGW(TAG, "Invalid queue type requested: %d", queue_type);
      res = false;
      break;
  }
  return res;
}

QueueHandle_t queue_manager_get_queue(QueueType_t queue_type) {
  switch (queue_type) {
    case MQTT:
      return mqtt_send_queue;
    case UART:
      return uart_send_queue;
    default:
      ESP_LOGW(TAG, "Invalid queue type requested: %d", queue_type);
      return NULL;
  }
}

void queue_manager_destroy(QueueType_t queue_type) {
  if (mqtt_send_queue) {
    vQueueDelete(mqtt_send_queue);
  }
  if (uart_send_queue) {
    vQueueDelete(uart_send_queue);
  }
}