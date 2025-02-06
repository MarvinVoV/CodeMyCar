#pragma once

// ============== 订阅主题  ==============
// 服务器通信主题
#define TOPIC_SERVER_CTRL "server/ctrl"  // 服务器控制指令

// ============== 发布主题  ==============
// 设备通信主题
#define TOPIC_DEVICE_SENSOR "device/sensor"  // 设备数据上报

// 系统级主题
#define TOPIC_SYS_HEARTBEAT "sys/heartbeat"  // 心跳包
#define TOPIC_SYS_LOG "sys/log"              // 系统日志
