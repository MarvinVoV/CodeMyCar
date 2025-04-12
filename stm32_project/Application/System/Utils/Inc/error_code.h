/*
 * error_code.h
 *
 *  Created on: Apr 7, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_UTILS_INC_ERROR_CODE_H_
#define SYSTEM_UTILS_INC_ERROR_CODE_H_
#include "stm32h7xx_hal.h"

// 错误域划分
typedef enum
{
    ERR_MODULE_CORE = 0x0000,   // 核心系统
    ERR_MODULE_MOTOR = 0x1000,  // 电机控制
    ERR_MODULE_STEER = 0x2000,  // 转向系统
    ERR_MODULE_SENSOR = 0x3000, // 传感器
    ERR_MODULE_COMMS = 0x4000,  // 通信模块
    ERR_MODULE_POWER = 0x5000,  // 电源管理
    ERR_MODULE_MOTION = 0x6000  // 运动控制
} ErrorModule;

// 错误严重等级
typedef enum
{
    ERR_LEVEL_DEBUG = 0x000,   // 调试信息
    ERR_LEVEL_WARNING = 0x100, // 可恢复错误
    ERR_LEVEL_ERROR = 0x200,   // 功能异常
    ERR_LEVEL_CRITICAL = 0x300 // 系统致命错误
} ErrorSeverity;

// 复合错误码结构
typedef union
{
    struct
    {
        uint16_t code : 12;    // 具体错误编号
        uint16_t level : 4;    // 错误等级
        uint16_t module : 12;  // 模块标识
        uint16_t reserved : 4; // 保留位
    };

    uint32_t value; // 完整错误码值
} ErrorCode;

// 错误码生成宏
#define MAKE_ERROR(module, level, code) \
((uint32_t)((module) | (level) | ((code) & 0xFFF)))


// 错误码解析宏
#define ERROR_MODULE(err)    ((err) & 0xF000)
#define ERROR_LEVEL(err)     ((err) & 0x0F00)
#define ERROR_CODE(err)      ((err) & 0x00FF)

// 通用错误码
#define ERR_SUCCESS         0x00000000  // 操作成功
#define ERR_INVALID_PARAM   MAKE_ERROR(ERR_MODULE_CORE, ERR_LEVEL_ERROR, 0x01)
#define ERR_HW_INIT_FAIL    MAKE_ERROR(ERR_MODULE_CORE, ERR_LEVEL_CRITICAL, 0x02)
#define ERR_MUTEX_TIMEOUT   MAKE_ERROR(ERR_MODULE_CORE, ERR_LEVEL_CRITICAL, 0x02)

// 电机模块错误
#define ERR_MOTOR_OVERHEAT  MAKE_ERROR(ERR_MODULE_MOTOR, ERR_LEVEL_CRITICAL, 0x01)
#define ERR_MOTOR_STALL     MAKE_ERROR(ERR_MODULE_MOTOR, ERR_LEVEL_ERROR, 0x02)

// 转向系统错误
#define ERR_STEER_TIMEOUT   MAKE_ERROR(ERR_MODULE_STEER, ERR_LEVEL_ERROR, 0x01)
#define ERR_STEER_RANGE     MAKE_ERROR(ERR_MODULE_STEER, ERR_LEVEL_WARNING, 0x02)
#define ERR_STEER_TOO_FREQUENT     MAKE_ERROR(ERR_MODULE_STEER, ERR_LEVEL_WARNING, 0x03)
#define ERR_HAL_SERVO_INVALID_ARG   MAKE_ERROR(ERR_MODULE_STEER, ERR_LEVEL_ERROR, 0x04)
#define ERR_HAL_SERVO_FAILURE   MAKE_ERROR(ERR_MODULE_STEER, ERR_LEVEL_ERROR, 0x05)


// 运动控制模块
#define MOTION_ERR_INVALID_PARAM     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x01)
#define MOTION_ERR_MODE_TRANSITION     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x02)
#define MOTION_ERR_EMERGENCY_ACTIVE     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x03)
#define MOTION_ERR_MODE_MISMATCH     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x04)
#define MOTION_ERR_PARAM_CLAMPED     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x05)
#define MOTION_ERR_CLEAR_FAILED     MAKE_ERROR(ERR_MODULE_MOTION, ERR_LEVEL_ERROR, 0x06)


// 错误描述接口
const char* GetErrorDescription(uint32_t errCode);

#endif /* SYSTEM_UTILS_INC_ERROR_CODE_H_ */
