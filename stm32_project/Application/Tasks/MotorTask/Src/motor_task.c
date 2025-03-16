/*
 * motor_task.c
 *
 *  Created on: Mar 15, 2025
 *      Author: marvin
 */

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "motor_task.h"
#include "ctrl_protocol.h"
#include "log_manager.h"
#include "motor_service.h"
#include "queue_manager.h"

static osThreadId_t servoTaskHandle = NULL;
uint16_t calculate_dynamic_delay(int remaining_steps, int total_steps, uint16_t base_delay);

void motor_task_init(MotionController* motion_controller)
{
    const osThreadAttr_t taskAttributes = {
        .name = "MotorTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    servoTaskHandle = osThreadNew(motor_task, motion_controller, &taskAttributes);
    if (servoTaskHandle == NULL)
    {
        LOG_ERROR(LOG_MODULE_SERVO, "Failed to create MotorTask");
        return;
    }
}

// 任务函数
void motor_task(void* arg)
{
    MotionController* ctrl = (MotionController*)arg;
    control_cmd_t* cmd;

    osMessageQueueId_t commandQueue = QueueManager_GetQueueByType(QUEUE_TYPE_MOTOR);
    if (commandQueue == NULL)
    {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to get CommandQueue");
        osThreadTerminate(NULL);
    }

    const TickType_t xControlPeriod = pdMS_TO_TICKS(10); // 10ms控制周期
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 阶段1：处理队列命令（非阻塞）
        if (osMessageQueueGet(commandQueue, &cmd, NULL, 0) == osOK)
        {
            if (cmd == NULL) continue;

            // todo 更新控制参数

            if (cmd->motor.mode == CTRL_MODE_EMERGENCY)
            {
                MotionService_EmergencyStop(ctrl);
            }
            else
            {
                // todo
                MotionService_SetVelocity(ctrl, cmd->motor.motion.diff.linear_vel, cmd->motor.motion.diff.angular_vel);
                // 释放内存
                vPortFree(cmd);
            }
        }
        // 阶段2：执行控制逻辑
        if(!ctrl->emergency_stop)
        {
            // 更新编码器状态（需原子操作保护）
            Motor_UpdateState(ctrl->left_motor);
            Motor_UpdateState(ctrl->right_motor);

            // 获取当前状态
            MotionService_UpdateMotorStatu(ctrl->left_motor, &ctrl->status[MOTOR_LEFT]);
            MotionService_UpdateMotorStatu(ctrl->right_motor, &ctrl->status[MOTOR_RIGHT]);

            // PID计算（注意线程安全）
            const float dt = 0.01f; // 10ms周期
            const float error_left = ctrl->target_rpm - ctrl->status[MOTOR_LEFT].current_rpm;
            const float output_left = PIDController_Update(&ctrl->left_pid, error_left, dt);
            Motor_SetSpeed(ctrl->left_motor, output_left);

            const float error_right = ctrl->target_rpm - ctrl->status[MOTOR_RIGHT].current_rpm;
            const float output_right = PIDController_Update(&ctrl->right_pid, error_right, dt);
            Motor_SetSpeed(ctrl->right_motor, output_right);
        }

        // 精确周期延迟
        vTaskDelayUntil(&xLastWakeTime, xControlPeriod);
    }
}
