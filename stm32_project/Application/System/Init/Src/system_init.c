/*
 * system_init.c
 *
 *  Created on: Apr 12, 2025
 *      Author: marvin
 */
#include "system_init.h"

#include "error_code.h"


static SystemContext sysCtx;

/**
 * @brief 底盘默认物理参数配置
 *
 * @note 所有长度单位统一为毫米(mm)，角度单位为度(deg)
 */
static const ChassisConfig DEFAULT_CHASSIS_CFG = {
    .wheelbaseMM = 144.0f,        // 轴距（前后轮中心距），单位：毫米(mm)
    .trackWidthMM = 167.0f,       // 轮距（左右轮中心距），单位：毫米(mm)
    .wheelRadiusMM = 32.5f,       // 驱动轮半径，单位：毫米(mm)
    .maxLinearVelocityMPS = 1.2f, // 最大允许线速度，单位：米/秒(m/s) 正值为前进，负值为后退
    .maxAngularVelRad = 8.2f,     // 最大允许角速度，单位：弧度/秒(rad/s)
    .maxSteerAngleDeg = 45.0f     // 前轮最大转向角度，单位：度(degree) 正值右转，负值左转
};

int System_Initialize()
{
    HAL_MotorConfig motorLeftHalConfig = {
        .pwm = {
            .tim = &htim3,
            .ch = TIM_CHANNEL_1
        },
        .encoder = {
            &htim4
        },
        .gpio = {
            .in1Port = GPIOF,
            .in1Pin = GPIO_PIN_7, //  IN1控制引脚GPIO端口 - 通常用于电机正转控制
            .in2Port = GPIOF,
            .in2Pin = GPIO_PIN_8 //  IN2控制引脚GPIO端口 - 通常用于电机反转控制
        }
    };
    HAL_MotorConfig motorRightHalConfig = {
        .pwm = {
            .tim = &htim3,
            .ch = TIM_CHANNEL_2
        },
        .encoder = {
            &htim5
        },
        .gpio = {
            .in1Port = GPIOF,
            .in1Pin = GPIO_PIN_6, //  IN1控制引脚GPIO端口 - 通常用于电机正转控制
            .in2Port = GPIOF,
            .in2Pin = GPIO_PIN_10 //  IN2控制引脚GPIO端口 - 通常用于电机反转控制
        }
    };

    MotorSpec motorSpec = {
        .encoderPPR = 13,
        .gearRatio = 30,
        .maxRPM = 400,
        .wheelRadiusMM = DEFAULT_CHASSIS_CFG.wheelbaseMM
    };

    PID_Controller motorPicController = {
        .params = {
            .kp = 1.0f,
            .ki = 0.1f,
            .kd = 0.05f,
            .integral_max = 100.0f,
            .output_max = 95.0f
        }
    };

    MotorDriver motorLeftDriver = {
        .hal_cfg = &motorLeftHalConfig,
        .spec = &motorSpec,
        .control = {
            .mode = MOTOR_DRIVER_MODE_STOP,
            .pidControl = &motorPicController,
            .targetRPM = 0.0f,
        }
    };
    MotorDriver motorRightDriver = {
        .hal_cfg = &motorRightHalConfig,
        .spec = &motorSpec,
        .control = {
            .mode = MOTOR_DRIVER_MODE_STOP,
            .pidControl = &motorPicController,
            .targetRPM = 0.0f,
        }
    };

    MotorService motorService = {
        .instances = {
            [MOTOR_LEFT_WHEEL] = {
                .driver = &motorLeftDriver
            },
            [MOTOR_RIGHT_WHEEL] = {
                .driver = &motorRightDriver
            }
        }
    };
    // 初始化电机服务
    if (MotorService_init(&motorService, &motorLeftDriver, &motorRightDriver) != ERR_SUCCESS)
    {
        return ERR_HW_INIT_FAIL;
    }


    HAL_ServoConfig servoHalConfig = {
        .pwmTim = &htim2,
        .channel = TIM_CHANNEL_1,
        .maxPulse = SERVO_MAX_PULSE,
        .minPulse = SERVO_MIN_PULSE
    };
    ServoDriver servoDriver = {
        .hw = &servoHalConfig,
        .spec = {
            .minAngle = SERVO_ANGLE_MIN,
            .maxAngle = SERVO_ANGLE_MAX,
            .minPulseUs = SERVO_MIN_PULSE,
            .maxPulseUs = SERVO_MAX_PULSE,
            .resolutionDeg = 0.1f
        }
    };

    SteerConfig steerConfig = {
        .minAngleDeg = -45,
        .maxAngleDeg = 45,
        .deadZoneDeg = 2,
        .updateIntervalMs = 20
    };
    SteerInstance steerInstance = {
        .driver = &servoDriver
    };
    // 初始化转向服务
    if (SteerService_init(&steerInstance, &steerConfig, &servoDriver) != ERR_SUCCESS)
    {
        return ERR_HW_INIT_FAIL;
    }


    // 初始化运动控制
    ChassisConfig chassisConfig = DEFAULT_CHASSIS_CFG;
    MotionContext motionContext = {
        .chassisConfig = chassisConfig,
        .motorService = &motorService,
        .steerServo = &steerInstance
    };
    if (MotionContext_Init(&motionContext, &motorService, &steerInstance, &chassisConfig) != ERR_SUCCESS)
    {
        return ERR_HW_INIT_FAIL;
    }

    // 初始化命令处理器
    CmdProcessorContext cmdContext = {
        .motionContext = &motionContext
    };
    CmdProcessor_Init(&cmdContext);


    sysCtx.motionContext = motionContext;
    sysCtx.motorService = motorService;
    sysCtx.cmdContext = cmdContext;
    sysCtx.steerInstance = steerInstance;

    return ERR_SUCCESS;
}

SystemContext* System_GetContext(void)
{
    return &sysCtx;
}
