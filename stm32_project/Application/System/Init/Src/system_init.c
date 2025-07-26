/*
 * system_init.c
 *
 *  Created on: Apr 12, 2025
 *      Author: marvin
 */
#include "system_init.h"

#include "error_code.h"

static SystemContext sysCtx;

static MotorSpec motorSpec;
static HAL_MotorConfig motorLeftHalConfig;
static HAL_MotorConfig motorRightHalConfig;
static PID_Controller motorPicController;
static MotorDriver motorLeftDriver;
static MotorDriver motorRightDriver;
static MotorService motorService;

static HAL_ServoConfig servoHalConfig;
static ServoDriver servoDriver;
static SteerConfig steerConfig;
static SteerInstance steerInstance;

static MotionContext motionContext;
static CmdProcessorContext commandContext;

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
    motorLeftHalConfig = (HAL_MotorConfig){
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

    motorRightHalConfig = (HAL_MotorConfig){
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

    motorSpec = (MotorSpec){
        .encoderPPR = 13,
        .gearRatio = 30,
        .maxRPM = 400,
        .wheelRadiusMM = DEFAULT_CHASSIS_CFG.wheelbaseMM
    };

    motorPicController = (PID_Controller){
        .params = {
            .kp = 1.0f,
            .ki = 0.1f,
            .kd = 0.05f,
            .integral_max = 100.0f,
            .output_max = 95.0f
        }
    };

    motorLeftDriver = (MotorDriver){
        .halCfg = &motorLeftHalConfig,
        .spec = &motorSpec,
        .control = {
            .mode = MOTOR_DRIVER_MODE_STOP,
            .pidCtrl = motorPicController,
            .targetRPM = 0.0f,
        }
    };

    motorRightDriver = (MotorDriver){
        .halCfg = &motorRightHalConfig,
        .spec = &motorSpec,
        .control = {
            .mode = MOTOR_DRIVER_MODE_STOP,
            .pidCtrl = motorPicController,
            .targetRPM = 0.0f,
        }
    };

    motorService = (MotorService){
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

    servoHalConfig = (HAL_ServoConfig){
        .pwmTim = &htim2,
        .channel = TIM_CHANNEL_1,
        .maxPulse = SERVO_MAX_PULSE,
        .minPulse = SERVO_MIN_PULSE
    };

    servoDriver = (ServoDriver){
        .hw = &servoHalConfig,
        .spec = {
            .minAngle = SERVO_ANGLE_MIN,
            .maxAngle = SERVO_ANGLE_MAX,
            .minPulseUs = SERVO_MIN_PULSE,
            .maxPulseUs = SERVO_MAX_PULSE,
            .resolutionDeg = 0.1f
        }
    };

    steerConfig = (SteerConfig){
        .minAngleDeg = 0,
        .maxAngleDeg = 180,
        .deadZoneDeg = 2,
        .updateIntervalMs = 20,
        .maxSpeedDegPerSec = 90.0f,      // 最大速度：90°/秒（即 0.025°/ms）
        .accelerationDegPerSec2 = 180.0f // 加速度：180°/秒²（即 0.18°/ms²）
    };

    steerInstance = (SteerInstance){
        .driver = &servoDriver
    };


    // 初始化转向服务
    if (SteerService_init(&steerInstance, &steerConfig, &servoDriver) != ERR_SUCCESS)
    {
        return ERR_HW_INIT_FAIL;
    }


    // 初始化运动控制
    motionContext = (MotionContext){
        .chassisConfig = DEFAULT_CHASSIS_CFG,
        .motorService = &motorService,
        .steerServo = &steerInstance
    };
    if (MotionContext_Init(&motionContext, &motorService, &steerInstance, &DEFAULT_CHASSIS_CFG) != ERR_SUCCESS)
    {
        return ERR_HW_INIT_FAIL;
    }

    // 初始化命令处理器
    commandContext = (CmdProcessorContext){
        .motionContext = &motionContext
    };
    CmdProcessor_Init(&commandContext, &motionContext);


    sysCtx.motionContext = &motionContext;
    sysCtx.motorService = &motorService;
    sysCtx.cmdContext = &commandContext;
    sysCtx.steerInstance = &steerInstance;

    return ERR_SUCCESS;
}

SystemContext* System_GetContext(void)
{
    return &sysCtx;
}
