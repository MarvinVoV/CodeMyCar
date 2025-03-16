from datetime import datetime
from enum import Enum
from typing import Optional, Union

from pydantic import BaseModel, Field, ConfigDict, field_validator
from pydantic_core.core_schema import ValidationInfo

from app.core.models.api_model import BaseResponse


class MotorDiffControl(BaseModel):
    """差速控制模式参数"""
    linear_vel: int = Field(
        ...,
        ge=-3000,
        le=3000,
        json_schema_extra={"example": 1000},
        description="线速度（mm/s），正负表示方向"
    )
    angular_vel: int = Field(
        ...,
        ge=-1800,
        le=1800,
        json_schema_extra={"example": 500},
        description="角速度（0.1°/s），正负表示方向"
    )
    accel: int = Field(
        50,
        ge=0,
        le=100,
        json_schema_extra={"example": 80},
        description="加速度（0-100%）"
    )


class MotorDirectControl(BaseModel):
    """直接控制模式参数"""
    left_speed: int = Field(
        ...,
        ge=-1000,
        le=1000,
        json_schema_extra={"example": 800},
        description="左电机速度（-1000~1000）"
    )
    right_speed: int = Field(
        ...,
        ge=-1000,
        le=1000,
        json_schema_extra={"example": -600},
        description="右电机速度（-1000~1000）"
    )
    left_accel: int = Field(
        50,
        ge=0,
        le=100,
        description="左电机加速度（0-100%）"
    )
    right_accel: int = Field(
        50,
        ge=0,
        le=100,
        description="右电机加速度（0-100%）"
    )


class MotorCommand(BaseModel):
    """电机控制指令参数模型"""
    mode: str = Field(
        ...,
        pattern="^(diff|direct|emergency)$",
        json_schema_extra={"example": "diff"},
        description="控制模式：diff-差速模式, direct-直接控制, emergency-紧急制动"
    )
    params: Union[MotorDiffControl, MotorDirectControl, None] = Field(
        None,
        description="控制参数（紧急模式无需参数）"
    )

    @field_validator('params')
    def validate_params(cls, v, info: ValidationInfo):
        mode = info.data.get('mode')
        if mode in ['diff', 'direct'] and v is None:
            raise ValueError(f"{mode}模式需要参数")
        if mode == 'emergency' and v is not None:
            raise ValueError("紧急模式不需要参数")
        return v


class ServoCommand(BaseModel):
    """舵机控制指令参数模型"""
    angle: int = Field(
        ...,
        ge=0,
        le=180,
        json_schema_extra={"example": 90},
        description="舵机目标角度（0-180度）"
    )
    speed: int = Field(
        50,
        ge=0,
        le=100,
        json_schema_extra={"example": 75},
        description="转向速度（0-100%）"
    )


class PingCommand(BaseModel):
    """调试指令参数模型"""
    message: str = Field(
        ...,
        min_length=1,
        max_length=255,
        json_schema_extra={"example": "System OK"},
        description="调试消息内容（最大255字符）"
    )


# ================ 主指令模型 ================

class CommandType(str, Enum):
    MOTOR = "motor"
    SERVO = "servo"
    PING = "ping"

class DeviceCommand(BaseModel):
    """设备控制指令模型"""
    command_type: CommandType
    payload: Union[MotorCommand, ServoCommand, PingCommand] = Field(
        ...,
        description="根据指令类型变化的载荷数据"
    )

    model_config = ConfigDict(
        json_schema_extra={
            "examples": [
                {
                    "command_type": "motor",
                    "payload": {
                        "mode": "diff",
                        "params": {
                            "linear_vel": 1000,
                            "angular_vel": 500,
                            "accel": 80
                        }
                    }
                },
                {
                    "command_type": "servo",
                    "payload": {
                        "angle": 90,
                        "speed": 50
                    }
                }
            ]
        }
    )


class StatusResponse(BaseResponse):
    """设备状态响应模型"""

    class StatusData(BaseModel):
        online: bool
        last_heartbeat: datetime

    data: StatusData
