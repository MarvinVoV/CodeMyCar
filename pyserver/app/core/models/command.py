from datetime import datetime
from enum import IntEnum
from typing import Union

from pydantic import Field, model_validator, RootModel, BaseModel

from core.models.api_model import BaseResponse


class MotionReqMode(IntEnum):
    """运动控制模式"""
    EMERGENCY_STOP = 0
    DIRECT_CONTROL = 1,
    DIFFERENTIAL = 2,
    STEER_ONLY = 3,
    SPIN_IN_PLACE = 4,
    MIXED_STEER = 5


class DiffCtrlReqParam(BaseModel):
    """差速控制参数"""
    linearVel: int = Field(
        ..., ge=0, le=65535,
        description="线速度（0.01 m/s步长，最大655.35 m/s）"
    )
    angularVel: int = Field(
        ..., ge=0, le=65535,
        description="角速度（0.001 rad/s步长，最大65.535 rad/s）"
    )


class DirectCtrlReqParam(BaseModel):
    """直接控制参数"""
    leftRpm: int = Field(
        ..., ge=-32768, le=32767,
        description="左电机转速（RPM，16位有符号整数）"
    )
    rightRpm: int = Field(
        ..., ge=-32768, le=32767,
        description="右电机转速（RPM，16位有符号整数）"
    )
    steerAngle: int = Field(
        ..., ge=-32768, le=32767,
        description="舵机角度（0.1度步长，16位有符号整数）"
    )


class AckermannReqParam(BaseModel):
    """阿克曼转向参数"""
    linearVel: int = Field(
        ..., ge=0, le=65535,
        description="线速度（0.01 m/s步长）"
    )
    steerAngle: int = Field(
        ..., ge=-32768, le=32767,
        description="转向角度（0.1度步长）"
    )


class MixedCtrlReqParam(BaseModel):
    """混合控制参数"""
    base: DiffCtrlReqParam
    steerAngle: int = Field(
        ..., ge=-32768, le=32767,
        description="舵机角度（0.1度步长）"
    )
    differentialGain: int = Field(
        ..., ge=0, le=100,
        description="差速增益（0-100%，步长0.392%）"
    )


class MotionCmdReqParams(RootModel):
    """运动控制参数联合体"""
    root: Union[
        DiffCtrlReqParam,
        DirectCtrlReqParam,
        AckermannReqParam,
        MixedCtrlReqParam
    ]


class MotionReqCmd(BaseModel):
    """运动控制指令"""
    mode: MotionReqMode
    params: MotionCmdReqParams
    duration: int = Field(
        ..., ge=0, le=0xFFFF,
        description="执行持续时间（毫秒，16位无符号）"
    )

    @model_validator(mode='after')
    def validate_params_mode(self):
        if self.mode == MotionReqMode.EMERGENCY_STOP and self.params is not None:
            raise ValueError("紧急停止模式不应包含参数")
        return self


class ControlReqCmd(BaseModel):
    ctrlId: int = Field(
        ..., ge=0, le=255,
        description="控制器ID（8位无符号）"
    )
    motion: MotionReqCmd  # 删除 fields 和 pingText 字段

    @model_validator(mode='after')
    def validate_motion(self):
        if not self.motion:
            raise ValueError("运动控制指令必须存在")
        return self

    # 删除原来的 validate_fields 方法
    # def validate_fields(self):
    #     ...


class StatusResponse(BaseResponse):
    """设备状态响应模型"""

    class StatusData(BaseModel):
        online: bool
        last_heartbeat: datetime

    data: StatusData
