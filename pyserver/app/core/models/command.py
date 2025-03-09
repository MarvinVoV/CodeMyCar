from datetime import datetime
from enum import Enum
from typing import Optional, Union

from pydantic import BaseModel, Field, ConfigDict

from app.core.models.api_model import BaseResponse


class ServoCommand(BaseModel):
    """舵机控制指令参数模型"""
    angle: int = Field(
        ...,
        ge=0,
        le=180,
        json_schema_extra={"examples": [90]},
        description="舵机转动角度（0-180度）"
    )



class PingCommand(BaseModel):
    """Ping测试参数模型"""
    echo_message: Optional[str] = Field(
        ...,
        min_length=1,
        json_schema_extra={"examples": ["Hello Device!"]},
        description="Ping回显消息"
    )


class CommandType(str, Enum):
    CONTROL = "control"
    SERVO = "servo"
    PING = "ping"


class DeviceCommand(BaseModel):
    """设备指令模型（v2 语法）"""
    command_type: CommandType
    payload: Union[ServoCommand, PingCommand, dict] = Field(
        ...,
        json_schema_extra={
            "examples": [
                {"angle": 90},  # SERVO 示例
                {"test_mode": "quick"}  # CONTROL 示例
            ]
        },
        description="根据command_type变化的载荷数据"
    )

    model_config = ConfigDict(
        json_schema_extra={
            "examples": [{
                "command_type": "servo",
                "payload": {"angle": 45}
            }]
        }
    )


class StatusResponse(BaseResponse):
    """设备状态响应模型"""

    class StatusData(BaseModel):
        online: bool
        last_heartbeat: datetime

    data: StatusData
