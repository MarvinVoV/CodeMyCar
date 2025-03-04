from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field, ConfigDict


class BaseRequest(BaseModel):
    """基础请求模型（Pydantic v2 语法）"""
    model_config = ConfigDict(
        json_schema_extra={
            "examples": [{
                "request_id": "req_123456",
                "timestamp": 1691234567.890
            }]
        }
    )

    request_id: str = Field(
        ...,
        min_length=8,
        max_length=32,
        json_schema_extra={"examples": ["req_123456"]},
        description="唯一请求标识"
    )
    timestamp: float = Field(
        default_factory=lambda: datetime.now().timestamp(),
        json_schema_extra={"examples": [1691234567.890]},
        description="客户端时间戳"
    )


class BaseResponse(BaseRequest):
    """基础响应模型"""
    model_config = ConfigDict(
        json_schema_extra={
            "examples": [{
                "request_id": "req_123456",
                "timestamp": 1691234567.890,
                "status": 200,
                "message": "success"
            }]
        }
    )

    status: int = Field(
        default=200,
        ge=200,
        le=599,
        json_schema_extra={"examples": [200]},
        description="HTTP状态码"
    )
    message: str = Field(
        default="success",
        json_schema_extra={"examples": ["操作成功"]},
        description="人类可读的消息"
    )


class ErrorResponse(BaseResponse):
    """错误响应模型"""
    error_code: str
    error_detail: Optional[dict] = None
