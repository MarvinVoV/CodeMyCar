from typing import Optional

from pydantic import BaseModel, Field, ConfigDict


class BaseRequest(BaseModel):
    pass


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


class CommandResponse(BaseResponse):
    """响应模型"""
    data: Optional[dict] = None
