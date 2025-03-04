from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional, Dict


class HealthCheckResponse(BaseModel):
    """健康检查响应模型"""
    status: str = Field(..., json_schema_extra={"examples": ["ok"]})
    version: str = Field(..., json_schema_extra={"examples": ["1.2.3"]})
    timestamp: str = Field(..., json_schema_extra={"examples": datetime.utcnow().isoformat()})
    details: Optional[Dict] = Field(
        default=None,
        json_schema_extra={"database": "connected"}
    )


class SystemConfigResponse(BaseModel):
    """系统配置响应模型"""
    environment: str = Field(..., json_schema_extra={"examples": ["prod"]})
    log_level: str = Field(..., json_schema_extra={"examples": ["INFO"]})
    api_version: str = Field(..., json_schema_extra={"examples": ["/api/v1"]})
    mqtt_broker: str = Field(..., json_schema_extra={"examples": ["broker.example.com"]})
