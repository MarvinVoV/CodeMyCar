from fastapi import APIRouter, Depends, status
from datetime import datetime
from app.core.schemas import (
    HealthCheckResponse,
    SystemConfigResponse
)
from core.exception.exceptions import CustomHTTPException
from app.config.settings import get_settings

router = APIRouter(prefix="/system", tags=["System"])


@router.get("/health", response_model=HealthCheckResponse)
async def health_check(settings=Depends(get_settings)):
    """系统健康检查端点"""
    try:
        # TODO 模拟检查数据库和MQTT连接状态
        health_status = {
            "status": "ok",
            "version": settings.PROTOCOL_VERSION,
            "timestamp": datetime.utcnow().isoformat(),
            "details": {
                "database": "connected",
                "mqtt": "connected" if hasattr(settings, "MQTT_BROKER") else "disabled"
            }
        }
        return health_status
    except Exception as e:
        raise CustomHTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            code=5002,
            message="Service Unavailable",
            detail=str(e)
        )


@router.get("/config", response_model=SystemConfigResponse)
async def get_system_config(settings=Depends(get_settings)):
    """获取系统配置（过滤敏感信息）"""
    return {
        "environment": settings.ENV,
        "log_level": settings.LOG_LEVEL,
        "api_version": settings.API_PREFIX,
        "mqtt_broker": settings.MQTT_BROKER
    }
