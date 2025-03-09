from datetime import datetime

from fastapi import APIRouter, Depends, Body

from app.config.dependencies import get_comm_service
from app.core.logger import get_logger
from app.core.models.api_model import CommandResponse
from app.core.models.api_model import ErrorResponse
from app.core.models.command import StatusResponse, DeviceCommand
from app.core.services.device_comm import DeviceCommunicationService
from app.core.exception.errorcode import ErrorCode
from app.core.exception.exceptions import AppException

logger = get_logger()

router = APIRouter(prefix="/api/v1/devices", tags=["Devices"])


@router.post("/{device_id}/commands",
             response_model=CommandResponse,
             responses={
                 400: {"model": ErrorResponse},
                 404: {"model": ErrorResponse},
                 500: {"model": ErrorResponse}
             })
async def send_device_command(
        device_id: str,
        command: DeviceCommand = Body(...),
        comm_service: DeviceCommunicationService = Depends(get_comm_service)
):
    """
    设备指令下发接口

    - **device_id**: 设备物理编号（格式：DEV-四位数字）
    - **command**: 设备指令对象
    - **returns**: 包含执行结果的响应对象
    """
    logger.info(
        "Processing device command",
        device_id=device_id,
        command_type=command.command_type.value
    )
    try:
        # 调用服务层处理核心逻辑
        result = await comm_service.process_command(
            device_id=device_id,
            command=command,
        )
        return result

    except AppException as e:
        logger.error("send command error", error=str(e))
        raise AppException(
            code=ErrorCode.UNKNOWN_ERROR,
            message="未知错误",
            detail={"unknown_error": e.args[0]}
        )


@router.get(
    "/status/{device_id}",
    response_model=StatusResponse
)
async def get_device_status(device_id: str):
    """获取设备状态接口"""
    return StatusResponse(
        data={
            "online": True,
            "last_heartbeat": datetime.now(),
            "device_id": device_id
        }
    )
