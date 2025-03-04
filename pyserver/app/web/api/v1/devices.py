from datetime import datetime

from fastapi import APIRouter, Depends, Body

from app.core.logger import get_logger
from app.core.models.command import StatusResponse, DeviceCommand
from app.core.models.api_model import BaseRequest, BaseResponse, ErrorResponse
from app.core.services.device_comm import DeviceCommunicationService
from app.config.dependencies import get_comm_service

router = APIRouter(prefix="/api/v1/devices", tags=["Devices"])
logger = get_logger("api.devices")


@router.post("/{device_id}/commands",
             response_model=BaseResponse,
             responses={
                 400: {"model": ErrorResponse},
                 404: {"model": ErrorResponse},
                 500: {"model": ErrorResponse}
             })
async def send_device_command(
        device_id: str,
        request: BaseRequest,
        command: DeviceCommand =  Body(...),
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
            request=request
        )
        # todo
        return result


    except Exception as e:
        pass
        # logger.warning("Device offline", device_id=device_id)
        # raise CustomHTTPException(
        #     status_code=503,
        #     code=ErrorCode.DEVICE_OFFLINE,
        #     message="设备离线",
        #     detail={"last_online": e.last_online.isoformat()}
        # )
    # except ProtocolEncodeError as e:
    #     logger.error("Protocol encode failed", error=str(e))
    #     raise CustomHTTPException(
    #         status_code=400,
    #         code=ErrorCode.INVALID_COMMAND,
    #         message="指令格式错误",
    #         detail={"validation_error": e.args[0]}
    #     )

@router.get(
    "/status/{device_id}",
    response_model=StatusResponse
)
async def get_device_status(
        device_id: str,
        request: BaseRequest
):
    """获取设备状态接口"""
    return StatusResponse(
        request_id=request.request_id,
        data={
            "online": True,
            "last_heartbeat": datetime.now(),
            "device_id": device_id
        }
    )
