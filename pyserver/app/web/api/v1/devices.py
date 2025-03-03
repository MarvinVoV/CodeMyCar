from fastapi import APIRouter, Depends
from app.device.mqtt_client import get_mqtt_client
from app.core.schemas import DeviceCommand

from app.core.exceptions import CustomHTTPException
from app.core.logger import get_logger

router = APIRouter(prefix="/api/v1/devices", tags=["Devices"])
logger = get_logger("api.devices")

@router.post("/{device_id}/commands")
async def send_device_command(
        device_id: str,
        command: DeviceCommand,
        mqtt=Depends(get_mqtt_client)
):
    # 结构化日志示例
    logger.info(
        "Sending device command",
        device_id=device_id,
        action="command_send"
    )
    try:
        topic = f"devices/{device_id}/commands"
        await mqtt.publish(topic, command.json())
        return {"status": "command_sent"}
    except Exception as e:
        logger.error(
            "Command failed",
            exc_info=e,
            stack_info=True,
            device_id=device_id
        )
        raise CustomHTTPException(
            status_code=500,
            code=5001,
            message="Device Communication Error",
            detail=str(e)
        )
