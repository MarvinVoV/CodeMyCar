from app.core.logger import get_logger

logger = get_logger("handler.status")


async def handle_device_status(data: dict):
    """处理设备状态上报"""
    try:
        device_id = data["device_id"]
        # TODO
        logger.info(f"Updated {device_id} status")

    except KeyError as e:
        logger.error(f"Missing field in status data: {str(e)}")