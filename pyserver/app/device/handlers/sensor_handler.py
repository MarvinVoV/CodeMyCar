# app/device/handlers/sensor_handler.py
from app.core.logger import get_logger

logger = get_logger("handler.sensor")


async def handle_sensor_data(data: dict):
    """处理传感器数据"""
    try:
        # todo
        logger.debug(f"Saved sensor data")

    except (KeyError, ValueError) as e:
        logger.error(f"Invalid sensor data: {str(e)}")
