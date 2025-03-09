from typing import Union

from paho.mqtt.client import topic_matches_sub

from app.core.logger import get_logger
from app.device.topic import Topic
from device.protocol.builder import LogParser

logger = get_logger(__name__)
logParser = LogParser()


async def handle_device_log(topic: str, payload: Union[bytes, dict, str]):
    logger.info(f"Received {len(payload)} bytes on {topic}")
    """处理设备状态上报"""
    try:
        # if topic as string contains esp32 then
        if 'esp32' in topic.lower():
            if isinstance(payload, bytes):
                # todo fix
                logger.log(payload.decode('utf-8'))
        elif 'stm32' in topic.lower():
            if isinstance(payload, bytes):
                logger.info(" ".join([f"{b:02x}" for b in payload]))
                log_entry = logParser.parse(payload)
                logger.info(f"Received log entry: {log_entry.to_json()}")
        else:
            pass



    #     if not topic_matches_sub(Topic.TOPIC_SUB_LOG, topic):
    #         logger.error(f"Invalid topic: {topic}")
    #     else:
    #         log_entry = logParser.parse(payload)
    #         logger.info(f"Received log entry: {log_entry.to_json()}")

    except KeyError as e:
        logger.error(f"Missing field in status data: {str(e)}")
