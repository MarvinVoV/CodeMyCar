from fastapi import Depends

from app.device.mqtt_client import MQTTClientWrapper, get_mqtt_client
from core.services.device_comm import DeviceCommunicationService


def get_comm_service(
        mqtt_client_wrapper: MQTTClientWrapper = Depends(get_mqtt_client)
) -> DeviceCommunicationService:
    """获取通信服务实例"""
    return DeviceCommunicationService(mqtt_client_wrapper)
