from fastapi import Depends
from fastapi import Request

from app.device.mqtt_client import MQTTClientWrapper
from core.services.device_comm import DeviceCommunicationService



def get_mqtt_client(request: Request) -> MQTTClientWrapper:
    """从应用状态中获取已初始化的MQTT客户端单例"""
    return request.app.state.mqtt_client

def get_comm_service(
        mqtt: MQTTClientWrapper = Depends(get_mqtt_client)
) -> DeviceCommunicationService:
    """获取通信服务实例"""
    return DeviceCommunicationService(mqtt)
