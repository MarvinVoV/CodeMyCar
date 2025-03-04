from app.core.models.api_model import BaseRequest
from app.core.models.command import DeviceCommand
from app.device.mqtt_client import MQTTClientWrapper
from core.models.api_model import BaseResponse


class DeviceCommunicationService:
    def __init__(
            self,
            mqtt_client: MQTTClientWrapper,
    ):
        self.mqtt = mqtt_client

    async def process_command(
            self,
            device_id: str,
            command: DeviceCommand,
            request: BaseRequest
    ) -> BaseResponse:
        """处理指令下发全流程"""
        # 1. 协议转换

        # 2. 指令下发
        # await self.mqtt.publish(
        #     topic=f"devices/{device_id}/cmds",
        #     payload=binary_data,
        #     qos=1
        # )

        return None

    async def check_device_online(self, device_id: str) -> bool:
        """检查设备在线状态（示例实现）"""
        # todo
        return True
