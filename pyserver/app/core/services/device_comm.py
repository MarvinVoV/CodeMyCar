from app.core.models.api_model import CommandResponse
from app.core.models.command import DeviceCommand, PingCommand, ServoCommand
from app.device.mqtt_client import MQTTClientWrapper
from app.device.protocol.builder import DevicePayload, ProtocolFrameBuilder
from app.device.protocol.common import ProtocolType
from app.core.logger import get_logger
from core.models.command import CommandType

logger = get_logger()

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
    ) -> CommandResponse:
        """处理指令下发全流程"""
        # 1. 构建指令帧
        frame = None
        cmd_type = command.command_type
        try:
            if cmd_type == CommandType.PING:
                if isinstance(command.payload, PingCommand):
                    ping_cmd = command.payload
                    payload = DevicePayload().build_text(ping_cmd.echo_message)
                    frame = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value).set_payload(
                        payload).build_frame()
                    # logger.info(" ".join(f"{byte:02X}" for byte in frame))
            elif cmd_type == CommandType.SERVO:
                if isinstance(command.payload, ServoCommand):
                    servo_cmd = command.payload
                    payload = DevicePayload().build_servo(servo_cmd.angle)
                    frame = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value).set_payload(
                        payload).build_frame()
                    # logger.info(" ".join(f"{byte:02X}" for byte in frame))
            else:
                return CommandResponse(
                    status=500,
                    message=f"指令类型 '{cmd_type}' 不支持"
                )

            # 2. 指令下发
            success = await self.mqtt.publish(
                topic=f"cmd/esp32/{device_id}/exec", # cmd/device_type/device_id/exec
                payload=frame
            )
            logger.info(f"指令下发结果：{success}")

            return CommandResponse(data={"command_type": cmd_type.value, "payload": command.payload})
        except Exception as e:
            logger.error(f"指令下发失败，错误信息：{str(e)}")
            return CommandResponse(
                status=500,
                message=f"指令下发失败，请检查设备{device_id}是否在线，错误信息：{str(e)}"
            )
