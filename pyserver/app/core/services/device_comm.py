from app.core.models.api_model import CommandResponse
from app.core.models.command import DeviceCommand, PingCommand, ServoCommand
from app.device.mqtt_client import MQTTClientWrapper
from app.device.protocol.builder import DevicePayload, ProtocolFrameBuilder
from app.device.protocol.common import ProtocolType


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
            if cmd_type == DeviceCommand.CommandType.PING:
                if isinstance(command.payload, PingCommand):
                    ping_cmd = command.payload
                    payload = DevicePayload().build_text(ping_cmd.echo_message)
                    frame = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value).set_payload(
                        payload).build_frame()
            elif cmd_type == DeviceCommand.CommandType.SERVO:
                if isinstance(command.payload, ServoCommand):
                    servo_cmd = command.payload
                    payload = DevicePayload().build_servo(servo_cmd.angle)
                    frame = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value).set_payload(
                        payload).build_frame()
            else:
                return CommandResponse(
                    status=500,
                    message=f"指令类型 '{cmd_type}' 不支持"
                )

            # 2. 指令下发
            await self.mqtt.publish(
                topic=f"cmd/{device_id}/+/exec",
                payload=frame
            )
            return CommandResponse(data=command)
        except Exception as e:
            return CommandResponse(
                status=500,
                message=f"指令下发失败，请检查设备是否在线，错误信息：{str(e)}"
            )
