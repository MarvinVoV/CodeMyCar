from app.core.logger import get_logger
from app.core.models.api_model import CommandResponse
from app.core.models.command import ControlReqCmd, MotionReqCmd, MotionReqMode
from app.device.mqtt_client import MQTTClientWrapper
from app.device.protocol.builder import DevicePayload, ProtocolFrameBuilder, MotionMode
from app.device.protocol.common import ProtocolType

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
            command: ControlReqCmd,
    ) -> CommandResponse:
        try:
            # 处理运动控制指令
            motion_cmd = command.motion
            if not isinstance(motion_cmd, MotionReqCmd):
                return CommandResponse(status=400, message="运动控制指令格式错误")
            params = motion_cmd.params.root
            duration = motion_cmd.duration
            if motion_cmd.mode == MotionReqMode.DIFFERENTIAL:
                payload = DevicePayload().add_motor_diff(
                    params.linearVel, params.angularVel, duration
                )
            elif motion_cmd.mode == MotionReqMode.DIRECT_CONTROL:
                payload = DevicePayload().add_motor_direct(
                    params.leftRpm, params.rightRpm, params.steerAngle, duration
                )
            elif motion_cmd.mode == MotionReqMode.STEER_ONLY:
                # 新增对STEER_ONLY模式的支持
                payload = DevicePayload().add_steering_only(
                    params.linearVel, params.steerAngle, duration
                )
            elif motion_cmd.mode == MotionReqMode.SPIN_IN_PLACE:
                # 新增对SPIN_IN_PLACE模式的支持（假设仅需要持续时间）
                payload = DevicePayload().add_spin_in_place(duration)
            elif motion_cmd.mode == MotionReqMode.MIXED_STEER:
                # 新增对MIXED_STEER模式的支持
                payload = DevicePayload().add_mixed_steering(
                    params.base.linearVel,
                    params.base.angularVel,
                    params.steerAngle,
                    params.differentialGain,
                    duration
                )
            else:
                return CommandResponse(
                    status=400,
                    message=f"不支持的运动控制模式: {motion_cmd.mode.name}"
                )
            frame = ProtocolFrameBuilder(ProtocolType.CONTROL.value).set_payload(payload).build_frame()
            success = await self.mqtt.publish(
                topic=f"cmd/esp32/{device_id}/exec",
                payload=frame
            )
            logger.info(f"指令下发结果：{success}")
            return CommandResponse(data={"motion_cmd_mode": motion_cmd.mode.name, "payload_size": len(frame)})
        except Exception as e:
            logger.error(f"指令处理失败：{str(e)}")
            return CommandResponse(
                status=500,
                message=f"设备{device_id}指令处理异常：{str(e)}"
            )
