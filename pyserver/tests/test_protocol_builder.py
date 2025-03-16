import unittest

from app.device.protocol.builder import DevicePayload, ControlCmd, CtrlField


class TestDevicePayload(unittest.TestCase):
    def setUp(self):
        pass

    def test_motor_payload(self):
        """测试电机控制载荷"""
        # 构建载荷
        payload = (DevicePayload(ctrl_id=0)
                   .add_motor_diff(
            linear=1000,
            angular=100,
            accel=80
        ).add_servo(
            angle=90,
            speed=100
        ).add_text(
            message='hello'
        ).raw_bytes)

        # 序列化与反序列化
        # 解析指令
        cmd = ControlCmd.from_bytes(payload)
        if cmd.ctrl_fields & CtrlField.MOTOR:
            print(f"电机模式: {cmd.motor_mode.name}")
            print(f"差速参数: {cmd.motor_data}")
        if cmd.ctrl_fields & CtrlField.SERVO:
            print(f"舵机参数: {cmd.servo_data}")
        if cmd.ctrl_fields & CtrlField.TEXT:
            print(f"文本参数: {cmd.text_data}")


if __name__ == "__main__":
    unittest.main()
