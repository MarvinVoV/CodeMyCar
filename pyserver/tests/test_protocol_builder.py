import unittest

from device.protocol import DevicePayload, MotorCommandType


class TestDevicePayload(unittest.TestCase):
    def setUp(self):
        """初始化测试环境"""
        self.test_acceleration = 50
        self.test_speeds = (120, 110)
        self.test_message = "Debug_Test_123"

    def test_motor_payload(self):
        """测试电机控制载荷"""
        # 构建载荷
        payload = DevicePayload().build_motor(
            cmd_type=MotorCommandType.FORWARD,
            left=self.test_speeds[0],
            right=self.test_speeds[1],
            acc=self.test_acceleration
        )

        # 序列化与反序列化
        parsed = payload.parse()

        # 验证结果
        self.assertEqual(parsed["type"], "motor")
        self.assertEqual(parsed["data"]["left"], self.test_speeds[0])
        self.assertEqual(parsed["data"]["acc"], self.test_acceleration)

    def test_invalid_motor_acceleration(self):
        """测试异常加速度值"""
        with self.assertRaises(ValueError):
            DevicePayload().build_motor(
                cmd_type=MotorCommandType.FORWARD,
                left=100,
                right=100,
                acc=101  # 超出范围
            )

    def test_text_message_truncation(self):
        """测试长消息截断功能"""
        long_msg = "A" * 100  # 超过64字节限制
        payload = DevicePayload().build_text(long_msg)
        parsed = payload.parse()

        self.assertEqual(len(parsed["message"]), 64)


if __name__ == "__main__":
    unittest.main()
