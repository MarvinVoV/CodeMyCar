import struct
import unittest

from app.device.protocol.builder import DevicePayload, ProtocolFrameBuilder, CtrlField
from app.device.protocol.common import ProtocolType, PROTOCOL_MAX_DATA_LEN
from app.device.protocol.parser import ProtocolParser
from app.device.protocol.builder import ControlCmd


class TestProtocolFrame(unittest.TestCase):
    def test_base_test(self):
        frame = struct.pack("<H B H", 0x0205, 0x01, 0x0304)
        # print hex
        print(" ".join("{:02x}".format(x) for x in frame))

    def test_frame_integrity(self):
        """测试完整帧构建与解析流程"""
        # 构建测试载荷
        payload = DevicePayload(ctrl_id=0).add_motor_diff(linear=1000, angular=100, accel=80).add_servo(angle=90, speed=100).add_text(message='hello')

        # 构建协议帧
        builder = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value)
        frame = builder.set_payload(payload).build_frame()

        print("\n")
        print(" ".join("{:02x}".format(x) for x in frame))
        # 解析协议帧
        parsed = ProtocolFrameBuilder.parse_frame(frame)

        # 验证元数据
        self.assertEqual(parsed["protocol_type"], ProtocolType.CONTROL.value)
        payload = parsed['payload']
        cmd = ControlCmd.from_bytes(payload)
        self.assertTrue(cmd.ctrl_fields & CtrlField.SERVO)
        print(cmd.servo_data.angle)

    def test_crc_error_detection(self):
        payload = DevicePayload(ctrl_id=0).add_text(message='text')
        """测试CRC错误检测"""
        valid_frame = ProtocolFrameBuilder(0x02).set_payload(payload).build_frame()

        # 篡改数据
        corrupted = bytearray(valid_frame)
        corrupted[10] ^= 0xFF  # 修改数据部分

        with self.assertRaises(ValueError):
            ProtocolFrameBuilder.parse_frame(bytes(corrupted))

    def test_boundary_conditions(self):
        """测试最大数据长度限制"""
        # 生成边界数据
        max_payload = b"\xFF" * PROTOCOL_MAX_DATA_LEN
        builder = ProtocolFrameBuilder(0x03).set_payload(max_payload)

        # 正常构建
        self.assertTrue(len(builder.build_frame()) > 0)

        # 超出限制
        with self.assertRaises(ValueError):
            builder.set_payload(max_payload + b"\x00").build_frame()

    def test_parser_text(self):
        payload = DevicePayload(ctrl_id=0).add_text("HelloWorld")

        print(" ".join("{:02x}".format(x) for x in payload.raw_bytes))
        frame = ProtocolFrameBuilder(ProtocolType.CONTROL).set_payload(payload).build_frame()
        print(" ".join("{:02x}".format(x) for x in frame))
        parsed = ProtocolFrameBuilder.parse_frame(frame)
        # 验证元数据
        self.assertEqual(parsed["protocol_type"], ProtocolType.CONTROL.value)
        payload = parsed['payload']
        cmd = ControlCmd.from_bytes(payload)
        self.assertTrue(cmd.ctrl_fields & CtrlField.TEXT)
        print(cmd.text_data.message)



if __name__ == "__main__":
    unittest.main()
