import struct
import unittest

from device.protocol import MotorCommandType, PROTOCOL_MAX_DATA_LEN, ProtocolType, ProtocolParser
from device.protocol import ProtocolFrameBuilder, DevicePayload


class TestProtocolFrame(unittest.TestCase):
    def test_base_test(self):
        frame = struct.pack("<H B H", 0x0205, 0x01, 0x0304)
        # print hex
        print(" ".join("{:02x}".format(x) for x in frame))

    def test_frame_integrity(self):
        """测试完整帧构建与解析流程"""
        # 构建测试载荷
        payload = DevicePayload().build_motor(
            cmd_type=MotorCommandType.TURN_LEFT,
            left=200,
            right=180,
            acc=80
        )

        # 构建协议帧
        builder = ProtocolFrameBuilder(protocol_type=ProtocolType.CONTROL.value)
        frame = builder.set_payload(payload).build_frame()

        print(" ".join("{:02x}".format(x) for x in frame))
        # 解析协议帧
        parsed = ProtocolFrameBuilder.parse_frame(frame)

        # 验证元数据
        self.assertEqual(parsed["protocol_type"], ProtocolType.CONTROL.value)
        self.assertEqual(len(parsed["payload"]), 6)

        # # 验证载荷一致性
        # reconstructed = DevicePayload.from_bytes(parsed["payload"])
        # self.assertEqual(reconstructed.parse()["data"]["left"], 200)

    def test_crc_error_detection(self):
        """测试CRC错误检测"""
        valid_frame = ProtocolFrameBuilder(0x02).set_payload(b"test").build_frame()

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
        payload = DevicePayload().build_text("HelloWorld")
        print(" ".join("{:02x}".format(x) for x in payload.raw_bytes))
        frame = ProtocolFrameBuilder(ProtocolType.CONTROL).set_payload(payload).build_frame()
        print(" ".join("{:02x}".format(x) for x in frame))
        parser = ProtocolParser()
        for byte in frame:
            result = parser.parse_byte(byte)
            if result:
                print("解析完成")
                raw_data = parser.frame['data']
                print(" ".join("{:02x}".format(x) for x in raw_data))
                payload = DevicePayload.from_bytes(raw_data)
                print(payload.parse())
                break

    def test_parser_motor(self):
        payload = DevicePayload().build_motor(
            cmd_type=MotorCommandType.TURN_LEFT,
            left=200,
            right=180,
            acc=80
        )
        frame = ProtocolFrameBuilder(ProtocolType.CONTROL).set_payload(payload).build_frame()
        parser = ProtocolParser()
        for byte in frame:
            result = parser.parse_byte(byte)
            if result:
                print("解析完成")
                raw_data = parser.frame['data']
                print(" ".join("{:02x}".format(x) for x in raw_data))
                payload = DevicePayload.from_bytes(raw_data)
                print(payload.parse())
                print(payload.get_motor_params())
                break

    def test_parser_servo(self):
        payload = DevicePayload().build_servo(angle=90)
        frame = ProtocolFrameBuilder(ProtocolType.CONTROL).set_payload(payload).build_frame()
        parser = ProtocolParser()
        for byte in frame:
            result = parser.parse_byte(byte)
            if result:
                print("解析完成")
                raw_data = parser.frame['data']
                payload = DevicePayload.from_bytes(raw_data)
                print(payload.parse())
                break


if __name__ == "__main__":
    unittest.main()
