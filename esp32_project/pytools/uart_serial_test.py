import serial
import sys
import argparse
from time import sleep
import struct

# 协议常量定义
HEADER = 0xAA
FRAME_END = 0x55
MAX_DATA_LEN = 512


# 命令类型枚举
class ProtocolType:
    SENSOR = 0x01
    CONTROL = 0x02
    LOG = 0x03


def calculate_checksum(data):
    """计算异或校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(protocol_type, data):
    """构建协议帧"""
    if len(data) > MAX_DATA_LEN:
        raise ValueError(f"Data length exceeds maximum limit ({MAX_DATA_LEN})")

    # 构建基础帧
    frame = bytes([HEADER, protocol_type]) + struct.pack("<H", len(data))

    # 添加数据
    frame += bytes(data)

    # 计算校验和
    checksum = calculate_checksum(frame)
    frame += bytes([checksum])

    # 添加帧尾
    frame += bytes([FRAME_END])

    return frame


def send_test_frame(port, baudrate, protocol_type, data):
    """发送测试帧"""
    try:
        with serial.Serial(port, baudrate, timeout=10) as ser:
            frame = build_frame(protocol_type, data)
            print(f"Sending frame: {frame.hex(' ').upper()}")
            if ser.is_open:
                print("Serial port is open")
            else:
                print("Serial port is not open")
            # check write result
            if ser.write(frame) != len(frame):
                print("Error: Failed to write data to serial port")
                sys.exit(1)
            sleep(0.1)  # 等待数据发送完成
    except Exception as e:
        print(f"Error: {str(e)}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="UART Serial Tester")
    parser.add_argument(
        "-p", "--port", default="/dev/tty.usbmodem575B0215341", help="Serial port name"
    )
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "-t",
        "--type",
        type=int,
        required=True,
        choices=[ProtocolType.SENSOR, ProtocolType.CONTROL, ProtocolType.LOG],
        help="Command type (1:SENSOR, 2:CONTROL, 3:LOG)",
    )
    parser.add_argument(
        "-d", "--data", default="", help="Hex data string (e.g. 'A1B2C3')"
    )

    args = parser.parse_args()

    # 转换数据格式
    try:
        data = bytes.fromhex(args.data) if args.data else bytes()
    except ValueError:
        print("Invalid hex data format")
        sys.exit(1)

    send_test_frame(args.port, args.baud, args.type, data)


if __name__ == "__main__":
    main()
