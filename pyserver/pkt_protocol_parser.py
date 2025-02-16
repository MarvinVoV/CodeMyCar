import struct
from enum import Enum, auto

# 定义协议常量
FRAME_HEADER = 0xAA55
FRAME_TAIL = 0x55AA
PROTOCOL_MAX_DATA_LEN = 512

# 协议类型
PROTOCOL_TYPE_SENSOR = 0x01
PROTOCOL_TYPE_CONTROL = 0x02
PROTOCOL_TYPE_LOG = 0x03


# 状态机定义为枚举
class ParseState(Enum):
    WAIT_HEADER_1 = auto()
    WAIT_HEADER_2 = auto()
    WAIT_TYPE = auto()
    WAIT_LENGTH_1 = auto()
    WAIT_LENGTH_2 = auto()
    WAIT_DATA = auto()
    WAIT_CRC_1 = auto()
    WAIT_CRC_2 = auto()
    WAIT_TAIL_1 = auto()
    WAIT_TAIL_2 = auto()


# 协议帧解析器
class ProtocolParser:
    def __init__(self):
        self.crc = None
        self.state = ParseState.WAIT_HEADER_1
        self.frame = None
        self.data_index = 0

    def parse_byte(self, byte):
        """解析一个字节"""
        if self.state == ParseState.WAIT_HEADER_1:
            if byte == (FRAME_HEADER & 0xFF):
                self.state = ParseState.WAIT_HEADER_2
        elif self.state == ParseState.WAIT_HEADER_2:
            if byte == ((FRAME_HEADER >> 8) & 0xFF):
                self.state = ParseState.WAIT_TYPE
        elif self.state == ParseState.WAIT_TYPE:
            self.frame = {"header": FRAME_HEADER, "type": byte, "len": 0, "data": []}
            self.state = ParseState.WAIT_LENGTH_1
        elif self.state == ParseState.WAIT_LENGTH_1:
            self.frame["len"] = byte
            self.state = ParseState.WAIT_LENGTH_2
        elif self.state == ParseState.WAIT_LENGTH_2:
            self.frame["len"] |= byte << 8
            if self.frame["len"] > PROTOCOL_MAX_DATA_LEN:
                raise ValueError("Data length exceeds maximum")
            self.state = ParseState.WAIT_DATA
        elif self.state == ParseState.WAIT_DATA:
            self.frame["data"].append(byte)
            self.data_index += 1
            if self.data_index == self.frame["len"]:
                self.state = ParseState.WAIT_CRC_1
        elif self.state == ParseState.WAIT_CRC_1:
            self.crc = byte
            self.state = ParseState.WAIT_CRC_2
        elif self.state == ParseState.WAIT_CRC_2:
            self.crc |= byte << 8
            # 可以在此校验CRC
            self.state = ParseState.WAIT_TAIL_1
        elif self.state == ParseState.WAIT_TAIL_1:
            if byte == (FRAME_TAIL & 0xFF):
                self.state = ParseState.WAIT_TAIL_2
        elif self.state == ParseState.WAIT_TAIL_2:
            if byte == ((FRAME_TAIL >> 8) & 0xFF):
                return 1  # 解析完成
            else:
                self.state = ParseState.WAIT_HEADER_1

        return 0  # 解析未完成

    def get_parsed_data(self):
        """获取解析出来的帧数据"""
        return self.frame

    def print_hex(self):
        """打印数据帧的 hex 格式，空格间隔"""
        data = self.frame
        hex_data = f"Header: {data['header']:04X}, Type: {data['type']:02X}, Length: {data['len']:04X}, "
        hex_data += "Data: " + " ".join(f"{byte:02X}" for byte in data["data"])
        hex_data += f", CRC: {self.crc:04X}, Tail: {FRAME_TAIL:04X}"
        print(hex_data)

    def print_plain_text(self):
        """打印数据帧的 plain text 格式，空格间隔"""
        data = self.frame['data']
        print("Plain Text: " + data)

    def print_log_data(self):
        """打印日志内容"""
        data = self.frame
        if data["type"] == PROTOCOL_TYPE_LOG:
            # 解析时间戳 (uint32_t)
            timestamp = struct.unpack_from('<I', bytes(data["data"]), 0)[0]
            # 解析模块标识 (uint16_t)
            module = struct.unpack_from('<H', bytes(data["data"]), 4)[0]
            # 解析日志级别 (uint8_t)
            level = struct.unpack_from('<B', bytes(data["data"]), 6)[0]
            # 解析日志正文 (固定长度字符串，128字节)
            message = struct.unpack_from('<128s', bytes(data["data"]), 7)[0].rstrip(b'\x00').decode("utf-8")

            print(f"Log data: timestamp={timestamp}, module={module}, level={level}, message={message}")
        else:
            print("Not a log frame")

# # 示例使用
# parser = ProtocolParser()

# # 假设接收到一个数据包的字节流
# data_stream = [
#     0x55,
#     0xAA,  # HEADER
#     0x01,  # type: SENSOR
#     0x02,
#     0x00,  # length: 512 bytes
#     *([0x01] * 512),  # data (512 bytes of 0x01)
#     0x12,
#     0x34,  # CRC
#     0xAA,
#     0x55,  # TAIL
# ]

# # 模拟解析过程
# for byte in data_stream:
#     result = parser.parse_byte(byte)
#     if result:
#         print("解析完成")
#         parser.print_hex()
#         break
