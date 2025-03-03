# 协议常量
from enum import IntEnum

FRAME_HEADER = 0xAA55
FRAME_TAIL = 0x55AA
PROTOCOL_MAX_DATA_LEN = 108

# 协议类型定义
class ProtocolType(IntEnum):
    SENSOR = 0x01
    CONTROL = 0x02
    LOG = 0x03

