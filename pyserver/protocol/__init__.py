"""
协议处理核心模块
"""
from .common import (
    FRAME_HEADER,
    FRAME_TAIL,
    PROTOCOL_MAX_DATA_LEN,
    ProtocolType
)
from .builder import MotorData, ControlCmd, MotorCommandType, CtrlType, DevicePayload, ProtocolFrameBuilder
from .parser import ProtocolParser

"""
设备通信协议处理核心包

包含协议常量定义、帧构建器、帧解析器等核心组件
主要功能：
- build_frame() 构建协议帧
- ProtocolParser 解析接收数据
- 提供 ProtocolType 等枚举类型
"""

__all__ = [
    'FRAME_HEADER', 'FRAME_TAIL', 'PROTOCOL_MAX_DATA_LEN',
    'ProtocolType', 'CtrlType', 'MotorCommandType',
    'MotorData', 'ControlCmd', 'ProtocolFrameBuilder',
    'ProtocolParser', 'DevicePayload'
]

__version__ = "1.0.0"
