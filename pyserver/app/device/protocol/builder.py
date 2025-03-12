import json
import struct
from enum import IntEnum
from typing import Union, Dict, Optional, NamedTuple

from app.device.protocol.common import PROTOCOL_MAX_DATA_LEN, FRAME_HEADER, FRAME_TAIL



# ---------- 基础类型定义 ----------
class CtrlType(IntEnum):
    CTRL_MOTOR = 0x01
    CTRL_SERVO = 0x02
    CTRL_TEXT = 0x03
    CTRL_MAX = 0x04


class MotorCommandType(IntEnum):
    STOP = 0x00
    FORWARD = 0x01
    BACKWARD = 0x02
    TURN_LEFT = 0x03
    TURN_RIGHT = 0x04


# ---------- 命令数据基类 ----------
class CommandData:
    @classmethod
    def from_bytes(cls, data: bytes):
        raise NotImplementedError

    def to_bytes(self) -> bytes:
        raise NotImplementedError

    @property
    def size(self) -> int:
        return len(self.to_bytes())


# ---------- 具体命令实现 ----------
class MotorData(CommandData):
    FORMAT = '<4B'

    def __init__(self, cmd_type: int, left: int, right: int, acc: int):
        self.cmd_type = cmd_type
        self.left = left
        self.right = right
        self.acc = acc

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT,
                           self.cmd_type, self.left, self.right, self.acc)

    @classmethod
    def from_bytes(cls, data: bytes):
        return cls(*struct.unpack_from(cls.FORMAT, data))


class ServoData(CommandData):
    FORMAT = '<B'

    def __init__(self, angle: int):
        self.angle = angle

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.angle)

    @classmethod
    def from_bytes(cls, data: bytes):
        return cls(struct.unpack(cls.FORMAT, data)[0])


class TextData(CommandData):
    def __init__(self, message: str):
        self.message = message[:64]

    def to_bytes(self) -> bytes:
        encoded = self.message.encode('utf-8')[:64]
        return struct.pack('<B', len(encoded)) + encoded

    @classmethod
    def from_bytes(cls, data: bytes):
        length = data[0]
        return cls(data[1:1 + length].decode('utf-8', errors='ignore'))


# ---------- 协议控制命令 ----------
class ControlCmd:
    def __init__(self, ctrl_id: int, ctrl_type: CtrlType, data: CommandData):
        self.ctrl_id = ctrl_id
        self.ctrl_type = ctrl_type
        self.data = data

    def to_bytes(self) -> bytes:
        header = struct.pack('<BB', self.ctrl_id, self.ctrl_type)
        return header + self.data.to_bytes()

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ControlCmd':
        if isinstance(data, list):
            data = bytes(data)
        ctrl_id, ctrl_type = struct.unpack_from('<BB', data)
        payload_data = data[2:]

        if ctrl_type == CtrlType.CTRL_MOTOR:
            data_cls = MotorData
        elif ctrl_type == CtrlType.CTRL_SERVO:
            data_cls = ServoData
        elif ctrl_type == CtrlType.CTRL_TEXT:
            data_cls = TextData
        else:
            raise ValueError("Unknown control type")

        return cls(ctrl_id, CtrlType(ctrl_type), data_cls.from_bytes(payload_data))


# ---------- 高层业务封装 ----------
class DevicePayload:
    def __init__(self, ctrl_id: int = 0):
        self.ctrl_id = ctrl_id
        self._command = None

    @classmethod
    def from_bytes(cls, data: bytes) -> 'DevicePayload':
        """静态方法：从字节流解析生成payload实例"""
        cmd = ControlCmd.from_bytes(data)
        instance = cls(cmd.ctrl_id)
        instance._command = cmd
        return instance

    def build_motor(self, cmd_type: int, left: int, right: int, acc: int) -> 'DevicePayload':
        if not (0 <= acc <= 100):
            raise ValueError("Acceleration must be 0-100")
        self._command = ControlCmd(
            self.ctrl_id,
            CtrlType.CTRL_MOTOR,
            MotorData(cmd_type, left, right, acc)
        )
        return self

    def build_servo(self, angle: int) -> 'DevicePayload':
        self._command = ControlCmd(
            self.ctrl_id,
            CtrlType.CTRL_SERVO,
            ServoData(angle)
        )
        return self

    def build_text(self, message: str) -> 'DevicePayload':
        self._command = ControlCmd(
            self.ctrl_id,
            CtrlType.CTRL_TEXT,
            TextData(message)
        )
        return self

    @property
    def raw_bytes(self) -> bytes:
        return self._command.to_bytes()

    def parse(self) -> Dict[str, Union[str, int, dict]]:
        """解析为结构化数据（增强版）"""
        if not self._command:
            return {"error": "No command loaded"}

        base_info = {
            "ctrl_id": self.ctrl_id,
            "ctrl_type": self._command.ctrl_type.name
        }

        if self._command.ctrl_type == CtrlType.CTRL_MOTOR:
            return {**base_info, "data": vars(self._command.data)}
        elif self._command.ctrl_type == CtrlType.CTRL_SERVO:
            return {**base_info, "angle": self._command.data.angle}
        elif self._command.ctrl_type == CtrlType.CTRL_TEXT:
            return {**base_info, "message": self._command.data.message}
        return {**base_info, "error": "Unknown data type"}

    def get_motor_params(self) -> Optional[dict]:
        """快速获取电机参数（类型安全访问）"""
        if self._command and self._command.ctrl_type == CtrlType.CTRL_MOTOR:
            return {
                "cmd_type": self._command.data.cmd_type,
                "left": self._command.data.left,
                "right": self._command.data.right,
                "acc": self._command.data.acc
            }
        return None


def crc16_ccitt(data):
    """CRC16-CCITT (0xFFFF) 校验实现"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF  # 保持16位
    return crc


class ProtocolFrameBuilder:
    def __init__(self, protocol_type: int):
        self.protocol_type = protocol_type
        self._payload = None

    def set_payload(self, payload: DevicePayload) -> 'ProtocolFrameBuilder':
        if isinstance(payload, DevicePayload):
            self._payload = payload.raw_bytes
        else:
            raise TypeError("Unsupported payload type")
        return self

    def build_frame(self):
        """构建完整协议帧"""
        if not self._payload:
            raise ValueError("Payload not initialized")

        # 数据长度校验
        if len(self._payload) > PROTOCOL_MAX_DATA_LEN:
            raise ValueError(f"Payload exceeds {PROTOCOL_MAX_DATA_LEN} bytes")

        # 构建基础帧结构
        frame = struct.pack(
            "<H B H",
            FRAME_HEADER,
            self.protocol_type,
            len(self._payload))

        # 添加数据
        frame += bytes(self._payload)

        # 计算校验和
        checksum = crc16_ccitt(frame)
        frame += struct.pack("<H", checksum)

        # 添加帧尾
        frame += struct.pack("<H", FRAME_TAIL)

        return frame

    @classmethod
    def parse_frame(cls, frame: bytes) -> dict:
        """解析接收到的协议帧"""
        if len(frame) < 8:
            raise ValueError("Invalid frame length")

        # 解包基础头
        header, proto_type, data_len = struct.unpack_from("<H B H", frame, 0)
        if header != FRAME_HEADER:
            raise ValueError("Invalid frame header")

        # 校验数据完整性 header 2 + proto_type 1 + data_len 2 + data[N] + checksum 2 + tail 2
        expected_len = 9 + data_len
        if len(frame) != expected_len:
            raise ValueError("Frame length mismatch")

        # 提取载荷数据
        payload_start = 5  # H(2) + B(1) + H(2)
        payload_end = payload_start + data_len
        payload_data = frame[payload_start:payload_end]

        # 校验和验证
        received_crc = struct.unpack_from("<H", frame, payload_end)[0]
        calculated_crc = crc16_ccitt(frame[:payload_end])
        if received_crc != calculated_crc:
            raise ValueError("CRC check failed")

        return {
            "protocol_type": proto_type,
            "payload": payload_data,
            "raw_frame": frame
        }


# ---------- STM32 LOG PAYLOAD --------------------
class LogEntry(NamedTuple):
    timestamp: int  # 时间戳（单位ms）
    module: int  # 模块标识
    level: int  # 日志级别
    message: str  # 日志正文

    def to_json(self, ensure_ascii: bool = False, indent: int = None) -> str:
        """
        将日志条目序列化为JSON字符串
        :param ensure_ascii: 是否转义非ASCII字符（默认False保留中文）
        :param indent: JSON缩进格式（默认紧凑格式）
        :return: JSON格式字符串
        """
        return json.dumps(
            {
                "timestamp": self.timestamp,
                "module": self.module,
                "level": self.level,
                "message": self.message.strip()
            },
            ensure_ascii=ensure_ascii,
            indent=indent
        )


class LogParser:
    """UART日志协议解析器"""

    # 协议固定参数
    HEADER_FORMAT = "<IHB"  # 格式：小端, uint32, uint16, uint8
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # # 4+2+1=7字节

    def __init__(self):
        self._header_struct = struct.Struct(self.HEADER_FORMAT)

    def parse(self, data: bytes) -> LogEntry:
        """解析二进制数据"""
        # 基础校验：至少包含头部数据
        if len(data) < self.HEADER_SIZE:
            raise ValueError(
                f"Data too short: minimum {self.HEADER_SIZE} bytes, got {len(data)}"
            )

        # 解包头部分（自动处理小端字节序）
        header_data = data[:self.HEADER_SIZE]
        timestamp, module, level = self._header_struct.unpack(header_data)

        # 处理消息部分（剩余字节）
        raw_msg = data[self.HEADER_SIZE:]
        message = self._trim_null_bytes(raw_msg)

        return LogEntry(
            timestamp=timestamp,
            module=module,
            level=level,
            message=message
        )

    def _trim_null_bytes(self, raw: bytes) -> str:
        """处理消息字段：截断到第一个空字符，并解码为字符串"""
        null_pos = raw.find(b'\x00')
        if null_pos != -1:
            raw = raw[:null_pos]
        return raw.decode('utf-8', errors='replace')

    @property
    def min_size(self) -> int:
        """返回协议最小有效数据长度"""
        return self.HEADER_SIZE + 1  # 至少包含1字节消息内容（含终止符）
