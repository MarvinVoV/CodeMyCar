import json
import struct
from enum import IntEnum, IntFlag
from typing import Union, Dict, Optional, NamedTuple

from app.device.protocol.common import PROTOCOL_MAX_DATA_LEN, FRAME_HEADER, FRAME_TAIL


# ---------- 基础类型定义 ----------
class CtrlField(IntFlag):
    MOTOR = 1 << 0  # 电机控制有效
    SERVO = 1 << 1  # 舵机控制有效
    TEXT = 1 << 7  # 调试信息有效


class MotorCtrlMode(IntEnum):
    DIFFERENTIAL = 0x01  # 差速模式
    DIRECT = 0x02  # 直接控制模式
    EMERGENCY = 0x80  # 紧急模式


class MotorDiffCtrl(NamedTuple):
    """ 差速模式参数"""
    linear_vel: int  # 线速度（-3000~3000）
    angular_vel: int  # 角速度（-1800~1800）
    accel: int  # 加速度（0-100）

    def to_bytes(self) -> bytes:
        return struct.pack('<hhB',
                           self.linear_vel,
                           self.angular_vel,
                           self.accel)

    @classmethod
    def from_bytes(cls, data: bytes):
        return cls(*struct.unpack_from('<hhB', data))


class MotorDirectCtrl(NamedTuple):
    """直接控制模式参数"""
    left_speed: int  # 左电机速度（-1000~1000）
    right_speed: int  # 右电机速度（-1000~1000）
    left_accel: int  # 左加速度（0-100%）
    right_accel: int  # 右加速度（0-100%）

    def to_bytes(self) -> bytes:
        return struct.pack('<hhBB',
                           self.left_speed,
                           self.right_speed,
                           self.left_accel,
                           self.right_accel)

    @classmethod
    def from_bytes(cls, data: bytes):
        return cls(*struct.unpack_from('<hhBB', data))


class ServoCtrl(NamedTuple):
    """舵机控制参数"""
    angle: int  # 角度（0~180）
    speed: int  # 速度（0-100%）

    def to_bytes(self) -> bytes:
        return struct.pack('<BB', self.angle, self.speed)

    @classmethod
    def from_bytes(cls, data: bytes):
        return cls(*struct.unpack_from('<BB', data))


class TextData(NamedTuple):
    message: str  # UTF-8字符串

    def to_bytes(self) -> bytes:
        encoded = self.message.encode('utf-8')[:255]
        return struct.pack('<B', len(encoded)) + encoded

    @classmethod
    def from_bytes(cls, data: bytes):
        length = data[0]
        return cls(data[1:1 + length].decode('utf-8', 'ignore'))


# ---------- 协议控制命令 ----------
class ControlCmd:
    def __init__(self):
        self.ctrl_id: int = 0
        self.ctrl_fields: CtrlField = CtrlField(0)
        self.motor_mode: Optional[MotorCtrlMode] = MotorCtrlMode.DIRECT
        self.motor_data: Optional[Union[MotorDiffCtrl, MotorDirectCtrl]] = MotorDirectCtrl(0, 0, 0, 0)
        self.servo_data: Optional[ServoCtrl] = ServoCtrl(0, 0)
        self.text_data: Optional[TextData] = TextData("")

    def to_bytes(self) -> bytes:
        # 基础字段打包
        header = struct.pack('<BB', self.ctrl_id, self.ctrl_fields)
        payload = bytearray()

        # 舵机控制数据
        payload += (self.servo_data if self.servo_data else ServoCtrl(0, 0)).to_bytes()

        # 电机数据
        motor_mode = self.motor_mode if self.motor_mode is not None else 2
        payload += struct.pack('<B', motor_mode)
        if motor_mode == MotorCtrlMode.DIFFERENTIAL:
            motor_data = self.motor_data if self.motor_data else MotorDiffCtrl(0, 0, 0)
            payload += motor_data.to_bytes()
        elif motor_mode == MotorCtrlMode.DIRECT:
            motor_data = self.motor_data if self.motor_data else MotorDirectCtrl(0, 0, 0, 0)
            payload += motor_data.to_bytes()
        elif motor_mode == MotorCtrlMode.EMERGENCY:
            pass  # 无数据
        else:
            # 模式0或未定义，填充6字节的0
            payload += b'\x00\x00\x00\x00\x00\x00'

        # 文本数据（无论是否启用，都包含）
        payload += (self.text_data if self.text_data else TextData("")).to_bytes()

        # 完整数据包
        return header + payload

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ControlCmd':
        cmd = cls()
        pos = 0

        # 解析基础头
        cmd.ctrl_id, cmd.ctrl_fields = struct.unpack_from('<BB', data, pos)
        pos += 2

        # 先解析舵机数据
        if cmd.ctrl_fields & CtrlField.SERVO:
            cmd.servo_data = ServoCtrl.from_bytes(data[pos:pos + 2])
            pos += 2

        # 再解析电机数据
        if cmd.ctrl_fields & CtrlField.MOTOR:
            cmd.motor_mode = MotorCtrlMode(data[pos])
            pos += 1
            if cmd.motor_mode == MotorCtrlMode.DIFFERENTIAL:
                cmd.motor_data = MotorDiffCtrl.from_bytes(data[pos:pos + 5])
                pos += 5
            elif cmd.motor_mode == MotorCtrlMode.DIRECT:
                cmd.motor_data = MotorDirectCtrl.from_bytes(data[pos:pos + 6])
                pos += 6

        # 解析文本数据
        if cmd.ctrl_fields & CtrlField.TEXT:
            length = data[pos]
            cmd.text_data = TextData.from_bytes(data[pos:pos + 1 + length])
            pos += 1 + length

        return cmd


# ---------- 高层业务封装 ----------
class DevicePayload:
    def __init__(self, ctrl_id: int = 0):
        self.cmd = ControlCmd()
        self.cmd.ctrl_id = ctrl_id

    def add_motor_diff(self, linear: int, angular: int, accel: int) -> 'DevicePayload':
        self.cmd.ctrl_fields |= CtrlField.MOTOR
        self.cmd.motor_mode = MotorCtrlMode.DIFFERENTIAL
        self.cmd.motor_data = MotorDiffCtrl(
            linear_vel=linear,
            angular_vel=angular,
            accel=accel
        )
        return self

    def add_motor_direct(self, left: int, right: int, accel_l: int, accel_r: int) -> 'DevicePayload':
        self.cmd.ctrl_fields |= CtrlField.MOTOR
        self.cmd.motor_mode = MotorCtrlMode.DIRECT
        self.cmd.motor_data = MotorDirectCtrl(
            left_speed=left,
            right_speed=right,
            left_accel=accel_l,
            right_accel=accel_r
        )
        return self

    def add_servo(self, angle: int, speed: int) -> 'DevicePayload':
        self.cmd.ctrl_fields |= CtrlField.SERVO
        self.cmd.servo_data = ServoCtrl(angle, speed)
        return self

    def add_text(self, message: str) -> 'DevicePayload':
        self.cmd.ctrl_fields |= CtrlField.TEXT
        self.cmd.text_data = TextData(message)
        return self

    @property
    def raw_bytes(self) -> bytes:
        return self.cmd.to_bytes()


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
