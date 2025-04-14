import json
import struct
from enum import IntEnum, IntFlag
from typing import Union, Dict, Optional, NamedTuple

from app.device.protocol.common import PROTOCOL_MAX_DATA_LEN, FRAME_HEADER, FRAME_TAIL


# ---------- 基础类型定义 ----------
class CtrlField(IntFlag):
    MOTION = 1 << 0  # 运动控制有效


class MotionMode(IntEnum):
    EMERGENCY_STOP = 0x00
    DIRECT_CONTROL = 0x01
    DIFFERENTIAL = 0x02
    STEER_ONLY = 0x03
    SPIN_IN_PLACE = 0x04
    MIXED_STEER = 0x05


class DiffCtrlParam(NamedTuple):
    linearVel: int
    angularVel: int

    def to_bytes(self) -> bytes:
        return struct.pack('<hh', self.linearVel, self.angularVel).ljust(7, b'\x00')


class DirectCtrlParam(NamedTuple):
    leftRpm: int
    rightRpm: int
    steerAngle: int

    def to_bytes(self) -> bytes:
        return struct.pack('<hhh', self.leftRpm, self.rightRpm, self.steerAngle).ljust(7, b'\x00')


class AckermannParam(NamedTuple):
    linearVel: int
    steerAngle: int

    def to_bytes(self) -> bytes:
        return struct.pack('<hh', self.linearVel, self.steerAngle).ljust(7, b'\x00')


class MixedCtrlParam(NamedTuple):
    base: DiffCtrlParam
    steerAngle: int
    differentialGain: int

    def to_bytes(self) -> bytes:
        return struct.pack('<hhhB', self.base.linearVel, self.base.angularVel, self.steerAngle, self.differentialGain)


class MotionCmd:
    def __init__(
            self,
            mode: MotionMode,
            params: Union[DiffCtrlParam, DirectCtrlParam, AckermannParam, MixedCtrlParam],  # 添加类型注解
            duration: int
    ):
        self.mode = mode
        self.params = params
        self.duration = duration

    def to_bytes(self) -> bytes:
        mode_byte = struct.pack('<B', self.mode.value)
        params_bytes = self.params.to_bytes()
        duration_bytes = struct.pack('<H', self.duration)
        return mode_byte + params_bytes + duration_bytes


class ControlCmd:
    def __init__(self):
        self.ctrl_id: int = 0
        self.ctrl_fields: CtrlField = CtrlField(0)
        self.motion: MotionCmd = MotionCmd(MotionMode.EMERGENCY_STOP, None, 0)

    def to_bytes(self) -> bytes:
        header = struct.pack('<BB', self.ctrl_id, self.ctrl_fields)
        motion_bytes = self.motion.to_bytes()
        return header + motion_bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ControlCmd':
        cmd = cls()
        pos = 0
        # 解析控制头
        cmd.ctrl_id, cmd.ctrl_fields = struct.unpack_from('<BB', data, pos)
        pos += 2
        # 解析motion部分
        mode_byte = data[pos]
        pos += 1
        params_data = data[pos:pos + 7]
        pos += 7
        duration = struct.unpack_from('<H', data, pos)[0]
        pos += 2
        # 根据mode解析params
        mode = MotionMode(mode_byte)
        if mode == MotionMode.EMERGENCY_STOP:
            params = None
        elif mode == MotionMode.DIRECT_CONTROL:
            left, right, steer = struct.unpack('<hhh', params_data[:6])
            params = DirectCtrlParam(left, right, steer)
        elif mode == MotionMode.DIFFERENTIAL:
            linear, angular = struct.unpack('<HH', params_data[:4])
            params = DiffCtrlParam(linear, angular)
        elif mode == MotionMode.STEER_ONLY:
            linear, steer = struct.unpack('<Hh', params_data[:4])
            params = AckermannParam(linear, steer)
        elif mode == MotionMode.MIXED_STEER:
            linear, angular, steer, gain = struct.unpack('<hhhB', params_data)
            base = DiffCtrlParam(linearVel=linear, angularVel=angular)
            params = MixedCtrlParam(base=base, steerAngle=steer, differentialGain=gain)
        elif mode == MotionMode.SPIN_IN_PLACE:  # 新增处理分支
            params = None
        cmd.motion = MotionCmd(mode, params, duration)
        return cmd


class DevicePayload:
    def __init__(self, ctrl_id: int = 0):
        self.cmd = ControlCmd()
        self.cmd.ctrl_id = ctrl_id

    def add_motor_diff(self, linear: int, angular: int, duration: int) -> 'DevicePayload':
        params = DiffCtrlParam(linearVel=linear, angularVel=angular)
        self.cmd.motion = MotionCmd(MotionMode.DIFFERENTIAL, params, duration)
        self.cmd.ctrl_fields |= CtrlField.MOTION
        return self

    def add_motor_direct(self, left: int, right: int, steer: int, duration: int) -> 'DevicePayload':
        params = DirectCtrlParam(leftRpm=left, rightRpm=right, steerAngle=steer)
        self.cmd.motion = MotionCmd(MotionMode.DIRECT_CONTROL, params, duration)
        self.cmd.ctrl_fields |= CtrlField.MOTION
        return self

    # 新增STEER_ONLY模式构造方法
    def add_steering_only(self, linear: int, steer: int, duration: int) -> 'DevicePayload':
        params = AckermannParam(linearVel=linear, steerAngle=steer)
        self.cmd.motion = MotionCmd(MotionMode.STEER_ONLY, params, duration)
        self.cmd.ctrl_fields |= CtrlField.MOTION
        return self

    # 新增SPIN_IN_PLACE模式构造方法
    def add_spin_in_place(self, duration: int) -> 'DevicePayload':
        params = None  # 该模式可能不需要参数
        self.cmd.motion = MotionCmd(MotionMode.SPIN_IN_PLACE, params, duration)
        self.cmd.ctrl_fields |= CtrlField.MOTION
        return self

    # 新增MIXED_STEER模式构造方法
    def add_mixed_steering(self, linear: int, angular: int, steer: int, gain: int, duration: int) -> 'DevicePayload':
        base = DiffCtrlParam(linearVel=linear, angularVel=angular)
        params = MixedCtrlParam(base=base, steerAngle=steer, differentialGain=gain)
        self.cmd.motion = MotionCmd(MotionMode.MIXED_STEER, params, duration)
        self.cmd.ctrl_fields |= CtrlField.MOTION
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
