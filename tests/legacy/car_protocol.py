"""M1 framing and telemetry: no serial writes, motor commands or Flash operations."""
from dataclasses import dataclass, fields
import binascii
import struct

MAGIC = b"\xa5\x5a"
VERSION = 1
MAX_PAYLOAD = 256
TIMEOUT_MS = 100
HEADER = struct.Struct("<2sBBHH")
PING, PONG, TELEMETRY = 0x01, 0x02, 0x10


@dataclass(frozen=True)
class Frame:
    kind: int
    seq: int
    payload: bytes = b""


def encode(frame):
    """编码有长度上限的二进制协议帧并追加 CRC。"""
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 256 bytes")
    header = HEADER.pack(MAGIC, VERSION, frame.kind, frame.seq, len(frame.payload))
    body = header[2:] + frame.payload
    return MAGIC + body + struct.pack("<I", binascii.crc32(body))


class Decoder:
    """Bounded stream parser. now_ms is a monotonic uint32 millisecond tick."""

    def __init__(self):
        """初始化独立解析状态或序号跟踪状态。"""
        self.buffer = bytearray()
        self.last_byte_ms = 0
        self.crc_errors = self.header_errors = self.timeouts = 0

    def _drain(self):
        """逐字节重新同步并提取 CRC 正确的完整帧。"""
        frames = []
        while self.buffer:
            if self.buffer[0] != MAGIC[0] or (
                len(self.buffer) >= 2 and self.buffer[:2] != MAGIC
            ):
                del self.buffer[0]
                continue
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, seq, size = HEADER.unpack_from(self.buffer)
            if version != VERSION or size > MAX_PAYLOAD:
                self.header_errors += 1
                del self.buffer[0]
                continue
            total = size + 12
            if len(self.buffer) < total:
                break
            crc, = struct.unpack_from("<I", self.buffer, total - 4)
            if binascii.crc32(self.buffer[2:total - 4]) != crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append(Frame(kind, seq, bytes(self.buffer[8:total - 4])))
            del self.buffer[:total]
        return frames

    def poll(self, now_ms):
        """处理不完整帧的接收超时并尝试恢复后续帧。"""
        frames = []
        elapsed = (now_ms - self.last_byte_ms) & 0xFFFFFFFF
        if self.buffer and elapsed >= TIMEOUT_MS:
            self.timeouts += 1
            while self.buffer:
                del self.buffer[0]
                frames.extend(self._drain())
        return frames

    def feed(self, data, now_ms):
        """追加输入字节流并返回已解析的完整帧。"""
        frames = self.poll(now_ms)
        for byte in data:
            self.buffer.append(byte)
            self.last_byte_ms = now_ms & 0xFFFFFFFF
            frames.extend(self._drain())
        return frames


@dataclass(frozen=True)
class Telemetry:
    uptime_ms: int
    target_left_rpm: int
    target_right_rpm: int
    actual_left_rpm: int
    actual_right_rpm: int
    command_left: int
    command_right: int
    distance_left_mm: int
    distance_right_mm: int
    state: int
    reserved: int
    fault_bits: int
    comm_errors: int
    firmware_version: int
    parameter_version: int

    FORMAT = struct.Struct("<I6h2iBBH3I")

    def pack(self):
        """编码早期协议测试样本；不是新车实时遥测格式。"""
        if self.reserved != 0:
            raise ValueError("reserved must be zero")
        return self.FORMAT.pack(*(getattr(self, f.name) for f in fields(self)))

    @classmethod
    def unpack(cls, payload):
        """解码早期协议测试样本并检查载荷长度。"""
        if len(payload) != cls.FORMAT.size:
            raise ValueError("telemetry must contain exactly 40 bytes")
        result = cls(*cls.FORMAT.unpack(payload))
        if result.reserved != 0:
            raise ValueError("unsupported telemetry reserved byte")
        return result


class SequenceTracker:
    """Report anomalies without dropping samples or guessing whether MCU reset."""

    def __init__(self):
        """初始化独立解析状态或序号跟踪状态。"""
        self.last = None

    def observe(self, seq):
        """比较序号并标记连续、重复或缺帧情况。"""
        if self.last is None:
            self.last = seq
            return "first"
        delta = (seq - self.last) & 0xFFFF
        if delta == 0:
            return "duplicate"
        if delta >= 0x8000:
            return "out_of_order_or_reset"
        self.last = seq
        return "ok" if delta == 1 else f"gap:{delta - 1}"
