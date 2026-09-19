import binascii
import ctypes as ct
import random
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests/legacy"))
from car_protocol import Decoder, Frame, PING, SequenceTracker, Telemetry, encode
from telemetry import decode_file


CALLBACK = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_uint8, ct.c_uint16,
                       ct.POINTER(ct.c_uint8), ct.c_uint16)


class CReceiver(ct.Structure):
    _fields_ = [("buffer", ct.c_uint8 * 268), ("used", ct.c_uint16),
                ("last_byte_ms", ct.c_uint32), ("crc_errors", ct.c_uint32),
                ("header_errors", ct.c_uint32), ("timeouts", ct.c_uint32),
                ("handler", CALLBACK), ("context", ct.c_void_p)]


class CDecoder:
    def __init__(self):
        self.frames = []
        self.rx = CReceiver()
        self.callback = CALLBACK(lambda _, kind, seq, data, size:
                                 self.frames.append(Frame(kind, seq, bytes(data[:size]))))
        LIB.car_receiver_init(ct.byref(self.rx), self.callback, None)

    def feed(self, data, now_ms):
        self.frames = []
        for byte in data:
            LIB.car_receiver_feed(ct.byref(self.rx), byte, now_ms)
        return self.frames

    def poll(self, now_ms):
        self.frames = []
        LIB.car_receiver_poll(ct.byref(self.rx), now_ms)
        return self.frames

    def __getattr__(self, name):
        return getattr(self.rx, name)


def c_encode(frame):
    output = (ct.c_uint8 * 268)()
    data = (ct.c_uint8 * len(frame.payload)).from_buffer_copy(frame.payload)
    size = LIB.car_encode(output, len(output), frame.kind, frame.seq, data, len(data))
    return bytes(output[:size])


class StreamCases:
    def setUp(self):
        self.rx = self.factory()
        self.frame = Frame(PING, 0x1234, b"\x00\xa5\x5a\xff")
        self.wire = encode(self.frame)

    def test_every_split_and_concatenation(self):
        for cut in range(len(self.wire) + 1):
            rx = self.factory()
            frames = rx.feed(self.wire[:cut], 0)
            frames += rx.feed(self.wire[cut:] + self.wire, 1)
            self.assertEqual(frames, [self.frame, self.frame])

    def test_bytewise_and_sync_overlap(self):
        frames = self.rx.feed(b"garbage\xa5", 0)
        for index, byte in enumerate(self.wire):
            frames += self.rx.feed(bytes([byte]), index)
        self.assertEqual(frames, [self.frame])

    def test_all_single_bit_errors_recover(self):
        for index in range(len(self.wire)):
            for bit in range(8):
                rx = self.factory()
                damaged = bytearray(self.wire)
                damaged[index] ^= 1 << bit
                frames = rx.feed(damaged + self.wire, 0) + rx.poll(100)
                self.assertEqual(frames, [self.frame], (index, bit))

    def test_oversize_and_version_rejected(self):
        for version, size in [(1, 257), (1, 65535), (2, 0)]:
            bad = struct.pack("<2sBBHH", b"\xa5\x5a", version, 1, 1, size)
            self.assertEqual(self.rx.feed(bad + self.wire, 0), [self.frame])
        self.assertEqual(self.rx.header_errors, 3)

    def test_timeout_boundary(self):
        self.assertEqual(self.rx.feed(self.wire[:9], 0), [])
        self.assertEqual(self.rx.poll(99), [])
        self.assertEqual(self.rx.timeouts, 0)
        self.assertEqual(self.rx.poll(100), [])
        self.assertEqual(self.rx.timeouts, 1)
        self.assertEqual(self.rx.feed(self.wire[9:] + self.wire, 101), [self.frame])

    def test_timeout_recovers_frame_behind_damaged_length(self):
        bad = struct.pack("<2sBBHH", b"\xa5\x5a", 1, 1, 1, 256)
        self.assertEqual(self.rx.feed(bad + self.wire, 1), [])
        self.assertEqual(self.rx.poll(101), [self.frame])

    def test_timeout_tick_wrap(self):
        self.rx.feed(self.wire[:5], 0xFFFFFFF0)
        self.rx.poll(0x53)
        self.assertEqual(self.rx.timeouts, 0)
        self.rx.poll(0x54)
        self.assertEqual(self.rx.timeouts, 1)

    def test_gap_expires_before_new_bytes(self):
        self.rx.feed(self.wire[:5], 1)
        self.assertEqual(self.rx.feed(self.wire, 101), [self.frame])
        self.assertEqual(self.rx.timeouts, 1)

    def test_duplicate_delivered_for_business_layer(self):
        self.assertEqual(self.rx.feed(self.wire * 2, 0), [self.frame, self.frame])

    def test_empty_and_max_payload(self):
        for payload in [b"", bytes(range(256))]:
            frame = Frame(0xFE, 0xFFFF, payload)
            self.assertEqual(self.rx.feed(encode(frame), 0), [frame])

    def test_seeded_random_stream(self):
        rng = random.Random(20260916)
        frames = [Frame(rng.randrange(256), i, rng.randbytes(rng.randrange(257)))
                  for i in range(200)]
        wire = b"".join(encode(frame) for frame in frames)
        received = []
        offset = 0
        while offset < len(wire):
            size = rng.randrange(1, 80)
            received += self.rx.feed(wire[offset:offset + size], 0)
            offset += size
        self.assertEqual(received, frames)


class PythonStreamTests(StreamCases, unittest.TestCase):
    factory = Decoder


class CStreamTests(StreamCases, unittest.TestCase):
    factory = CDecoder


class InteropTests(unittest.TestCase):
    def test_crc_known_answer(self):
        data = (ct.c_uint8 * 9).from_buffer_copy(b"123456789")
        self.assertEqual(LIB.car_crc32(data, 9), 0xCBF43926)
        self.assertEqual(LIB.car_crc32(None, 0), 0)

    def test_wire_layout_and_both_encoders(self):
        rng = random.Random(42)
        for size in range(257):
            frame = Frame(0x10, 0x1234, rng.randbytes(size))
            # 固定字段独立写出，避免只做同一份实现的回环测试。
            body = b"\x01\x10\x34\x12" + size.to_bytes(2, "little") + frame.payload
            expected = b"\xa5\x5a" + body + binascii.crc32(body).to_bytes(4, "little")
            self.assertEqual(encode(frame), expected)
            self.assertEqual(c_encode(frame), expected)
            self.assertEqual(Decoder().feed(c_encode(frame), 0), [frame])

    def test_invalid_encode_capacity_and_payload(self):
        output = (ct.c_uint8 * 268)()
        data = (ct.c_uint8 * 257)()
        self.assertEqual(LIB.car_encode(output, 268, 1, 0, data, 257), 0)
        self.assertEqual(LIB.car_encode(output, 11, 1, 0, None, 0), 0)
        self.assertEqual(LIB.car_encode(output, 268, 1, 0, None, 1), 0)
        with self.assertRaises(ValueError):
            encode(Frame(1, 0, bytes(257)))

    def test_sequence_gap_duplicate_wrap_and_reset_ambiguity(self):
        tracker = SequenceTracker()
        self.assertEqual([tracker.observe(n) for n in (65534, 65535, 0, 0, 3, 2, 4)],
                         ["first", "ok", "ok", "duplicate", "gap:2",
                          "out_of_order_or_reset", "ok"])

    def test_telemetry_and_csv_reject_bad_payload(self):
        sample = Telemetry(123, -10, 20, -9, 19, -100, 100, -500, 600,
                           1, 0, 3, 7, 1, 2)
        self.assertEqual(len(sample.pack()), 40)
        self.assertEqual(Telemetry.unpack(sample.pack()), sample)
        with self.assertRaises(ValueError):
            Telemetry.unpack(sample.pack()[:-1])
        bad = bytearray(sample.pack())
        bad[25] = 1
        with self.assertRaises(ValueError):
            Telemetry.unpack(bad)
        with tempfile.TemporaryDirectory() as folder:
            raw, csv = Path(folder) / "wire.bin", Path(folder) / "samples.csv"
            raw.write_bytes(encode(Frame(0x10, 1, sample.pack())) +
                            encode(Frame(0x10, 2, b"bad")))
            report = decode_file(raw, csv, simulated=True)
            self.assertEqual(report["samples"], 1)
            self.assertEqual(report["invalid_telemetry"], 1)
            self.assertIn("SIMULATED,1,first,123,-10,20", csv.read_text())


def load_library(path):
    global LIB
    LIB = ct.CDLL(str(path.resolve()))
    byte_pointer = ct.POINTER(ct.c_uint8)
    LIB.car_crc32.argtypes = [byte_pointer, ct.c_size_t]
    LIB.car_crc32.restype = ct.c_uint32
    LIB.car_encode.argtypes = [byte_pointer, ct.c_size_t, ct.c_uint8, ct.c_uint16,
                               byte_pointer, ct.c_uint16]
    LIB.car_encode.restype = ct.c_size_t
    LIB.car_receiver_init.argtypes = [ct.POINTER(CReceiver), CALLBACK, ct.c_void_p]
    LIB.car_receiver_init.restype = None
    LIB.car_receiver_feed.argtypes = [ct.POINTER(CReceiver), ct.c_uint8, ct.c_uint32]
    LIB.car_receiver_feed.restype = None
    LIB.car_receiver_poll.argtypes = [ct.POINTER(CReceiver), ct.c_uint32]
    LIB.car_receiver_poll.restype = None


if __name__ == "__main__":
    load_library(Path(sys.argv.pop(1)))
    unittest.main(verbosity=2)
