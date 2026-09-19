"""Production C ESP32 sender -> production C STM32 receiver, with Flash/transport faults."""
import ctypes as C
import struct
import sys
import unittest
import zlib

import test_ota_receiver as model  # Consumes shared-library argument.

EXCHANGE = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_uint8, C.c_uint16, C.c_void_p,
                      C.c_uint16, C.c_void_p, C.c_uint32)
MARK = C.CFUNCTYPE(C.c_uint32, C.c_void_p)
IDENTITY = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p, C.c_uint32, C.c_uint32)


class Info(C.Structure):
    _fields_ = [('size', C.c_uint32), ('crc', C.c_uint32), ('version', C.c_uint32)]


class Link(C.Structure):
    _fields_ = [('exchange', EXCHANGE), ('mark', MARK), ('wait_identity', IDENTITY), ('context', C.c_void_p)]


class Report(C.Structure):
    _fields_ = [(name, C.c_uint32) for name in
               ('result', 'stage', 'transferred', 'retries', 'blocks', 'session', 'version', 'remote_status', 'running_version')]


model.lib.sender_install.argtypes = [C.POINTER(Link), C.POINTER(Info), C.c_void_p, C.c_uint32, C.POINTER(Report)]
model.lib.sender_install.restype = C.c_int


class Target:
    def __init__(self, fault=''):
        self.device = model.Device()
        self.fault, self.calls, self.dropped, self.waits = fault, [], set(), []
        self.running, self.generation = 100, 1
        self.tag = b'SC01'
        self.callbacks = (EXCHANGE(self.exchange), MARK(lambda _: self.generation), IDENTITY(self.identity))
        self.link = Link(*self.callbacks, None)

    def exchange(self, ctx, kind, seq, pointer, size, output, timeout):
        payload = C.string_at(pointer, size)
        self.calls.append((kind, seq, payload))
        if self.fault == 'silent':
            return 0
        if self.fault == 'io':
            return -1
        if kind == 0x20:
            self.running, self.tag = None, b'M2B2'
            ack = (0, 0, kind, 0, 0, 0, 0)
        elif self.running is not None:
            if kind != 0x33:
                return 0
            ack = (0, 4, kind, 0, 0, 0, self.running)
        else:
            ack = self.device.call(kind, payload, seq, now=100)
        if kind == 0x34 and ack[0] == 0:
            self.running = ack[6]
            self.tag = b'SC01'
        self.generation += 1
        if self.fault == 'three_drops' and kind in (0x30, 0x31, 0x32) and kind not in self.dropped:
            self.dropped.add(kind)
            return 0
        if (self.fault == 'reset_ack' and kind == 0x34) or (self.fault == 'enter_ack' and kind == 0x20):
            return 0
        ack = list(ack)
        if self.fault == 'offset' and kind == 0x31:
            ack[5] += 128
        if self.fault == 'session' and kind == 0x30:
            ack[4] ^= 1
        if self.fault == 'version' and kind == 0x33:
            ack[6] = 99
        if self.fault == 'reset_state' and kind == 0x34:
            ack[1] = 1
        if self.fault == 'reserved' and kind == 0x30:
            ack[3] = 1
        C.memmove(output, struct.pack('<4B3I', *ack), 16)
        return 1

    def identity(self, ctx, wanted, after, timeout):
        tag = C.string_at(wanted, 4)
        self.waits.append((tag, after))
        if (self.fault == 'no_boot' and tag == b'M2B2') or (self.fault == 'no_app' and tag == b'SC01'):
            return 0
        return int(tag == self.tag and self.generation > after)

    def run(self, data=None, version=100, crc=None, session=42):
        data = model.image() if data is None else data
        info = Info(len(data), zlib.crc32(data) if crc is None else crc, version)
        report = Report()
        result = model.lib.sender_install(C.byref(self.link), C.byref(info), C.create_string_buffer(data), session, C.byref(report))
        return result, report


class SenderTests(unittest.TestCase):
    def test_success_versions_and_boundaries(self):
        for version, size in ((100, 258), (101, 259), (102, 47104)):
            target = Target()
            result, report = target.run(model.image(size), version)
            self.assertEqual(result, 0)
            self.assertEqual(report.running_version, version)
            self.assertEqual(report.transferred, size)
            self.assertEqual(target.device.boot_result(), 0)

    def test_dropped_acks_retransmit_identically_without_rewrite(self):
        target = Target('three_drops')
        result, report = target.run()
        self.assertEqual((result, report.retries), (0, 3))
        self.assertEqual(target.device.events.count('erase'), 1)
        self.assertEqual(target.device.events.count('write'), 8)
        self.assertEqual(target.device.events.count('finish'), 1)
        for kind in (0x30, 0x31, 0x32):
            calls = [x for x in target.calls if x[0] == kind]
            self.assertEqual(calls[0], calls[1])

    def test_reset_ack_loss_requires_fresh_app_and_query(self):
        target = Target('reset_ack')
        result, report = target.run()
        self.assertEqual(result, 0)
        self.assertEqual(target.calls[-1][0], 0x33)
        self.assertEqual(report.running_version, 100)

    def test_enter_ack_loss_recovers_from_boot_identity(self):
        self.assertEqual(Target('enter_ack').run()[0], 0)

    def test_invalid_crc_never_sends_enter_or_begin(self):
        target = Target()
        self.assertEqual(target.run(crc=0)[0], 1)
        self.assertEqual(target.calls, [])

    def test_invalid_vector_never_sends_enter_or_begin(self):
        target = Target()
        data = bytearray(model.image())
        struct.pack_into('<I', data, 4, 0x08004001)
        self.assertEqual(target.run(bytes(data))[0], 1)
        self.assertEqual(target.calls, [])

    def test_invalid_version_or_session_never_sends(self):
        for version, session in ((0, 42), (4, 42), (3, 0)):
            target = Target()
            self.assertEqual(target.run(version=version, session=session)[0], 1)
            self.assertEqual(target.calls, [])

    def test_no_boot_identity_cannot_erase(self):
        target = Target('no_boot')
        self.assertEqual(target.run()[0], 5)
        self.assertNotIn('erase', target.device.events)

    def test_no_app_identity_does_not_report_success(self):
        self.assertEqual(Target('no_app').run()[0], 6)

    def test_wrong_offset_stops_before_commit(self):
        target = Target('offset')
        self.assertEqual(target.run()[0], 3)
        self.assertNotIn('finish', target.device.events)

    def test_reply_session_reserved_and_version_checked(self):
        for fault in ('session', 'reserved', 'version', 'reset_state'):
            with self.subTest(fault=fault):
                self.assertEqual(Target(fault).run()[0], 3)

    def test_write_rejection_not_retried_as_timeout(self):
        target = Target()
        target.device.fail = 'write'
        result, report = target.run()
        self.assertEqual((result, report.remote_status, report.retries), (4, 6, 0))
        self.assertNotIn('finish', target.device.events)

    def test_silent_link_has_bounded_identical_retries(self):
        target = Target('silent')
        self.assertEqual(target.run()[0], 5)
        self.assertEqual(len(target.calls), 3)
        self.assertEqual(len(set(target.calls)), 1)

    def test_transport_io_error_stops(self):
        self.assertEqual(Target('io').run()[0], 7)


if __name__ == '__main__':
    unittest.main(verbosity=2)
