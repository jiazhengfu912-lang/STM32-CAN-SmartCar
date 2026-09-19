"""Exercise the actual C OTA receiver with fault-injected storage and reboot checks."""
import ctypes as c
from pathlib import Path
import struct
import sys
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'firmware/stm32_bootloader'))
import package

lib = c.CDLL(str(Path(sys.argv.pop(1)).resolve()))
lib.ota_init.argtypes = [c.c_void_p, c.c_void_p]
lib.ota_handle.argtypes = [c.c_void_p, c.c_uint8, c.c_uint16, c.c_void_p, c.c_uint16, c.c_uint32, c.c_void_p]
lib.ota_handle.restype = c.c_uint8
lib.ota_poll.argtypes = [c.c_void_p, c.c_uint32]
lib.boot_validate.argtypes = [c.c_void_p, c.c_void_p, c.c_size_t]
lib.boot_validate.restype = c.c_int
NoArgs = c.CFUNCTYPE(c.c_int)
Erase = c.CFUNCTYPE(c.c_int, c.c_uint32)
Data = c.CFUNCTYPE(c.c_int, c.c_uint32, c.c_void_p, c.c_uint16)
Finish = c.CFUNCTYPE(c.c_int, c.c_uint32, c.c_uint32, c.c_uint32)


class Ops(c.Structure):
    _fields_ = [('invalidate', NoArgs), ('erase', Erase), ('write', Data), ('equal', Data), ('finish', Finish)]


def image(size=1024):
    body = bytearray((i * 31 + 7) & 255 for i in range(size))
    struct.pack_into('<II', body, 0, 0x20005000, package.APP_BASE + 257)
    return bytes(body)


class Device:
    def __init__(self):
        self.flash = bytearray(b'\xff' * 65536)
        self.flash[:0x4000] = b'\xa5' * 0x4000
        self.flash[0xFC00:] = b'\x5a' * 1024
        old = image()
        self.flash[0x4000:0x4000+len(old)] = old
        self.flash[0xF800:0xF820] = package.metadata(old)
        self.events = []
        self.fail = None
        self.commit_cut = None
        self.callbacks = [NoArgs(self.invalidate), Erase(self.erase), Data(self.write),
                          Data(self.equal), Finish(self.finish)]
        self.ops = Ops(*self.callbacks)
        self.reboot()

    def reboot(self):
        self.rx = c.create_string_buffer(128)
        lib.ota_init(self.rx, c.byref(self.ops))

    def boot_result(self, record=None):
        record = bytes(self.flash[0xF800:0xF820]) if record is None else record
        app = bytes(self.flash[0x4000:0xF800])
        return lib.boot_validate(c.create_string_buffer(record), c.create_string_buffer(app), len(app))

    def invalidate(self):
        self.events.append('invalidate')
        if self.fail == 'invalidate':
            return 0
        self.flash[0xF800:0xFC00] = b'\xff' * 1024
        return 1

    def erase(self, length):
        self.events.append('erase')
        if bytes(self.flash[0xF800:0xF820]) != b'\xff' * 32:
            return 0
        if self.fail == 'erase':
            return 0
        end = (length + 1023) // 1024 * 1024
        if end > package.APP_CAPACITY:
            return 0
        self.flash[0x4000:0x4000+end] = b'\xff' * end
        return 1

    def write(self, offset, pointer, size):
        self.events.append('write')
        if offset + size > package.APP_CAPACITY or self.fail == 'write':
            return 0
        data = c.string_at(pointer, size)
        start = 0x4000 + offset
        if bytes(self.flash[start:start+size]) != b'\xff' * size:
            return 0
        self.flash[start:start+size] = data
        return 1

    def equal(self, offset, pointer, size):
        self.events.append('equal')
        return int(bytes(self.flash[0x4000+offset:0x4000+offset+size]) == c.string_at(pointer, size))

    def finish(self, length, crc, version):
        self.events.append('finish')
        header = struct.pack('<6I', package.MAGIC, 1, package.APP_BASE, length, crc, version)
        record = header + struct.pack('<II', zlib.crc32(header), package.COMMITTED)
        if self.boot_result(record) != 0:
            return 0
        for half in range(16):
            if self.commit_cut == half:
                return 0
            start = 0xF800 + half * 2
            self.flash[start:start+2] = record[half*2:half*2+2]
        return 1

    def call(self, kind, data=b'', seq=1, now=0):
        reply = c.create_string_buffer(16)
        status = lib.ota_handle(self.rx, kind, seq, c.create_string_buffer(data), len(data), now, reply)
        fields = struct.unpack('<4B3I', reply.raw)
        assert fields[0] == status and fields[2] == kind and fields[3] == 0
        return fields

    def begin(self, app, session=10, seq=1, now=0, crc=None):
        return self.call(0x30, struct.pack('<5I', session, package.APP_BASE, len(app),
                         zlib.crc32(app) if crc is None else crc, 3), seq, now)

    def data(self, offset, body, seq=2, session=10, now=0):
        return self.call(0x31, struct.pack('<2I', session, offset) + body, seq, now)

    def fill(self, app):
        for offset in range(0, len(app), 128):
            result = self.data(offset, app[offset:offset+128], seq=2+offset//128)
            assert result[0] == 0 and result[5] == min(offset+128, len(app))

    def end(self, seq=400):
        return self.call(0x32, struct.pack('<I', 10), seq)


class OtaTests(unittest.TestCase):
    def test_success_and_reset_permission(self):
        d, app = Device(), image()
        self.assertEqual(d.call(0x34, struct.pack('<I', 10))[0], 3)
        self.assertEqual(d.begin(app)[0], 0)
        self.assertEqual(d.events, ['invalidate', 'erase'])
        self.assertNotEqual(d.boot_result(), 0)
        d.fill(app)
        self.assertEqual(d.end()[0:2], (0, 2))
        self.assertEqual(d.boot_result(), 0)
        self.assertEqual(d.call(0x34, struct.pack('<I', 10))[0], 0)
        self.assertEqual(d.flash[:0x4000], b'\xa5' * 0x4000)
        self.assertEqual(d.flash[0xFC00:], b'\x5a' * 1024)

    def test_lost_begin_ack_does_not_erase_again(self):
        d, app = Device(), image()
        d.begin(app)
        d.data(0, app[:128])
        self.assertEqual(d.begin(app)[5], 128)
        self.assertEqual(d.events.count('erase'), 1)
        self.assertEqual(bytes(d.flash[0x4000:0x4080]), app[:128])

    def test_lost_data_ack_only_compares_and_returns_128(self):
        d, app = Device(), image()
        d.begin(app)
        for _ in range(3):
            reply = d.data(0, app[:128])
            self.assertEqual((reply[0], reply[5]), (0, 128))
        self.assertEqual(d.events.count('write'), 1)
        self.assertEqual(d.events.count('equal'), 2)

    def test_conflicting_duplicate_is_rejected(self):
        d, app = Device(), image()
        d.begin(app)
        d.data(0, app[:128])
        for body, seq in ((b'x' * 128, 2), (app[:126], 2), (app[:128], 3)):
            r = d.data(0, body, seq=seq)
            self.assertEqual((r[0], r[5]), (5, 128))
        self.assertEqual(d.events.count('write'), 1)

    def test_old_block_after_next_block_is_not_last_duplicate(self):
        d, app = Device(), image()
        d.begin(app)
        d.data(0, app[:128])
        d.data(128, app[128:256], seq=3)
        self.assertEqual(d.data(0, app[:128])[0], 4)

    def test_invalid_begin_never_touches_storage(self):
        for session, base, length, version in ((0, package.APP_BASE, 1024, 3),
             (10, 0x08000000, 1024, 3), (10, 0x0800FC00, 1024, 3),
             (10, package.APP_BASE, 0, 3), (10, package.APP_BASE, 47105, 3),
             (10, package.APP_BASE, 0xFFFFFFFF, 3), (10, package.APP_BASE, 1024, 0)):
            d = Device()
            r = d.call(0x30, struct.pack('<5I', session, base, length, 0, version))
            self.assertEqual(r[0], 1)
            self.assertEqual(d.events, [])
            self.assertEqual(d.boot_result(), 0)

    def test_malformed_packets_do_not_write(self):
        for kind in (0x30, 0x31, 0x32, 0x33, 0x34, 0x3F):
            for size in (1, 2, 3, 7, 137, 256):
                d = Device()
                self.assertNotEqual(d.call(kind, b'\xff' * size)[0], 0)
                self.assertEqual(d.events, [])

    def test_stale_session_and_busy_session(self):
        d, app = Device(), image()
        d.begin(app)
        self.assertEqual(d.data(0, app[:128], session=11)[0], 3)
        self.assertEqual(d.begin(app, session=11)[0], 9)
        self.assertEqual(d.begin(app, crc=0)[0], 5)
        self.assertEqual(d.events, ['invalidate', 'erase'])

    def test_offset_overflow_order_and_alignment(self):
        d, app = Device(), image()
        d.begin(app)
        for offset, body in ((128, app[:128]), (0xFFFFFFFF, b'xx'), (1024, b'xx'),
                             (1022, b'xxxx'), (1, b'xx'), (0, b'xxx')):
            self.assertEqual(d.data(offset, body)[0], 4)
        self.assertEqual(d.events, ['invalidate', 'erase'])

    def test_early_end_does_not_commit(self):
        d, app = Device(), image()
        d.begin(app)
        d.data(0, app[:128])
        self.assertEqual(d.end()[0], 2)
        self.assertNotIn('finish', d.events)
        self.assertNotEqual(d.boot_result(), 0)

    def test_lost_end_ack_does_not_commit_twice(self):
        d, app = Device(), image()
        d.begin(app)
        d.fill(app)
        for _ in range(3):
            self.assertEqual(d.end()[0], 0)
        self.assertEqual(d.events.count('finish'), 1)
        self.assertEqual(d.end(seq=401)[0], 5)

    def test_crc_or_vectors_rejected_at_end(self):
        for mode in ('crc', 'vector'):
            d = Device()
            app = image() if mode == 'crc' else b'\x00' * 1024
            d.begin(app, crc=0 if mode == 'crc' else None)
            d.fill(app)
            self.assertEqual(d.end()[0:2], (7, 3))
            self.assertNotEqual(d.boot_result(), 0)

    def test_invalidate_failure_prevents_erase(self):
        d, app = Device(), image()
        d.fail = 'invalidate'
        self.assertEqual(d.begin(app)[0], 6)
        self.assertEqual(d.events, ['invalidate'])
        self.assertEqual(d.boot_result(), 0)  # 原完整镜像尚未被擦除。

    def test_erase_or_write_failure_keeps_record_invalid(self):
        for fail in ('erase', 'write'):
            d, app = Device(), image()
            d.fail = fail
            r = d.begin(app)
            if fail == 'write':
                r = d.data(0, app[:128])
            self.assertEqual(r[0:2], (6, 3))
            self.assertNotEqual(d.boot_result(), 0)

    def test_every_partial_commit_is_rejected_after_reboot(self):
        for cut in range(16):
            d, app = Device(), image()
            d.commit_cut = cut
            d.begin(app)
            d.fill(app)
            self.assertEqual(d.end()[0], 7)
            d.reboot()
            self.assertNotEqual(d.boot_result(), 0, cut)

    def test_interruption_after_each_data_block(self):
        app = image()
        for blocks in range(9):
            d = Device()
            d.begin(app)
            for index in range(blocks):
                d.data(index*128, app[index*128:(index+1)*128], seq=index+2)
            d.reboot()
            self.assertNotEqual(d.boot_result(), 0)
            self.assertEqual(d.data(0, app[:128])[0], 3)
            self.assertEqual(d.begin(app, session=11)[0], 0)

    def test_timeout_tick_wrap_and_query_does_not_keep_alive(self):
        d, app = Device(), image()
        start = 0xFFFFFF00
        d.begin(app, now=start)
        self.assertEqual(d.call(0x33, now=(start+9999) & 0xFFFFFFFF)[1], 1)
        self.assertEqual(d.call(0x33, now=(start+10000) & 0xFFFFFFFF)[1], 3)
        self.assertEqual(d.begin(app, now=10001)[0], 2)
        self.assertEqual(d.begin(app, session=11, now=10001)[0], 0)

    def test_maximum_size_and_odd_final_byte(self):
        for size in (259, package.APP_CAPACITY):
            d, app = Device(), image(size)
            d.begin(app)
            d.fill(app)
            self.assertEqual(d.end()[0], 0)
            self.assertEqual(d.boot_result(), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
