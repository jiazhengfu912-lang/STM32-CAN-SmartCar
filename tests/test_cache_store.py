"""Exercise the production C cache store on real temporary files (not ESP32 Flash)."""
import ctypes as C
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware/stm32_bootloader'))
from package import APP_BASE, APP_CAPACITY, metadata


class Info(C.Structure):
    _fields_ = [('size', C.c_uint32), ('crc', C.c_uint32), ('version', C.c_uint32)]


class Upload(C.Structure):
    _fields_ = [('file', C.c_void_p), ('written', C.c_size_t), ('expected', C.c_size_t),
                ('part_path', C.c_char_p), ('ready_path', C.c_char_p), ('failed', C.c_int)]


LIB = C.CDLL(sys.argv.pop(1))
PTR = C.POINTER(C.c_ubyte)
LIB.cache_verify.argtypes = [C.c_char_p, C.POINTER(Info), PTR, C.c_size_t]
LIB.cache_start.argtypes = [C.POINTER(Upload), C.c_char_p, C.c_char_p, C.c_size_t]
LIB.cache_write.argtypes = [C.POINTER(Upload), C.c_char_p, C.c_size_t]
LIB.cache_finish.argtypes = [C.POINTER(Upload), C.POINTER(Info), PTR, C.c_size_t]
LIB.cache_abort.argtypes = [C.POINTER(Upload)]
LIB.cache_abort.restype = None


def image(size=1928):
    data = bytearray((i * 31) & 255 for i in range(size))
    struct.pack_into('<II', data, 0, 0x20005000, APP_BASE + 257)
    return bytes(data)


def package(data=None, version=3):
    data = image() if data is None else data
    return metadata(data, version) + data


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.part = Path(self.temp.name) / 'car.part'
        self.ready = Path(self.temp.name) / 'car.ready'
        self.part_b, self.ready_b = str(self.part).encode(), str(self.ready).encode()
        self.upload, self.info = Upload(), Info()
        self.scratch = (C.c_ubyte * APP_CAPACITY)()
        self.addCleanup(lambda: LIB.cache_abort(C.byref(self.upload)))

    def verify(self):
        return LIB.cache_verify(self.ready_b, C.byref(self.info), self.scratch, APP_CAPACITY)

    def start(self, count):
        return LIB.cache_start(C.byref(self.upload), self.part_b, self.ready_b, count)

    def write(self, data):
        return LIB.cache_write(C.byref(self.upload), data, len(data))

    def finish(self):
        return LIB.cache_finish(C.byref(self.upload), C.byref(self.info), self.scratch, APP_CAPACITY)

    def publish(self, data):
        self.assertEqual(self.start(len(data)), 1)
        for offset in range(0, len(data), 127):
            self.assertEqual(self.write(data[offset:offset + 127]), 1)
        return self.finish()

    def test_publish_and_reopen(self):
        data = package()
        self.assertEqual(self.publish(data), 1)
        self.assertFalse(self.part.exists())
        self.assertEqual(self.ready.read_bytes(), data)
        self.assertEqual(self.verify(), 1)
        self.assertEqual((self.info.size, self.info.version), (1928, 3))

    def test_complete_replacement(self):
        self.ready.write_bytes(package(version=2))
        self.assertEqual(self.publish(package()), 1)
        self.assertEqual(self.ready.read_bytes(), package())

    def test_only_sixty_percent_does_not_publish(self):
        data = package()
        self.assertEqual(self.start(len(data)), 1)
        self.assertEqual(self.write(data[:len(data) * 6 // 10]), 1)
        self.assertEqual(self.finish(), 0)
        self.assertEqual(self.verify(), 0)

    def test_partial_upload_keeps_old_ready(self):
        old = package(version=2)
        self.ready.write_bytes(old)
        self.start(len(package()))
        self.write(package()[:500])
        self.assertEqual(self.finish(), 0)
        self.assertEqual(self.ready.read_bytes(), old)

    def test_corrupt_body_keeps_old_ready(self):
        old = package(version=2)
        self.ready.write_bytes(old)
        damaged = bytearray(package())
        damaged[-1] ^= 1
        self.assertEqual(self.publish(bytes(damaged)), 0)
        self.assertEqual(self.ready.read_bytes(), old)

    def test_invalid_records_and_trailing_data(self):
        valid = package()
        cases = [valid[:20], valid[:-1], valid + b'x']
        for offset in (0, 4, 8, 12, 16, 20, 24, 28):
            damaged = bytearray(valid)
            damaged[offset] ^= 1
            cases.append(bytes(damaged))
        for data in cases:
            with self.subTest(size=len(data), prefix=data[:32].hex()):
                self.ready.write_bytes(data)
                self.assertEqual(self.verify(), 0)
                self.assertEqual(self.info.size, 0)

    def test_bad_vectors_with_matching_crc(self):
        for stack, entry in ((0, APP_BASE + 257), (0x20005000, APP_BASE + 256),
                             (0x20005000, APP_BASE + 1), (0x20005000, APP_BASE + 4001)):
            data = bytearray(image())
            struct.pack_into('<II', data, 0, stack, entry)
            self.assertEqual(self.publish(package(data)), 0)

    def test_zero_version_rejected(self):
        self.assertEqual(self.publish(package(version=0)), 0)

    def test_size_limits(self):
        for count in (0, 289, APP_CAPACITY + 33, 0xFFFFFFFF):
            self.assertEqual(self.start(count), 0)
        self.assertFalse(self.part.exists())

    def test_maximum_and_odd_images(self):
        for size in (258, 259, APP_CAPACITY):
            self.assertEqual(self.publish(package(image(size))), 1)
            self.assertEqual(self.info.size, size)

    def test_overshoot_poisoned_until_abort(self):
        data = package()
        self.start(len(data))
        self.assertEqual(self.write(data + b'!'), 0)
        self.assertEqual(self.write(data), 0)
        self.assertEqual(self.finish(), 0)
        self.assertFalse(self.ready.exists())

    def test_orphan_complete_part_never_ready(self):
        self.part.write_bytes(package())
        self.assertEqual(self.verify(), 0)

    def test_corruption_after_publication_detected(self):
        self.assertEqual(self.publish(package()), 1)
        damaged = bytearray(self.ready.read_bytes())
        damaged[-1] ^= 1
        self.ready.write_bytes(damaged)
        self.assertEqual(self.verify(), 0)

    def test_abort_closes_and_preserves_old(self):
        old = package(version=2)
        self.ready.write_bytes(old)
        self.start(len(package()))
        self.write(package()[:400])
        LIB.cache_abort(C.byref(self.upload))
        self.assertFalse(self.part.exists())
        self.assertEqual(self.ready.read_bytes(), old)

    def test_rename_failure_not_success(self):
        self.ready.mkdir()
        self.assertEqual(self.publish(package()), 0)
        self.assertEqual(self.info.size, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
