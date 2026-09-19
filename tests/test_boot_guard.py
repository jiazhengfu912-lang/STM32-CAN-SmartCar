"""Fault-inject the actual C startup guard, using independently packed images."""
import ctypes
from pathlib import Path
import re
import struct
import sys
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "firmware/stm32_bootloader"))
import package

LIBRARY = ctypes.CDLL(str(Path(sys.argv.pop(1)).resolve()))
LIBRARY.boot_validate.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
LIBRARY.boot_validate.restype = ctypes.c_int


def image(size=1024, stack=0x20005000, reset=package.APP_BASE + 257):
    data = bytearray((i * 17 + 3) & 255 for i in range(size))
    struct.pack_into("<II", data, 0, stack, reset)
    return bytes(data)


def record_fields(record, index, value):
    data = bytearray(record)
    struct.pack_into("<I", data, index * 4, value)
    struct.pack_into("<I", data, 24, zlib.crc32(data[:24]))
    return bytes(data)


def validate(record, app, available=None):
    return LIBRARY.boot_validate(ctypes.create_string_buffer(record),
                                 ctypes.create_string_buffer(app),
                                 len(app) if available is None else available)


class GuardTests(unittest.TestCase):
    def test_layout_matches_firmware_and_linkers(self):
        header = (ROOT / "firmware/stm32_bootloader/boot_layout.h").read_text(encoding="utf-8")
        for name, value in {"BOOT_FLASH_BASE": package.FLASH_BASE, "BOOT_APP_BASE": package.APP_BASE,
                            "BOOT_APP_LIMIT": package.APP_LIMIT, "BOOT_META_BASE": package.META_BASE,
                            "BOOT_PARAM_BASE": package.PARAM_BASE, "BOOT_COMMITTED": package.COMMITTED,
                            "BOOT_MAGIC": package.MAGIC, "BOOT_FORMAT": package.FORMAT}.items():
            match = re.search(r"#define\s+" + name + r"\s+(0x[0-9A-F]+|\d+)U", header)
            self.assertEqual(int(match[1], 0), value)
        for name, base, capacity in (("boot", package.FLASH_BASE, 16384),
                                     ("app", package.APP_BASE, package.APP_CAPACITY)):
            numbers = re.findall(r"LR_IROM1\s+(0x\w+)\s+(0x\w+)",
                                 (ROOT / f"firmware/stm32_bootloader/{name}.sct").read_text(encoding="utf-8"))[0]
            self.assertEqual(tuple(int(n, 16) for n in numbers), (base, capacity))

    def test_valid_and_maximum_image(self):
        for size in (258, 1024, package.APP_CAPACITY):
            app = image(size)
            self.assertEqual(validate(package.metadata(app), app), 0)
            package.check_vectors(app, package.APP_BASE, package.APP_CAPACITY)

    def test_complete_app_without_commit_is_rejected(self):
        app = image()
        self.assertEqual(validate(package.metadata(app, committed=False), app), 1)
        self.assertEqual(validate(b"\xff" * 32, app), 1)

    def test_every_interrupted_metadata_prefix(self):
        app = image()
        record = package.metadata(app)
        for cut in range(32):
            self.assertNotEqual(validate(record[:cut] + b"\xff" * (32 - cut), app), 0, cut)

    def test_every_metadata_bit_flip(self):
        app = image()
        record = package.metadata(app)
        for bit in range(256):
            damaged = bytearray(record)
            damaged[bit // 8] ^= 1 << (bit % 8)
            self.assertNotEqual(validate(bytes(damaged), app), 0, bit)

    def test_wrong_target_format_or_magic_even_with_header_crc(self):
        app = image()
        for index, value in ((0, 0), (1, 2), (2, 0x08000000), (2, 0x0800FC00)):
            self.assertEqual(validate(record_fields(package.metadata(app), index, value), app), 3)

    def test_lengths_and_available_prevent_out_of_bounds(self):
        app = image()
        for size in (0, 8, 256, 257, 1025, package.APP_CAPACITY + 1, 0xFFFFFFFF):
            self.assertEqual(validate(record_fields(package.metadata(app), 3, size), app), 4)
        self.assertEqual(validate(package.metadata(app), app, available=8), 4)

    def test_invalid_stack_or_reset_even_with_image_crc(self):
        for stack in (0x20000000, 0x1FFFFFF8, 0x20005008, 0x20004FFF):
            app = image(stack=stack)
            self.assertEqual(validate(package.metadata(app), app), 5)
        for reset in (package.APP_BASE + 256, package.APP_BASE + 1,
                      0x08000001, package.APP_BASE + 1025, package.APP_LIMIT + 1):
            app = image(reset=reset)
            self.assertEqual(validate(package.metadata(app), app), 5)

    def test_legitimate_vectors_do_not_hide_partial_image(self):
        app = image()
        record = package.metadata(app)
        for cut in range(8, len(app)):
            partial = app[:cut] + b"\xff" * (len(app) - cut)
            if partial != app:
                self.assertEqual(validate(record, partial), 6, cut)

    def test_body_corruption_rejected(self):
        app = image()
        for offset in (256, 512, len(app) - 1):
            damaged = bytearray(app)
            damaged[offset] ^= 1
            self.assertEqual(validate(package.metadata(app), bytes(damaged)), 6)

    def test_current_built_images(self):
        app = (ROOT / "out/app/app.bin").read_bytes()
        record = package.metadata(app, 100)
        self.assertEqual(validate(record, app), 0)
        bad = app[:-1] + bytes([app[-1] ^ 1])
        self.assertEqual(validate(record, bad), 6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
