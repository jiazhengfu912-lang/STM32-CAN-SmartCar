"""Build M2A SWD installation artifacts; does not access any device."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib

FLASH_BASE = 0x08000000
APP_BASE = 0x08004000
APP_LIMIT = 0x0800F800
APP_CAPACITY = APP_LIMIT - APP_BASE
META_BASE = APP_LIMIT
PARAM_BASE = 0x0800FC00
MAGIC = 0x3141544F
FORMAT = 1
COMMITTED = 0xA55A3CC3


def metadata(image, version=1, committed=True):
    """生成含目标地址、正文 CRC 和最后提交标记的元数据。"""
    header = struct.pack("<6I", MAGIC, FORMAT, APP_BASE, len(image),
                         zlib.crc32(image), version)
    return header + struct.pack("<II", zlib.crc32(header),
                                COMMITTED if committed else 0xFFFFFFFF)


def check_vectors(image, base, capacity):
    """校验镜像长度、初始栈和 Thumb 复位入口是否落在允许区域。"""
    if not 258 <= len(image) <= capacity:
        raise ValueError("image size outside partition")
    stack, reset = struct.unpack_from("<II", image)
    if not (0x20000000 < stack <= 0x20005000 and stack % 8 == 0):
        raise ValueError("invalid initial stack")
    if not (reset & 1 and base + 256 <= (reset & ~1) <= base + len(image) - 2):
        raise ValueError("reset vector outside image code")
    return {"stack": hex(stack), "reset": hex(reset)}


def main():
    """解析命令行参数并执行本模块提供的离线操作。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boot", required=True, type=Path)
    parser.add_argument("--app", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--firmware-version", type=int, default=1)
    args = parser.parse_args()
    if not 1 <= args.firmware_version <= 0xFFFFFFFF:
        parser.error("firmware version must be a positive uint32")
    boot, app = args.boot.read_bytes(), args.app.read_bytes()
    report = {"result": "PASS", "source": "BUILD_ONLY", "firmware_version": args.firmware_version}
    for name, image, base, size in (("boot", boot, FLASH_BASE, APP_BASE - FLASH_BASE),
                                    ("app", app, APP_BASE, APP_CAPACITY)):
        report[name] = dict(bytes=len(image), base=hex(base), capacity=size,
                            sha256=hashlib.sha256(image).hexdigest(),
                            **check_vectors(image, base, size))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    record = metadata(app, args.firmware_version)
    (args.output_dir / "metadata_committed.bin").write_bytes(record)
    (args.output_dir / "metadata_uncommitted.bin").write_bytes(metadata(app, args.firmware_version, committed=False))
    # 精确损坏一个正文位，保留合法向量与原有效记录，用于 CRC 拦截实验。
    damaged = bytearray(app)
    damaged[-1] ^= 1
    (args.output_dir / "app_corrupt.bin").write_bytes(damaged)
    report["metadata"] = {"address": hex(META_BASE), "bytes": len(record),
                          "image_crc32": f"{zlib.crc32(app):08X}",
                          "sha256": hashlib.sha256(record).hexdigest()}
    report["reserved_parameters"] = {"address": hex(PARAM_BASE), "bytes": 1024}
    (args.output_dir / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
