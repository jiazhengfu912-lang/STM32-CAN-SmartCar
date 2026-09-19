"""Create a SIMULATED byte stream or decode a saved stream into CSV."""
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path

from car_protocol import Decoder, Frame, SequenceTracker, TELEMETRY, Telemetry, encode


def decode_file(source, destination, simulated=False):
    """将早期测试字节流解码为 CSV，保留模拟数据标识。"""
    decoder, tracker = Decoder(), SequenceTracker()
    count = invalid = ignored = 0
    columns = ["source", "frame_seq", "sequence_status", *Telemetry.__dataclass_fields__]
    with source.open("rb") as raw, destination.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=columns)
        writer.writeheader()

        def save(frames):
            """将已校验样本及序号诊断结果写入 CSV。"""
            nonlocal count, invalid, ignored
            for frame in frames:
                if frame.kind != TELEMETRY:
                    ignored += 1
                    continue
                try:
                    sample = Telemetry.unpack(frame.payload)
                except ValueError:
                    invalid += 1
                    continue
                writer.writerow({"source": "SIMULATED" if simulated else "CAPTURE_FILE",
                                 "frame_seq": frame.seq,
                                 "sequence_status": tracker.observe(frame.seq), **asdict(sample)})
                count += 1

        # 原始文件没有字节到达时间，块大小不代表 UART 帧边界。
        while block := raw.read(73):
            save(decoder.feed(block, 0))
        save(decoder.poll(100))
    return {"samples": count, "crc_errors": decoder.crc_errors,
            "header_errors": decoder.header_errors, "timeouts": decoder.timeouts,
            "invalid_telemetry": invalid, "ignored_frames": ignored}


def main():
    """解析命令行参数并执行本模块提供的离线操作。"""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("simulate")
    demo.add_argument("--output-dir", type=Path, required=True)
    decode = sub.add_parser("decode")
    decode.add_argument("input", type=Path)
    decode.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "simulate":
        args.output_dir.mkdir(parents=True, exist_ok=True)
        source = args.output_dir / "simulated_telemetry.bin"
        destination = args.output_dir / "simulated_telemetry.csv"
        wire = bytearray(b"noise")
        for index in range(30):
            actual = min(index * 2, 30)
            sample = Telemetry(index * 100, 30, 30, actual, actual,
                               (30 - actual) * 20, (30 - actual) * 20,
                               index * 10, index * 10, 1, 0, 0, 0, 1, 1)
            packet = encode(Frame(TELEMETRY, index, sample.pack()))
            if index == 7:
                damaged = bytearray(packet)
                damaged[12] ^= 1
                wire.extend(damaged)
            else:
                wire.extend(packet)
            if index == 12:
                wire.extend(packet)
        source.write_bytes(wire)
        result = decode_file(source, destination, simulated=True)
        result["source"] = "SIMULATED"
    else:
        if args.input.resolve() == args.output.resolve():
            parser.error("input and output must differ")
        result = decode_file(args.input, args.output)
        destination = args.output
    print(json.dumps({**result, "csv": str(destination.resolve())}, indent=2))


if __name__ == "__main__":
    main()
