"""只读采集ESP32 USB诊断，保存原始字节、CRC检查结果、CSV和SHA256。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import sys
import time
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'desktop'))
from imu_protocol import decode_imu


def decode(data):
    """按协议CRC识别IMU帧；保留无效状态行与序号，不插值补样。"""
    rows = []; bad = 0; pos = 0
    while pos+12 <= len(data):
        at = data.find(b'\xa5\x5a', pos)
        if at < 0 or at+8 > len(data): break
        size = struct.unpack_from('<H', data, at+6)[0]
        if data[at+2] != 1 or size > 256: pos = at+1; continue
        end = at+12+size
        if end > len(data): break
        if zlib.crc32(data[at+2:end-4]) != struct.unpack_from('<I', data, end-4)[0]:
            bad += 1; pos = at+1; continue
        if data[at+3] == 0x12:
            row = decode_imu(data[at+8:end-4])
            row['frame_sequence'] = struct.unpack_from('<H', data, at+4)[0]
            rows.append(row)
        pos = end
    return {'samples': rows, 'bad_crc': bad, 'trailing_bytes': len(data)-pos}


def main():
    """采集期间不写串口、不切换DTR/RTS、不发送控制或心跳。"""
    import serial
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM3')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True, help='本轮版本、参数和实物工况JSON')
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600: parser.error('seconds must be in (0, 3600]')
    metadata=json.loads(args.metadata.read_text(encoding='utf-8-sig'))
    for field in ('version','hardware_condition','nav_config'):
        if field not in metadata: parser.error('metadata missing '+field)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    port = serial.Serial(); port.port=args.port; port.baudrate=115200; port.timeout=.2
    port.dtr=False; port.rts=False; data=bytearray(); error=None
    try:
        port.open(); print('USB reader ready', flush=True); end=time.monotonic()+args.seconds
        while time.monotonic()<end: data.extend(port.read(4096))
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc)
        raise
    finally:
        if port.is_open: port.close()
        (args.output/'serial.bin').write_bytes(data)
        result=decode(data); result.update(error=error, source='ESP32_USB_READ_ONLY', rate_hz=50)
        (args.output/'imu.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        with (args.output/'imu.csv').open('w',encoding='utf-8-sig',newline='') as stream:
            rows=result['samples']
            if rows:
                writer=csv.DictWriter(stream,fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
                writer.writeheader(); writer.writerows(rows)
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
        (args.output/'SHA256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')


if __name__ == '__main__': main()
