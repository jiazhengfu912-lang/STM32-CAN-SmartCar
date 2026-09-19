"""只读ESP32 USB，保存原始串口字节并校验解码停车后导出的TRC3启动诊断。"""
import argparse
import json
from pathlib import Path
import struct
import time
import zlib
import serial


def decode(data):
    """跳过状态文本，按长度和CRC校验协议帧；不把损坏帧当有效采样。"""
    rows=[];bad=0;pos=0
    while pos+12<=len(data):
        at=data.find(b'\xa5\x5a',pos)
        if at<0:break
        if at+8>len(data):break
        size=struct.unpack_from('<H',data,at+6)[0]
        if data[at+2]!=1 or size>256:pos=at+1;continue
        end=at+12+size
        if end>len(data):break
        if zlib.crc32(data[at+2:end-4])!=struct.unpack_from('<I',data,end-4)[0]:
            bad+=1;pos=at+1;continue
        p=data[at+8:end-4]
        if data[at+3]==0x11 and size==28 and p[:4]==b'TRC3':
            fields=struct.unpack('<I8h4B',p[4:])
            rows.append(dict(zip(['uptime_ms','target_left','target_right','rpm_left','rpm_right',
                'output_left','output_right','integral_left','integral_right','starting_left',
                'starting_right','state','fault'],fields)))
        pos=end
    return {'samples':rows,'bad_crc':bad}


def main():
    """有界只读采集，DTR/RTS关闭后打开串口，结束保存原字节和解码JSON。"""
    p=argparse.ArgumentParser();p.add_argument('--port',default='COM3')
    p.add_argument('--seconds',type=float,default=45);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    s=serial.Serial();s.port=a.port;s.baudrate=115200;s.timeout=.2;s.dtr=False;s.rts=False
    data=bytearray()
    try:
        s.open();print('USB reader ready',flush=True);deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline:data.extend(s.read(4096))
    finally:
        if s.is_open:s.close()
        (a.output/'serial.bin').write_bytes(data)
        result=decode(data);(a.output/'startup.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('samples',len(result['samples']),'bad_crc',result['bad_crc'],flush=True)


if __name__=='__main__':main()
