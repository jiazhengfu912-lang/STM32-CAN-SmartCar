"""用完整无效控制包测量持久连接时延；网关拒绝后不进入 STM32 UART。"""
import http.client
import json
import struct
import time
from pathlib import Path
from urllib.parse import urlsplit


def main():
    """重复安全探测并保存分布；此测量不能代替实际心跳与停机验收。"""
    root=Path(__file__).resolve().parents[1]
    config=json.loads((root/'config.local.json').read_text(encoding='utf-8'))
    address=urlsplit(config['gateway'])
    connection=http.client.HTTPConnection(address.hostname,address.port or 80,timeout=2)
    samples=[];ports=set()
    try:
        for _ in range(100):
            started=time.monotonic()
            connection.request('POST','/control',body=struct.pack('<B3xIIhh',4,0,0,0,0),
                               headers={'Authorization':'Bearer '+config['token']})
            response=connection.getresponse();body=json.loads(response.read())
            assert response.status==400 and body.get('error')=='control_argument', 'Invalid request was not rejected safely'
            samples.append(round((time.monotonic()-started)*1000,1))
            if connection.sock is not None:ports.add(connection.sock.getsockname()[1])
            time.sleep(.1)
    finally:connection.close()
    report={'scope':'INVALID_OPCODE_NO_UART','count':len(samples),'connections':len(ports),
            'max_ms':max(samples),'mean_ms':round(sum(samples)/len(samples),1),
            'over100':sum(value>=100 for value in samples),'over300':sum(value>=300 for value in samples),
            'latencies':samples}
    (root/'out/control_latency_latest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({key:value for key,value in report.items() if key!='latencies'}))


if __name__=='__main__':main()
