"""SmartCar 本地控制台：仅监听回环地址，设备令牌只保存在本机配置文件中。"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import csv
import io
import json
import secrets
import struct
import socket
import sys
import threading
import time
import http.client
from urllib.parse import urlsplit
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'firmware/stm32_bootloader'))
import package
import zlib


class Gateway:
    """封装有限超时的设备请求；不会在后台重复发送任何运动命令。"""
    def __init__(self, config):
        """加载指定局域网设备地址与认证令牌，绕过系统代理。"""
        self.url=config.get('gateway','').rstrip('/')
        self.token=config.get('token','')
        # 控制/升级与遥测分别复用连接，避免短心跳重复 TCP 握手；各通道独立串行。
        self.connections={}
        self.speed_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.speed_socket.setblocking(False)
        self.channel_locks={'control':threading.Lock(),'telemetry':threading.Lock()}

    def request(self, path, data=None, headers=None, timeout=1):
        """复用有限超时的 HTTP 连接；失败即丢弃连接，不重发可能已执行的命令。"""
        if not self.url: raise ValueError('尚未配置 ESP32 地址，请先运行 tools/configure.py')
        if path=='/control' and data is not None and len(data)==16 and data[0]==2:
            return self.send_speed(data)
        channel='telemetry' if path=='/telemetry' else 'control' 
        h={'Authorization':'Bearer '+self.token,'Content-Type':'application/octet-stream'}
        h.update(headers or {})
        with self.channel_locks[channel]:
            connection=self.connections.get(channel)
            if connection is None:
                address=urlsplit(self.url)
                connection=http.client.HTTPConnection(address.hostname,address.port or 80,timeout=timeout)
                self.connections[channel]=connection
            connection.timeout=timeout
            if connection.sock is not None: connection.sock.settimeout(timeout)
            try:
                connection.request('GET' if data is None else 'POST',path,body=data,headers=h)
                response=connection.getresponse()
                body=response.read()
                if response.status==404: raise ValueError('设备尚未提供此接口，请安装 SmartCar 网关固件')
                if response.status>=400: raise ValueError(body.decode('utf-8',errors='replace'))
                return json.loads(body)
            except Exception:
                connection.close()
                self.connections.pop(channel,None)
                raise

    def send_speed(self, payload):
        """只发送当前速度包一次；返回本机发送结果，STM32 是否接受以遥测序号和状态为准。"""
        if len(self.token)!=32: raise ValueError('设备令牌配置无效')
        address=urlsplit(self.url)
        packet=b'SCU1'+self.token.encode('ascii')+payload
        count=self.speed_socket.sendto(packet,(address.hostname,3232))
        if count!=len(packet): raise OSError('速度数据包未完整发送')
        return {'result':0,'delivery':'sent_unconfirmed','transport':'udp-v1'}


class Console:
    """串行化控制与升级，保存最新真实遥测；网络失败不伪造零速或在线状态。"""
    def __init__(self, gateway):
        """初始化控制会话、有限记录缓存和同源请求令牌。"""
        self.gateway=gateway; self.lock=threading.Lock(); self.csrf=secrets.token_hex(24)
        self.session=0; self.sequence=0; self.owner=None; self.rows=[]
        self.control_trace=[]
        self.data_lock=threading.Lock(); self.latest={'valid':False,'error':'尚未连接设备'}
        self.upgrade={'state':'idle'}; self.running=True; self.last_sample=None; self.received_at=0

    def poll(self):
        """后台只读取遥测，绝不自动发送心跳、解锁或速度指令。"""
        while self.running:
            # 控制心跳不能阻塞遥测刷新；仅上传或安装固件时暂停轮询。
            with self.data_lock: upgrading=self.upgrade['state'] in ('uploading','installing')
            if not upgrading:
                try:
                    data=self.gateway.request('/telemetry',timeout=.2)
                    with self.data_lock:
                        self.latest=data; self.received_at=time.monotonic()
                        stamp=(data.get('uptime_ms'),data.get('session'))
                        if data.get('valid') and stamp!=self.last_sample:
                            self.last_sample=stamp
                            self.rows.append({'pc_time':time.time(),**data})
                            if len(self.rows)>36000: del self.rows[:1000]
                except Exception as error:
                    # 单次失败不刷新旧数据时间；API 仍在原始有效期届满时判定过期。
                    with self.data_lock: self.latest={**self.latest,'error':str(error)}
            time.sleep(.1)

    def control(self, command):
        """发送当前浏览器的即时命令；忙时拒绝，避免排队积压后突然执行旧动作。"""
        if not self.lock.acquire(blocking=False): raise ValueError('设备忙，请停止操作后重试')
        started=time.monotonic()
        event={'pc_time':time.time(),'op':command.get('op'),'phase':'rejected'}
        try:
            op=command.get('op'); owner=command.get('owner')
            if type(op)!=int or op not in (0,1,2,3) or not isinstance(owner,str) or not 1<=len(owner)<=64:
                raise ValueError('控制请求格式错误')
            if abs(time.time()*1000-float(command.get('at',0)))>250: raise ValueError('控制请求已过期')
            left=command.get('left',0); right=command.get('right',0)
            if type(left)!=int or type(right)!=int or not -720<=left<=720 or not -720<=right<=720:
                raise ValueError('轮速超限')
            if op in (1,3) and (left or right): raise ValueError('解锁和清故障必须使用零目标')
            if op==2 and (self.owner!=owner or not self.session): raise ValueError('当前页面没有控制权')
            if op==1:
                if self.owner is not None and self.owner!=owner: raise ValueError('另一页面占用控制权，请先停止')
                self.session=secrets.randbelow(0xFFFFFFFE)+1; self.sequence=0
            self.sequence+=1
            payload=struct.pack('<B3xIIhh',op,self.session,self.sequence,left,right)
            event.update(sequence=self.sequence,session=self.session,left=left,right=right,
                         request_age_ms=time.time()*1000-float(command.get('at',0)),phase='sending')
            result=self.gateway.request('/control',payload,timeout=.4)
            event.update(phase='returned',delivery=result.get('delivery','ack'),result=result.get('result'))
            if result.get('result')!=0: raise ValueError('STM32 拒绝请求：'+str(result))
            if op==1: self.owner=owner
            if op in (0,3): self.owner=None; self.session=0
            return result
        except Exception as error:
            event['error']=str(error)
            raise
        finally:
            event['duration_ms']=(time.monotonic()-started)*1000
            # 仅保留有限诊断，不存页面身份或认证令牌；UDP返回仍不是送达确认。
            with self.data_lock:
                self.control_trace.append(event)
                if len(self.control_trace)>2000: del self.control_trace[:200]
            self.lock.release()

    def install(self, blob):
        """先独立验证 OTA 文件，再上传并安装；整个操作中禁止发送运动请求。"""
        if not 290<=len(blob)<=47136: raise ValueError('OTA 包长度错误')
        magic,fmt,base,size,crc,version,hcrc,commit=struct.unpack('<8I',blob[:32])
        image=blob[32:]
        if (magic,fmt,base,commit)!=(package.MAGIC,1,package.APP_BASE,package.COMMITTED) or version<100 or size!=len(image) or zlib.crc32(image)!=crc or zlib.crc32(blob[:24])!=hcrc:
            raise ValueError('OTA 包校验失败')
        package.check_vectors(image,base,package.APP_CAPACITY)
        if not self.lock.acquire(blocking=False): raise ValueError('设备忙')
        try:
            status=self.gateway.request('/status')
            if status.get('stm32')!='M2B2':
                telemetry=self.gateway.request('/telemetry')
                if not telemetry.get('valid') or telemetry.get('state')!=0: raise ValueError('请先禁止驱动；当前不是可升级状态')
            self.owner=None; self.session=0
            with self.data_lock:
                self.latest={'valid':False,'error':'升级期间暂停遥测'}
                self.upgrade={'state':'uploading','version':version}
            self.gateway.request('/cache',blob,timeout=35)
            with self.data_lock: self.upgrade={'state':'installing','version':version}
            result=self.gateway.request('/install',struct.pack('<III',size,crc,version),{'X-Install-Id':uuid.uuid4().hex},timeout=60)
            if result.get('result')!='PASS': raise ValueError(str(result))
            with self.data_lock: self.upgrade={'state':'complete',**result}
            return result
        except Exception as error:
            with self.data_lock: self.upgrade={'state':'failed','error':str(error)}
            raise
        finally: self.lock.release()


class Handler(BaseHTTPRequestHandler):
    """只暴露固定静态文件与本地 API，不提供任意路径或任意 URL 代理。"""
    protocol_version='HTTP/1.1'
    disable_nagle_algorithm=True
    def log_message(self, *args):
        """关闭逐请求日志，避免高频遥测刷屏及凭据意外进入日志。"""

    def reply(self, body, kind='application/json', status=200):
        """发送带长度和禁止缓存头的响应，兼容页面关闭时的连接断开。"""
        if not isinstance(body,bytes): body=json.dumps(body,ensure_ascii=False).encode()
        try:
            self.send_response(status); self.send_header('Content-Type',kind+'; charset=utf-8')
            self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff'); self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError): self.close_connection=True

    def local(self):
        """限制 Host 为本机端口，阻止 DNS 重绑定访问控制 API。"""
        return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')

    def do_GET(self):
        """提供页面、状态与 CSV 下载；状态缺失时显示离线而非虚构反馈。"""
        if not self.local(): self.reply({'error':'invalid host'},status=403); return
        app=self.server.console
        files={'/':('index.html','text/html'),'/app.js':('app.js','text/javascript'),'/style.css':('style.css','text/css')}
        if self.path in files:
            filename,kind=files[self.path]; self.reply((Path(__file__).parent/filename).read_bytes(),kind); return
        if self.path=='/api/state':
            with app.data_lock:
                telemetry=dict(app.latest)
                elapsed=int((time.monotonic()-app.received_at)*1000)
                if 'age_ms' in telemetry: telemetry['age_ms']+=elapsed
                if elapsed>=500 or telemetry.get('age_ms',0)>=500: telemetry['valid']=False
                result={'telemetry':telemetry,'upgrade':dict(app.upgrade),'csrf':app.csrf}
            self.reply(result); return
        if self.path=='/api/record.csv':
            with app.data_lock: rows=list(app.rows)
            out=io.StringIO(newline='')
            if rows:
                writer=csv.DictWriter(out,fieldnames=list(rows[0]),extrasaction='ignore'); writer.writeheader(); writer.writerows(rows)
            self.reply(out.getvalue().encode('utf-8-sig'),'text/csv'); return
        if self.path=='/api/control_trace':
            with app.data_lock: trace=list(app.control_trace)
            self.reply(trace); return
        self.reply({'error':'not found'},status=404)

    def do_POST(self):
        """校验同源防护与有限正文后执行一次控制或固件安装请求。"""
        app=self.server.console
        if not self.local() or self.headers.get('X-Console-Token')!=app.csrf:
            self.close_connection=True # 未读取的正文不能留给下一条持久连接请求。
            self.reply({'error':'同源校验失败'},status=403); return
        body_complete=False
        try:
            self.connection.settimeout(3)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=47136: raise ValueError('请求长度错误')
            body=self.rfile.read(size)
            if len(body)!=size: raise ValueError('请求不完整')
            body_complete=True
            if self.path=='/api/control': result=app.control(json.loads(body))
            elif self.path=='/api/upgrade': result=app.install(body)
            else: self.reply({'error':'not found'},status=404); return
            self.reply(result)
        except Exception as error:
            if not body_complete: self.close_connection=True
            self.reply({'error':str(error)},status=400)


def main():
    """启动本机控制台及只读遥测线程；配置不完整时仍能打开离线界面。"""
    parser=argparse.ArgumentParser(); parser.add_argument('--port',type=int,default=18765)
    args=parser.parse_args(); config=ROOT/'config.local.json'
    values=json.loads(config.read_text(encoding='utf-8')) if config.exists() else {}
    console=Console(Gateway(values))
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler); server.console=console
    thread=threading.Thread(target=console.poll,daemon=True); thread.start()
    print(f'SmartCar console: http://127.0.0.1:{args.port}',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: console.running=False; server.server_close()


if __name__=='__main__': main()
