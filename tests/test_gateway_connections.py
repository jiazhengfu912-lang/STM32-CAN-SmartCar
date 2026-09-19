"""用本机 HTTP 替身检查持久连接、通道隔离和失败不重试，不访问电机。"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock,patch
import struct
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'desktop'))
spec=importlib.util.spec_from_file_location('gateway_under_test',os.environ.get('SMARTCAR_SERVER_TEST',str(ROOT/'desktop/server.py')))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Handler(BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'
    def log_message(self,*args):
        """不打印本机请求日志。"""
    def do_GET(self):
        """记录真实连接端口并应答，供测试判断连接是否复用。"""
        self.server.calls.append((self.path,self.client_address[1]))
        body=b'{"ok":true}'
        self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_POST(self):
        """模拟拒绝命令，检查主机不会自动再次发送。"""
        self.rfile.read(int(self.headers.get('Content-Length',0)))
        self.server.calls.append((self.path,self.client_address[1]))
        body=b'{"error":"rejected"}'
        self.send_response(409);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)


class GatewayTests(unittest.TestCase):
    def setUp(self):
        """启动仅监听本机的网关替身。"""
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.server.calls=[]
        self.worker=threading.Thread(target=self.server.serve_forever,daemon=True);self.worker.start()
        self.gateway=module.Gateway({'gateway':f'http://127.0.0.1:{self.server.server_port}','token':'test'})
    def tearDown(self):
        """关闭测试连接及服务，不留下后台监听。"""
        for connection in getattr(self.gateway,'connections',{}).values():connection.close()
        self.gateway.speed_socket.close()
        self.server.shutdown();self.server.server_close();self.worker.join()
    def test_reuse_and_separate_channels(self):
        """同通道复用 TCP，遥测不会与控制共享正在使用的连接。"""
        for path in ('/status','/status','/telemetry','/telemetry'):self.gateway.request(path)
        ports=[port for _,port in self.server.calls]
        self.assertEqual(ports[0],ports[1]);self.assertEqual(ports[2],ports[3]);self.assertNotEqual(ports[0],ports[2])
    def test_speed_does_not_wait_for_http_ack(self):
        """UDP 只发送当前速度包一次，不把发送成功伪装成 STM32 应答。"""
        self.gateway.token='0123456789abcdef0123456789abcdef'
        fake=Mock();fake.sendto.return_value=52
        payload=struct.pack('<B3xIIhh',2,7,8,180,-180)
        with patch.object(self.gateway,'speed_socket',fake):
            result=self.gateway.request('/control',payload)
        self.assertEqual(result['delivery'],'sent_unconfirmed')
        fake.sendto.assert_called_once_with(b'SCU1'+self.gateway.token.encode()+payload,('127.0.0.1',3232))
        self.assertEqual(self.server.calls,[])

    def test_udp_failure_is_not_retried_or_fallback(self):
        """发送失败立即返回，不补发旧速度，也不偷偷退回阻塞 HTTP。"""
        self.gateway.token='0123456789abcdef0123456789abcdef'
        fake=Mock();fake.sendto.side_effect=BlockingIOError('busy')
        with patch.object(self.gateway,'speed_socket',fake):
            with self.assertRaises(BlockingIOError):self.gateway.request('/control',struct.pack('<B3xIIhh',2,1,1,0,0))
        self.assertEqual(fake.sendto.call_count,1);self.assertEqual(self.server.calls,[])

    def test_error_is_not_retried(self):
        """拒绝的控制只发送一次，丢弃连接后由下一次独立请求重新连接。"""
        with self.assertRaises(ValueError):self.gateway.request('/control',b'test')
        self.assertEqual(len(self.server.calls),1)
        self.assertNotIn('control',getattr(self.gateway,'connections',{}))
        self.gateway.request('/status');self.assertEqual(len(self.server.calls),2)
        self.assertNotEqual(self.server.calls[0][1],self.server.calls[1][1])


if __name__=='__main__':unittest.main(verbosity=2)
