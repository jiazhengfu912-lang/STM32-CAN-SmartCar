"""用真实本地HTTP处理器及假网关验证连接复用，不接触实物。"""
import http.client
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'desktop'))
from server import Handler, Console


class FakeGateway:
    """仅记录测试控制，避免测试向设备发包。"""
    def request(self,path,data=None,**kwargs):
        """返回确定的测试应答。"""
        return {'result':0,'state':1}


class HttpTests(unittest.TestCase):
    """通过实际TCP请求检查状态、控制及错误响应均可复用连接。"""
    def test_persistent_control_and_state(self):
        """连续请求保留同一个socket，并保证错误响应不破坏后续消息边界。"""
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        server.console=Console(FakeGateway())
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        conn=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=1)
        try:
            conn.request('GET','/api/state');response=conn.getresponse()
            self.assertEqual(response.version,11)
            self.assertFalse(response.will_close)
            data=json.loads(response.read());sock=conn.sock
            command={'op':1,'owner':'test','at':time.time()*1000,'left':0,'right':0}
            conn.request('POST','/api/control',json.dumps(command),{'X-Console-Token':data['csrf']})
            response=conn.getresponse();self.assertEqual(response.status,200);response.read()
            self.assertIs(conn.sock,sock)
            conn.request('POST','/api/control','{}',{'X-Console-Token':data['csrf']})
            response=conn.getresponse();self.assertEqual(response.status,400);response.read()
            conn.request('GET','/api/state');response=conn.getresponse()
            self.assertEqual(response.status,200);response.read();self.assertIs(conn.sock,sock)
            conn.request('GET','/api/control_trace');response=conn.getresponse()
            self.assertEqual(response.status,200);trace=json.loads(response.read())
            self.assertEqual(trace[0]['sequence'],1)
            self.assertEqual(trace[0]['phase'],'returned')
            self.assertNotIn('csrf',trace[0]);self.assertNotIn('owner',trace[0])
        finally:
            conn.close();server.shutdown();server.server_close();worker.join()


if __name__=='__main__':unittest.main(verbosity=2)
