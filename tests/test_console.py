"""用模拟网关验证电脑端拒绝危险请求；此测试不访问任何真实设备。"""
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'desktop'))
from server import Console, package


class FakeGateway:
    def __init__(self):
        """建立可记录请求的网关替身，不创建网络连接。"""
        self.calls=[];self.state=0

    def request(self,path,data=None,headers=None,timeout=1):
        """提供可预测的应答，用于验证主机校验与调用顺序。"""
        self.calls.append((path,data))
        if path=='/control':return {'result':0,'state':1}
        if path=='/status':return {'stm32':'SC01'}
        if path=='/telemetry':return {'valid':True,'state':self.state}
        if path=='/install':return {'result':'PASS','running_version':100}
        return {}


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        """为每个测试创建互不共享的会话。"""
        self.gateway=FakeGateway();self.app=Console(self.gateway)

    def command(self,op=1,**kwargs):
        """构造当前时间的控制请求，允许测试覆盖边界字段。"""
        return {'op':op,'owner':'test','at':time.time()*1000,'left':0,'right':0,**kwargs}

    def test_session_owner_and_disarm(self):
        """未使能及其他页面不能驱动车辆；停止后原控制权失效。"""
        with self.assertRaises(ValueError):self.app.control(self.command(2))
        self.app.control(self.command());self.app.control(self.command(2,left=360))
        with self.assertRaises(ValueError):self.app.control(self.command(2,owner='other'))
        self.app.control(self.command(0));self.assertIsNone(self.app.owner)

    def test_stale_and_invalid_requests_never_forward(self):
        """过期、超限和非整数速度不能进入串口转发。"""
        for args in [{'at':0},{'left':721},{'left':1.5},{'op':7},{'left':1}]:
            with self.assertRaises(ValueError):self.app.control(self.command(**args))
        self.assertEqual(self.gateway.calls,[])

    def test_busy_does_not_queue(self):
        """正在安装或发送指令时拒绝新运动请求，不建立延迟队列。"""
        self.app.lock.acquire()
        try:
            with self.assertRaises(ValueError):self.app.control(self.command())
        finally:self.app.lock.release()
        self.assertEqual(self.gateway.calls,[])

    def test_control_trace_distinguishes_sent_from_rejected(self):
        """记录实际转发的序号与拒绝原因，不能把本机发送当成STM32已接收。"""
        self.app.control(self.command())
        self.app.control(self.command(2))
        with self.assertRaises(ValueError):self.app.control(self.command(2,at=0))
        trace=self.app.control_trace
        self.assertEqual([r.get('sequence') for r in trace],[1,2,None])
        self.assertEqual(trace[1]['phase'],'returned')
        self.assertEqual(trace[2]['phase'],'rejected')
        self.assertNotIn('owner',trace[1])
        self.assertGreaterEqual(trace[1]['duration_ms'],0)

    def poll_once(self):
        """运行一轮真实遥测循环，不等待定时器或接触硬件。"""
        def stop_loop(_):
            """在本轮末尾结束后台循环。"""
            self.app.running=False
        self.app.running=True
        with patch('server.time.sleep',side_effect=stop_loop):self.app.poll()

    def test_control_inflight_does_not_starve_telemetry(self):
        """控制锁被占用时仍读取遥测，避免持续心跳使网页误判数据过期。"""
        self.app.lock.acquire()
        try:self.poll_once()
        finally:self.app.lock.release()
        self.assertEqual([path for path,_ in self.gateway.calls],['/telemetry'])
        self.assertTrue(self.app.latest['valid'])

    def test_single_poll_failure_keeps_original_sample_time(self):
        """一次读取失败不伪造新样本，也不提前丢弃仍在有效期内的上一帧。"""
        self.app.latest={'valid':True,'state':1,'age_ms':20}
        self.app.received_at=123.0
        with patch.object(self.gateway,'request',side_effect=TimeoutError('probe timeout')):
            self.poll_once()
        self.assertTrue(self.app.latest['valid'])
        self.assertEqual(self.app.received_at,123.0)
        self.assertEqual(self.app.latest['age_ms'],20)
        self.assertEqual(self.app.latest['error'],'probe timeout')

    def test_upload_and_install_pause_telemetry(self):
        """上传及安装仍暂停轮询，避免与长时间升级操作竞争网关。"""
        self.app.lock.acquire()
        try:
            for state in ('uploading','installing'):
                self.app.upgrade={'state':state}
                self.poll_once()
        finally:self.app.lock.release()
        self.assertEqual(self.gateway.calls,[])

    def image(self):
        """构造有合法向量与 CRC 的最小应用测试包。"""
        image=bytearray(300);struct.pack_into('<II',image,0,0x20005000,0x08004101)
        return package.metadata(image,100)+image

    def test_bad_image_never_contacts_device(self):
        """损坏包在主机端即被拒绝，不能触发擦除。"""
        bad=bytearray(self.image());bad[-1]^=1
        with self.assertRaises(ValueError):self.app.install(bytes(bad))
        self.assertEqual(self.gateway.calls,[])

    def test_armed_cannot_upload(self):
        """车辆处于使能状态时不能开始上传与安装。"""
        self.gateway.state=1
        with self.assertRaises(ValueError):self.app.install(self.image())
        self.assertNotIn('/cache',[path for path,_ in self.gateway.calls])

    def test_verified_upload_then_install(self):
        """确认上传完成后才请求安装，并保留可显示的最终结果。"""
        self.assertEqual(self.app.install(self.image())['result'],'PASS')
        self.assertEqual([p for p,_ in self.gateway.calls][-2:],['/cache','/install'])


if __name__=='__main__':unittest.main(verbosity=2)
