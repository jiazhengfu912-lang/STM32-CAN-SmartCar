"""验证IMU协议、CSV解码、导航所有权和独立数据年龄，不连接硬件。"""
from pathlib import Path
import struct
import sys
import time
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'desktop')); sys.path.insert(0,str(ROOT/'tools'))
from imu_protocol import decode_imu
from server import Console
from capture_imu import decode


class FakeGateway:
    """记录调用，不创建网络或串口。"""
    def __init__(self):
        """初始化测试调用列表。"""
        self.calls=[]

    def request(self,path,data=None,**kwargs):
        """模拟一次成功应答，用于断言没有重试。"""
        self.calls.append((path,data)); return {'result':0,'state':1}


class ImuTests(unittest.TestCase):
    """执行生产解码和控制接口的边界验证。"""
    def payload(self):
        """构造明确标为测试的协议帧。"""
        p=bytearray(112);p[:4]=bytes([1,3,2,0]);struct.pack_into('<3I',p,4,1000,990,1)
        struct.pack_into('<7h',p,16,-8192,0,8192,-340,0,0,-65)
        struct.pack_into('<7f',p,32,.1,.2,.3,1,2,.5,-.1)
        return p

    def test_decode_independent_age_and_finite(self):
        """运动在线不能令过期IMU有效；坐标依赖独立标志。"""
        p=self.payload();r=decode_imu(p,100)
        self.assertTrue(r['position_valid']);self.assertEqual(r['age_ms'],110);self.assertEqual(r['ax'],-8192)
        self.assertFalse(decode_imu(p,490)['valid']);self.assertFalse(decode_imu(p,0,False)['position_valid'])
        p[1]=1;self.assertFalse(decode_imu(p)['position_valid'])
        struct.pack_into('<f',p,32,float('nan'));self.assertFalse(decode_imu(p)['valid'])
        self.assertFalse(decode_imu(b'')['valid'])

    def test_timestamp_wrap_crc_and_invalid_samples(self):
        """跨时基回绕仍计算真实年龄，CRC损坏帧不作为有效采样。"""
        p=self.payload();struct.pack_into('<II',p,4,4,0xfffffff0)
        self.assertEqual(decode_imu(p)['age_ms'],20)
        head=b'\xa5\x5a'+struct.pack('<BBHH',1,0x12,3,len(p));frame=head+p
        frame+=struct.pack('<I',zlib.crc32(frame[2:]))
        bad=bytearray(frame);bad[20]^=1
        result=decode(b'noise'+bad+frame)
        self.assertEqual(result['bad_crc'],1);self.assertEqual(len(result['samples']),1)
        self.assertEqual(result['samples'][0]['frame_sequence'],3)

    def test_navigation_owner_and_validation(self):
        """导航复用控制权，非法请求不得转发，停止后不能续发导航心跳。"""
        gateway=FakeGateway();app=Console(gateway)
        command={'op':3,'args':[5],'owner':'test','at':time.time()*1000}
        with self.assertRaises(ValueError):app.navigation(command)
        app.control({'op':1,'owner':'test','at':time.time()*1000})
        app.navigation(command);path,payload=gateway.calls[-1]
        self.assertEqual(path,'/navigation');self.assertEqual(len(payload),32)
        self.assertEqual(struct.unpack_from('<f',payload,12)[0],5)
        with self.assertRaises(ValueError):app.navigation({**command,'owner':'other'})
        with self.assertRaises(ValueError):app.navigation({**command,'args':[float('nan')]})
        with self.assertRaises(ValueError):app.navigation({**command,'op':5})
        app.control({'op':4,'owner':'test','at':time.time()*1000})
        self.assertEqual(gateway.calls[-1][1][0],4)
        app.control({'op':0,'owner':'test','at':time.time()*1000})
        with self.assertRaises(ValueError):app.control({'op':4,'owner':'test','at':time.time()*1000})


if __name__=='__main__':unittest.main(verbosity=2)
