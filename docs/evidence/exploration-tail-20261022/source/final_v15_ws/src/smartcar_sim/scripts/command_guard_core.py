"""仿真速度指令的有界接收与超时逻辑；不连接实车。"""
import math


class CommandGuard:
    def __init__(self, timeout=0.3, linear_limit=0.25, angular_limit=1.5):
        self.timeout=timeout
        self.linear_limit=linear_limit
        self.angular_limit=angular_limit
        self.last_time=None
        self.command=(0.0,0.0)

    def receive(self, linear, angular, now):
        """无效数据清零；仅收到新的有效消息才更新时间。"""
        if not all(math.isfinite(v) for v in [linear,angular,now]):
            self.last_time=None;self.command=(0.0,0.0)
            return
        self.command=(max(-self.linear_limit,min(self.linear_limit,linear)),max(-self.angular_limit,min(self.angular_limit,angular)))
        self.last_time=now

    def output(self, now):
        """使用单调墙钟，仿真暂停不会延长指令有效期。"""
        if self.last_time is None or not math.isfinite(now):return (0.0,0.0)
        age=now-self.last_time
        if age<0 or age>=self.timeout:return (0.0,0.0)
        return self.command
