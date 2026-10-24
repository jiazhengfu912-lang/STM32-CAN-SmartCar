"""生命周期状态与查询均有时效，显式恢复不继承旧查询的等待状态。"""


class LifecycleReadiness:
    def __init__(self, names, timeout=3.):
        """用单调墙钟记录响应和待决查询，不把缓存active当成永久就绪。"""
        self.names=tuple(names);self.timeout=timeout;self.reset()

    def reset(self):
        """显式恢复后必须重新确认每一个关键生命周期节点。"""
        self.states={};self.pending={}

    def requested(self, name, now):
        """每次请求单独计时，丢失响应不会永久占住查询槽。"""
        self.pending[name]=now

    def received(self, name, state, now):
        """仅新鲜的实际响应才能确认active。"""
        self.pending.pop(name,None);self.states[name]=(state,now)

    def unavailable(self, name):
        """服务消失或请求失败时丢弃旧active证据。"""
        self.states.pop(name,None)

    def expired(self, now):
        """返回超时查询，由ROS适配层取消并移除待决请求。"""
        return [name for name,sent in self.pending.items() if now-sent>=self.timeout]

    def ready(self, now):
        """全部节点均为新鲜active响应才允许发新导航目标。"""
        return all(name in self.states and self.states[name][0]==3 and
                   0.<=now-self.states[name][1]<self.timeout for name in self.names)

    def snapshot(self, now):
        """输出等待原因所需的纯数据，避免诊断日志包含Future对象。"""
        return {'states':{name:{'id':state,'age_s':now-stamp} for name,(state,stamp) in self.states.items()},
                'pending_age_s':{name:now-stamp for name,stamp in self.pending.items()}}
