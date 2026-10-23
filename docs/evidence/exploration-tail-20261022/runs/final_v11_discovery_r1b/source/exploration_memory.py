"""观察位置和失败的有限记忆；不把整片前沿永久拉黑。"""
import math
import numpy as np


def local_context(grid, resolution, origin, point):
    """按固定世界坐标采样局部占据类别，地图扩张不改变比较坐标。"""
    offsets=np.arange(-1.2,1.201,.1);x,y=np.meshgrid(point[0]+offsets,point[1]+offsets)
    ix=np.floor((x-origin[0])/resolution).astype(int);iy=np.floor((y-origin[1])/resolution).astype(int)
    inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0]);result=np.full(x.shape,-1,dtype=np.int8)
    v=grid[iy[inside],ix[inside]];result[inside]=np.where(v<0,-1,np.where(v<=20,0,1))
    return result


class ViewpointMemory:
    """成功视点等待相关地图变化；失败最多重试一次且必须经过冷却。"""
    def __init__(self):
        """每个会话独立保存访问证据，不继承旧会话控制状态。"""
        self.entries=[]

    def record(self, target, status, grid, resolution, origin, sim):
        """在动作结束后的地图上记录背景，防止用动作成功冒充区域已观测。"""
        point=(target['x'],target['y']);near=next((e for e in self.entries if math.dist(point,e['point'])<.35),None)
        attempts=1 if near is None else near['attempts']+1
        if near is not None:self.entries.remove(near)
        self.entries.append({'point':point,'status':status,'attempts':attempts,'retry_sim':sim+30.,
                             'context':local_context(grid,resolution,origin,point)})

    def filter(self, candidates, grid, resolution, origin, sim):
        """局部地图发生实质变化可重新观察；无变化时区分冷却和已耗尽视点。"""
        active=[]
        for entry in self.entries:
            context=local_context(grid,resolution,origin,entry['point'])
            if np.count_nonzero(context!=entry['context'])>=max(8,.05*context.size):continue
            active.append(entry)
        self.entries=active;available=[];cooling=0;exhausted=0;retry=None
        for target in candidates:
            matches=[e for e in active if math.dist((target['x'],target['y']),e['point'])<.4]
            if not matches:available.append(target);continue
            blocked=False
            for entry in matches:
                if entry['status']==4 or entry['attempts']>=2:
                    exhausted+=1;blocked=True;break
                if sim<entry['retry_sim']:
                    cooling+=1;retry=entry['retry_sim'] if retry is None else min(retry,entry['retry_sim']);blocked=True;break
            if not blocked:available.append(target)
        return available,{'cooling_views':cooling,'exhausted_views':exhausted,'retry_sim':retry}
