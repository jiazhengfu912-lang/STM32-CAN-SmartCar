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


def world_target_keys(points, resolution):
    """按固定世界栅格识别具体未知目标，地图扩张不把新目标误当作已经看过。"""
    return {tuple(row) for row in np.floor(np.asarray(points,dtype=float)/resolution+1.e-8).astype(int)}


def navigation_failure(goal, status, paused=False):
    """超时请求及其取消结果只计一次，用户暂停/取消不冒充导航失败。"""
    if paused or goal.get('failure_counted') or (status!=6 and not goal.get('timed_out')):return False
    goal['failure_counted']=True;return True


class EmptyConfirmation:
    def __init__(self):
        """独立保存同类空结果确认，避免新候选出现后继续沿用旧完成计时。"""
        self.clear()

    def clear(self):
        """新有效结果清除计时和地图消息计数。"""
        self.reason=None;self.since=None;self.versions=set()

    def candidates(self, planned=False):
        """几何候选推翻几何空结果，实际可行路径才推翻Nav2不可行结论。"""
        if planned or self.reason is None or not self.reason.startswith('NAV2_'):self.clear()

    def observe(self, reason, sim, version):
        """同类原因至少六仿真秒且三个不同地图消息，冷却等待不终止。"""
        if reason=='VIEWPOINTS_COOLDOWN':self.clear();return False
        if reason!=self.reason:self.clear();self.reason=reason;self.since=sim
        self.versions.add(version)
        return sim-self.since>=6. and len(self.versions)>=3


class ViewpointMemory:
    """成功视点等待相关地图变化；失败最多重试一次且必须经过冷却。"""
    def __init__(self):
        """每个会话独立保存访问证据，不继承旧会话控制状态。"""
        self.entries=[]

    def record(self, target, status, grid, resolution, origin, sim):
        """在动作结束后的地图上记录背景，防止用动作成功冒充区域已观测。"""
        point=(target['x'],target['y']);targets=target.get('target_observation_world');near=None
        keys=world_target_keys(targets,resolution) if targets is not None else None
        for entry in self.entries:
            radius=.2 if status==4 and keys is not None else .35
            if math.dist(point,entry['point'])>=radius:continue
            if status==4 and keys is not None and entry['status']==4 and entry.get('targets') is not None:
                if world_target_keys(entry['targets'],resolution)!=keys:continue
            near=entry;break
        attempts=1 if near is None or near['status']==4 else near['attempts']+1
        if near is not None:self.entries.remove(near)
        self.entries.append({'point':point,'status':status,'attempts':attempts,'retry_sim':sim+30.,'targets':targets,
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
            matches=[];cells=target.get('target_visible_cells');keys=None
            if cells is not None:
                yy,xx=np.divmod(np.asarray(cells),grid.shape[1])
                keys=world_target_keys(np.column_stack((origin[0]+(xx+.5)*resolution,origin[1]+(yy+.5)*resolution)),resolution)
            for entry in active:
                radius=.2 if entry['status']==4 and entry.get('targets') is not None else .4
                if math.dist((target['x'],target['y']),entry['point'])>=radius:continue
                if entry['status']==4 and entry.get('targets') is not None and keys is not None:
                    if not keys<=world_target_keys(entry['targets'],resolution):continue
                matches.append(entry)
            if not matches:available.append(target);continue
            blocked=False
            for entry in matches:
                if entry['status']==4 or entry['attempts']>=2:
                    exhausted+=1;blocked=True;break
                if sim<entry['retry_sim']:
                    cooling+=1;retry=entry['retry_sim'] if retry is None else min(retry,entry['retry_sim']);blocked=True;break
            if not blocked:available.append(target)
        return available,{'cooling_views':cooling,'exhausted_views':exhausted,'retry_sim':retry}
