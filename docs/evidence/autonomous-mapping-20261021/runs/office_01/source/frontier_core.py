"""仅从在线栅格和当前位姿选择安全、可达的未知区观察点，不读取场景真值。"""
from collections import deque
import math
import numpy as np


def dilate(mask, radius):
    """按栅格圆盘扩张障碍或未知区，边界外也视为不可通行。"""
    n=int(math.ceil(radius));out=np.zeros_like(mask,dtype=bool)
    padded=np.pad(mask,n,constant_values=True)
    h,w=mask.shape
    for dy in range(-n,n+1):
        for dx in range(-n,n+1):
            if dx*dx+dy*dy<=radius*radius:
                out|=padded[n+dy:n+dy+h,n+dx:n+dx+w]
    return out


def select_frontiers(data, resolution, origin, robot, excluded=(), clearance=.40, min_length=.5):
    """已知自由区内广搜；前沿聚类后选退后观察点，禁止穿越未知或墙体。"""
    grid=np.asarray(data,dtype=np.int16)
    if grid.ndim!=2 or min(grid.shape)<3 or not math.isfinite(resolution) or resolution<=0:
        raise ValueError('无效地图尺寸或分辨率')
    if not all(math.isfinite(v) for v in [*origin,*robot]):raise ValueError('地图原点或位姿非有限数')
    free=(grid>=0)&(grid<=20);unknown=grid<0
    adjacent=np.zeros_like(free)
    adjacent[1:]|=unknown[:-1];adjacent[:-1]|=unknown[1:]
    adjacent[:,1:]|=unknown[:,:-1];adjacent[:,:-1]|=unknown[:,1:]
    frontier=free&adjacent
    safe=free&~dilate(~free,clearance/resolution)
    h,w=grid.shape
    rx=int(math.floor((robot[0]-origin[0])/resolution));ry=int(math.floor((robot[1]-origin[1])/resolution))
    ys,xs=np.nonzero(safe)
    if not len(xs):return [],{'frontier_cells':int(frontier.sum()),'clusters':0,'reachable_cells':0,'reason':'NO_SAFE_KNOWN_SPACE'}
    # 实际机器人不在安全自由栅格时，不把它瞬移到墙的另一侧再计算可达性。
    if not (0<=rx<w and 0<=ry<h and safe[ry,rx]):
        return [],{'frontier_cells':int(frontier.sum()),'clusters':0,'reachable_cells':0,'reason':'ROBOT_NOT_IN_SAFE_SPACE'}
    distance=np.full((h,w),-1,dtype=np.int32);distance[ry,rx]=0;queue=deque([(ry,rx)])
    while queue:
        y,x=queue.popleft()
        for ny,nx in [(y-1,x),(y+1,x),(y,x-1),(y,x+1)]:
            if 0<=ny<h and 0<=nx<w and safe[ny,nx] and distance[ny,nx]<0:
                distance[ny,nx]=distance[y,x]+1;queue.append((ny,nx))
    seen=np.zeros_like(free);clusters=[]
    for sy,sx in zip(*np.nonzero(frontier)):
        if seen[sy,sx]:continue
        group=[];queue=deque([(sy,sx)]);seen[sy,sx]=True
        while queue:
            y,x=queue.popleft();group.append((y,x))
            for dy in [-1,0,1]:
                for dx in [-1,0,1]:
                    ny,nx=y+dy,x+dx
                    if 0<=ny<h and 0<=nx<w and frontier[ny,nx] and not seen[ny,nx]:
                        seen[ny,nx]=True;queue.append((ny,nx))
        if len(group)*resolution>=min_length:clusters.append(np.asarray(group))
    candidates=[]
    for group in clusters:
        center=group.mean(axis=0);fy,fx=group[np.argmin(((group-center)**2).sum(axis=1))]
        n=int(math.ceil(1.1/resolution));y0,y1=max(0,fy-n),min(h,fy+n+1);x0,x1=max(0,fx-n),min(w,fx+n+1)
        cy,cx=np.nonzero(distance[y0:y1,x0:x1]>=0);cy+=y0;cx+=x0
        if not len(cx):continue
        near=np.hypot(cx-fx,cy-fy)*resolution
        valid=(near<=1.1)&(distance[cy,cx]*resolution>=.45)
        for i in np.flatnonzero(valid):
            wx=origin[0]+(cx[i]+.5)*resolution;wy=origin[1]+(cy[i]+.5)*resolution
            if any(math.hypot(wx-ex,wy-ey)<radius for ex,ey,radius in excluded):valid[i]=False
        if not valid.any():continue
        indices=np.flatnonzero(valid)
        # 越靠近前沿越有利于新观测，路径距离惩罚防止无意义远行。
        index=indices[np.argmin(distance[cy[indices],cx[indices]]*resolution+2*near[indices])]
        gx=origin[0]+(cx[index]+.5)*resolution;gy=origin[1]+(cy[index]+.5)*resolution
        front_x=origin[0]+(fx+.5)*resolution;front_y=origin[1]+(fy+.5)*resolution
        path_length=float(distance[cy[index],cx[index]]*resolution)
        candidates.append({'x':float(gx),'y':float(gy),'yaw':math.atan2(front_y-gy,front_x-gx),
                           'frontier_cells':len(group),'path_distance_m':path_length,
                           'score':path_length-.6*math.sqrt(len(group)*resolution)})
    candidates.sort(key=lambda c:c['score'])
    return candidates,{'frontier_cells':int(frontier.sum()),'clusters':len(clusters),
                       'reachable_cells':int((distance>=0).sum()),'candidates':len(candidates),
                       'reason':'CANDIDATES' if candidates else ('NO_FRONTIERS' if not clusters else 'NO_REACHABLE_FRONTIERS')}
