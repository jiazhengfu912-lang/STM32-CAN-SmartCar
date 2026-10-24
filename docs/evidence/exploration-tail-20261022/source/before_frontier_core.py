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


def reachable_distances(safe, start):
    """同样四邻接安全广搜，使用原生列表避免逐栅格NumPy标量开销阻塞任务心跳。"""
    h,w=safe.shape;allowed=safe.ravel().tolist();distance=[-1]*(h*w);parent=[-1]*(h*w)
    first=start[0]*w+start[1];distance[first]=0;queue=deque([first])
    while queue:
        cell=queue.popleft();y,x=divmod(cell,w)
        neighbors=[]
        if y:neighbors.append(cell-w)
        if y+1<h:neighbors.append(cell+w)
        if x:neighbors.append(cell-1)
        if x+1<w:neighbors.append(cell+1)
        for other in neighbors:
            if allowed[other] and distance[other]<0:
                distance[other]=distance[cell]+1;parent[other]=cell;queue.append(other)
    return np.asarray(distance,dtype=np.int32).reshape((h,w)),parent


def path_headings(safe, parent, start, goal, resolution):
    """估计起步和到达切线；360度雷达无需每次抵达后再强制转向未知区。"""
    sy,sx=start;gy,gx=goal;h,w=safe.shape
    steps=max(abs(gy-sy),abs(gx-sx))*2+1
    ys=np.rint(np.linspace(sy,gy,steps)).astype(int);xs=np.rint(np.linspace(sx,gx,steps)).astype(int)
    if safe[ys,xs].all():
        angle=math.atan2(gy-sy,gx-sx);return angle,angle
    cells=[];cell=gy*w+gx;first=sy*w+sx
    while cell!=first and cell>=0:
        cells.append(divmod(cell,w));cell=parent[cell]
    cells.append(start);cells.reverse();offset=min(len(cells)-1,max(1,round(.6/resolution)))
    first_y,first_x=cells[offset];last_y,last_x=cells[-offset-1]
    return math.atan2(first_y-sy,first_x-sx),math.atan2(gy-last_y,gx-last_x)


def predicted_gain(grid, resolution, origin, point, radius=3.):
    """仅从当前地图射线估计未知区观测面积，已知墙体遮挡后的未知区不计收益。"""
    angles=np.arange(72)*2*math.pi/72;step=resolution/2
    ranges=np.arange(step,radius+step/2,step)
    xs=np.floor((point[0]+np.cos(angles[:,None])*ranges-origin[0])/resolution).astype(int)
    ys=np.floor((point[1]+np.sin(angles[:,None])*ranges-origin[1])/resolution).astype(int)
    h,w=grid.shape;inside=(xs>=0)&(xs<w)&(ys>=0)&(ys<h)
    values=grid[np.clip(ys,0,h-1),np.clip(xs,0,w-1)]
    blocked=np.maximum.accumulate((~inside)|(values>20),axis=1)
    return float((((values<0)&~blocked)*ranges).sum())*step*2*math.pi/72


def tree_route(parent, width, first, second):
    """沿已知安全广搜树求两点连接，不用穿墙直线低估离开后的返回代价。"""
    a=first[0]*width+first[1];b=second[0]*width+second[1];left=[];indices={}
    while a>=0:
        indices[a]=len(left);left.append(a);a=parent[a]
    right=[]
    while b>=0 and b not in indices:right.append(b);b=parent[b]
    if b<0:return []
    cells=left[:indices[b]+1]+right[::-1]
    return [divmod(cell,width) for cell in cells]


def prioritize_local(candidates, parent, width, resolution):
    """远行前顺路补盲：比较现在绕行和离开后返回的树路径代价，避免房间间往返。"""
    if not candidates:return
    anchor=candidates[0]
    if anchor['path_distance_m']<4. or anchor['estimated_cost_s']<35.:return
    opportunities=[]
    for target in candidates[1:]:
        if (target['frontier_group']==anchor['frontier_group'] or
            not .75<=target['path_distance_m']<=3. or target['predicted_gain_m2']<.8 or
            math.hypot(target['x']-anchor['x'],target['y']-anchor['y'])<2.5):continue
        route=tree_route(parent,width,target['cell'],anchor['cell'])
        if len(route)<2:continue
        separation=(len(route)-1)*resolution;offset=min(len(route)-1,max(1,round(.6/resolution)))
        y,x=route[0];ny,nx=route[offset];direction=math.atan2(ny-y,nx-x)
        turn=abs(math.atan2(math.sin(direction-target['yaw']),math.cos(direction-target['yaw'])))
        onward=separation/.2+turn/.35+2.
        detour=max(.2,target['estimated_cost_s']+onward-anchor['estimated_cost_s'])
        if detour<=.6*onward:opportunities.append((target['predicted_gain_m2']/detour,target,separation,detour))
    if opportunities:
        _,target,separation,detour=max(opportunities,key=lambda item:item[0])
        target['route_reason']='finish_local_before_departure';target['avoided_return_path_m']=separation
        target['added_detour_s']=detour;candidates.remove(target);candidates.insert(0,target)


def select_frontiers(data, resolution, origin, robot, excluded=(), clearance=.40, min_length=.8, robot_yaw=0.):
    """已知自由区内广搜；前沿聚类后选退后观察点，禁止穿越未知或墙体。"""
    grid=np.asarray(data,dtype=np.int16)
    if grid.ndim!=2 or min(grid.shape)<3 or not math.isfinite(resolution) or resolution<=0:
        raise ValueError('无效地图尺寸或分辨率')
    if not all(math.isfinite(v) for v in [*origin,*robot,robot_yaw]):raise ValueError('地图原点或位姿非有限数')
    free=(grid>=0)&(grid<=20);unknown=grid<0
    adjacent=np.zeros_like(free)
    adjacent[1:]|=unknown[:-1];adjacent[:-1]|=unknown[1:]
    adjacent[:,1:]|=unknown[:,:-1];adjacent[:,:-1]|=unknown[:,1:]
    frontier=free&adjacent
    safe_goal=free&~dilate(~free,clearance/resolution)
    # 到达容差和新扫描可使边界移动；优先留出额外观察余量，窄处仍保留原合法目标。
    preferred_goal=free&~dilate(~free,max(clearance,.65)/resolution)
    # 通行圆与现有Nav2的0.35m半径+0.02m padding一致；观察目标再留0.40m余量。
    # 否则已到达的合法位置可能因一格地图更新被误当作完全无路。
    safe=free&~dilate(~free,.37/resolution)
    h,w=grid.shape
    rx=int(math.floor((robot[0]-origin[0])/resolution));ry=int(math.floor((robot[1]-origin[1])/resolution))
    ys,xs=np.nonzero(safe)
    if not len(xs):return [],{'frontier_cells':int(frontier.sum()),'clusters':0,'reachable_cells':0,'reason':'NO_SAFE_KNOWN_SPACE'}
    # 实际机器人不在安全自由栅格时，不把它瞬移到墙的另一侧再计算可达性。
    if not (0<=rx<w and 0<=ry<h and safe[ry,rx]):
        return [],{'frontier_cells':int(frontier.sum()),'clusters':0,'reachable_cells':0,'reason':'ROBOT_NOT_IN_SAFE_SPACE'}
    distance,parent=reachable_distances(safe,(ry,rx))
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
    candidates=[];used=set()
    for group_index,group in enumerate(clusters):
        center=group.mean(axis=0)
        representatives=[int(np.argmin(((group-center)**2).sum(axis=1)))]
        # 长而弯曲的前沿不能只检查质心；在边缘与分布采样处寻找可达观察点。
        representatives += [int(np.argmin(group[:,i])) for i in [0,1]]
        representatives += [int(np.argmax(group[:,i])) for i in [0,1]]
        representatives += list(range(0,len(group),max(1,len(group)//8)))
        for representative in dict.fromkeys(representatives):
            fy,fx=group[representative]
            n=int(math.ceil(1.1/resolution));y0,y1=max(0,fy-n),min(h,fy+n+1);x0,x1=max(0,fx-n),min(w,fx+n+1)
            cy,cx=np.nonzero((distance[y0:y1,x0:x1]>=0)&safe_goal[y0:y1,x0:x1]);cy+=y0;cx+=x0
            if not len(cx):continue
            near=np.hypot(cx-fx,cy-fy)*resolution
            valid=(near<=1.1)&(distance[cy,cx]*resolution>=.45)
            wx=origin[0]+(cx+.5)*resolution;wy=origin[1]+(cy+.5)*resolution
            for ex,ey,radius in excluded:
                valid&=(wx-ex)**2+(wy-ey)**2>=radius*radius
            if not valid.any():continue
            preferred=valid&preferred_goal[cy,cx]
            if preferred.any():valid=preferred
            indices=np.flatnonzero(valid)
            index=indices[np.argmin(distance[cy[indices],cx[indices]]*resolution+2*near[indices])]
            gx=origin[0]+(cx[index]+.5)*resolution;gy=origin[1]+(cy[index]+.5)*resolution
            key=(int(cy[index]),int(cx[index]))
            if key in used:continue
            used.add(key)
            path_length=float(distance[cy[index],cx[index]]*resolution)
            initial_yaw,arrival_yaw=path_headings(safe,parent,(ry,rx),key,resolution)
            turn=abs(math.atan2(math.sin(initial_yaw-robot_yaw),math.cos(initial_yaw-robot_yaw)))
            gain=predicted_gain(grid,resolution,origin,(gx,gy))
            # 收益按预计秒数摊销，短距离频繁转向/切目标的开销也计入；速度边界保持原值。
            seconds=path_length/.2+turn/.35+2.
            candidates.append({'x':float(gx),'y':float(gy),'yaw':arrival_yaw,'frontier_cells':len(group),
                               'cell':[int(cy[index]),int(cx[index])],'frontier_group':group_index,
                               'path_distance_m':path_length,'predicted_gain_m2':gain,'initial_turn_rad':turn,
                               'goal_margin_m':max(clearance,.65) if preferred.any() else clearance,
                               'estimated_cost_s':seconds,'score':-gain/seconds})
    if candidates:
        best_gain=max(c['predicted_gain_m2'] for c in candidates)
        coarse=[c for c in candidates if best_gain>=3. and c['path_distance_m']>=1.2 and
                c['predicted_gain_m2']>=max(2.,.6*best_gain)]
        # 先做有充分收益的大步探索；没有这种目标时完整保留短距离补盲，不伪报完成。
        for c in candidates:c['stage']='coarse' if c in coarse else 'fine'
        candidates.sort(key=lambda c:(bool(coarse) and c['stage']!='coarse',c['score'],c['estimated_cost_s']))
        prioritize_local(candidates,parent,w,resolution)
    return candidates,{'frontier_cells':int(frontier.sum()),'clusters':len(clusters),
                       'reachable_cells':int((distance>=0).sum()),'candidates':len(candidates),
                       'reason':'CANDIDATES' if candidates else ('NO_FRONTIERS' if not clusters else 'NO_REACHABLE_FRONTIERS')}
