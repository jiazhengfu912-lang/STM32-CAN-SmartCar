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


def predicted_gain(grid, resolution, origin, point, radius=3., rays=72):
    """仅从当前地图射线估计未知区观测面积，已知墙体遮挡后的未知区不计收益。"""
    angles=np.arange(rays)*2*math.pi/rays;step=resolution/2
    ranges=np.arange(step,radius+step/2,step)
    xs=np.floor((point[0]+np.cos(angles[:,None])*ranges-origin[0])/resolution).astype(int)
    ys=np.floor((point[1]+np.sin(angles[:,None])*ranges-origin[1])/resolution).astype(int)
    h,w=grid.shape;inside=(xs>=0)&(xs<w)&(ys>=0)&(ys<h)
    values=grid[np.clip(ys,0,h-1),np.clip(xs,0,w-1)]
    blocked=np.maximum.accumulate((~inside)|(values>20),axis=1)
    return float((((values<0)&~blocked)*ranges).sum())*step*2*math.pi/rays


def visible_unknown_cells(grid, resolution, origin, point, radius=3., boundary=None):
    """直接朝未知边界格投射细步长射线，避免固定角分辨率漏掉窄缝；只计首次未知。"""
    if boundary is None:
        free=(grid>=0)&(grid<=20);adjacent=np.zeros_like(free)
        adjacent[1:]|=free[:-1];adjacent[:-1]|=free[1:]
        adjacent[:,1:]|=free[:,:-1];adjacent[:,:-1]|=free[:,1:]
        boundary=np.nonzero((grid<0)&adjacent)
    ys,xs=boundary;dx=origin[0]+(xs+.5)*resolution-point[0];dy=origin[1]+(ys+.5)*resolution-point[1]
    distances=np.hypot(dx,dy);valid=(distances>0)&(distances<=radius)
    ys,xs,dx,dy,distances=[a[valid] for a in (ys,xs,dx,dy,distances)]
    if not len(xs):return []
    fractions=np.minimum(np.arange(0,radius+resolution/2,resolution/2)[None,:]/distances[:,None],1.)
    xx=np.floor((point[0]+dx[:,None]*fractions-origin[0])/resolution).astype(int)
    yy=np.floor((point[1]+dy[:,None]*fractions-origin[1])/resolution).astype(int)
    values=grid[yy,xx];other=(xx!=xs[:,None])|(yy!=ys[:,None])
    blocked=((values>20)|((values<0)&other)).any(axis=1)
    return (ys[~blocked]*grid.shape[1]+xs[~blocked]).astype(int).tolist()


def target_still_visible(grid, resolution, origin, point, world_targets):
    """发目标前按世界坐标复查指定未知边界，不能用旁边另一片未知替代任务目标。"""
    points=np.asarray(world_targets,dtype=float)
    if not len(points):return False
    ix=np.floor((points[:,0]-origin[0])/resolution).astype(int);iy=np.floor((points[:,1]-origin[1])/resolution).astype(int)
    inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0]);ix=ix[inside];iy=iy[inside]
    unknown=grid[iy,ix]<0;wanted=set((iy[unknown]*grid.shape[1]+ix[unknown]).tolist())
    return bool(wanted&set(visible_unknown_cells(grid,resolution,origin,point)))


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


def assign_observation_regions(candidates, safe, resolution, radius=3.):
    """沿已知安全通路聚合附近视点，阻隔墙两侧不按欧氏距离误并成一个区域。"""
    remaining=set(range(len(candidates)));regions=0
    for index,target in enumerate(candidates):
        if index not in remaining:continue
        distances,_=reachable_distances(safe,tuple(target['cell']))
        members=[i for i in remaining if distances[tuple(candidates[i]['cell'])]>=0 and
                 distances[tuple(candidates[i]['cell'])]*resolution<=radius]
        for i in members:candidates[i]['observation_region']=regions
        remaining.difference_update(members);regions+=1
    return regions


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
    # 保留原0.40米合法观察点；实际Nav2路径上可能提前到达的一段另行检查，避免整体收窄可观测位置。
    safe_goal=free&~dilate(~free,clearance/resolution)
    robust_goal=free&~dilate(~free,(clearance+.15+.02)/resolution)
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
        # 小前沿也进入补盲；长度仅区分阶段，不能据此把可观测窄缝当作已完成。
        clusters.append(np.asarray(group))
    adjacent_free=np.zeros_like(free)
    adjacent_free[1:]|=free[:-1];adjacent_free[:-1]|=free[1:]
    adjacent_free[:,1:]|=free[:,:-1];adjacent_free[:,:-1]|=free[:,1:]
    boundary=np.nonzero(unknown&adjacent_free)
    candidates=[];used=set();geometric=0;occluded=0;excluded_count=0
    for group_index,group in enumerate(clusters):
        wanted=set()
        for fy,fx in group:
            for dy,dx in [(1,0),(-1,0),(0,1),(0,-1)]:
                ny,nx=fy+dy,fx+dx
                if 0<=ny<h and 0<=nx<w and unknown[ny,nx]:wanted.add(int(ny*w+nx))
        target_ids=np.asarray(sorted(wanted),dtype=int);target_boundary=np.divmod(target_ids,w)
        center=group.mean(axis=0)
        representatives=[int(np.argmin(((group-center)**2).sum(axis=1)))]
        # 长而弯曲的前沿不能只检查质心；在边缘与分布采样处寻找可达观察点。
        representatives += [int(np.argmin(group[:,i])) for i in [0,1]]
        representatives += [int(np.argmax(group[:,i])) for i in [0,1]]
        representatives += list(range(0,len(group),max(1,len(group)//8)))
        for representative in dict.fromkeys(representatives):
            fy,fx=group[representative]
            n=int(math.ceil(2.5/resolution));y0,y1=max(0,fy-n),min(h,fy+n+1);x0,x1=max(0,fx-n),min(w,fx+n+1)
            cy,cx=np.nonzero((distance[y0:y1,x0:x1]>=0)&safe_goal[y0:y1,x0:x1]);cy+=y0;cx+=x0
            if not len(cx):continue
            near=np.hypot(cx-fx,cy-fy)*resolution
            valid=(near<=2.5)&(distance[cy,cx]*resolution>=.45)
            wx=origin[0]+(cx+.5)*resolution;wy=origin[1]+(cy+.5)*resolution
            before=int(valid.sum())
            for ex,ey,radius in excluded:
                valid&=(wx-ex)**2+(wy-ey)**2>=radius*radius
            excluded_count+=before-int(valid.sum())
            if not valid.any():continue
            indices=np.flatnonzero(valid)
            order=indices[np.argsort(distance[cy[indices],cx[indices]]*resolution+2*near[indices])]
            tiers=[preferred_goal[cy,cx],robust_goal[cy,cx]&~preferred_goal[cy,cx],~robust_goal[cy,cx]]
            for tier in tiers:
                sampled=[];accepted=0
                for index in order[tier[order]]:
                    key=(int(cy[index]),int(cx[index]))
                    if key in used or any(math.hypot(key[0]-a,key[1]-b)*resolution<.25 for a,b in sampled):continue
                    sampled.append(key);geometric+=1;gx=float(wx[index]);gy=float(wy[index])
                    target_visible=visible_unknown_cells(grid,resolution,origin,(gx,gy),boundary=target_boundary)
                    if not target_visible:
                        occluded+=1
                        if len(sampled)>=6:break
                        continue
                    visible=visible_unknown_cells(grid,resolution,origin,(gx,gy),boundary=boundary)
                    used.add(key);accepted+=1;path_length=float(distance[cy[index],cx[index]]*resolution)
                    initial_yaw,arrival_yaw=path_headings(safe,parent,(ry,rx),key,resolution)
                    turn=abs(math.atan2(math.sin(initial_yaw-robot_yaw),math.cos(initial_yaw-robot_yaw)))
                    gain=predicted_gain(grid,resolution,origin,(gx,gy),rays=360);seconds=path_length/.2+turn/.35+2.
                    candidates.append({'x':gx,'y':gy,'yaw':arrival_yaw,'frontier_cells':len(group),
                        'cell':list(key),'frontier_group':group_index,'small_frontier':len(group)*resolution<min_length,
                        'path_distance_m':path_length,'predicted_gain_m2':gain,'visible_gain_m2':len(visible)*resolution**2,
                        'target_visible_gain_m2':len(target_visible)*resolution**2,
                        'information_gain_m2':max(gain,len(target_visible)*resolution**2),
                        'visible_cells':visible,'target_visible_cells':target_visible,'initial_turn_rad':turn,
                        'goal_margin_m':max(clearance,.65) if preferred_goal[key] else (clearance+.17 if robust_goal[key] else clearance),
                        'estimated_cost_s':seconds,'score':-gain/seconds})
                    if accepted>=2 or len(sampled)>=6:break
    regions=0
    if candidates:
        best_gain=max(c['predicted_gain_m2'] for c in candidates)
        coarse=[c for c in candidates if best_gain>=3. and c['path_distance_m']>=1.2 and
                c['predicted_gain_m2']>=max(2.,.6*best_gain)]
        # 先做有充分收益的大步探索；没有这种目标时完整保留短距离补盲，不伪报完成。
        for c in candidates:
            c['stage']='coarse' if c in coarse else 'fine'
            if not coarse:c['score']=-c['information_gain_m2']/c['estimated_cost_s']
        candidates.sort(key=lambda c:(bool(coarse) and c['stage']!='coarse',c['score'],c['estimated_cost_s']))
        if not coarse:regions=assign_observation_regions(candidates,safe,resolution)
        # 最终顺序由真实Nav2路径和有限区域前瞻决定，不再用广搜树作候选间代价。
    return candidates,{'frontier_cells':int(frontier.sum()),'clusters':len(clusters),
                       'small_clusters':sum(len(g)*resolution<min_length for g in clusters),
                       'observation_regions':regions,
                       'geometric_views':geometric,'occluded_views':occluded,'excluded_cells':excluded_count,
                       'reachable_cells':int((distance>=0).sum()),'candidates':len(candidates),
                       'reason':'CANDIDATES' if candidates else ('NO_FRONTIERS' if not clusters else
                           ('NO_OBSERVABLE_FRONTIERS' if geometric and occluded==geometric else 'NO_REACHABLE_FRONTIERS'))}
