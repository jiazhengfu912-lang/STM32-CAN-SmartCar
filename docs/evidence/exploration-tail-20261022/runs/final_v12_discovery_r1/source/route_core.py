"""用Nav2返回路径评价行驶代价和区域访问顺序；不读取仿真真值。"""
import math
import numpy as np


def angle_difference(first, second):
    """把转角差规范到最短旋转角。"""
    return abs(math.atan2(math.sin(first-second),math.cos(first-second)))


def path_cost(points, yaw, costmap=None):
    """计算真实路径长度、起终切线和膨胀代价积分；规划CPU耗时不当行驶时间。"""
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or len(p)<2 or p.shape[1]!=2 or not np.isfinite(p).all():return None
    ds=np.linalg.norm(np.diff(p,axis=0),axis=1);length=float(ds.sum())
    if length<.05:return None
    cumulative=np.r_[0.,np.cumsum(ds)]
    first=min(len(p)-1,max(1,int(np.searchsorted(cumulative,.4))))
    last=max(0,min(len(p)-2,int(np.searchsorted(cumulative,length-.4))))
    initial=math.atan2(*(p[first]-p[0])[::-1]);arrival=math.atan2(*(p[-1]-p[last])[::-1])
    sampled=p[np.unique(np.r_[0,np.searchsorted(cumulative,np.arange(.4,length,.4)),len(p)-1])]
    directions=np.arctan2(np.diff(sampled[:,1]),np.diff(sampled[:,0]))
    turns=sum(angle_difference(a,b) for a,b in zip(directions,directions[1:]))
    weighted=0.;maximum=0
    if costmap is not None:
        g,res,origin=costmap;mid=(p[:-1]+p[1:])/2
        ix=np.floor((mid[:,0]-origin[0])/res).astype(int);iy=np.floor((mid[:,1]-origin[1])/res).astype(int)
        if ((ix<0)|(ix>=g.shape[1])|(iy<0)|(iy>=g.shape[0])).any():return None
        values=g[iy,ix].astype(int)
        if (values>=253).any():return None
        weighted=float(np.dot(ds,values/252.));maximum=int(values.max())
    turn=angle_difference(initial,yaw)
    return {'length_m':length,'initial_yaw':initial,'arrival_yaw':arrival,'initial_turn_rad':turn,
            'path_turn_rad':turns,'inflation_integral_m':weighted,'maximum_cost':maximum,
            'estimated_cost_s':length/.2+turn/.35+.25*turns/.35+2.*weighted+2.}


def stopping_approach(points, goal, safe, resolution, origin, tolerance=.15):
    """实际路径进入原到达容差前后必须保持原观察余量，允许沿窄通道安全抵达。"""
    points=np.asarray(points,dtype=float);tail=points[np.linalg.norm(points-np.asarray(goal),axis=1)<=tolerance+resolution]
    if not len(tail):return None
    ix=np.floor((tail[:,0]-origin[0])/resolution).astype(int);iy=np.floor((tail[:,1]-origin[1])/resolution).astype(int)
    if ((ix<0)|(ix>=safe.shape[1])|(iy<0)|(iy>=safe.shape[0])).any() or not safe[iy,ix].all():return None
    return tail.tolist()


def shortlist(candidates, limit=8):
    """保留高收益及附近独立观察区域，避免候选预算全被同一长前沿占据。"""
    if not candidates:return []
    chosen=[]
    def add(target):
        """只加入空间不同且有新增可见格的候选。"""
        if any(math.dist((target['x'],target['y']),(x['x'],x['y']))<.45 for x in chosen):return
        visible=set(target.get('target_visible_cells',target['visible_cells']))
        if any(visible<=set(x['visible_cells']) for x in chosen):return
        chosen.append(target)
    fine=all(c['stage']=='fine' for c in candidates) if all('stage' in c for c in candidates) else False
    def region(target):
        """尾段按安全通路区域保留代表，不能让几个小前沿占满全部规划预算。"""
        return target.get('observation_region',target['frontier_group'])
    for target in candidates:
        if not chosen or region(target) not in [region(x) for x in chosen]:add(target)
        if len(chosen)>=min(limit if fine else 2,limit):break
    local=[c for c in candidates if c['path_distance_m']<=4.5]
    for target in sorted(local,key=lambda c:-c['visible_gain_m2']/c['estimated_cost_s']):
        if len(chosen)>=limit:break
        add(target)
        if len(chosen)>=min(4,limit):break
    if len(chosen)<limit and local:add(max(local,key=lambda c:c['visible_gain_m2']))
    for target in candidates:
        if len(chosen)>=limit:break
        add(target)
    return chosen


def worthwhile_local(target, coarse):
    """主探索期间只插入面积显著或明确可见边界足够多的局部观察，过滤零星噪声。"""
    return not coarse or target['predicted_gain_m2']>=.8 or target['visible_gain_m2']>=.025


def open_visit_order(costs, edges):
    """有限观察点的开放访问动态规划，不强制回起点；缺失的有向路径禁止当作可走。"""
    valid=[i for i,c in enumerate(costs) if c is not None]
    if not valid:return [],math.inf
    states={(1<<k,i):(costs[i]['estimated_cost_s'],[i]) for k,i in enumerate(valid)}
    for mask in range(1,1<<len(valid)):
        for i in valid:
            current=states.get((mask,i))
            if current is None:continue
            for k,j in enumerate(valid):
                edge=edges.get((i,j))
                if mask&(1<<k) or edge is None:continue
                key=(mask|(1<<k),j);cost=current[0]+edge['estimated_cost_s']
                if key not in states or cost<states[key][0]:states[key]=(cost,current[1]+[j])
    # 路径缺失时只承诺实际可连接的最大子集，剩余区域下批重新评估，不虚构连接。
    best=min(states,key=lambda key:(-key[0].bit_count(),states[key][0]))
    cost,order=states[best];return order,cost


def rank_routes(candidates, costs, edges=None):
    """实际路径收益排序后，比较先补本区的插入代价；重复观测范围不重复奖励。"""
    edges=edges or {};valid=[i for i,c in enumerate(costs) if c is not None]
    if not valid:return None
    coarse=any(candidates[i]['stage']=='coarse' for i in valid)
    if not coarse and edges:
        order,total=open_visit_order(costs,edges);chosen=order[0]
        target=dict(candidates[chosen]);target['nav2_path']=costs[chosen]
        target['yaw']=costs[chosen]['arrival_yaw'];target['route_reason']='nav2_open_visit_order'
        target['nav2_anchor_index']=chosen;target['nav2_chosen_index']=chosen
        target['nav2_visit_order']=order;target['nav2_visit_cost_s']=total
        return target
    def utility(i):
        """大步用预计面积，尾段用明确可见未知边界，均按实际路径耗时摊销。"""
        t=candidates[i];gain=t['predicted_gain_m2'] if coarse else t['visible_gain_m2']
        return (coarse and t['stage']!='coarse',-gain/costs[i]['estimated_cost_s'])
    anchor=min(valid,key=utility);chosen=anchor;best=0.;anchor_view=set(candidates[anchor]['visible_cells'])
    for i in valid:
        edge=edges.get((i,anchor))
        if i==anchor or edge is None:continue
        if not worthwhile_local(candidates[i],coarse):continue
        new=set(candidates[i]['visible_cells'])-anchor_view
        if not new:continue
        added=max(.2,costs[i]['estimated_cost_s']+edge['estimated_cost_s']-costs[anchor]['estimated_cost_s'])
        # 接近当前区域且现在插入显著便宜时，先完成这一小片，再跨区。
        if costs[i]['length_m']<=4. and added<=.6*edge['estimated_cost_s']:
            score=len(new)/added
            if score>best:chosen=i;best=score
    target=dict(candidates[chosen]);target['nav2_path']=costs[chosen]
    target['yaw']=costs[chosen]['arrival_yaw'];target['route_reason']='nav2_local_insertion' if chosen!=anchor else 'nav2_gain_per_cost'
    target['nav2_anchor_index']=anchor;target['nav2_chosen_index']=chosen
    if chosen!=anchor:
        target['added_detour_s']=max(.2,costs[chosen]['estimated_cost_s']+edges[(chosen,anchor)]['estimated_cost_s']-costs[anchor]['estimated_cost_s'])
    return target
