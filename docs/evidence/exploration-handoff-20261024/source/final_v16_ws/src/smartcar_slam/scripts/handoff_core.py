"""前沿搜索预备结果的有效期；不预发导航目标，也不接触速度控制。"""
import math
import numpy as np


def prepared_search_valid(context, robot, sim, goal_index, action_status):
    """仅复用刚成功到达目标的近端搜索，失败、过期或起点偏移都重新搜索。"""
    if context['source_goal'] is None:return True
    return (context['source_goal']==goal_index and action_status==4 and
            0.<=sim-context['sim_s']<=6. and math.dist(robot,context['robot'])<=.15)


def may_prepare(distance, map_version, sent_map_version, cancelling):
    """只在末段且已经收到行驶中新地图时预备一次纯计算，不抢占Nav2规划动作。"""
    return (math.isfinite(distance) and 0.<=distance<=.8 and
            map_version>sent_map_version and not cancelling)


def refresh_prepared(candidates, source_width, source_resolution, source_origin, grid, resolution, origin):
    """把预备未知格投到最新地图，仅保留仍未知的指定目标；新障碍交由实际路径复查。"""
    refreshed=[]
    def remap(cells):
        """通过世界坐标重编码，地图扩张、原点或分辨率变化不复用旧编号。"""
        yy,xx=np.divmod(np.asarray(cells,dtype=int),source_width)
        x=source_origin[0]+(xx+.5)*source_resolution;y=source_origin[1]+(yy+.5)*source_resolution
        ix=np.floor((x-origin[0])/resolution).astype(int);iy=np.floor((y-origin[1])/resolution).astype(int)
        inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0]);ix,iy=ix[inside],iy[inside]
        unknown=grid[iy,ix]<0
        return np.unique(iy[unknown]*grid.shape[1]+ix[unknown]).tolist()
    for target in candidates:
        wanted=remap(target.get('target_visible_cells',target['visible_cells']))
        if not wanted:continue
        copy=dict(target);visible=remap(target['visible_cells'])
        copy.update(visible_cells=visible,target_visible_cells=wanted,
                    visible_gain_m2=len(visible)*resolution**2,target_visible_gain_m2=len(wanted)*resolution**2,
                    cell=[int(math.floor((target['y']-origin[1])/resolution)),int(math.floor((target['x']-origin[0])/resolution))])
        refreshed.append(copy)
    return refreshed


def assigned_target_known(grid, resolution, origin, world_points):
    """所有指定未知格在当前地图都已变为已知，才可免除额外等一帧地图。"""
    points=np.asarray(world_points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.isfinite(points).all():return False
    ix=np.floor((points[:,0]-origin[0])/resolution).astype(int);iy=np.floor((points[:,1]-origin[1])/resolution).astype(int)
    inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0])
    return bool(inside.all() and (grid[iy,ix]>=0).all())
