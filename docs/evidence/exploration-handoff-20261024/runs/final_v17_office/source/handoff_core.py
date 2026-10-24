"""以指定目标的真实观测证据决定是否需要额外等一帧地图。"""
import numpy as np


def assigned_target_known(grid, resolution, origin, world_points):
    """所有指定未知格在当前地图都已变为已知，才可免除额外等一帧地图。"""
    points=np.asarray(world_points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.isfinite(points).all():return False
    ix=np.floor((points[:,0]-origin[0])/resolution).astype(int);iy=np.floor((points[:,1]-origin[1])/resolution).astype(int)
    inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0])
    return bool(inside.all() and (grid[iy,ix]>=0).all())
