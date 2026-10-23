"""顺序异步查询真实Nav2路径；不阻塞心跳，也不与正在执行的导航抢规划动作。"""
import json
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from frontier_core import dilate, visible_unknown_cells
from route_core import path_cost, shortlist, rank_routes, worthwhile_local


class RoutePlanner:
    def __init__(self, node):
        """只创建规划查询接口，速度、取消导航与停车仍由原探索状态机负责。"""
        self.node=node;self.client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        self.costmap_client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
        self.busy=False;self.done=False;self.abandoned=False;self.retired=False;self.handle=None;self.batch=0
        self.target=None;self.reason=None;self.deadline=0.;self.future=None

    def begin(self, candidates, robot, yaw, grid, resolution, origin, version):
        """每批保存地图上下文；候选预算有限，所有动作查询串行执行。"""
        if self.busy or self.retired or not self.client.server_is_ready() or not self.costmap_client.service_is_ready():return False
        self.batch+=1;self.busy=True;self.done=False;self.abandoned=False;self.target=None;self.reason=None
        self.robot=robot;self.yaw=yaw;self.grid=grid;self.resolution=resolution;self.origin=origin;self.version=version
        self.candidates=shortlist(candidates,8 if any(c['stage']=='coarse' for c in candidates) else 6)
        self.costs=[None]*len(self.candidates);self.edges={};self.queue=[];self.pairs=False
        self.deadline=time.monotonic()+10.;self.handle=None
        self.future=self.costmap_client.call_async(GetCostmap.Request());self.future.add_done_callback(self.costmap_ready)
        return True

    def cancel(self):
        """暂停或终止时丢弃整批结果，等待已有查询结束再允许新批次。"""
        self.abandoned=True
        if self.handle is not None:self.handle.cancel_goal_async()

    def tick(self, now):
        """查询超时只取消规划并保持停车，绝不在未知查询状态下发送运动目标。"""
        if self.busy and now>self.deadline and not self.abandoned:
            self.reason='NAV2_PLANNING_TIMEOUT';self.node.event('planning_timeout',batch=self.batch);self.cancel()
        if self.busy and now>self.deadline+10.:
            self.retired=True;self.finish('NAV2_PLANNING_CANCEL_TIMEOUT')

    def finish(self, reason=None):
        """发布一批有限查询的结果，错误和被取消情况都保留具体原因。"""
        self.busy=False;self.done=True;self.handle=None
        self.reason=reason or self.reason or ('CANDIDATES' if self.target else 'NAV2_UNREACHABLE_FRONTIERS')
        self.node.event('planning_batch_finished',batch=self.batch,reason=self.reason,
                        candidates=len(self.candidates),valid_paths=sum(x is not None for x in self.costs),
                        pair_paths=len(self.edges),selected=self.target)

    def costmap_ready(self, future):
        """从实际全局代价图获取膨胀代价，保存原值和时间戳，不用SLAM占据值冒充。"""
        try:
            if self.retired:return
            if self.abandoned:self.finish('PLANNING_CANCELLED');return
            msg=future.result().map;meta=msg.metadata
            if msg.header.frame_id!='map' or not meta.resolution or len(msg.data)!=meta.size_x*meta.size_y:
                raise ValueError('全局代价地图无效')
            q=meta.origin.orientation
            if abs(q.x)+abs(q.y)+abs(q.z)>1.e-6 or abs(abs(q.w)-1)>1.e-6:raise ValueError('代价图不是轴对齐地图')
            self.costmap=(np.asarray(msg.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x),float(meta.resolution),
                          (meta.origin.position.x,meta.origin.position.y))
            folder=self.node.folder/'planning_maps';folder.mkdir(exist_ok=True)
            np.savez_compressed(folder/f'batch_{self.batch:04d}.npz',costmap=self.costmap[0],
                costmap_resolution=self.costmap[1],costmap_origin=self.costmap[2],occupancy=self.grid,
                resolution=self.resolution,origin=self.origin,sim_s=self.node.sim(),map_version=self.version,
                costmap_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1.e9)
            self.queue=[(None,i) for i in range(len(self.candidates))];self.advance()
        except Exception as exc:
            self.node.event('planning_error',batch=self.batch,error=str(exc));self.finish('NAV2_COSTMAP_UNAVAILABLE')

    def pose(self, x, y, yaw):
        """候选间规划显式给出安全起点，所有坐标使用同一个map坐标系。"""
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.position.x=float(x);msg.pose.position.y=float(y)
        msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
        return msg

    def advance(self):
        """先求机器人到候选的路径，再求少量附近区域到最佳区域的插入路径。"""
        if self.abandoned:self.finish('PLANNING_CANCELLED');return
        if not self.queue and not self.pairs:
            self.pairs=True;anchor=rank_routes(self.candidates,self.costs)
            if anchor is not None:
                index=anchor['nav2_anchor_index'];view=set(self.candidates[index]['visible_cells'])
                valid=[i for i,c in enumerate(self.costs) if c is not None]
                coarse=any(self.candidates[i]['stage']=='coarse' for i in valid)
                local=[i for i,c in enumerate(self.costs) if c is not None and i!=index and c['length_m']<=4.
                       and set(self.candidates[i]['visible_cells'])-view and
                       worthwhile_local(self.candidates[i],coarse)]
                self.queue=([(i,index) for i in sorted(local,key=lambda i:self.costs[i]['estimated_cost_s'])[:3]] if coarse else
                            [(i,j) for i in valid for j in valid if i!=j])
        if not self.queue:
            self.target=rank_routes(self.candidates,self.costs,self.edges);self.finish();return
        self.edge=self.queue.pop(0);first,second=self.edge
        start=(*self.robot,self.yaw) if first is None else (self.candidates[first]['x'],self.candidates[first]['y'],self.costs[first]['arrival_yaw'])
        target=self.candidates[second];self.start_yaw=start[2]
        request=ComputePathToPose.Goal();request.start=self.pose(*start)
        request.goal=self.pose(target['x'],target['y'],target['yaw']);request.use_start=True;request.planner_id='GridBased'
        self.deadline=time.monotonic()+10.;self.handle=None
        self.future=self.client.send_goal_async(request);self.future.add_done_callback(self.accepted)

    def accepted(self, future):
        """动作拒绝也记录，不把空路径当作零成本路径。"""
        try:
            handle=future.result()
            if self.retired:
                if handle is not None and handle.accepted:handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                self.record(None,[],0.,'REJECTED');self.advance();return
            self.handle=handle
            if self.abandoned:handle.cancel_goal_async()
            self.future=handle.get_result_async();self.future.add_done_callback(self.received)
        except Exception as exc:
            self.node.event('planning_error',batch=self.batch,error=str(exc));self.finish('NAV2_PLANNING_ERROR')

    def received(self, future):
        """验证返回终点仍有观察价值，并保留完整路径、结果和实际规划CPU时间。"""
        try:
            if self.retired:return
            response=future.result();result=response.result;points=[(p.pose.position.x,p.pose.position.y) for p in result.path.poses]
            cost=path_cost(points,self.start_yaw,self.costmap) if response.status==4 else None
            if cost is not None:
                end=points[-1];ix=int((end[0]-self.origin[0])/self.resolution);iy=int((end[1]-self.origin[1])/self.resolution)
                safe=(self.grid>=0)&(self.grid<=20)&~dilate(~((self.grid>=0)&(self.grid<=20)),.4/self.resolution)
                visible=visible_unknown_cells(self.grid,self.resolution,self.origin,end)
                target=self.candidates[self.edge[1]]
                if not (0<=ix<self.grid.shape[1] and 0<=iy<self.grid.shape[0] and safe[iy,ix]
                        and set(visible)&set(target['visible_cells']) and math.dist(end,(target['x'],target['y']))<=.15):cost=None
            elapsed=result.planning_time.sec+result.planning_time.nanosec/1.e9
            self.record(cost,points,elapsed,response.status);self.handle=None;self.advance()
        except Exception as exc:
            self.node.event('planning_error',batch=self.batch,error=str(exc));self.finish('NAV2_PLANNING_ERROR')

    def record(self, cost, points, elapsed, status):
        """逐条写出实际Nav2查询，候选和候选间路径都可复核，不用广搜估算代替。"""
        first,second=self.edge
        if first is None:self.costs[second]=cost
        elif cost is not None:self.edges[(first,second)]=cost
        row={'wall_s':time.monotonic()-self.node.wall_start,'sim_s':self.node.sim(),'batch':self.batch,
             'map_version':self.version,'from_index':first,'to_index':second,'candidates':self.candidates,
             'action_status':status,'planning_cpu_s':elapsed,'path':points,'cost':cost}
        with (self.node.folder/'nav2_paths.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
        self.node.event('nav2_path_evaluated',batch=self.batch,from_index=first,to_index=second,status=status,cost=cost)
