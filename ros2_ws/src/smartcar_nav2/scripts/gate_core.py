"""Wall-clock freshness and bounded navigation commands, independent of ROS."""
import math


def map_authority_ok(source, publishers, nodes):
    """建图和已知地图分别要求唯一地图发布者，并拒绝两套定位同时运行。"""
    if source=='slam':
        return publishers==['slam_toolbox'] and not {'amcl','map_server'}.intersection(nodes)
    if source=='amcl':
        return publishers==['map_server'] and 'slam_toolbox' not in nodes
    return False


def slam_localization_ok(now, map_time, sim_time, tf_stamp, values):
    """用持续有效的SLAM地图/TF确认定位，避免静止时漏收一次性pose而死等。"""
    if map_time is None or sim_time is None or len(values)!=7:
        return False
    if not all(math.isfinite(v) for v in [now,map_time,sim_time,tf_stamp,*values]):
        return False
    return (0<=now-map_time<=10. and -1.<=sim_time-tf_stamp<=1.5 and
            abs(sum(v*v for v in values[3:])-1.)<=.01)


class NavigationGate:
    def __init__(self):
        self.command = (0., 0.)
        self.command_time = self.scan_time = self.odom_time = self.clock_time = None
        self.clock_value = None
        self.localized = False
        self.estop = False
        self.points = []

    def receive_command(self, vx, wz, now):
        if not all(math.isfinite(v) for v in [vx, wz, now]):
            self.command = (0., 0.)
            self.command_time = None
            return
        self.command = (max(-.15, min(.3, vx)), max(-.8, min(.8, wz)))
        self.command_time = now

    def receive_clock(self, value, now):
        if self.clock_value is not None and value < self.clock_value:
            self.localized = False
        if self.clock_value is None or value != self.clock_value:
            self.clock_time = now
        self.clock_value = value

    def output(self, now):
        if self.estop:
            return 0., 0., 'STOP_LATCHED'
        if not self.localized:
            return 0., 0., 'WAIT_LOCALIZATION'
        for name, stamp, maximum in [('CLOCK', self.clock_time, .5), ('SCAN', self.scan_time, .6),
                                     ('ODOM', self.odom_time, .5), ('COMMAND', self.command_time, .3)]:
            if stamp is None or not 0 <= now-stamp < maximum:
                return 0., 0., name+'_STALE'
        vx, wz = self.command
        for x, y in self.points:
            if abs(wz) > .05 and math.hypot(x, y) < .33:
                return 0., 0., 'OBSTACLE_NEAR_TURN'
            if vx > .01 and -.14 < x < .39 and abs(y) < .21:
                return 0., 0., 'OBSTACLE_AHEAD'
            if vx < -.01 and -.28 < x < .25 and abs(y) < .21:
                return 0., 0., 'OBSTACLE_BEHIND'
        return vx, wz, 'READY'
