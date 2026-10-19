"""Wall-clock freshness and bounded navigation commands, independent of ROS."""
import math


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
