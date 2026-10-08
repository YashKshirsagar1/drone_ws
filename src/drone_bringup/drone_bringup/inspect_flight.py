"""Fly one optimized lap around the building, camera on the wall, then land.

    ros2 run drone_bringup inspect --ros-args -p altitude:=3.0

Plans with `loop_planner.plan_loop` at start-up (a fraction of a second), then:
climb where it stands, fly to the start of the loop, track the loop, fly home
and land.

Tracking is the planned velocity as feedforward plus a P term on the position
error, in the world frame, rotated into the body frame `cmd_vel` expects. The
camera (the drone's nose) is pointed at the nearest point of the building.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_bringup.flight import Touchdown, clamp, world_to_body, yaw_from_quaternion
from drone_bringup.loop_planner import LoopConfig, nearest_point, plan_loop


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Inspect(Node):

    def __init__(self):
        super().__init__('inspect')
        self.declare_parameter('altitude', 3.0)
        self.declare_parameter('gain', 0.8)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        p = self.get_parameter
        self.altitude = p('altitude').value
        self.gain = p('gain').value
        self.yaw_gain = p('yaw_gain').value
        self.max_speed = p('max_speed').value
        self.max_yaw_rate = p('max_yaw_rate').value

        self.cfg = LoopConfig()
        self.traj, info = plan_loop(self.cfg)
        if not info['success']:
            raise RuntimeError(f'planner failed: {info["status"]}')
        self.get_logger().info(
            f'planned lap: {info["lap_time"]:.1f} s, min clearance '
            f'{info["min_clearance"]:.2f} m, solved in {info["solve_seconds"]:.2f} s')

        self.enable_pub = self.create_publisher(Bool, '/quadcopter/enable', 10)
        self.cmd_pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.pose = None            # (x, y, z, yaw)
        self.launch_z = None
        self.home = None
        self.state = 'arming'
        self.lap_start = None
        self.touchdown = Touchdown()
        self.done = False
        self._ticks = 0
        self.create_timer(0.05, self._tick)

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = (pos.x, pos.y, pos.z, yaw_from_quaternion(msg.pose.pose.orientation))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _face_building(self, point):
        target = nearest_point(point, self.cfg.box)
        return math.atan2(target[1] - point[1], target[0] - point[0])

    def _send(self, wx, wy, climb, yaw_target, yaw_ff=0.0):
        """Publish a world-frame velocity and a yaw target as a body-frame Twist."""
        x, y, z, yaw = self.pose
        speed = math.hypot(wx, wy)
        if speed > self.max_speed:
            wx, wy = wx * self.max_speed / speed, wy * self.max_speed / speed
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = world_to_body(wx, wy, yaw)
        cmd.linear.z = climb
        cmd.angular.z = clamp(yaw_ff + self.yaw_gain * wrap(yaw_target - yaw), self.max_yaw_rate)
        self.cmd_pub.publish(cmd)

    def _hold_altitude(self):
        return clamp(self.gain * (self.launch_z + self.altitude - self.pose[2]), 1.5)

    def _go(self, point, climb, yaw_target):
        """P control toward `point`; returns the distance left."""
        dx, dy = point[0] - self.pose[0], point[1] - self.pose[1]
        self._send(self.gain * dx, self.gain * dy, climb, yaw_target)
        return math.hypot(dx, dy)

    def _tick(self):
        self._ticks += 1
        if self.state == 'arming':
            self.enable_pub.publish(Bool(data=True))
            if self._ticks >= 10 and self.pose is not None:
                self.launch_z = self.pose[2]
                self.home = self.pose[:2]
                self.state = 'climb'
                self.get_logger().info(f'climbing to {self.altitude:.1f} m')
            elif self._ticks == 100:
                self.get_logger().warn('no /odom yet - is the sim running?')
            return

        x, y, z, yaw = self.pose
        start = self.traj.p[0]

        if self.state == 'climb':
            self._go(self.home, self._hold_altitude(), yaw)
            if abs(self.launch_z + self.altitude - z) < 0.15:
                self.state = 'to_start'
                self.get_logger().info('flying to the start of the loop')

        elif self.state == 'to_start':
            d = self._go(start, self._hold_altitude(), self._face_building(start))
            if d < 0.2 and abs(wrap(self._face_building(start) - yaw)) < 0.1:
                self.state = 'lap'
                self.lap_start = self._now()
                self.get_logger().info(f'starting the {self.traj.duration:.1f} s lap')

        elif self.state == 'lap':
            t = self._now() - self.lap_start
            p_ref, v_ref, _, _ = self.traj.at(t)
            ex, ey = p_ref[0] - x, p_ref[1] - y
            # Yaw feedforward: rate of the face-the-building heading along the plan.
            dt = 0.1
            p_next = self.traj.at(t + dt)[0]
            yaw_ref = self._face_building(p_ref)
            yaw_ff = wrap(self._face_building(p_next) - yaw_ref) / dt
            self._send(v_ref[0] + self.gain * ex, v_ref[1] + self.gain * ey,
                       self._hold_altitude(), yaw_ref, yaw_ff)
            if self._ticks % 40 == 0:
                self.get_logger().info(
                    f't={t:5.1f}s  tracking error {math.hypot(ex, ey):.2f} m')
            if t >= self.traj.duration:
                self.state = 'home'
                self.get_logger().info('lap done, flying home')

        elif self.state == 'home':
            if self._go(self.home, self._hold_altitude(), yaw) < 0.2:
                self.state = 'descend'
                self.touchdown.reset()
                self.get_logger().info('over home, landing')

        elif self.state == 'descend':
            if self.touchdown.update(z, self._now()):
                self.cmd_pub.publish(Twist())
                self.enable_pub.publish(Bool(data=False))
                self.get_logger().info('landed')
                self.done = True
                return
            rate = 0.7 if z - self.launch_z > 0.6 else 0.3
            self._go(self.home, -rate, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = Inspect()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
