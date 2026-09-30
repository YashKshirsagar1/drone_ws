"""Take off at point A, cruise to point B, land there.

    ros2 run drone_bringup mission --ros-args \
        -p b_x:=6.0 -p b_y:=-2.0 -p cruise_altitude:=4.0

Points are (x, y) in the world frame, the same frame `/odom` reports, and the
altitude is above the launch point. The drone climbs where it stands, flies to
A if it is not already there, cruises to B, then descends onto it and disarms.

`cmd_vel` is a *body-frame* velocity, so every horizontal command here is the
world-frame error rotated by the current yaw.

The altitude the drone is at when it arms is taken as ground level, which is
what makes `cruise_altitude` mean "above the launch point" and tells the
descent when to ease off. Starting a mission already airborne still lands
safely, just slowly - the final approach spends the whole descent at its
gentlest rate.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_bringup.flight import Touchdown, clamp, world_to_body, yaw_from_quaternion


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        self.declare_parameter('a_x', 0.0)
        self.declare_parameter('a_y', 0.0)
        self.declare_parameter('b_x', 5.0)
        self.declare_parameter('b_y', 4.0)
        self.declare_parameter('cruise_altitude', 3.0)
        self.declare_parameter('cruise_speed', 1.5)
        self.declare_parameter('climb_speed', 1.5)
        self.declare_parameter('descend_speed', 0.7)
        self.declare_parameter('position_tolerance', 0.2)
        self.declare_parameter('altitude_tolerance', 0.15)
        self.declare_parameter('gain', 0.9)

        p = self.get_parameter
        self.point_a = (p('a_x').value, p('a_y').value)
        self.point_b = (p('b_x').value, p('b_y').value)
        self.cruise_altitude = p('cruise_altitude').value
        self.cruise_speed = p('cruise_speed').value
        self.climb_speed = p('climb_speed').value
        self.descend_speed = p('descend_speed').value
        self.position_tolerance = p('position_tolerance').value
        self.altitude_tolerance = p('altitude_tolerance').value
        self.gain = p('gain').value

        self.enable_pub = self.create_publisher(Bool, '/quadcopter/enable', 10)
        self.cmd_pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.pose = None              # (x, y, z, yaw)
        self.launch_z = None
        self.state = 'arming'
        self.touchdown = Touchdown()
        self.done = False
        self._ticks = 0
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'A ({self.point_a[0]:.2f}, {self.point_a[1]:.2f}) -> '
            f'B ({self.point_b[0]:.2f}, {self.point_b[1]:.2f}) '
            f'at {self.cruise_altitude:.2f} m')

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = (pos.x, pos.y, pos.z, yaw_from_quaternion(msg.pose.pose.orientation))

    @property
    def target_z(self):
        return self.launch_z + self.cruise_altitude

    def _climb_rate(self, target_z):
        return clamp(self.gain * (target_z - self.pose[2]), self.climb_speed)

    def _go(self, point, climb):
        """Command a body-frame velocity toward `point`, and return the distance left."""
        x, y, _, yaw = self.pose
        dx, dy = point[0] - x, point[1] - y
        distance = math.hypot(dx, dy)

        # P control on the world-frame error, capped, then rotated into the
        # body frame the controller expects.
        if distance > 1e-6:
            speed = min(self.cruise_speed, self.gain * distance)
            wx, wy = dx / distance * speed, dy / distance * speed
        else:
            wx, wy = 0.0, 0.0

        cmd = Twist()
        cmd.linear.x, cmd.linear.y = world_to_body(wx, wy, yaw)
        cmd.linear.z = climb
        self.cmd_pub.publish(cmd)
        return distance

    def _descend(self):
        """Sink onto B, holding position, until the drone stops going down."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.touchdown.update(self.pose[2], now):
            return True

        # Ease off near the ground so it settles instead of bouncing.
        agl = self.pose[2] - self.launch_z
        rate = self.descend_speed if agl > 0.6 else max(0.25, self.descend_speed * 0.4)
        self._go(self.point_b, -rate)
        return False

    def _finish(self):
        self.cmd_pub.publish(Twist())
        self.enable_pub.publish(Bool(data=False))
        x, y, z, _ = self.pose
        error = math.hypot(self.point_b[0] - x, self.point_b[1] - y)
        self.get_logger().info(
            f'landed at ({x:.2f}, {y:.2f}, {z:.2f}) - {error:.2f} m from B')
        self.state = 'landed'
        self.done = True

    def _tick(self):
        self._ticks += 1

        if self.state == 'arming':
            # The controller ignores cmd_vel until enabled, and the bridge may
            # still be connecting, so repeat for half a second.
            self.enable_pub.publish(Bool(data=True))
            if self._ticks >= 10 and self.pose is not None:
                self.launch_z = self.pose[2]
                self.state = 'climb'
                self.get_logger().info(
                    f'climbing to {self.cruise_altitude:.2f} m above launch')
            elif self._ticks == 100:
                self.get_logger().warn('no /odom yet - is the sim running?')
            return

        if self.pose is None:
            return

        if self.state == 'climb':
            self._go(self.point_a, self._climb_rate(self.target_z))
            if abs(self.target_z - self.pose[2]) <= self.altitude_tolerance:
                self.state = 'to_a'
                self.get_logger().info('at altitude, heading for A')

        elif self.state == 'to_a':
            if self._go(self.point_a, self._climb_rate(self.target_z)) <= self.position_tolerance:
                self.state = 'to_b'
                self.get_logger().info('over A, cruising to B')

        elif self.state == 'to_b':
            if self._go(self.point_b, self._climb_rate(self.target_z)) <= self.position_tolerance:
                self.state = 'descend'
                self.touchdown.reset()
                self.get_logger().info('over B, descending')

        elif self.state == 'descend':
            if self._descend():
                self._finish()


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
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
