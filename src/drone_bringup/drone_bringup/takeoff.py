"""Arm the multicopter controller and climb to a target altitude, then hold.

    ros2 run drone_bringup takeoff --ros-args -p altitude:=5.0
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class Takeoff(Node):

    def __init__(self):
        super().__init__('takeoff')
        self.declare_parameter('altitude', 3.0)
        self.declare_parameter('climb_speed', 1.5)
        self.declare_parameter('tolerance', 0.15)
        self.declare_parameter('gain', 0.9)

        self.altitude = self.get_parameter('altitude').value
        self.climb_speed = self.get_parameter('climb_speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.gain = self.get_parameter('gain').value

        self.enable_pub = self.create_publisher(Bool, '/quadcopter/enable', 10)
        self.cmd_pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.z = None
        self.state = 'arming'
        self._ticks = 0
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'climbing to {self.altitude:.2f} m')

    def _on_odom(self, msg):
        self.z = msg.pose.pose.position.z

    def _track_altitude(self):
        """Command a climb rate proportional to the remaining error.

        A constant climb rate cut to zero on arrival overshoots by a metre or
        so - the controller zeroes the *velocity*, but only after the drone has
        already carried past the target. Tapering the command near the target
        both lands it on the setpoint and holds it there.
        """
        error = self.altitude - self.z
        speed = max(-self.climb_speed, min(self.climb_speed, self.gain * error))
        cmd = Twist()
        cmd.linear.z = speed
        self.cmd_pub.publish(cmd)

    def _tick(self):
        self._ticks += 1

        if self.state == 'arming':
            # The controller only accepts velocity commands once enabled, and
            # the bridge may still be connecting, so repeat for a second.
            self.enable_pub.publish(Bool(data=True))
            if self._ticks >= 10 and self.z is not None:
                self.state = 'climbing'
            elif self._ticks == 50:
                self.get_logger().warn('no /odom yet - is the sim running?')
            return

        if self.state == 'climbing' and abs(self.altitude - self.z) <= self.tolerance:
            self.state = 'holding'
            self.get_logger().info(f'reached {self.z:.2f} m, holding')

        # Keep tracking in both states, so the hold corrects any drift.
        self._track_altitude()


def main(args=None):
    rclpy.init(args=args)
    node = Takeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
