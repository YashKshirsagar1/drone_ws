"""Fly the quadcopter from the keyboard.

    ros2 run drone_bringup teleop_key

Needs a real terminal - it puts stdin into cbreak mode to read single
keypresses. Run it in its own terminal, alongside the sim:

    ros2 launch drone_bringup drone_sim.launch.py

Movement keys are held, not latched: releasing them lets the drone drift back
to a hover, because a terminal gives us keypresses but never key *releases*.
Auto-repeat while a key is held keeps the command alive, and `hold_time` is how
long a command outlives the last keypress.
"""

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_bringup.flight import Touchdown, clamp

# Each entry is (forward, left, up, yaw-left) in multiples of the speed scales.
# The arrows carry the four directions you steer with most; w/s/a/d cover the
# rest without moving your hand.
MOVE = {
    '\x1b[A': (0.0, 0.0, 1.0, 0.0),    # up arrow    - climb
    '\x1b[B': (0.0, 0.0, -1.0, 0.0),   # down arrow  - descend
    '\x1b[D': (0.0, 1.0, 0.0, 0.0),    # left arrow  - slide left
    '\x1b[C': (0.0, -1.0, 0.0, 0.0),   # right arrow - slide right
    'w': (1.0, 0.0, 0.0, 0.0),         # forward
    's': (-1.0, 0.0, 0.0, 0.0),        # back
    'a': (0.0, 0.0, 0.0, 1.0),         # yaw left
    'd': (0.0, 0.0, 0.0, -1.0),        # yaw right
}

HELP = """
quadcopter teleop
-----------------
  up / down arrow    climb / descend
  left / right arrow slide left / right
  w / s              forward / back
  a / d              yaw left / right
  space              stop and hover here
  t                  take off to the target altitude
  l                  land straight down, then disarm
  e                  toggle the motors (arm / disarm)
  + / -              speed scale up / down
  ?                  show these keys
  q                  quit (disarms first)
"""


class NoTerminal(RuntimeError):
    """Raised when stdin is not a terminal, so no keypresses can arrive."""


class TeleopKey(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self.declare_parameter('speed', 1.2)          # m/s, horizontal
        self.declare_parameter('climb_speed', 1.0)    # m/s, vertical
        self.declare_parameter('yaw_speed', 0.8)      # rad/s
        self.declare_parameter('altitude', 3.0)       # m above launch, target for `t`
        self.declare_parameter('hold_time', 0.4)      # s, command lifetime
        self.declare_parameter('gain', 0.9)

        self.speed = self.get_parameter('speed').value
        self.climb_speed = self.get_parameter('climb_speed').value
        self.yaw_speed = self.get_parameter('yaw_speed').value
        self.altitude = self.get_parameter('altitude').value
        self.hold_time = self.get_parameter('hold_time').value
        self.gain = self.get_parameter('gain').value

        self.enable_pub = self.create_publisher(Bool, '/quadcopter/enable', 10)
        self.cmd_pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.z = None
        self.launch_z = None
        self.armed = False
        self._enable_ticks = 0
        self._warned_no_odom = False
        self._last_command = {}
        self.mode = 'manual'          # manual | takeoff | landing
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.last_key = 0.0
        self.touchdown = Touchdown()
        self.quit = False

        self._fd = sys.stdin.fileno()
        if not os.isatty(self._fd):
            raise NoTerminal(
                'teleop_key needs a terminal to read keypresses from, and this '
                'stdin is not one.\n'
                'Run it directly in a terminal window - not through a pipe, '
                'not from a launch file,\n'
                'and not via a wrapper that captures output (an editor task, '
                '`ros2 launch`, or\n'
                "Claude Code's `!` prefix all detach stdin).\n"
                'To fly without a terminal, use the mission node instead:\n'
                '    ros2 run drone_bringup mission --ros-args -p b_x:=5.0 -p b_y:=4.0')
        self._saved = termios.tcgetattr(self._fd)
        # cbreak, not raw: it leaves output post-processing and Ctrl-C alone, so
        # log lines still start at column zero and SIGINT still works.
        tty.setcbreak(self._fd, termios.TCSADRAIN)

        self.say(HELP)
        self.create_timer(0.05, self._tick)

    def say(self, text):
        sys.stdout.write(text.rstrip('\n') + '\n')
        sys.stdout.flush()

    def restore_terminal(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def _on_odom(self, msg):
        self.z = msg.pose.pose.position.z
        if self.launch_z is None:
            self.launch_z = self.z

    def _keys(self):
        """Every keypress waiting on stdin, arrow escape sequences included."""
        if not select.select([sys.stdin], [], [], 0)[0]:
            return []
        data = os.read(self._fd, 64).decode(errors='ignore')
        keys, i = [], 0
        while i < len(data):
            if data[i] == '\x1b' and data[i + 1:i + 3].startswith('['):
                keys.append(data[i:i + 3])
                i += 3
            else:
                keys.append(data[i])
                i += 1
        return keys

    def _arm(self, armed):
        self.armed = armed
        self.enable_pub.publish(Bool(data=armed))
        self.say('motors armed' if armed else 'motors disarmed')

    def _handle(self, key):
        if key in MOVE:
            if not self.armed:
                self._arm(True)
            if self.mode != 'manual':
                self.say('manual control')
                self.mode = 'manual'
            self.cmd = MOVE[key]
            self.last_key = self.now()
            return

        now = self.now()
        if now - self._last_command.get(key, 0.0) < self.hold_time:
            return
        self._last_command[key] = now

        if key == ' ':
            self.mode = 'manual'
            self.cmd = (0.0, 0.0, 0.0, 0.0)
            self.last_key = 0.0
            self.say('hover')
        elif key == 't':
            if self.mode == 'takeoff':
                return
            if not self.armed:
                self._arm(True)
            self.mode = 'takeoff'
            self.cmd = (0.0, 0.0, 0.0, 0.0)
            self.say(f'taking off to {self.altitude:.2f} m')
        elif key == 'l':
            if self.mode == 'landing' or not self.armed:
                return          # already on the way down, or already down
            self.mode = 'landing'
            self.cmd = (0.0, 0.0, 0.0, 0.0)
            self.touchdown.reset()
            self.say('landing')
        elif key == 'e':
            self._arm(not self.armed)
        elif key in '+=':
            self.speed, self.climb_speed = self.speed * 1.25, self.climb_speed * 1.25
            self.say(f'speed {self.speed:.2f} m/s, climb {self.climb_speed:.2f} m/s')
        elif key in '-_':
            self.speed, self.climb_speed = self.speed * 0.8, self.climb_speed * 0.8
            self.say(f'speed {self.speed:.2f} m/s, climb {self.climb_speed:.2f} m/s')
        elif key == '?':
            self.say(HELP)
        elif key in ('q', '\x03'):
            self.quit = True

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _auto_climb(self):
        """Track the target altitude, tapering the climb rate near it."""
        if self.z is None:
            return 0.0
        error = (self.launch_z + self.altitude) - self.z
        if abs(error) <= 0.15 and self.mode == 'takeoff':
            self.say(f'holding {self.z - self.launch_z:.2f} m')
            self.mode = 'manual'
        return clamp(self.gain * error, self.climb_speed)

    def _auto_descend(self):
        """Descend until the drone stops moving down, then disarm."""
        if self.z is None:
            return 0.0

        if self.touchdown.update(self.z, self.now()):
            self.say('touchdown')
            self.mode = 'manual'
            self.cmd = (0.0, 0.0, 0.0, 0.0)
            self._arm(False)
            return 0.0

        # Ease off close to the ground so it settles instead of bouncing.
        agl = self.z - self.launch_z
        rate = min(self.climb_speed, 0.7) if agl > 0.6 else max(0.25, self.climb_speed * 0.4)
        return -rate

    def _tick(self):
        for key in self._keys():
            self._handle(key)
        if self.quit:
            return

        self._enable_ticks += 1
        if self._enable_ticks % 10 == 0:
            self.enable_pub.publish(Bool(data=self.armed))

        # Silence here means the sim is not up, or the bridge is not running -
        # worth saying, because every key would otherwise look broken.
        if self.z is None and self._enable_ticks == 60 and not self._warned_no_odom:
            self._warned_no_odom = True
            self.say('no /odom yet - is the sim running? '
                     '(ros2 launch drone_bringup drone_sim.launch.py)')

        if self.mode == 'manual' and self.now() - self.last_key > self.hold_time:
            self.cmd = (0.0, 0.0, 0.0, 0.0)

        fwd, left, up, yaw = self.cmd
        cmd = Twist()
        cmd.linear.x = fwd * self.speed
        cmd.linear.y = left * self.speed
        cmd.linear.z = up * self.climb_speed
        cmd.angular.z = yaw * self.yaw_speed

        if self.mode == 'takeoff':
            cmd.linear.z = self._auto_climb()
        elif self.mode == 'landing':
            cmd.linear.z = self._auto_descend()

        if self.armed:
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopKey()
    except NoTerminal as exc:
        print(f'\n{exc}\n')
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Leave the drone hovering rather than mid-command, and always hand the
        # terminal back the way we found it.
        try:
            node.cmd_pub.publish(Twist())
            node.enable_pub.publish(Bool(data=False))
        except Exception:
            pass
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('teleop stopped, motors disarmed')


if __name__ == '__main__':
    main()
