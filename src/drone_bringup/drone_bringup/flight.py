"""Small helpers shared by the flight nodes.

`MulticopterVelocityControl` takes its linear velocity in the vehicle's *body*
frame and its angular velocity as a yaw rate, while `/odom` reports position in
the world frame. Anything that chases a world-frame position therefore has to
rotate the error into the body frame first - see `world_to_body`.
"""

import math


def clamp(value, limit):
    """Clamp `value` into [-limit, limit]."""
    return max(-limit, min(limit, value))


def yaw_from_quaternion(q):
    """Yaw in radians from a `geometry_msgs/Quaternion`."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def world_to_body(dx, dy, yaw):
    """Rotate a world-frame horizontal vector into the body frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


class Touchdown:
    """Spots the moment a descending drone stops going down.

    Watching the altitude is a more honest way to find the ground than
    comparing it against a hard-coded height: the spawn pose sits above where
    the model comes to rest (this one drops 0.16 m onto its own collision box),
    and anything the drone lands *on* would move the ground anyway.

    So: while a descent is commanded, feed every altitude in here. Once `z` has
    not dropped by `epsilon` for `settle` seconds, the drone is resting on
    something. At the slowest descent rate the nodes command, 0.25 m/s, it
    covers 0.2 m in that window - ten times `epsilon` - so a real descent never
    looks like a touchdown.
    """

    def __init__(self, settle=0.8, epsilon=0.02):
        self.settle = settle
        self.epsilon = epsilon
        self.reset()

    def reset(self):
        self._lowest = None
        self._since = None

    def update(self, z, now):
        """Return True once the altitude has held still long enough."""
        if self._lowest is None or z < self._lowest - self.epsilon:
            self._lowest, self._since = z, now
            return False
        return now - self._since >= self.settle
