"""Time-parameterised horizontal trajectories with camera yaw.

A `Trajectory` is knots of (p, v, a, psi, w) with constant jerk `j` and yaw
acceleration `wd` over each segment - exactly what the optimizer produces, so
evaluating between knots reproduces its solution rather than interpolating it.

`orbit` builds the paper's baseline in the same form: a circle at constant
speed with the camera locked on the centre.
"""

import math

import numpy as np


class Trajectory:

    def __init__(self, t, p, v, a, j, psi, w, wd):
        self.t = np.asarray(t, dtype=float)
        self.p, self.v, self.a = (np.asarray(x, dtype=float) for x in (p, v, a))
        self.j = np.asarray(j, dtype=float)
        self.psi, self.w = np.asarray(psi, dtype=float), np.asarray(w, dtype=float)
        self.wd = np.asarray(wd, dtype=float)
        if not (len(self.t) == len(self.p) == len(self.psi) == len(self.j) + 1):
            raise ValueError('need one more knot than segments')

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0])

    def at(self, time):
        """State at `time`, clamped to the ends: (p, v, psi, w)."""
        time = min(max(time, self.t[0]), self.t[-1])
        k = min(int(np.searchsorted(self.t, time, side='right')) - 1, len(self.j) - 1)
        s = time - self.t[k]
        j, wd = self.j[k], self.wd[k]
        p = self.p[k] + self.v[k] * s + self.a[k] * s**2 / 2 + j * s**3 / 6
        v = self.v[k] + self.a[k] * s + j * s**2 / 2
        psi = self.psi[k] + self.w[k] * s + wd * s**2 / 2
        w = self.w[k] + wd * s
        return p, v, psi, w

    def sample(self, step):
        """Evaluate every `step` seconds; returns arrays keyed t, p, v, psi, w."""
        times = np.arange(self.t[0], self.t[-1] + step / 2, step)
        states = [self.at(t) for t in times]
        return {
            't': times,
            'p': np.array([s[0] for s in states]),
            'v': np.array([s[1] for s in states]),
            'psi': np.array([s[2] for s in states]),
            'w': np.array([s[3] for s in states]),
        }

    def to_dict(self):
        return {k: getattr(self, k).tolist() for k in ('t', 'p', 'v', 'a', 'j', 'psi', 'w', 'wd')}

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: np.asarray(v) for k, v in d.items()})


def orbit(centre, radius, speed, start_angle=0.0, step=0.05):
    """One counter-clockwise lap of a circle, camera on the centre.

    This is the 3DR Solo's orbit mode as the paper sets it up: a centre, a
    radius and a speed, nothing else.
    """
    cx, cy = centre
    rate = speed / radius
    duration = 2 * math.pi / rate
    t = np.linspace(0.0, duration, max(2, math.ceil(duration / step)) + 1)
    th = start_angle + rate * t
    c, s = np.cos(th), np.sin(th)
    p = np.column_stack([cx + radius * c, cy + radius * s])
    v = speed * np.column_stack([-s, c])
    a = speed * rate * np.column_stack([-c, -s])
    jerk = speed * rate**2 * np.column_stack([s, -c])
    # Looking at the centre is looking back along the radius.
    psi = th + math.pi
    return Trajectory(t, p, v, a, jerk[:-1], psi, np.full_like(t, rate), np.zeros(len(t) - 1))
