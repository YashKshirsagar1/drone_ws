"""Plan one lap around a rectangular building by trajectory optimization.

The small version of `coverage_planner`: no coverage model, one box, a few
dozen knots, so it solves in well under a second and a few MB of memory. (The
full planner's soft-OR coverage term couples every knot to every wall sample;
at 160 knots its IPOPT Hessian ran the VM out of memory.)

    state per knot     p, v, a   (horizontal, world frame)
    input per segment  j         (jerk)
    free scalar        T         (lap time; knots are T/N apart)

    minimise    w_stand * sum (distance to building - standoff)^2
              + w_jerk  * integral |j|^2
              + w_time  * T

    subject to  exact triple-integrator dynamics between knots
                closed loop: last knot == first knot
                counter-clockwise: (p - centre) x v > 0 at every knot
                distance to building >= min_clearance at every knot
                |v| <= max_speed, |a| <= max_accel

The camera is not optimised: the flier points it at the nearest point of the
building, which for a box is head-on to whichever wall is closest.
"""

import math
import time
from dataclasses import dataclass

import casadi as ca
import numpy as np

from drone_bringup.trajectory import Trajectory


@dataclass
class LoopConfig:
    box: tuple = (6.0, 14.0, -4.0, 4.0)   # building footprint x_min, x_max, y_min, y_max
    standoff: float = 3.0                  # m, preferred distance to the wall
    min_clearance: float = 2.0             # m, hard limit
    max_speed: float = 1.5                 # m/s
    max_accel: float = 1.0                 # m/s^2
    knots: int = 30
    w_standoff: float = 5.0
    w_jerk: float = 0.1
    w_time: float = 0.2


def box_distance(p, box):
    """Distance from `p` to the box (works on numpy arrays and CasADi symbols).

    Exact outside the box. `max(q, 0)` is smoothed with a tight soft-plus so
    IPOPT gets a twice-differentiable function around the corners.
    """
    x0, x1, y0, y1 = box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
    k = 10.0
    if isinstance(p, (ca.SX, ca.MX)):
        qx = ca.sqrt((p[0] - cx)**2 + 1e-6) - hx
        qy = ca.sqrt((p[1] - cy)**2 + 1e-6) - hy
        return ca.sqrt((ca.log1p(ca.exp(k * qx)) / k)**2 + (ca.log1p(ca.exp(k * qy)) / k)**2)
    q = np.abs(np.asarray(p, dtype=float) - [cx, cy]) - [hx, hy]
    return float(np.linalg.norm(np.maximum(q, 0.0)))


def nearest_point(p, box):
    """Closest point of the box footprint to `p` (numpy)."""
    x0, x1, y0, y1 = box
    return np.array([min(max(p[0], x0), x1), min(max(p[1], y0), y1)])


def plan_loop(cfg=None, verbose=False):
    """Solve for one lap. Returns (Trajectory, info dict)."""
    cfg = cfg or LoopConfig()
    N = cfg.knots
    x0, x1, y0, y1 = cfg.box
    centre = np.array([(x0 + x1) / 2, (y0 + y1) / 2])

    opti = ca.Opti()
    P = opti.variable(2, N + 1)
    V = opti.variable(2, N + 1)
    A = opti.variable(2, N + 1)
    J = opti.variable(2, N)
    T = opti.variable()
    h = T / N

    cost = cfg.w_time * T
    for k in range(N):
        # Constant jerk over the segment: exact triple-integrator step.
        opti.subject_to(P[:, k + 1] == P[:, k] + h * V[:, k] + h**2 / 2 * A[:, k] + h**3 / 6 * J[:, k])
        opti.subject_to(V[:, k + 1] == V[:, k] + h * A[:, k] + h**2 / 2 * J[:, k])
        opti.subject_to(A[:, k + 1] == A[:, k] + h * J[:, k])
        cost += cfg.w_jerk * h * ca.sumsqr(J[:, k])

    opti.subject_to(P[:, N] == P[:, 0])
    opti.subject_to(V[:, N] == V[:, 0])
    opti.subject_to(A[:, N] == A[:, 0])

    for k in range(N + 1):
        d = box_distance(P[:, k], cfg.box)
        opti.subject_to(d >= cfg.min_clearance)
        cost += cfg.w_standoff * (d - cfg.standoff)**2 / N
        r = P[:, k] - centre
        opti.subject_to(r[0] * V[1, k] - r[1] * V[0, k] >= 0.1)
        opti.subject_to(ca.sumsqr(V[:, k]) <= cfg.max_speed**2)
        opti.subject_to(ca.sumsqr(A[:, k]) <= cfg.max_accel**2)
    opti.subject_to(opti.bounded(5.0, T, 600.0))
    opti.minimize(cost)

    # Seed: a circle that clears the corners, starting on the -x side
    # (nearest the drone's spawn at the origin).
    radius = math.hypot((x1 - x0) / 2, (y1 - y0) / 2) + cfg.standoff
    T0 = 2 * math.pi * radius / (0.8 * cfg.max_speed)
    th = math.pi + np.linspace(0, 2 * math.pi, N + 1)
    rate = 2 * math.pi / T0
    s, c = np.sin(th), np.cos(th)
    opti.set_initial(P, np.vstack([centre[0] + radius * c, centre[1] + radius * s]))
    opti.set_initial(V, radius * rate * np.vstack([-s, c]))
    opti.set_initial(A, radius * rate**2 * np.vstack([-c, -s]))
    opti.set_initial(J, 0)
    opti.set_initial(T, T0)

    opti.solver('ipopt', {'print_time': False},
                {'print_level': 5 if verbose else 0, 'max_iter': 500, 'sb': 'yes'})
    start = time.monotonic()
    try:
        sol = opti.solve()
        ok, val = True, sol.value
    except RuntimeError:
        ok, val = False, opti.debug.value
    seconds = time.monotonic() - start

    Tv = float(val(T))
    p, v, a = (np.asarray(val(X)).T for X in (P, V, A))
    j = np.asarray(val(J)).T
    t = np.linspace(0.0, Tv, N + 1)
    zeros = np.zeros(N + 1)
    traj = Trajectory(t, p, v, a, j, zeros, zeros, zeros[:-1])

    dense = traj.sample(0.05)
    info = {
        'success': ok,
        'status': opti.stats()['return_status'],
        'solve_seconds': seconds,
        'lap_time': Tv,
        'min_clearance': min(box_distance(q, cfg.box) for q in dense['p']),
        'max_speed': float(np.linalg.norm(dense['v'], axis=1).max()),
    }
    return traj, info


def main():
    traj, info = plan_loop(verbose=False)
    for k, v in info.items():
        print(f'{k:>14}: {v:.2f}' if isinstance(v, float) else f'{k:>14}: {v}')


if __name__ == '__main__':
    main()
