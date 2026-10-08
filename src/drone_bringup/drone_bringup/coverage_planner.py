"""Plan an inspection loop around a building by trajectory optimization.

Gottumukkala & Murphy fly a spline through photo positions an operator picked
by hand, and turn the camera perpendicular to the direction of travel. Neither
the path nor the camera is chosen with the building in mind, so tight
waypoints give sharp turns and the camera often points past the wall.

Here nothing is picked by hand. The footprint and a few limits go in; one
nonlinear program (CasADi + IPOPT) chooses the path, the speed along it, the
camera heading and the lap time together:

    state per knot    p, v, a  (horizontal, world frame)   psi, w  (camera yaw)
    input per segment j  (jerk)                             wd      (yaw accel)
    free scalar       T  (lap time; knots are T/N apart)

    minimise    w_cov    * wall length the camera never sees well
              + w_stand  * integral of (distance to wall - standoff)^2
              + w_face   * integral of (1 - cos(camera, wall normal))
              + w_jerk   * integral of |j|^2
              + w_yaw    * integral of wd^2
              + w_time   * T

    subject to  exact triple-integrator dynamics between knots
                a closed loop: the last knot equals the first (yaw + 2 pi)
                winding once counter-clockwise around the building
                distance to every wall >= min_clearance at every knot
                |v| <= max_speed, |a| <= max_accel, |j| <= max_jerk
                |w| <= max_yaw_rate, |wd| <= max_yaw_accel

"Seeing a wall well" is the paper's own definition, made smooth: a wall piece
counts as seen from a knot when it is in range, inside the camera's field of
view, and viewed within `max_view_angle` of head-on. A wall piece is covered
if any knot sees it - computed as a soft OR, 1 - prod(1 - q), so the gradient
reaches every knot that could see it.

Occlusion is not modelled in the optimizer (it is not smooth); the range and
view-angle limits keep the optimizer from claiming walls it could only see
through the building, and `inspection_metrics` scores flights with true line of
sight.

The flight altitude is fixed, as in the paper: the camera looks horizontally
at mid-wall, so this is a 2-D problem plus yaw.
"""

import math
import time
from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from drone_bringup.building import resample_loop
from drone_bringup.trajectory import Trajectory


@dataclass
class PlannerConfig:
    standoff: float = 3.0            # m, preferred distance to the wall
    min_clearance: float = 2.0       # m, hard limit
    max_speed: float = 2.0           # m/s, the paper's 3DR Solo cap
    max_accel: float = 1.0           # m/s^2, the controller allows 2
    max_jerk: float = 2.0            # m/s^3
    max_yaw_rate: float = 0.6        # rad/s
    max_yaw_accel: float = 0.8       # rad/s^2
    camera_fov: float = math.radians(60.0)       # horizontal, as on the Solo
    max_view_angle: float = math.radians(40.0)   # from head-on
    max_view_range: float = 8.0      # m
    knots: int = 160
    wall_spacing: float = 0.5        # m between wall samples for coverage
    sharpness: float = 4.0           # 1/m, smooth min/max of the distance field
    weights: dict = field(default_factory=lambda: {
        'coverage': 50.0, 'standoff': 0.5, 'facing': 2.0,
        'jerk': 0.2, 'yaw': 2.0, 'time': 0.5,
    })


@dataclass
class PlanResult:
    trajectory: Trajectory
    success: bool
    status: str
    solve_seconds: float
    predicted_coverage: float        # fraction of wall length, by the smooth model
    min_clearance: float             # exact, sampled densely along the result


def _smooth_max(values, k):
    """(1/k) log sum exp(k v): a smooth upper bound on max(values), within log(n)/k."""
    m = ca.mmax(values)
    return m + ca.log(ca.sum1(ca.exp(k * (values - m)))) / k


def smooth_distance(building, k):
    """A CasADi function p -> smooth distance to the building.

    Per box it is the smooth max of the four face distances (positive outside),
    which is exact on the face-normal sides and an underestimate near corners;
    across boxes it is a smooth min. Away from corners it is within
    log(4)/k + log(boxes)/k of the true distance; the planner re-checks
    clearance against the exact distance afterwards.
    """
    p = ca.SX.sym('p', 2)
    per_box = []
    for x0, x1, y0, y1 in building.boxes:
        faces = ca.vertcat(x0 - p[0], p[0] - x1, y0 - p[1], p[1] - y1)
        per_box.append(_smooth_max(faces, k))
    d = -_smooth_max(-ca.vertcat(*per_box), k)
    return ca.Function('distance', [p], [d, ca.gradient(d, p)])


def _sigmoid(x):
    return 1.0 / (1.0 + ca.exp(-x))


def _log_unseen(wall_pts, wall_normals, cfg):
    """A CasADi function (p, psi) -> log(1 - q) for every wall piece.

    q is how well the camera at `p` facing `psi` sees each piece: in range,
    inside the field of view, and close enough to head-on. Summed over knots
    and exponentiated, it gives the chance each piece is never seen.
    """
    p = ca.SX.sym('p', 2)
    psi = ca.SX.sym('psi')
    M = len(wall_pts)
    S = ca.DM(wall_pts.T)                      # 2 x M
    Nrm = ca.DM(wall_normals.T)
    ray = ca.repmat(p, 1, M) - S               # wall -> camera
    r = ca.sqrt(ca.sum1(ray**2) + 1e-6)        # 1 x M
    heading = ca.vertcat(ca.cos(psi), ca.sin(psi))
    in_range = _sigmoid(4.0 * (cfg.max_view_range - r))
    head_on = _sigmoid(20.0 * (ca.sum1(ray * Nrm) / r - math.cos(cfg.max_view_angle)))
    in_fov = _sigmoid(20.0 * (-(heading.T @ ray) / r - math.cos(cfg.camera_fov / 2)))
    q = in_range * head_on * in_fov
    return ca.Function('log_unseen', [p, psi], [ca.log(1 - 0.999 * q).T])


def plan(building, config=None, verbose=False):
    """Solve for one inspection lap. Returns a `PlanResult`."""
    cfg = config or PlannerConfig()
    w = cfg.weights
    N = cfg.knots

    dist = smooth_distance(building, cfg.sharpness)
    wall_pts, wall_normals, wall_len = building.walls(cfg.wall_spacing)

    opti = ca.Opti()
    P = opti.variable(2, N + 1)
    V = opti.variable(2, N + 1)
    A = opti.variable(2, N + 1)
    J = opti.variable(2, N)
    psi = opti.variable(1, N + 1)
    om = opti.variable(1, N + 1)
    omd = opti.variable(1, N)
    T = opti.variable()
    dt = T / N

    # -- dynamics, exact for piecewise-constant jerk and yaw acceleration ---
    for k in range(N):
        j, h = J[:, k], dt
        opti.subject_to(P[:, k + 1] == P[:, k] + V[:, k] * h + A[:, k] * h**2 / 2 + j * h**3 / 6)
        opti.subject_to(V[:, k + 1] == V[:, k] + A[:, k] * h + j * h**2 / 2)
        opti.subject_to(A[:, k + 1] == A[:, k] + j * h)
        opti.subject_to(psi[k + 1] == psi[k] + om[k] * h + omd[k] * h**2 / 2)
        opti.subject_to(om[k + 1] == om[k] + omd[k] * h)

    # -- one closed lap ------------------------------------------------------
    opti.subject_to(P[:, N] == P[:, 0])
    opti.subject_to(V[:, N] == V[:, 0])
    opti.subject_to(A[:, N] == A[:, 0])
    opti.subject_to(psi[N] == psi[0] + 2 * math.pi)
    opti.subject_to(om[N] == om[0])

    # Winding once counter-clockwise around a point inside the building rules
    # out shortcuts through it and laps that only cover one side.
    biggest = np.argmax((building.boxes[:, 1] - building.boxes[:, 0])
                        * (building.boxes[:, 3] - building.boxes[:, 2]))
    b = building.boxes[biggest]
    centre = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2])
    winding = 0
    for k in range(N):
        r0, r1 = P[:, k] - centre, P[:, k + 1] - centre
        winding += ca.atan2(r0[0] * r1[1] - r0[1] * r1[0], r0[0] * r1[0] + r0[1] * r1[1])
    opti.subject_to(winding == 2 * math.pi)

    # -- limits --------------------------------------------------------------
    opti.subject_to(opti.bounded(5.0, T, 1000.0))
    opti.subject_to(ca.sum1(V**2) <= cfg.max_speed**2)
    opti.subject_to(ca.sum1(A**2) <= cfg.max_accel**2)
    opti.subject_to(ca.sum1(J**2) <= cfg.max_jerk**2)
    opti.subject_to(opti.bounded(-cfg.max_yaw_rate, om, cfg.max_yaw_rate))
    opti.subject_to(opti.bounded(-cfg.max_yaw_accel, omd, cfg.max_yaw_accel))

    # -- costs ---------------------------------------------------------------
    standoff_cost = 0
    facing_cost = 0
    for k in range(N):
        d, g = dist(P[:, k])
        heading = ca.vertcat(ca.cos(psi[k]), ca.sin(psi[k]))
        toward_wall = -g / ca.sqrt(ca.sumsqr(g) + 1e-9)
        opti.subject_to(d >= cfg.min_clearance)
        standoff_cost += (d - cfg.standoff) ** 2 * dt
        facing_cost += (1 - ca.dot(heading, toward_wall)) * dt

    # Every (knot, wall piece) pair is in the model - P is symbolic, so there
    # is nothing to prefilter on - but the sigmoids are steep enough that
    # far-off pairs contribute nothing numerically.
    unseen = _log_unseen(wall_pts, wall_normals, cfg).map(N)(P[:, :N], psi[:N])
    seen = 1 - ca.exp(ca.sum2(unseen))
    coverage = ca.dot(ca.DM(wall_len), seen) / float(wall_len.sum())

    jerk_cost = ca.sum1(ca.sum2(J**2)) * dt
    yaw_cost = ca.sum2(omd**2) * dt
    opti.minimize(w['coverage'] * (1 - coverage)
                  + w['standoff'] * standoff_cost
                  + w['facing'] * facing_cost
                  + w['jerk'] * jerk_cost
                  + w['yaw'] * yaw_cost
                  + w['time'] * T)

    # -- first guess: the stand-off contour, flown at a steady speed --------
    loop = resample_loop(building.offset_contour(cfg.standoff), N)
    loop = np.vstack([loop, loop[:1]])
    length = float(np.sum(np.linalg.norm(np.diff(loop, axis=0), axis=1)))
    T0 = length / (0.7 * cfg.max_speed)
    tangent = np.gradient(loop, axis=0)
    tangent[0] = tangent[-1] = loop[1] - loop[-2]
    v0 = tangent / np.linalg.norm(tangent, axis=1, keepdims=True) * length / T0
    # Camera 90 degrees left of travel: counter-clockwise, that is the wall.
    yaw0 = np.unwrap(np.arctan2(v0[:, 1], v0[:, 0]) + math.pi / 2)
    yaw0 = yaw0[0] + (yaw0 - yaw0[0]) * (2 * math.pi) / (yaw0[-1] - yaw0[0])
    opti.set_initial(P, loop.T)
    opti.set_initial(V, v0.T)
    opti.set_initial(A, 0)
    opti.set_initial(J, 0)
    opti.set_initial(psi, yaw0[None])
    opti.set_initial(om, 2 * math.pi / T0)
    opti.set_initial(omd, 0)
    opti.set_initial(T, T0)

    opti.solver('ipopt', {'print_time': False, 'expand': True}, {
        'print_level': 5 if verbose else 0,
        'max_iter': 3000,
        'tol': 1e-6,
        'acceptable_tol': 1e-4,
        'mu_strategy': 'adaptive',
    })

    start = time.monotonic()
    try:
        sol = opti.solve()
        stats, ok = sol.stats(), True
        value = sol.value
    except RuntimeError:
        # Hand back the last iterate so the caller can see what went wrong,
        # but say so - never fly it without checking `success`.
        stats, ok = opti.stats(), False
        value = opti.debug.value
    elapsed = time.monotonic() - start

    traj = Trajectory(
        t=np.linspace(0.0, float(value(T)), N + 1),
        p=np.asarray(value(P)).T, v=np.asarray(value(V)).T, a=np.asarray(value(A)).T,
        j=np.asarray(value(J)).T,
        psi=np.asarray(value(psi)).ravel(), w=np.asarray(value(om)).ravel(),
        wd=np.asarray(value(omd)).ravel(),
    )
    dense = traj.sample(0.02)
    return PlanResult(
        trajectory=traj,
        success=ok,
        status=stats.get('return_status', '?'),
        solve_seconds=elapsed,
        predicted_coverage=float(value(coverage)),
        min_clearance=float(building.distance(dense['p']).min()),
    )
