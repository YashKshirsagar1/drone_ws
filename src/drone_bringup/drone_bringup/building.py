"""The building being inspected, as a union of axis-aligned boxes.

Everything here is plain numpy geometry in the horizontal plane: the planner
uses it to seed its first guess and to check the result, and the metrics use it
to decide which walls the camera saw. The optimizer itself needs a smooth
version of `distance`, which lives in `coverage_planner`.

Boxes are footprints `(x_min, x_max, y_min, y_max)` in the world frame. Boxes
may touch or overlap; faces shared between two boxes are interior and are not
walls.
"""

import math

import numpy as np
import yaml


class Building:

    def __init__(self, boxes, height):
        self.boxes = np.array(boxes, dtype=float).reshape(-1, 4)
        self.height = float(height)
        if np.any(self.boxes[:, 1] <= self.boxes[:, 0]) or np.any(self.boxes[:, 3] <= self.boxes[:, 2]):
            raise ValueError('every box needs x_min < x_max and y_min < y_max')

    @classmethod
    def load(cls, path):
        with open(path) as f:
            spec = yaml.safe_load(f)
        return cls(spec['boxes'], spec['height'])

    # -- point queries -------------------------------------------------------

    def distance(self, points):
        """Signed distance to the footprint: positive outside, negative inside.

        Exact outside. Inside it is the depth into the box the point is deepest
        in, which is all anything here needs from it.
        """
        p = np.atleast_2d(points)[:, None, :]                       # (n, 1, 2)
        centre = (self.boxes[:, [0, 2]] + self.boxes[:, [1, 3]]) / 2  # (m, 2)
        half = (self.boxes[:, [1, 3]] - self.boxes[:, [0, 2]]) / 2
        q = np.abs(p - centre) - half                                # (n, m, 2)
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=2)
        inside = np.minimum(q.max(axis=2), 0.0)
        d = (outside + inside).min(axis=1)
        return d if np.ndim(points) > 1 else float(d[0])

    def contains(self, points):
        return self.distance(points) < 0.0

    def centroid(self):
        """Area-weighted centre of the boxes (overlaps counted twice - keep boxes disjoint)."""
        area = (self.boxes[:, 1] - self.boxes[:, 0]) * (self.boxes[:, 3] - self.boxes[:, 2])
        centre = (self.boxes[:, [0, 2]] + self.boxes[:, [1, 3]]) / 2
        return (centre * area[:, None]).sum(axis=0) / area.sum()

    def corners(self):
        b = self.boxes
        return np.concatenate([b[:, [0, 2]], b[:, [1, 2]], b[:, [1, 3]], b[:, [0, 3]]])

    # -- walls and sight lines -----------------------------------------------

    def walls(self, spacing=0.25):
        """Sample the exposed walls.

        Returns `(points, normals, lengths)`: each sample is the middle of a
        wall piece `lengths` long, with its outward unit normal. Pieces whose
        outside is inside another box are interior faces and are dropped.
        """
        points, normals, lengths = [], [], []
        for x0, x1, y0, y1 in self.boxes:
            for a, b, n in (((x0, y0), (x1, y0), (0.0, -1.0)),
                            ((x1, y0), (x1, y1), (1.0, 0.0)),
                            ((x1, y1), (x0, y1), (0.0, 1.0)),
                            ((x0, y1), (x0, y0), (-1.0, 0.0))):
                a, b, n = np.array(a), np.array(b), np.array(n)
                count = max(1, math.ceil(np.linalg.norm(b - a) / spacing))
                t = (np.arange(count) + 0.5) / count
                pts = a + t[:, None] * (b - a)
                exposed = ~self.contains(pts + 0.01 * n)
                points.append(pts[exposed])
                normals.append(np.repeat(n[None], exposed.sum(), axis=0))
                lengths.append(np.full(exposed.sum(), np.linalg.norm(b - a) / count))
        return np.concatenate(points), np.concatenate(normals), np.concatenate(lengths)

    def ray_hit(self, origin, direction):
        """Distance along a ray to the first box it enters, or `inf`.

        `direction` need not be unit length; the answer is in multiples of it.
        A ray starting inside a box hits at 0.
        """
        o = np.asarray(origin, dtype=float)
        d = np.asarray(direction, dtype=float)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / d
            tx = (self.boxes[:, [0, 1]] - o[0]) * inv[0]
            ty = (self.boxes[:, [2, 3]] - o[1]) * inv[1]
        # An axis the ray is parallel to: either always inside that slab or never.
        if d[0] == 0.0:
            inside = (self.boxes[:, 0] <= o[0]) & (o[0] <= self.boxes[:, 1])
            tx = np.where(inside[:, None], [-np.inf, np.inf], [np.inf, -np.inf])
        if d[1] == 0.0:
            inside = (self.boxes[:, 2] <= o[1]) & (o[1] <= self.boxes[:, 3])
            ty = np.where(inside[:, None], [-np.inf, np.inf], [np.inf, -np.inf])
        near = np.maximum(tx.min(axis=1), ty.min(axis=1))
        far = np.minimum(tx.max(axis=1), ty.max(axis=1))
        hit = (near <= far) & (far >= 0.0)
        return float(np.maximum(near[hit], 0.0).min()) if hit.any() else math.inf

    def blocked(self, a, b):
        """True if the straight line from `a` to `b` passes through the building."""
        return self.ray_hit(a, np.subtract(b, a)) < 1.0 - 1e-6

    # -- the stand-off contour -----------------------------------------------

    def offset_contour(self, standoff, resolution=0.1):
        """The closed loop at exactly `standoff` from the walls, counter-clockwise.

        Used as the optimizer's first guess. Where a gap between walls is
        narrower than twice the stand-off the loop simply bridges it.
        """
        import contourpy

        lo = self.boxes[:, [0, 2]].min(axis=0) - standoff - 2.0
        hi = self.boxes[:, [1, 3]].max(axis=0) + standoff + 2.0
        xs = np.arange(lo[0], hi[0] + resolution, resolution)
        ys = np.arange(lo[1], hi[1] + resolution, resolution)
        gx, gy = np.meshgrid(xs, ys)
        field = self.distance(np.column_stack([gx.ravel(), gy.ravel()])).reshape(gx.shape)

        lines = contourpy.contour_generator(xs, ys, field).lines(standoff)
        loop = max(lines, key=len)
        if np.linalg.norm(loop[0] - loop[-1]) < 1e-9:
            loop = loop[:-1]
        if _signed_area(loop) < 0:
            loop = loop[::-1]
        return loop


def _signed_area(loop):
    x, y = loop[:, 0], loop[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def resample_loop(loop, count):
    """`count` points spaced evenly by arc length around a closed polyline."""
    closed = np.vstack([loop, loop[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.linspace(0.0, s[-1], count, endpoint=False)
    return np.column_stack([np.interp(targets, s, closed[:, i]) for i in range(2)])
