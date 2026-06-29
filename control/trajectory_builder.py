"""
trajectory_builder.py
---------------------
Bézier-curve trajectory planner for navigating around WRO traffic pillars.

Ported from the original TrajectoryBuilder.py with:
  - Fixed imports (no hardcoded Windows paths)
  - Renamed class to TrajectoryBuilder (snake_case consistent)
  - Docstrings added throughout
  - Test/demo block preserved under __main__

The planner builds a smooth path through a sequence of via-points offset
from each detected pillar, respecting WRO passing rules:
    Red   pillar → pass to the RIGHT
    Green pillar → pass to the LEFT

Usage:
    builder = TrajectoryBuilder(resolution_cm=5.0)
    points, steering, speed, seg_lengths = builder.bezier_curve_from_pillars(
        pillars, g0, d0, df, theta
    )
"""

import numpy as np
from scipy.ndimage import gaussian_filter1d


class TrajectoryBuilder:
    """Builds Bézier-curve trajectories around detected pillars."""

    last_calculated_trajectory = None   # class-level cache

    def __init__(self, resolution_cm: float = 5.0):
        """
        Args:
            resolution_cm : Arc-length spacing between waypoints (cm).
                            Smaller = smoother but more points.
        """
        self.resolution_cm = resolution_cm

    # ── Public API ────────────────────────────────────────────────────────────

    def bezier_curve_from_pillars(self, pillars, g0: np.ndarray,
                                   d0: np.ndarray, df: np.ndarray,
                                   theta: float) -> tuple:
        """
        Build a trajectory through all given pillars.

        Args:
            pillars : Array-like of objects with .x_cm, .y_cm, .color, .section_nb
            g0      : Start position [x, y] in cm
            d0      : Start direction unit vector [dx, dy]
            df      : End direction unit vector [dx, dy]
            theta   : Robot heading angle in radians (used for offset direction)

        Returns:
            (points, steering_angles, safe_speeds, segment_lengths)
            All arrays aligned by index.
        """
        pillars = np.asarray(pillars)
        n = pillars.shape[0]

        # Build via-points: start + one offset point per pillar
        v = np.zeros((n + 1, 2))
        v[0] = g0
        for i in range(1, n + 1):
            p = pillars[i - 1]
            x, y, color, sec = p.x_cm, p.y_cm, p.color, p.section_nb
            offset = 20   # cm to pass beside the pillar
            if sec == 1:
                v[i] = [x - offset * np.sin(theta), y - offset * np.cos(theta)]                     if color == "red" else                        [x + offset * np.sin(theta), y + offset * np.cos(theta)]
            elif sec == 2:
                v[i] = [x - offset * np.cos(theta), y + offset * np.sin(theta)]                     if color == "red" else                        [x + offset * np.cos(theta), y - offset * np.sin(theta)]
            elif sec == 3:
                v[i] = [x + offset * np.sin(theta), y + offset * np.cos(theta)]                     if color == "red" else                        [x - offset * np.sin(theta), y - offset * np.cos(theta)]
            else:
                v[i] = [x + offset * np.cos(theta), y - offset * np.sin(theta)]                     if color == "red" else                        [x - offset * np.cos(theta), y + offset * np.sin(theta)]

        # Compute tangent directions at each via-point
        dv = np.zeros_like(v)
        dv[0]  = d0
        dv[-1] = df
        kc = 0.0085   # smoothing constant
        for i in range(1, dv.shape[0] - 1):
            tdv    = v[i + 1] - v[i]
            dv[i]  = tdv + kc * np.linalg.norm(tdv) * (v[i] - v[i - 1])

        # Sample each segment
        all_points, all_steering, all_speed, seg_lengths = [], [], [], []
        for i in range(len(v) - 1):
            pts, steer, spd = self._sample_segment(v[i], v[i+1], dv[i], dv[i+1])
            all_points.append(pts)
            all_steering.append(steer)
            all_speed.append(spd)
            seg_lengths.append(len(pts))

        points   = np.vstack(all_points)
        steering = np.concatenate(all_steering)
        speed    = np.concatenate(all_speed)

        # Cache result
        TrajectoryBuilder.last_calculated_trajectory = {
            "x":             [round(float(xi), 2) for xi in points[:, 0]],
            "y":             [round(float(yi), 2) for yi in points[:, 1]],
            "steering_angle": [round(float(a),  2) for a in steering],
            "safe_speed":    [round(float(s),  2) for s in speed],
        }

        return (np.round(points, 2),
                np.round(steering, 2),
                np.round(speed, 2),
                seg_lengths)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _sample_segment(self, P0: np.ndarray, P3: np.ndarray,
                         v0: np.ndarray, v3: np.ndarray) -> tuple:
        """
        Sample a cubic Bézier segment by arc length.

        Returns (points, steering_angles, safe_speeds).
        """
        v0 = v0 / (np.linalg.norm(v0) + 1e-9)
        v3 = v3 / (np.linalg.norm(v3) + 1e-9)
        d  = np.linalg.norm(P3 - P0) / 3
        P1 = P0 + d * v0
        P2 = P3 - d * v3

        def bezier(t):
            return ((1-t)**3*P0 + 3*(1-t)**2*t*P1 +
                    3*(1-t)*t**2*P2 + t**3*P3)

        # Dense sample for arc-length parameterisation
        t_dense = np.linspace(0, 1, 1000)
        curve   = np.array([bezier(t) for t in t_dense])
        dists   = np.sqrt(np.sum(np.diff(curve, axis=0)**2, axis=1))
        cumlen  = np.insert(np.cumsum(dists), 0, 0)
        total   = cumlen[-1]

        n_pts   = max(int(total / self.resolution_cm), 2)
        t_samp  = np.interp(np.linspace(0, total, n_pts), cumlen, t_dense)
        points  = np.array([bezier(t) for t in t_samp])

        # Curvature → steering angle
        dx  = np.gradient(points[:, 0], t_samp)
        dy  = np.gradient(points[:, 1], t_samp)
        d2x = np.gradient(dx, t_samp)
        d2y = np.gradient(dy, t_samp)

        curv  = np.abs(dx*d2y - dy*d2x) / (dx**2 + dy**2)**1.5
        R     = 1.0 / (curv + 1e-9)
        sign  = np.sign(dx*d2y - dy*d2x)
        L     = 13   # wheelbase in cm
        steer = -sign * np.degrees(np.arctan(L / R))

        # Speed profile: slower when steering sharply
        a_y_max   = 2.0    # m/s²
        max_speed = 3.0    # m/s
        safe_spd  = np.minimum(
            np.sqrt(a_y_max * (L/100) /
                    (np.tan(np.radians(np.abs(steer))) + 1e-9)),
            max_speed
        )
        safe_spd = np.clip(safe_spd, 0.5, max_speed)

        return points, steer, safe_spd


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    class _Pillar:
        def __init__(self, x, y, color, sec):
            self.x_cm = x; self.y_cm = y
            self.color = color; self.section_nb = sec

    pillars = np.array([
        _Pillar(60, 100, "red",   1),
        _Pillar(40, 200, "green", 1),
        _Pillar(100, 260, "red",  2),
        _Pillar(260, 150, "green",3),
        _Pillar(100,  40, "red",  4),
    ])

    tb = TrajectoryBuilder(3.0)
    pts, steer, spd, _ = tb.bezier_curve_from_pillars(
        pillars, np.array([50, 50]), np.array([1, 0]), np.array([0, -1]), 0)

    plt.figure(); plt.plot(pts[:,0], pts[:,1]); plt.axis("equal")
    plt.title("Trajectory"); plt.show()
