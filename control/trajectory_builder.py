"""
trajectory_builder.py
---------------------
Builds smooth Bézier-curve trajectories around detected traffic pillars.

The trajectory is represented as three parallel arrays (all length N_POINTS):
    points          : np.ndarray (N, 2)  — (x, y) waypoints in cm
    steering_angles : np.ndarray (N,)    — target steering angle at each point (degrees)
    speeds          : np.ndarray (N,)    — target motor speed at each point (0-100)

Usage:
    builder = TrajectoryBuilder()
    points, steering, speeds = builder.build(pillars, start_dir, end_dir)
"""

import numpy as np
from config import TrajParams, RobotParams


class TrajectoryBuilder:
    """
    Generates Bézier-curve paths that navigate around coloured pillars.

    Rules (WRO Future Engineers):
        - Pass to the RIGHT of red pillars.
        - Pass to the LEFT of green pillars.
    """

    PILLAR_OFFSET_CM = 20.0   # How far to the side to aim past a pillar (cm)
    N = TrajParams.N_POINTS

    # ── Public API ────────────────────────────────────────────────────────────

    def build(
        self,
        pillars: list,
        start_dir: np.ndarray,
        end_dir: np.ndarray,
    ) -> tuple:
        """
        Build a trajectory through/around the given pillars.

        Args:
            pillars   : List of TrafficSign objects with .x_cm, .y_cm, .color.
                        Already sorted nearest-first.
            start_dir : Unit vector (2,) — robot's current heading direction.
            end_dir   : Unit vector (2,) — desired heading at the end of the curve.

        Returns:
            (points, steering_angles, speeds) — three np.ndarray arrays of
            length N_POINTS.
        """
        if not pillars:
            return self._straight_trajectory(start_dir)

        # Build a sequence of via-points offset from each pillar
        via_points = [np.array([0.0, 0.0])]  # robot is at origin
        for p in pillars[:3]:                  # use at most 3 nearest pillars
            via_points.append(self._offset_point(p))
        via_points = np.array(via_points)

        # Fit a cubic Bézier through the via-points
        points = self._bezier_curve(via_points, start_dir, end_dir)

        # Derive steering angles from the curve's tangent vectors
        steering = self._steering_from_tangents(points)

        # Assign speeds — slower when steering sharply
        speeds = self._speed_profile(steering)

        return points, steering, speeds

    def bezier_curve_from_pillars(
        self,
        pillars,
        g0: np.ndarray,
        d0: np.ndarray,
        df: np.ndarray,
    ) -> tuple:
        """
        Legacy interface used by Super_main.py.
        Wraps build() to match the original call signature.

        Returns:
            (points, steering, speeds, segment_lengths)
        """
        points, steering, speeds = self.build(list(pillars), d0, df)
        # Compute cumulative segment lengths
        diffs = np.diff(points, axis=0)
        seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
        return points, steering, speeds, seg_len

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _offset_point(self, pillar) -> np.ndarray:
        """
        Return a via-point offset from the pillar in the correct direction.

        Red  → pass to the RIGHT → offset in +x direction (robot frame).
        Green→ pass to the LEFT  → offset in -x direction (robot frame).
        """
        offset = self.PILLAR_OFFSET_CM
        if pillar.color == "red":
            return np.array([pillar.x_cm + offset, pillar.y_cm])
        else:  # green
            return np.array([pillar.x_cm - offset, pillar.y_cm])

    def _bezier_curve(
        self,
        via: np.ndarray,
        d0: np.ndarray,
        df: np.ndarray,
    ) -> np.ndarray:
        """
        Fit a cubic Bézier curve: start at via[0], end at via[-1],
        with tangents d0 and df.  Intermediate via-points are used to
        position control points P1 and P2.
        """
        p0 = via[0]
        p3 = via[-1]

        # Scale tangent handles by 1/3 of total arc length estimate
        chord = np.linalg.norm(p3 - p0)
        handle = chord / 3.0

        p1 = p0 + handle * d0 / (np.linalg.norm(d0) + 1e-9)
        p2 = p3 - handle * df / (np.linalg.norm(df) + 1e-9)

        t = np.linspace(0, 1, self.N)[:, None]  # (N,1) for broadcasting
        points = (
            (1 - t) ** 3 * p0
            + 3 * (1 - t) ** 2 * t * p1
            + 3 * (1 - t) * t ** 2 * p2
            + t ** 3 * p3
        )
        return points  # (N, 2)

    def _straight_trajectory(self, direction: np.ndarray) -> tuple:
        """Fallback: straight line of 150 cm in the given direction."""
        d = direction / (np.linalg.norm(direction) + 1e-9)
        t = np.linspace(0, 150, self.N)[:, None]
        points = t * d
        steering = np.zeros(self.N)
        speeds   = np.full(self.N, TrajParams.DEFAULT_SPEED, dtype=float)
        return points, steering, speeds

    def _steering_from_tangents(self, points: np.ndarray) -> np.ndarray:
        """
        Compute steering angle (degrees) from the curvature of the path.
        Uses the Ackermann formula: steer = arctan(L / R)
        where R is the local radius of curvature.
        """
        L = RobotParams.WHEELBASE_CM
        n = len(points)
        steering = np.zeros(n)

        for i in range(1, n - 1):
            p_prev, p_curr, p_next = points[i - 1], points[i], points[i + 1]
            # Menger curvature
            a = np.linalg.norm(p_curr - p_prev)
            b = np.linalg.norm(p_next - p_curr)
            c = np.linalg.norm(p_next - p_prev)
            area2 = abs(np.cross(p_curr - p_prev, p_next - p_prev))
            if area2 < 1e-6 or a * b * c < 1e-6:
                continue
            R = (a * b * c) / area2
            # Sign: cross product of consecutive tangents
            t1 = p_curr - p_prev
            t2 = p_next - p_curr
            sign = 1.0 if np.cross(t1, t2) > 0 else -1.0
            steering[i] = sign * np.degrees(np.arctan2(L, R))

        # Propagate to endpoints
        steering[0]  = steering[1]
        steering[-1] = steering[-2]
        return steering

    def _speed_profile(self, steering: np.ndarray) -> np.ndarray:
        """Reduce speed proportionally to steering angle magnitude."""
        max_steer = 30.0
        base  = TrajParams.DEFAULT_SPEED
        slow  = TrajParams.CORNER_SPEED
        ratio = np.clip(np.abs(steering) / max_steer, 0, 1)
        return base - ratio * (base - slow)
