"""
ekf_estimator.py
----------------
Extended Kalman Filter pose estimator using filterpy.

State vector: [x_cm, y_cm, theta_rad]
Inputs      : wheel encoder speeds (v_l, v_r), steering angle, IMU heading

Based on ekf4class.py — the most complete EKF version in the original repo.
Uses sensor-only measurements (no commanded values) for robustness.

Requires:  pip install filterpy

Usage:
    ekf = EKFEstimator(wheelbase_cm=13.0)
    pose = ekf.update(v_l, v_r, delta_rad, imu_theta_rad)
    # pose = [x_cm, y_cm, theta_rad]
"""

import numpy as np
import time

try:
    from filterpy.kalman import ExtendedKalmanFilter
    _FILTERPY = True
except ImportError:
    _FILTERPY = False
    print("[EKF] filterpy not installed — using simple odometry fallback")


class EKFEstimator:
    """
    Sensor-fused pose estimator.

    If filterpy is available: full EKF with covariance propagation.
    Otherwise: simple Ackermann dead-reckoning (fallback).
    """

    def __init__(self, wheelbase_cm: float = 13.0):
        self.wheelbase  = wheelbase_cm / 100.0   # convert to metres internally
        self.prev_time  = time.time()
        self._pose      = np.array([0.0, 0.0, 0.0])   # fallback state

        if _FILTERPY:
            self.ekf        = ExtendedKalmanFilter(dim_x=3, dim_z=3)
            self.ekf.x      = np.array([0., 0., 0.])
            self.ekf.P     *= 0.0
            self.ekf.Q      = np.eye(3) * 0.01
            self.ekf.R      = np.diag([0.01, 0.01, 0.002])

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, v_l: float, v_r: float,
               delta_meas: float, imu_theta_rad: float) -> np.ndarray:
        """
        Run one EKF predict + update step.

        Args:
            v_l           : Left  wheel speed (cm/s).
            v_r           : Right wheel speed (cm/s).
            delta_meas    : Measured steering angle (radians).
            imu_theta_rad : Absolute heading from IMU integration (radians).

        Returns:
            np.ndarray [x_cm, y_cm, theta_rad] updated pose.
        """
        now = time.time()
        dt  = max(now - self.prev_time, 1e-4)
        self.prev_time = now

        # Convert cm/s → m/s for internal calculation
        v = (v_l + v_r) / 2.0 / 100.0

        if not _FILTERPY:
            return self._dead_reckoning(v, delta_meas, dt)

        # ── Predict ───────────────────────────────────────────────────────────
        self.ekf.F = self._jacobian(self.ekf.x, dt, v, delta_meas)
        self.ekf.x = self._fx(self.ekf.x, dt, v, delta_meas)
        self.ekf.predict()

        # ── Update (replace yaw with IMU reading) ─────────────────────────────
        z          = self.ekf.x.copy()
        z[2]       = imu_theta_rad
        self.ekf.update(z=z,
                        HJacobian=lambda x: np.eye(3),
                        Hx=lambda x: x)

        # Convert x back to cm
        result    = self.ekf.x.copy()
        result[0] *= 100.0
        result[1] *= 100.0
        return result

    def reset(self) -> None:
        """Reset pose to origin."""
        self._pose = np.array([0.0, 0.0, 0.0])
        if _FILTERPY:
            self.ekf.x = np.array([0., 0., 0.])
            self.ekf.P = np.zeros((3, 3))

    # ── Internal ──────────────────────────────────────────────────────────────

    def _fx(self, x: np.ndarray, dt: float,
             v: float, delta: float) -> np.ndarray:
        """Ackermann kinematic motion model (state in metres)."""
        dx     = v * np.cos(x[2]) * dt
        dy     = v * np.sin(x[2]) * dt
        dtheta = (v / self.wheelbase) * np.tan(delta) * dt
        return x + np.array([dx, dy, dtheta])

    def _jacobian(self, x: np.ndarray, dt: float,
                   v: float, delta: float) -> np.ndarray:
        return np.array([
            [1, 0, -v * np.sin(x[2]) * dt],
            [0, 1,  v * np.cos(x[2]) * dt],
            [0, 0,  1],
        ])

    def _dead_reckoning(self, v: float, delta: float,
                         dt: float) -> np.ndarray:
        """Simple fallback when filterpy is not available."""
        x, y, theta = self._pose
        self._pose = np.array([
            x     + v * np.cos(theta) * dt * 100,   # back to cm
            y     + v * np.sin(theta) * dt * 100,
            theta + (v / self.wheelbase) * np.tan(delta) * dt,
        ])
        return self._pose.copy()
