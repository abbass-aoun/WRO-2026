"""
ekf_estimator.py
----------------
Extended Kalman Filter (EKF) for fusing wheel-encoder odometry with
IMU yaw rate to estimate the robot's 2-D pose: [x, y, theta].

State vector:
    x = [x_cm, y_cm, theta_rad]

Inputs (control vector):
    u = [v_cms, steer_deg]
        v_cms     — forward speed derived from encoders (cm/s)
        steer_deg — steering angle from servo encoder / commanded angle

Measurement:
    z = [d_yaw]  — change in heading from IMU (rad/s × dt)

Usage:
    ekf = EKFEstimator()
    pose = ekf.update(pose, dt, speed, steer, enc_r, enc_l, steer_enc, imu_z)
"""

import numpy as np
from config import RobotParams


class EKFEstimator:
    """
    Differential-drive EKF with Ackermann steering kinematics.

    Call update() once per control loop iteration.
    """

    def __init__(self):
        # Process noise covariance Q — tune these!
        self.Q = np.diag([1.0, 1.0, np.radians(2.0)]) ** 2

        # Measurement noise covariance R (yaw from IMU)
        self.R = np.diag([np.radians(1.0)]) ** 2

        # Estimate covariance (starts uncertain)
        self.P = np.eye(3) * 100.0

    # ── Public API ────────────────────────────────────────────────────────────

    def update(
        self,
        pose: list,
        dt: float,
        speed_cms: float,
        steer_deg: float,
        enc_right: int,
        enc_left: int,
        steer_enc: float,
        imu_yaw_rate: float,
    ) -> list:
        """
        Run one EKF predict + update cycle.

        Args:
            pose         : Current [x, y, theta] (will be updated in-place).
            dt           : Time step in seconds.
            speed_cms    : Forward speed from encoders (cm/s).
            steer_deg    : Commanded steering angle (degrees).
            enc_right    : Right encoder pulse count (unused directly —
                           speed_cms already derived from encoders).
            enc_left     : Left encoder pulse count.
            steer_enc    : Potentiometer-measured actual steering angle (deg).
            imu_yaw_rate : IMU yaw rate (rad/s).

        Returns:
            Updated [x, y, theta] pose list.
        """
        x = np.array(pose, dtype=float)

        # ── Predict ───────────────────────────────────────────────────────────
        x_pred, F = self._motion_model(x, speed_cms, steer_deg, dt)
        P_pred = F @ self.P @ F.T + self.Q

        # ── Update (IMU yaw measurement) ──────────────────────────────────────
        dtheta_imu = imu_yaw_rate * dt                # measured heading change
        z          = np.array([dtheta_imu])

        H          = np.array([[0.0, 0.0, 1.0]])      # we observe theta
        z_pred     = np.array([x_pred[2] - x[2]])     # predicted dtheta
        y_innov    = z - z_pred

        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)           # Kalman gain

        x_upd      = x_pred + (K @ y_innov).flatten()
        self.P     = (np.eye(3) - K @ H) @ P_pred

        # Normalise heading to [-π, π]
        x_upd[2]   = np.arctan2(np.sin(x_upd[2]), np.cos(x_upd[2]))

        pose[0], pose[1], pose[2] = float(x_upd[0]), float(x_upd[1]), float(x_upd[2])
        return pose

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _motion_model(x: np.ndarray, v: float, steer_deg: float,
                      dt: float) -> tuple:
        """
        Ackermann kinematic motion model.

        Returns:
            (x_new, F) where F is the Jacobian of the motion model.
        """
        L       = RobotParams.WHEELBASE_CM
        theta   = x[2]
        delta   = np.radians(steer_deg)

        dx      = v * np.cos(theta) * dt
        dy      = v * np.sin(theta) * dt
        dtheta  = (v / L) * np.tan(delta) * dt

        x_new   = x + np.array([dx, dy, dtheta])

        # Jacobian F = d(x_new) / d(x)
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1                      ],
        ])

        return x_new, F
