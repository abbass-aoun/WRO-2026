"""
allEncodersClass.py
-------------------
Wheel encoder and IMU interface for the WRO car.

Reads left and right rear wheel IR encoders via GPIO interrupts and
computes linear / angular speeds and cumulative distances.
Also reads yaw rate from the onboard MPU-6050 IMU over I2C.

Hardware:
    Left  encoder → GPIO 27 (default)
    Right encoder → GPIO 19 (default)
    IMU (MPU-6050) → I2C address 0x68

Physical constants (adjust if your wheel is different):
    pulses_per_rev  = 50    pulses per full wheel revolution
    wheel_circ_cm   = 20.48 wheel circumference in cm
                           (≈ 2π × 3.26 cm radius)

Usage:
    enc = RobotEncoders()
    v_l, v_r = enc.get_linear_speeds()   # cm/s
    yaw      = enc.get_yaw_rate()        # deg/s from IMU
    d_l, d_r = enc.get_distances()       # cm travelled since reset
    enc.reset_all()
"""

from gpiozero import Button
from mpu6050 import mpu6050
from time import time, sleep
import math


class RobotEncoders:
    """
    Combines wheel encoders and MPU-6050 IMU into one interface.

    Speed is computed from the time between the last encoder pulse and
    now. If no pulse has arrived in the last second the wheel is
    considered stationary (speed = 0).
    """

    def __init__(
        self,
        wheel_left_pin:  int   = 27,
        wheel_right_pin: int   = 19,
        pulses_per_rev:  int   = 50,
        wheel_circ_cm:   float = 20.48,
    ):
        self.PULSES_PER_REV = pulses_per_rev
        self.WHEEL_CIRC     = wheel_circ_cm
        self._cm_per_pulse  = wheel_circ_cm / pulses_per_rev

        # Pulse counters
        self.left_count  = 0
        self.right_count = 0

        # Distance accumulators
        self.left_total_distance  = 0.0
        self.right_total_distance = 0.0

        # Timestamps of last pulse (used for instantaneous speed)
        self.left_last_time  = time()
        self.right_last_time = time()

        # GPIO encoder buttons (active on falling edge with pull-up)
        self.left_sensor  = Button(wheel_left_pin,  pull_up=True)
        self.right_sensor = Button(wheel_right_pin, pull_up=True)
        self.left_sensor.when_pressed  = self._left_pulse
        self.right_sensor.when_pressed = self._right_pulse

        # IMU — MPU-6050 on I2C address 0x68
        self.imu = mpu6050(0x68)

    # ── Interrupt callbacks ───────────────────────────────────────────────────

    def _left_pulse(self) -> None:
        self.left_count          += 1
        self.left_total_distance += self._cm_per_pulse
        self.left_last_time       = time()

    def _right_pulse(self) -> None:
        self.right_count          += 1
        self.right_total_distance += self._cm_per_pulse
        self.right_last_time       = time()

    # ── Speed ─────────────────────────────────────────────────────────────────

    def get_linear_speeds(self) -> tuple:
        """
        Return (v_left, v_right) instantaneous linear wheel speeds in cm/s.

        Speed is estimated from the time elapsed since the last encoder
        pulse. If no pulse arrived in the last second, speed is 0.
        """
        now  = time()
        dt_l = now - self.left_last_time
        dt_r = now - self.right_last_time
        v_l  = self._cm_per_pulse / dt_l if dt_l < 1.0 else 0.0
        v_r  = self._cm_per_pulse / dt_r if dt_r < 1.0 else 0.0
        return v_l, v_r

    def get_angular_speeds(self) -> tuple:
        """
        Return (omega_left, omega_right) wheel angular speeds in rad/s.
        """
        now  = time()
        dt_l = now - self.left_last_time
        dt_r = now - self.right_last_time
        rev_l   = (1 / self.PULSES_PER_REV) / dt_l if dt_l < 1.0 else 0.0
        rev_r   = (1 / self.PULSES_PER_REV) / dt_r if dt_r < 1.0 else 0.0
        omega_l = 2 * math.pi * rev_l
        omega_r = 2 * math.pi * rev_r
        return omega_l, omega_r

    # ── Distance ──────────────────────────────────────────────────────────────

    def get_distances(self) -> tuple:
        """Return (left_cm, right_cm) total distance since last reset."""
        return self.left_total_distance, self.right_total_distance

    # ── IMU ───────────────────────────────────────────────────────────────────

    def get_yaw_rate(self) -> float:
        """
        Return yaw rate in deg/s from MPU-6050 gyroscope.

        Uses the Y-axis gyro channel — change to 'x' or 'z' depending
        on how the IMU is mounted on your robot.
        """
        gyro = self.imu.get_gyro_data()
        return gyro["y"]

    # ── Combined state ────────────────────────────────────────────────────────

    def get_motion_state(self) -> dict:
        """
        Return a dict with left_speed, right_speed, and yaw_rate.

        Convenient for logging or passing to the EKF estimator.
        """
        v_l, v_r = self.get_linear_speeds()
        yaw      = self.get_yaw_rate()
        return {
            "left_speed":  v_l,
            "right_speed": v_r,
            "yaw_rate":    yaw,
        }

    # ── Reset ─────────────────────────────────────────────────────────────────

    def reset_wheels(self) -> None:
        """Reset pulse counts, distances, and timestamps."""
        self.left_count           = 0
        self.right_count          = 0
        self.left_total_distance  = 0.0
        self.right_total_distance = 0.0
        self.left_last_time       = time()
        self.right_last_time      = time()

    def reset_all(self) -> None:
        """Alias for reset_wheels() — resets everything."""
        self.reset_wheels()
