"""
wheelencoder.py
---------------
IR wheel encoder interface for both rear wheels.

Tracks pulse counts, cumulative distances, and instantaneous speeds.
Direction-aware: call set_directions() whenever the motor direction changes.

Wiring:
    Left  wheel encoder D0 → GPIO 27 (Pin 13)
    Right wheel encoder D0 → GPIO 19

Physical defaults:
    pulses_per_rev  = 20
    wheel_circ_cm   = 20.41 cm  (≈ 2π × 3.25 cm radius)

Usage:
    enc = WheelEncoder()
    enc.set_directions(1, 1)         # both forward
    v_l, v_r = enc.get_speeds()      # cm/s
    d_l, d_r = enc.get_distances()   # cm since last reset
"""

from gpiozero import Button
from time import time


class WheelEncoder:
    """Dual-wheel encoder with direction tracking."""

    def __init__(self, pin_left: int = 27, pin_right: int = 19,
                 pulses_per_rev: int = 20, wheel_circ_cm: float = 20.41):
        self.PULSES_PER_REV    = pulses_per_rev
        self.WHEEL_CIRCUMFERENCE = wheel_circ_cm
        self._cm_per_pulse     = wheel_circ_cm / pulses_per_rev

        # Counters
        self.left_count            = 0
        self.right_count           = 0
        self.left_total_distance   = 0.0
        self.right_total_distance  = 0.0

        # Timing (for instantaneous speed)
        self.left_last_time  = time()
        self.right_last_time = time()

        # Direction (+1 forward, -1 backward)
        self.left_direction  = 1
        self.right_direction = 1

        # GPIO
        self.left_sensor  = Button(pin_left,  pull_up=True)
        self.right_sensor = Button(pin_right, pull_up=True)
        self.left_sensor.when_pressed  = self.left_pulse
        self.right_sensor.when_pressed = self.right_pulse

    def set_directions(self, left_dir: int, right_dir: int) -> None:
        """Set wheel directions: +1 = forward, -1 = backward."""
        self.left_direction  = 1 if left_dir  >= 0 else -1
        self.right_direction = 1 if right_dir >= 0 else -1

    def left_pulse(self) -> None:
        self.left_count += 1
        self.left_total_distance += self.left_direction * self._cm_per_pulse
        self.left_last_time = time()

    def right_pulse(self) -> None:
        self.right_count += 1
        self.right_total_distance += self.right_direction * self._cm_per_pulse
        self.right_last_time = time()

    def get_speeds(self) -> tuple:
        """Return (v_left, v_right) in cm/s. Zero if no pulse in last second."""
        now     = time()
        dt_l    = now - self.left_last_time
        dt_r    = now - self.right_last_time
        v_l = self.left_direction  * (self._cm_per_pulse / dt_l if dt_l < 1 else 0.0)
        v_r = self.right_direction * (self._cm_per_pulse / dt_r if dt_r < 1 else 0.0)
        return v_l, v_r

    def get_distances(self) -> tuple:
        """Return (left_cm, right_cm) total signed distance since last reset."""
        return self.left_total_distance, self.right_total_distance

    def reset(self) -> None:
        """Zero all counters and distances."""
        self.left_count            = 0
        self.right_count           = 0
        self.left_total_distance   = 0.0
        self.right_total_distance  = 0.0
        self.left_last_time        = time()
        self.right_last_time       = time()
