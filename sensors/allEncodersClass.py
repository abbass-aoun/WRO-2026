"""
allEncodersClass.py
-------------------
Wheel encoder abstraction providing linear speed readings for both rear wheels.

TODO: Paste the real contents of allEncodersClass.py here.
      It should define a RobotEncoders class with:
          get_linear_speeds() -> (v_left, v_right)   # in cm/s or m/s
"""

# ── STUB — replace with real implementation ───────────────────────────────────

import threading
import time
from gpiozero import Button
from config import Pins, RobotParams


class RobotEncoders:
    """
    Reads left and right rear wheel encoders and computes linear speeds.

    Uses interrupt-driven pulse counting on GPIO input pins.
    """

    WINDOW_S = 0.05   # Speed averaging window (seconds)

    def __init__(self):
        self._left_count  = 0
        self._right_count = 0
        self._lock        = threading.Lock()

        self._left_btn  = Button(Pins.ENCODER_LEFT,  pull_up=True)
        self._right_btn = Button(Pins.ENCODER_RIGHT, pull_up=True)

        self._left_btn.when_pressed  = self._on_left
        self._right_btn.when_pressed = self._on_right

        # Speed tracking
        self._last_time   = time.time()
        self._last_left   = 0
        self._last_right  = 0

    def _on_left(self):
        with self._lock:
            self._left_count += 1

    def _on_right(self):
        with self._lock:
            self._right_count += 1

    def get_counts(self) -> tuple:
        """Return (left_count, right_count) cumulative pulse counts."""
        with self._lock:
            return self._left_count, self._right_count

    def get_linear_speeds(self) -> tuple:
        """
        Return (v_left, v_right) instantaneous linear wheel speeds (cm/s).

        Computed from pulse delta over a short time window.
        """
        now = time.time()
        dt  = now - self._last_time
        if dt < self.WINDOW_S:
            time.sleep(self.WINDOW_S - dt)
            now = time.time()
            dt  = now - self._last_time

        with self._lock:
            left_now  = self._left_count
            right_now = self._right_count

        d_left  = left_now  - self._last_left
        d_right = right_now - self._last_right

        cpp = RobotParams.CM_PER_PULSE
        v_l = (d_left  * cpp) / dt
        v_r = (d_right * cpp) / dt

        self._last_time  = now
        self._last_left  = left_now
        self._last_right = right_now

        return v_l, v_r

    def reset(self):
        """Zero all counters."""
        with self._lock:
            self._left_count  = 0
            self._right_count = 0
        self._last_left  = 0
        self._last_right = 0
