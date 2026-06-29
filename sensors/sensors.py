"""
sensors.py
----------
Unified interface for all on-robot sensors:
    - 4× VL53L0X Time-of-Flight distance sensors (I2C)
    - TCS3200 colour sensor (GPIO)
    - 2× IR wheel encoders (GPIO)

Usage:
    sensors = Sensors()
    distances = sensors.read_distances()   # [FR1, FR2, FL2, FL1] in mm
    color     = sensors.read_color()       # "red" | "green" | "blue" |
                                           # "orange" | "yellow" | "unknown"
    enc_l, enc_r = sensors.read_encoders() # cumulative pulse counts
"""

import time
import threading
from gpiozero import Button
from config import Pins


# ── VL53L0X helper (requires vl53l0x library or smbus2 fallback) ─────────────
try:
    import VL53L0X  # pip install vl53l0x
    _VL53_AVAILABLE = True
except ImportError:
    _VL53_AVAILABLE = False


# ── TCS3200 colour sensor ─────────────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class Sensors:
    """Aggregates all sensor readings into a single easy-to-use object."""

    def __init__(self):
        self._enc_left  = 0
        self._enc_right = 0
        self._lock      = threading.Lock()

        self._init_encoders()
        self._init_color_sensor()
        self._distance_sensors = []   # TODO: init VL53L0X sensors

    # ── Encoders ──────────────────────────────────────────────────────────────

    def _init_encoders(self):
        """Attach interrupt callbacks to wheel encoder GPIO pins."""
        self._btn_left  = Button(Pins.ENCODER_LEFT,  pull_up=True)
        self._btn_right = Button(Pins.ENCODER_RIGHT, pull_up=True)
        self._btn_left.when_pressed  = self._on_left_pulse
        self._btn_right.when_pressed = self._on_right_pulse

    def _on_left_pulse(self):
        with self._lock:
            self._enc_left += 1

    def _on_right_pulse(self):
        with self._lock:
            self._enc_right += 1

    def read_encoders(self) -> tuple:
        """Return (left_count, right_count) cumulative pulse counts."""
        with self._lock:
            return self._enc_left, self._enc_right

    def reset_encoders(self):
        """Zero both encoder counters."""
        with self._lock:
            self._enc_left  = 0
            self._enc_right = 0

    def encoder_to_speed_cms(self, count_a: int, count_b: int, dt: float) -> tuple:
        """
        Convert encoder pulse delta to wheel speed in cm/s.

        Args:
            count_a, count_b : pulse counts since last call
            dt               : elapsed time in seconds

        Returns:
            (v_left_cms, v_right_cms)
        """
        from config import RobotParams
        cpp = RobotParams.CM_PER_PULSE
        v_l = (count_a * cpp) / dt if dt > 0 else 0.0
        v_r = (count_b * cpp) / dt if dt > 0 else 0.0
        return v_l, v_r

    # ── Colour Sensor (TCS3200) ───────────────────────────────────────────────

    def _init_color_sensor(self):
        """Configure TCS3200 GPIO pins."""
        if not _GPIO_AVAILABLE:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Pins.COLOR_S0,  GPIO.OUT)
        GPIO.setup(Pins.COLOR_S1,  GPIO.OUT)
        GPIO.setup(Pins.COLOR_S2,  GPIO.OUT)
        GPIO.setup(Pins.COLOR_S3,  GPIO.OUT)
        GPIO.setup(Pins.COLOR_OUT, GPIO.IN)
        # 20% frequency scaling
        GPIO.output(Pins.COLOR_S0, GPIO.HIGH)
        GPIO.output(Pins.COLOR_S1, GPIO.LOW)

    def _read_channel(self, s2: bool, s3: bool, sample_ms: int = 10) -> float:
        """Read one RGB channel from TCS3200. Returns pulses per sample."""
        if not _GPIO_AVAILABLE:
            return 0.0
        GPIO.output(Pins.COLOR_S2, GPIO.HIGH if s2 else GPIO.LOW)
        GPIO.output(Pins.COLOR_S3, GPIO.HIGH if s3 else GPIO.LOW)
        time.sleep(0.001)
        count   = 0
        end     = time.time() + sample_ms / 1000.0
        last    = GPIO.input(Pins.COLOR_OUT)
        while time.time() < end:
            val = GPIO.input(Pins.COLOR_OUT)
            if val != last:
                count += 1
                last = val
        return count / 2.0  # each pulse = rising + falling edge

    def read_color_raw(self) -> tuple:
        """Return raw (R, G, B) frequency counts from the TCS3200."""
        r = self._read_channel(False, False)
        g = self._read_channel(True,  True)
        b = self._read_channel(False, True)
        return r, g, b

    def read_color(self) -> str:
        """
        Return the dominant floor colour as a string.

        Returns:
            "orange" | "blue" | "red" | "green" | "unknown"

        Orange and blue are the WRO corner-line colours.
        Red and green are the traffic-pillar colours.
        """
        r, g, b = self.read_color_raw()
        total = r + g + b
        if total < 5:
            return "unknown"

        # Normalise
        rn = r / total
        gn = g / total
        bn = b / total

        # Simple threshold rules — calibrate these for your sensor!
        if rn > 0.5 and gn > 0.3 and bn < 0.25:
            return "orange"
        if bn > 0.5 and rn < 0.3:
            return "blue"
        if rn > 0.6 and gn < 0.3 and bn < 0.25:
            return "red"
        if gn > 0.5 and rn < 0.3:
            return "green"
        return "unknown"

    # ── Distance Sensors (VL53L0X) ────────────────────────────────────────────

    def read_distances(self) -> list:
        """
        Return list of distances [FR1, FR2, FL2, FL1] in mm.

        TODO: Initialize and address VL53L0X sensors using their XSHUT pins:
              Pins.DIST_XSHUT_1 through DIST_XSHUT_4.
              Use the vl53l0x library or smbus2 direct I2C access.
        """
        if not self._distance_sensors:
            return [9999, 9999, 9999, 9999]
        return [s.get_distance() for s in self._distance_sensors]

    def cleanup(self):
        """Release GPIO resources."""
        if _GPIO_AVAILABLE:
            GPIO.cleanup()
