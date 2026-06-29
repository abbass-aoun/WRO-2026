"""
steeringencoder.py
------------------
Potentiometer-based steering angle encoder using a rotary encoder on GPIO.

Wiring:
    A (CLK) → GPIO 5 (Pin 29)
    B (DT)  → GPIO 6 (Pin 31)

The encoder counts steps from center. Steps are mapped to degrees using
linear interpolation between full-left and full-right step counts.

Usage:
    enc = SteeringEncoder()
    angle = enc.get_angle()   # degrees, negative=left, positive=right
    enc.reset()               # re-zero at current position
"""

from gpiozero import RotaryEncoder


class SteeringEncoder:
    """Rotary encoder mapped to steering angle in degrees."""

    def __init__(self, pin_a: int = 5, pin_b: int = 6,
                 full_left: int = -50, full_right: int = 50,
                 max_angle: float = 30.0):
        self.STEPS_FULL_LEFT  = full_left
        self.STEPS_FULL_RIGHT = full_right
        self.MAX_ANGLE        = max_angle
        self.encoder          = RotaryEncoder(a=pin_a, b=pin_b, max_steps=0)

    def get_angle(self) -> float:
        """Return current steering angle in degrees (negative=left)."""
        steps = max(self.STEPS_FULL_LEFT,
                    min(self.STEPS_FULL_RIGHT, self.encoder.steps))
        return round((steps / self.STEPS_FULL_RIGHT) * self.MAX_ANGLE, 1)

    def reset(self) -> None:
        """Zero the encoder at the current position."""
        self.encoder.steps = 0
