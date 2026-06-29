"""
servoClass.py
-------------
Servo motor abstraction for the WRO car steering system.

TODO: Paste the real contents of servoClass.py here.
      It should define a myServo class with:
          __init__(self, pin, center_angle, max_deviation)
          set_servo_angle(self, angle)
"""

# ── STUB — replace with real implementation ───────────────────────────────────

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


class myServo:
    """
    Wrapper around a PWM servo motor.

    Args:
        pin           : GPIO pin number (BCM).
        center_angle  : Angle in degrees corresponding to servo center.
        max_deviation : Maximum degrees of travel either side of center.
    """

    def __init__(self, pin: int, center_angle: float = 78,
                 max_deviation: float = 27):
        self.pin           = pin
        self.center_angle  = center_angle
        self.max_deviation = max_deviation
        # gpiozero Servo maps -1..1 to min..max pulse width
        self._servo = Servo(pin, min_pulse_width=0.5/1000,
                            max_pulse_width=2.5/1000)
        self.set_servo_angle(0)   # start centered

    def set_servo_angle(self, angle: float) -> None:
        """
        Move servo to the given angle (degrees from center).

        Args:
            angle : Degrees from center. Positive = one direction,
                    negative = other. Clamped to ±max_deviation.
        """
        angle = max(-self.max_deviation, min(self.max_deviation, angle))
        # Map [-max_deviation, +max_deviation] → [-1, 1]
        self._servo.value = angle / self.max_deviation
