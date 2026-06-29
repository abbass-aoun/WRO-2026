"""
servoClass.py
-------------
Servo motor control using lgpio for precise PWM pulse generation.

The servo is driven at 50 Hz. Pulse widths range from 500 µs (0°) to
2500 µs (180°). The car uses a relative angle API so callers don't need
to know the absolute center position.

Hardware:
    Servo signal pin → GPIO 12 (configurable)
    Center angle     → 78° (straight ahead for this car)
    Max deviation    → 27° either side

Usage:
    servo = myServo(pin=12, center_angle=78, max_deviation=27)
    servo.set_servo_angle(10)    # steer 10° right of center
    servo.set_servo_angle(-15)   # steer 15° left of center
    servo.cleanup()              # release GPIO on shutdown
"""

import lgpio
import time


class myServo:
    """
    PWM servo wrapper using lgpio.

    Angles are given relative to center:
        0   = straight ahead
        +N  = right (or left, depending on mounting)
        -N  = opposite direction
    Clamped to ±max_deviation automatically.
    """

    def __init__(self, servo_pin: int,
                 center_angle: float = 90,
                 max_deviation: float = 30):
        self.chip       = lgpio.gpiochip_open(0)
        self.servo_pin  = servo_pin
        self.center     = center_angle
        self.deviation  = max_deviation
        self.pwm_freq   = 50          # Hz — standard servo frequency

        lgpio.gpio_claim_output(self.chip, self.servo_pin)

        # Start centered
        self.set_servo_angle(0)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_servo_angle(self, relative_angle: float) -> None:
        """
        Move servo to a position relative to its center.

        Args:
            relative_angle : Degrees from center. Clamped to ±max_deviation.
                             Positive = center + angle, negative = center - angle.
        """
        absolute_angle = max(
            self.center - self.deviation,
            min(self.center + self.deviation, self.center + relative_angle)
        )
        pulse_us = self.angle_to_pulse(absolute_angle)

        # lgpio tx_pwm expects duty cycle as a percentage (0–100)
        duty = pulse_us * 1e-6 * self.pwm_freq * 100
        lgpio.tx_pwm(self.chip, self.servo_pin, self.pwm_freq, duty)

    def cleanup(self) -> None:
        """Release lgpio chip handle. Call on shutdown."""
        lgpio.gpiochip_close(self.chip)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def angle_to_pulse(angle: float) -> int:
        """
        Convert an absolute servo angle (0–180°) to a pulse width in µs.

        Maps:
            0°   → 500 µs  (full one way)
            90°  → 1500 µs (center)
            180° → 2500 µs (full other way)
        """
        angle = max(0, min(180, angle))
        return int(500 + (angle / 180) * 2000)
