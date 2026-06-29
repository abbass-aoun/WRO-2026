"""
car_controller.py
-----------------
Low-level interface for the drive motor (via L298N) and steering servo.

Wiring (see docs/pinschart.md):
    Motor  : IN1=GPIO14, IN2=GPIO15, ENA=GPIO21 (PWM)
    Servo  : Signal=GPIO12 (PWM, 50 Hz)

Usage:
    ctrl = CarController()
    ctrl.set_all("f", speed=50, steer=0.0)   # forward, half speed, straight
    ctrl.set_all("b", speed=30, steer=-15.0) # backward, slow, turn left
    ctrl.stop()
"""

from gpiozero import Motor, Servo, PWMOutputDevice
from config import Pins, RobotParams
import time


class CarController:
    """
    Drives the DC motor and steering servo.

    Speed is expressed as 0-100 (percent of maximum).
    Steering angle is in degrees: negative = left, positive = right.
    """

    def __init__(self):
        # Drive motor — gpiozero Motor abstracts IN1/IN2 direction
        self._motor = Motor(forward=Pins.MOTOR_IN1, backward=Pins.MOTOR_IN2,
                            enable=Pins.MOTOR_ENA, pwm=True)

        # Steering servo — value range [-1, 1] maps to [right_max, left_max]
        self._servo = Servo(Pins.SERVO_STEERING)

        self.stop()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_all(self, direction: str, speed: float, steer_deg: float) -> None:
        """
        One-call interface used by the main control loop.

        Args:
            direction : "f" for forward, "b" for backward.
            speed     : Motor speed 0-100.
            steer_deg : Steering angle in degrees (negative=left, positive=right).
        """
        self.set_steer(steer_deg)
        if direction == "f":
            self.drive_forward(speed)
        else:
            self.drive_backward(speed)

    def drive_forward(self, speed: float) -> None:
        """Drive forward at given speed (0-100)."""
        self._motor.forward(self._clamp_speed(speed))

    def drive_backward(self, speed: float) -> None:
        """Drive backward at given speed (0-100)."""
        self._motor.backward(self._clamp_speed(speed))

    def set_steer(self, angle_deg: float) -> None:
        """
        Set steering angle.

        Args:
            angle_deg: Degrees from center. Negative = left, positive = right.
                       Clamped to ±MAX_STEER_DEG.
        """
        MAX = 30.0  # mechanical limit — adjust for your servo
        angle_deg = max(-MAX, min(MAX, angle_deg))
        # Map [-MAX, +MAX] → [-1, +1] for gpiozero Servo
        self._servo.value = angle_deg / MAX

    def stop(self) -> None:
        """Cut motor power and center steering."""
        self._motor.stop()
        self._servo.value = 0  # center

    def brake(self, duration_s: float = 0.3) -> None:
        """Active brake: briefly reverse then stop."""
        self._motor.backward(0.3)
        time.sleep(duration_s)
        self._motor.stop()

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp_speed(speed: float) -> float:
        """Convert 0-100 scale to 0.0-1.0, clamped."""
        return max(0.0, min(1.0, speed / 100.0))

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass
