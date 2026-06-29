"""
car_controller.py
-----------------
Low-level interface for the drive motor (L298N) and steering servo.

Dependencies:
    servoClass.py       — myServo wrapper (in control/)
    allEncodersClass.py — RobotEncoders wrapper (in sensors/)

Wiring (see docs/pinschart.md):
    Motor : IN1=GPIO14, IN2=GPIO15, ENA=GPIO21 (PWM)
    Servo : Signal=GPIO12 (PWM)

Usage:
    ctrl = CarController(in1_pin=14, in2_pin=15, ena_pin=21, servo_pin=12)
    ctrl.setAll("f", speed=0.6, angle=10)   # forward, 60% speed, steer right
    ctrl.brake(encoders)                     # PID-controlled stop
    ctrl.stop()
"""

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import time, sleep
from control.servoClass import myServo


class CarController:
    """
    Controls the DC drive motor and steering servo.

    Speed is expressed as 0.0–1.0 (fraction of maximum).
    Steering angle is in degrees relative to servo center (78°).
    """

    def __init__(self, in1_pin: int, in2_pin: int,
                 ena_pin: int, servo_pin: int):
        # Motor direction pins
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)
        # Motor speed (PWM)
        self.ena = PWMOutputDevice(ena_pin)
        # Steering servo — center_angle and max_deviation tuned for this car
        self.servo = myServo(servo_pin, center_angle=78, max_deviation=27)

    # ── Motor ─────────────────────────────────────────────────────────────────

    def set_motor(self, direction: str, speed: float = 1.0) -> None:
        """
        Drive the motor.

        Args:
            direction : "f" = forward, "b" = backward, anything else = stop.
            speed     : 0.0 – 1.0 (clamped automatically).
        """
        speed = max(0.0, min(1.0, speed))
        if direction == "f":
            self.in1.on()
            self.in2.off()
            self.ena.value = speed
        elif direction == "b":
            self.in1.off()
            self.in2.on()
            self.ena.value = speed
        else:
            self.stop()

    # ── Steering ──────────────────────────────────────────────────────────────

    def set_steering(self, angle: float) -> None:
        """
        Set steering angle.

        Args:
            angle : Degrees from servo center (78°).
                    Clamped to ±max_deviation (27°) by myServo.
        """
        self.servo.set_servo_angle(angle)
        sleep(0.05)   # allow servo to reach position

    # ── Combined ──────────────────────────────────────────────────────────────

    def setAll(self, direction: str, speed: float, angle: float) -> None:
        """One-call interface: set steering then motor together."""
        self.set_steering(angle)
        self.set_motor(direction, speed)

    # ── Stop / Brake ──────────────────────────────────────────────────────────

    def stop(self) -> None:
        """Cut motor power immediately."""
        self.in1.off()
        self.in2.off()
        self.ena.value = 0.0

    def brake(self, encoders, kp: float = 15, ki: float = 0.4,
              kd: float = 4, tolerance: float = 0.05,
              log_fn=None) -> None:
        """
        PID-controlled deceleration to a full stop.

        Alternates between braking and releasing the motor each iteration
        to avoid current spikes while stopping quickly.

        Args:
            encoders  : RobotEncoders instance — provides get_linear_speeds().
            kp, ki, kd: PID gains (tuned for this hardware).
            tolerance : Stop when |avg_speed| < tolerance (m/s or cm/s).
            log_fn    : Optional callable(timestamp, v_l, v_r) for logging.
        """
        integral   = 0.0
        last_error = 0.0
        last_time  = time()
        count      = 0

        while True:
            v_l, v_r  = encoders.get_linear_speeds()
            avg_speed  = (v_l + v_r) / 2.0
            error      = -avg_speed       # target = 0

            now        = time()
            dt         = now - last_time
            last_time  = now
            if dt == 0:
                continue

            integral  += error * dt
            derivative = (error - last_error) / dt
            last_error = error

            control = kp * error + ki * integral + kd * derivative
            control = max(0.0, min(1.0, abs(control)))

            # Alternate brake / release to prevent stall current
            if count % 2 == 0:
                self.set_motor("b" if avg_speed > 0 else "f", control)
            else:
                self.set_motor("f" if avg_speed > 0 else "b", 0)

            count += 1

            if log_fn:
                log_fn(now, v_l, v_r)

            if abs(avg_speed) < tolerance:
                break

            sleep(0.01)

        self.stop()
