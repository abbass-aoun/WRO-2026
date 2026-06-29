"""
pid_controller.py
-----------------
Generic PID controller base class.

Subclassed by DrivingPIDController and SteeringPIDController.

Usage:
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05,
                        output_limits=(-1, 1), windup_limit=10)
    output = pid.compute(error, dt)
"""

import time


class PIDController:
    """Reusable PID controller with anti-windup and output clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limits: tuple = (-1.0, 1.0),
                 windup_limit: float = 10.0):
        self.kp            = kp
        self.ki            = ki
        self.kd            = kd
        self.output_limits = output_limits
        self.windup_limit  = windup_limit

        self.integral   = 0.0
        self.last_error = 0.0
        self.last_time  = None

    def compute(self, error: float, dt: float = None) -> float:
        """
        Compute PID output for the given error.

        Args:
            error : Current error value (setpoint - measurement).
            dt    : Time step in seconds. If None, measured internally.

        Returns:
            Clamped output value.
        """
        if dt is None:
            now = time.time()
            dt  = (now - self.last_time) if self.last_time else 0.01
            self.last_time = now

        P = self.kp * error

        self.integral += error * dt
        self.integral  = max(-self.windup_limit,
                             min(self.windup_limit, self.integral))
        I = self.ki * self.integral

        D = self.kd * ((error - self.last_error) / dt if dt > 0 else 0.0)
        self.last_error = error

        output = P + I + D
        return max(self.output_limits[0], min(self.output_limits[1], output))

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self.integral   = 0.0
        self.last_error = 0.0
        self.last_time  = None
