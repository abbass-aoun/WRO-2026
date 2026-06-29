"""
stop_pid.py
-----------
PID controller that drives both rear wheels to a halt.

Usage:
    pid = StopPID()
    while not stopped:
        command = pid.compute(left_speed, right_speed, dt)
        car.set_motor(command)
"""

from config import PIDGains


class StopPID:
    """Symmetric PID controller for braking both wheels to zero speed."""

    def __init__(
        self,
        kp: float = PIDGains.STOP_KP,
        ki: float = PIDGains.STOP_KI,
        kd: float = PIDGains.STOP_KD,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral  = 0.0
        self._last_error = 0.0

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self._integral   = 0.0
        self._last_error = 0.0

    def compute(self, v_left: float, v_right: float, dt: float) -> float:
        """
        Compute a motor command that decelerates both wheels toward zero.

        Args:
            v_left:  Current left-wheel speed  (cm/s, positive = forward).
            v_right: Current right-wheel speed (cm/s, positive = forward).
            dt:      Time elapsed since last call (seconds).

        Returns:
            Motor command magnitude (float). Positive = brake forward motion,
            negative = brake reverse motion. Pass to CarController.set_motor().
        """
        if dt <= 0:
            return 0.0

        v_avg  = 0.5 * (v_left + v_right)
        error  = -v_avg                          # target speed is 0

        self._integral  += error * dt
        d_error          = (error - self._last_error) / dt
        self._last_error = error

        return self.kp * error + self.ki * self._integral + self.kd * d_error
