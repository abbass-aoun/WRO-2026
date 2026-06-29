"""
driving_pid_controller.py
-------------------------
PID controller for longitudinal (speed) control.

Computes a motor speed command based on the robot's distance error
from the desired trajectory point.

TODO: implement compute_position_error() using trajectory nearest-point logic.
"""

from control.pid_controller import PIDController


class DrivingPIDController(PIDController):
    """Longitudinal PID — controls rear-wheel motor speed."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1,
                 output_limits: tuple = (-1.0, 1.0),
                 windup_limit: float = 10.0):
        super().__init__(kp, ki, kd, output_limits, windup_limit)

    def compute_position_error(self, curr_x: float, curr_y: float,
                               curr_theta: float, trajectory, par_s: float) -> float:
        """
        Compute longitudinal position error.

        TODO: find nearest trajectory point and return distance along path.
        """
        return 0.0   # stub

    def compute(self, curr_x: float, curr_y: float, curr_theta: float,
                trajectory, par_s: float) -> float:
        """Compute motor speed command from trajectory error."""
        error = self.compute_position_error(curr_x, curr_y, curr_theta,
                                            trajectory, par_s)
        return super().compute(error)
