"""
steering_pid_controller.py
--------------------------
PID controller for lateral (steering) control.

Computes a steering angle command based on the robot's heading error
relative to the desired trajectory tangent.

TODO: implement compute_steering_error() using trajectory nearest-point logic.
"""

from control.pid_controller import PIDController


class SteeringPIDController(PIDController):
    """Lateral PID — controls front steering servo angle."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1,
                 output_limits: tuple = (-30.0, 30.0),
                 windup_limit: float = 10.0):
        super().__init__(kp, ki, kd, output_limits, windup_limit)

    def compute_steering_error(self, curr_x: float, curr_y: float,
                               curr_theta: float, trajectory, par_s: float) -> float:
        """
        Compute lateral steering error.

        TODO: find nearest trajectory point, return heading delta.
        """
        return 0.0   # stub

    def compute(self, curr_x: float, curr_y: float, curr_theta: float,
                trajectory, par_s: float) -> float:
        """Compute steering angle command from trajectory error."""
        error = self.compute_steering_error(curr_x, curr_y, curr_theta,
                                             trajectory, par_s)
        return super().compute(error)
