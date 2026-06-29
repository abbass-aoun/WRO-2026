"""
main.py
-------
WRO 2026 Autonomous Car — main control loop.

Start sequence:
    1. System initialises sensors, camera, and controllers.
    2. Robot waits for the physical start button to be pressed.
    3. Main loop runs until 3 laps are completed or 'q' is pressed.

Run on the Raspberry Pi:
    python3 main.py
"""

import time
import keyboard
import numpy as np
from gpiozero import Button

from config          import Pins, TrackParams, TrajParams
from control.car_controller    import CarController
from control.stop_pid          import StopPID
from control.trajectory_builder import TrajectoryBuilder
from sensors.sensors            import Sensors
from vision.vision_system       import Vision
from localization.ekf_estimator import EKFEstimator
from utils.transforms           import (
    find_nearest_point,
    compute_tracking_error,
    transform_pillars_to_local,
    transform_pillars_to_global,
    section_end_direction,
)


# ─────────────────────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────────────────────

class RobotState:
    """All mutable state in one place — replaces scattered globals."""

    def __init__(self):
        # Position [x_cm, y_cm, theta_rad]
        self.current_pose        = [0.0, 0.0, 0.0]
        self.local_reference     = [0.0, 0.0, 0.0]

        # Race progress
        self.lap_nb              = 1
        self.section_nb          = 1
        self.direction           = 0          # 0=CW, 1=CCW
        self.in_corner           = False
        self.new_section         = True
        self.final_section       = False
        self.time_section_update = time.time()

        # Detected objects
        self.pillars             = []         # TrafficSign list
        self.parking_exists      = False
        self.parking_location    = [0, 0, 0]

        # Planned path
        self.trajectory          = None       # (points, steering, speeds)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def process_data(state: RobotState, vision: Vision, sensors: Sensors) -> None:
    """Update state with the latest sensor and camera readings."""
    vision.capture_and_detect()

    state.lap_nb     = vision.lap_number
    state.section_nb = vision.side_number
    state.pillars    = []

    for obj in vision.current_side_registry:
        if obj.color in ("red", "green") and obj.x_cm is not None:
            obj.section_nb = state.section_nb
            state.pillars.append(obj)

    if vision.parking_marker and vision.side_number == 1:
        state.parking_exists = True
        pm = vision.parking_marker
        state.parking_location = [state.section_nb, pm.x_cm, pm.y_cm]


def calculate_trajectory(state: RobotState, builder: TrajectoryBuilder) -> None:
    """Build a new Bézier trajectory from current pillar detections."""
    forward_pillars = [
        p for p in state.pillars if p.y_cm is not None and p.y_cm > 0
    ]
    forward_pillars.sort(key=lambda p: np.hypot(p.x_cm, p.y_cm))
    selected = forward_pillars[:3]

    end_dir = section_end_direction(state.section_nb, state.direction)
    start_dir = np.array([np.cos(state.current_pose[2]),
                          np.sin(state.current_pose[2])])

    if selected:
        pts, steer, spd = builder.build(selected, start_dir, end_dir)
    else:
        pts, steer, spd = builder.build([], start_dir, end_dir)

    state.trajectory = (pts, steer, spd)


def take_step(state: RobotState, drive: CarController) -> tuple:
    """Command the robot one step along the stored trajectory."""
    if state.trajectory is None:
        drive.stop()
        return "f", 0, 0

    pts, steer, spd = state.trajectory
    ix = find_nearest_point(pts, state.current_pose, TrajParams.SEARCH_RADIUS_CM)

    if ix is None or ix >= len(pts) - 1:
        drive.stop()
        return "f", 0, 0

    target_steer = float(steer[ix])
    target_speed = float(spd[ix])

    # Determine forward vs backward from dot product with trajectory tangent
    traj_vec  = pts[ix + 1] - pts[ix]
    robot_vec = pts[ix + 1] - np.array(state.current_pose[:2])
    direction = "f" if np.dot(traj_vec, robot_vec) > 0 else "b"

    drive.set_all(direction, target_speed, target_steer)
    return direction, target_speed, target_steer


def advance_section(state: RobotState) -> None:
    """Increment section/lap counters and check for race completion."""
    if state.section_nb == 4 and state.lap_nb < TrackParams.TOTAL_LAPS:
        state.section_nb = 1
        state.lap_nb    += 1
    elif state.section_nb < 4:
        state.section_nb += 1
    else:
        state.final_section = True


def stop_robot(drive: CarController, sensors: Sensors) -> None:
    """Bring the robot to a controlled halt using StopPID."""
    pid  = StopPID()
    enc_l_prev, enc_r_prev = sensors.read_encoders()
    t_prev = time.time()

    for _ in range(20):
        time.sleep(0.05)
        enc_l, enc_r = sensors.read_encoders()
        dt = time.time() - t_prev

        v_l, v_r = sensors.encoder_to_speed_cms(
            enc_l - enc_l_prev, enc_r - enc_r_prev, dt
        )
        cmd = pid.compute(v_l, v_r, dt)
        direction = "f" if cmd > 0 else "b"
        drive.set_all(direction, abs(cmd), 0)

        enc_l_prev, enc_r_prev = enc_l, enc_r
        t_prev = time.time()

        if abs(v_l) < 0.5 and abs(v_r) < 0.5:
            break

    drive.stop()


def park(drive: CarController) -> None:
    """
    Execute parking manoeuvre.
    TODO: implement based on parking_location coordinates.
    """
    print("[PARK] Parking manoeuvre not yet implemented.")
    drive.stop()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=== WRO 2026 Autonomous Car ===")
    print("Initialising systems...")

    state   = RobotState()
    drive   = CarController()
    builder = TrajectoryBuilder()
    ekf     = EKFEstimator()
    sensors = Sensors()
    vision  = Vision()
    btn     = Button(Pins.START_BUTTON)

    print("Systems ready. Waiting for start button...")
    btn.wait_for_press()
    print("Button pressed — starting race loop.")

    prev_enc_l, prev_enc_r = sensors.read_encoders()
    prev_time = time.time()

    try:
        while True:
            # ── Sensor update ─────────────────────────────────────────────────
            process_data(state, vision, sensors)
            floor_color = sensors.read_color()

            # ── Odometry + EKF ────────────────────────────────────────────────
            now    = time.time()
            dt     = max(now - prev_time, 1e-4)
            prev_time = now

            enc_l, enc_r = sensors.read_encoders()
            d_enc_l = enc_l - prev_enc_l
            d_enc_r = enc_r - prev_enc_r
            prev_enc_l, prev_enc_r = enc_l, enc_r

            v_l, v_r = sensors.encoder_to_speed_cms(d_enc_l, d_enc_r, dt)
            speed_avg = 0.5 * (v_l + v_r)

            # EKF update (imu_yaw_rate = 0 until IMU is wired up)
            state.current_pose = ekf.update(
                state.current_pose, dt, speed_avg,
                steer_deg=0, enc_right=d_enc_r, enc_left=d_enc_l,
                steer_enc=0, imu_yaw_rate=0
            )

            # ── Trajectory: build or reuse ─────────────────────────────────
            if state.trajectory is None:
                calculate_trajectory(state, builder)
            else:
                err = compute_tracking_error(
                    state.trajectory, state.current_pose,
                    TrajParams.SEARCH_RADIUS_CM
                )
                if err and abs(err["lateral_error"]) > TrackParams.TRACKING_ERROR_THRESHOLD:
                    calculate_trajectory(state, builder)

            # ── Drive ─────────────────────────────────────────────────────────
            take_step(state, drive)

            # ── Corner / section detection via floor colour ────────────────────
            debounce = TrackParams.SECTION_DEBOUNCE_S
            elapsed  = time.time() - state.time_section_update

            if floor_color == "blue" and elapsed > debounce:
                if state.new_section:
                    state.direction      = 1       # CCW
                    state.new_section    = False
                    state.in_corner      = True
                else:
                    state.new_section    = True
                    state.in_corner      = False
                    advance_section(state)
                state.time_section_update = time.time()

            elif floor_color == "orange" and elapsed > debounce:
                if state.new_section:
                    state.direction      = 0       # CW
                    state.new_section    = False
                    state.in_corner      = True
                else:
                    state.new_section    = True
                    state.in_corner      = False
                    advance_section(state)
                state.time_section_update = time.time()

            # ── Race completion ────────────────────────────────────────────────
            if state.final_section:
                print(f"Final section reached after {state.lap_nb} laps.")
                if state.parking_exists:
                    park(drive)
                else:
                    stop_robot(drive, sensors)
                break

            # ── Emergency quit (keyboard) ─────────────────────────────────────
            if keyboard.is_pressed("q"):
                print("Manual quit.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        drive.stop()
        vision.release()
        sensors.cleanup()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
