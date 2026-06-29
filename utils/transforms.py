"""
transforms.py
-------------
2-D coordinate frame transformation utilities.

The robot operates in three nested frames:
    1. Global frame     — fixed world origin at start position.
    2. Local reference  — updated at the start of each section.
    3. Robot FPV frame  — camera / robot body frame (x=right, y=forward).

All angles in radians internally; accept/return lists [x, y, theta].
"""

import numpy as np
from typing import List


# ── Frame transforms ──────────────────────────────────────────────────────────

def local_to_global(local_point: np.ndarray, ref_pose: list) -> np.ndarray:
    """
    Transform a 2-D point from the local reference frame to the global frame.

    Args:
        local_point : np.ndarray [x, y] in the local frame (cm).
        ref_pose    : [x0, y0, theta] — local frame origin in global frame.

    Returns:
        np.ndarray [x, y] in the global frame.
    """
    ox, oy, theta = ref_pose
    R = _rotation(theta)
    return np.array([ox, oy]) + R @ local_point


def global_to_local(global_point: np.ndarray, ref_pose: list) -> np.ndarray:
    """
    Transform a 2-D point from the global frame to the local reference frame.
    """
    ox, oy, theta = ref_pose
    R_inv = _rotation(-theta)
    return R_inv @ (global_point - np.array([ox, oy]))


def transform_pillars_to_local(pillars: list, ref_pose: list) -> list:
    """
    Return a new list of TrafficSign objects with x_cm, y_cm expressed
    in the local reference frame instead of the robot FPV frame.

    Args:
        pillars  : List of TrafficSign objects (x_cm, y_cm in robot frame).
        ref_pose : [x0, y0, theta] — robot pose in local frame.

    Returns:
        New list of TrafficSign objects (shallow-copied) with updated coords.
    """
    x0, y0, theta = ref_pose
    R = _rotation(-theta)
    result = []
    for p in pillars:
        dx, dy = p.x_cm - x0, p.y_cm - y0
        x_local, y_local = R @ np.array([dx, dy])
        new_p = _copy_sign(p)
        new_p.x_cm = x_local
        new_p.y_cm = y_local
        result.append(new_p)
    return result


def transform_pillars_to_global(pillars: list, ref_pose: list) -> list:
    """
    Return a new list of TrafficSign objects with x_cm, y_cm expressed
    in the global frame.
    """
    x0, y0, theta = ref_pose
    R = _rotation(theta)
    result = []
    for p in pillars:
        x_g, y_g = R @ np.array([p.x_cm, p.y_cm]) + np.array([x0, y0])
        new_p = _copy_sign(p)
        new_p.x_cm = x_g
        new_p.y_cm = y_g
        result.append(new_p)
    return result


# ── Trajectory helpers ────────────────────────────────────────────────────────

def find_nearest_point(traj_points: np.ndarray,
                       robot_pos: list,
                       radius: float) -> int | None:
    """
    Find the index of the trajectory point nearest to robot_pos within radius.

    Args:
        traj_points : (N, 2) array of waypoints.
        robot_pos   : [x, y, theta] current pose.
        radius      : Search radius in cm.

    Returns:
        Index of nearest point, or None if none found within radius.
    """
    pos   = np.array(robot_pos[:2])
    diffs = traj_points - pos
    dists_sq = np.einsum("ij,ij->i", diffs, diffs)
    mask  = dists_sq <= radius ** 2
    if not np.any(mask):
        return None
    indices     = np.where(mask)[0]
    min_sub_idx = np.argmin(dists_sq[mask])
    return int(indices[min_sub_idx])


def compute_tracking_error(trajectory: tuple, robot_pose: list,
                           radius: float = 10.0) -> dict | None:
    """
    Compute lateral and heading error between robot and nearest trajectory point.

    Args:
        trajectory  : (points, steering, speeds) tuple from TrajectoryBuilder.
        robot_pose  : [x, y, theta] current pose.
        radius      : Nearest-point search radius (cm).

    Returns:
        Dict with keys lateral_error, heading_error, target_speed, target_steer.
        None if no valid point found.
    """
    points, steering, speeds = trajectory[0], trajectory[1], trajectory[2]
    ix = find_nearest_point(points, robot_pose, radius)

    if ix is None or ix >= len(points) - 1:
        return None

    target     = points[ix]
    next_pt    = points[ix + 1]
    traj_vec   = next_pt - target
    traj_theta = np.arctan2(traj_vec[1], traj_vec[0])

    x, y, theta = robot_pose
    err_vec        = target - np.array([x, y])
    heading_vec    = np.array([np.cos(theta), np.sin(theta)])
    lateral_error  = float(np.cross(heading_vec, err_vec))
    heading_error  = float(np.arctan2(np.sin(traj_theta - theta),
                                      np.cos(traj_theta - theta)))

    return {
        "lateral_error":  lateral_error,
        "heading_error":  heading_error,
        "target_speed":   float(speeds[ix]),
        "target_steer":   float(steering[ix]),
    }


def section_end_direction(section_nb: int, direction: int) -> np.ndarray:
    """
    Return the expected robot heading unit vector at the end of a section.

    Args:
        section_nb : 1-4 (current track section).
        direction  : 0 = clockwise (CW), 1 = counter-clockwise (CCW).

    Returns:
        np.ndarray unit vector [dx, dy].
    """
    cw_dirs  = {1: [1, 0], 2: [0, -1], 3: [-1, 0], 4: [0, 1]}
    ccw_dirs = {1: [1, 0], 2: [0,  1], 3: [-1, 0], 4: [0, -1]}
    dirs     = ccw_dirs if direction == 1 else cw_dirs
    return np.array(dirs.get(section_nb, [0, 1]), dtype=float)


# ── Private helpers ───────────────────────────────────────────────────────────

def _rotation(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _copy_sign(p):
    """Shallow-copy a TrafficSign (avoids circular import)."""
    from vision.vision_system import TrafficSign
    new_p = TrafficSign(p.color, p.x_px, p.y_px, p.bbox, p.bbox_width_px)
    new_p.section_nb = p.section_nb
    return new_p
