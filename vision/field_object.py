"""
field_object.py
---------------
Base class for all detected objects on the WRO track.

Subclassed by TrafficSign (red/green pillars) and ParkingLotMarker (magenta).
Provides pixel-to-cm geometry calculation using the pinhole camera model.
"""

import uuid
import numpy as np


class FieldObject:
    """Represents a single detected object in the camera frame."""

    # Class-level counters (shared across all instances)
    live_objects = 0
    all_objects  = 0

    def __init__(self, color: str, x_px: float, y_px: float,
                 bbox: tuple, bbox_width_px: float):
        self.id             = uuid.uuid4()
        self.color          = color
        self.x_px           = x_px
        self.y_px           = y_px
        self.bbox           = bbox            # (x, y, w, h) in pixels
        self.bbox_width_px  = bbox_width_px
        self.alive          = True

        # Real-world geometry (filled by calculate_geometry)
        self.x_cm               = None   # lateral offset from robot centre (cm)
        self.y_cm               = None   # forward distance (cm)
        self.distance_cm        = None   # alias for y_cm
        self.angle_horizontal   = None   # degrees left/right
        self.angle_vertical     = None   # degrees up/down
        self.initial_x_cm       = None
        self.initial_y_cm       = None
        self.initial_set        = False

        FieldObject.live_objects += 1
        FieldObject.all_objects  += 1

    # ── Geometry ──────────────────────────────────────────────────────────────

    def calculate_geometry(self, frame_width: int, frame_height: int,
                           real_width_cm: float, focal_length_px: float) -> None:
        """
        Compute real-world position from bounding-box pixel measurements.

        Uses the pinhole camera model:
            distance_cm = (real_width_cm * focal_length_px) / bbox_width_px
            x_cm        = (pixel_offset_x * distance_cm) / focal_length_px
        """
        cx = frame_width  // 2
        cy = frame_height // 2

        offset_x = self.x_px - cx
        offset_y = self.y_px - cy

        self.distance_cm      = (real_width_cm * focal_length_px) / self.bbox_width_px
        self.x_cm             = (offset_x * self.distance_cm) / focal_length_px
        self.y_cm             = self.distance_cm
        self.angle_horizontal = np.degrees(np.arctan2(self.x_cm, self.y_cm))
        self.angle_vertical   = np.degrees(np.arctan2(offset_y, focal_length_px))

        if not self.initial_set:
            self.initial_x_cm = self.x_cm
            self.initial_y_cm = self.y_cm
            self.initial_set  = True

    # ── State updates ─────────────────────────────────────────────────────────

    def update_position(self, x_px: float, y_px: float,
                        bbox: tuple, bbox_width_px: float) -> None:
        """Refresh pixel position from new detection."""
        self.x_px          = x_px
        self.y_px          = y_px
        self.bbox          = bbox
        self.bbox_width_px = bbox_width_px
        self.alive         = True

    def mark_dead(self) -> None:
        """Mark object as no longer visible."""
        if self.alive:
            self.alive = False
            FieldObject.live_objects -= 1
