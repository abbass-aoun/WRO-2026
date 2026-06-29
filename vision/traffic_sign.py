"""
traffic_sign.py
---------------
Represents a coloured traffic pillar on the WRO track.

Red  pillar → robot must pass to the RIGHT.
Green pillar → robot must pass to the LEFT.
"""

from vision.field_object import FieldObject


class TrafficSign(FieldObject):
    """A red or green traffic pillar detected by the vision system."""

    def __init__(self, color: str, x_px: float, y_px: float,
                 bbox: tuple, bbox_width_px: float):
        super().__init__(color, x_px, y_px, bbox, bbox_width_px)
        self.section_nb = 0   # filled by main loop

        if color == "red":
            self.direction = "right"   # pass to the right
        elif color == "green":
            self.direction = "left"    # pass to the left
        else:
            self.direction = "unknown"
