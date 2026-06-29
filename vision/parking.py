"""
parking.py
----------
Represents the magenta parking-lot marker detected during Lap 1.

Stores its position for use in the parking manoeuvre at the end of the race.
"""

from vision.field_object import FieldObject


class ParkingLotMarker(FieldObject):
    """Magenta marker indicating the parking zone."""

    def __init__(self, x_px: float, y_px: float, bbox: tuple,
                 bbox_width_px: float, loop_count: int):
        super().__init__("magenta", x_px, y_px, bbox, bbox_width_px)
        self.marker_type  = "parking"
        self.seen_at_loop = loop_count
        print(f"[PARKING] Marker detected at loop {loop_count}")
