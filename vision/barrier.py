"""
barrier.py
----------
Represents a black barrier/wall detected by the vision system.

Classifies the barrier as left, right, or front based on its
horizontal pixel position in the frame.
"""


class Barrier:
    """A black barrier detected in the camera frame."""

    def __init__(self, x_px: float, y_px: float, bbox_width_px: float,
                 frame_width: int, distance_cm: float):
        self.color        = "black"
        self.distance_cm  = distance_cm
        self.x_px         = x_px
        self.bbox_width_px = bbox_width_px

        # Classify left / front / right by horizontal position
        if x_px < frame_width / 3:
            self.position = "left"
        elif x_px > 2 * frame_width / 3:
            self.position = "right"
        else:
            self.position = "front"
