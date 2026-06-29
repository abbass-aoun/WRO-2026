"""
vision_system.py
----------------
Camera capture and computer-vision detection pipeline.

Detects:
    - Red and green traffic pillars (coloured cylinders on the track)
    - Magenta parking markers (Challenge round only)

Each detected object is returned as a TrafficSign instance with:
    .color        — "red" | "green" | "magenta"
    .x_cm         — lateral distance from robot centre (cm, right = positive)
    .y_cm         — forward distance from robot (cm)
    .x_px, .y_px  — pixel centroid in the frame
    .bbox         — (x, y, w, h) bounding box in pixels
    .bbox_width_px— bounding box width (used for distance estimation)

Usage:
    vision = Vision()
    vision.capture_and_detect()
    for obj in vision.detections:
        print(obj.color, obj.x_cm, obj.y_cm)
    vision.release()
"""

import cv2
import numpy as np
from config import VisionParams


# ── Data class ────────────────────────────────────────────────────────────────

class TrafficSign:
    """Represents one detected object in the scene."""

    def __init__(self, color: str, x_px: float, y_px: float,
                 bbox: tuple, bbox_width_px: float):
        self.color         = color
        self.x_px          = x_px
        self.y_px          = y_px
        self.bbox          = bbox          # (x, y, w, h)
        self.bbox_width_px = bbox_width_px
        self.x_cm: float | None = None    # filled by depth estimation
        self.y_cm: float | None = None
        self.section_nb: int    = 0        # filled by main loop


# ── Vision pipeline ───────────────────────────────────────────────────────────

class Vision:
    """Wraps camera capture and HSV colour segmentation."""

    # HSV ranges (H: 0-179, S: 0-255, V: 0-255 in OpenCV)
    _RANGES = {
        "red":     [(np.array([0,  120, 70]),  np.array([10,  255, 255])),
                    (np.array([170, 120, 70]),  np.array([180, 255, 255]))],
        "green":   [(np.array([40,  70,  70]),  np.array([80,  255, 255]))],
        "magenta": [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
    }

    MIN_CONTOUR_AREA = 500   # px² — ignore tiny blobs

    def __init__(self):
        self.cap              = None
        self.detections: list = []        # TrafficSign objects from latest frame
        self.parking_marker   = None      # TrafficSign | None
        self.lap_number: int  = 1
        self.side_number: int = 1
        self.current_side_registry: list = []

        # Camera intrinsics for distance estimation
        self._focal_px    = VisionParams.FOCAL_LENGTH_PX
        self._pillar_w_cm = VisionParams.PILLAR_WIDTH_CM
        self._frame_cx    = VisionParams.CAMERA_WIDTH / 2.0

        self._open_camera()

    # ── Public API ────────────────────────────────────────────────────────────

    def capture_and_detect(self) -> list:
        """
        Grab one frame, run detection, update self.detections.

        Returns:
            List of TrafficSign objects detected in this frame.
        """
        frame = self._grab_frame()
        if frame is None:
            return []

        self.detections      = []
        self.parking_marker  = None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

        for color, ranges in self._RANGES.items():
            mask = self._build_mask(blurred, ranges)
            signs = self._detect_objects(mask, color, frame)
            if color == "magenta" and signs:
                self.parking_marker = signs[0]
            else:
                self.detections.extend(signs)

        self.current_side_registry = self.detections[:]
        return self.detections

    # Legacy name used by Super_main.py
    def init_camera_and_detect(self):
        self.capture_and_detect()

    def release(self):
        """Release the camera resource."""
        if self.cap and self.cap.isOpened():
            self.cap.release()

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _open_camera(self):
        """Open the first available camera."""
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  VisionParams.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VisionParams.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          VisionParams.CAMERA_FPS)

    def _grab_frame(self):
        """Grab and return one BGR frame, or None on failure."""
        if not self.cap or not self.cap.isOpened():
            self._open_camera()
        ret, frame = self.cap.read()
        return frame if ret else None

    @staticmethod
    def _build_mask(hsv: np.ndarray, ranges: list) -> np.ndarray:
        """Combine one or more HSV ranges into a single binary mask."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        # Morphological clean-up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _detect_objects(self, mask: np.ndarray, color: str,
                        frame: np.ndarray) -> list:
        """Find contours and convert each to a TrafficSign with cm coords."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w / 2.0
            cy = y + h / 2.0

            sign = TrafficSign(color, cx, cy, (x, y, w, h), w)
            sign.x_cm, sign.y_cm = self._estimate_position(cx, w)
            results.append(sign)

        # Sort nearest-first
        results.sort(key=lambda s: s.y_cm if s.y_cm else float("inf"))
        return results

    def _estimate_position(self, cx_px: float, width_px: float) -> tuple:
        """
        Estimate object position relative to robot using the pinhole model.

        Args:
            cx_px    : horizontal pixel centroid of the bounding box
            width_px : pixel width of the bounding box

        Returns:
            (x_cm, y_cm) — lateral and forward distance in centimetres.
            x_cm: negative = left of robot, positive = right.
            y_cm: forward distance (always positive).
        """
        if width_px < 1:
            return None, None

        y_cm = (self._focal_px * self._pillar_w_cm) / width_px
        x_cm = (cx_px - self._frame_cx) * y_cm / self._focal_px
        return round(x_cm, 1), round(y_cm, 1)
