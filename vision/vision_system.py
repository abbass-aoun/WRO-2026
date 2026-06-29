"""
vision_system.py
----------------
Camera capture and computer-vision detection pipeline for the WRO car.

Uses PiCamera2 for capture and OpenCV HSV segmentation to detect:
    - Red  traffic pillars  → TrafficSign(color="red")
    - Green traffic pillars → TrafficSign(color="green")
    - Magenta parking marker → ParkingLotMarker  (Lap 1 Side 1 only)

Object memory:
    Lap 1 : detects and registers up to 2 objects per side for 15 frames,
             then freezes the side registry.
    Lap 2+ : replays Lap 1 memory and updates geometry from live detections.

Usage:
    vision = Vision(debug=False)
    vision.init_camera_and_detect()          # call each loop iteration
    objects = vision.current_side_registry   # list of TrafficSign
    vision.next_side()                       # call when crossing a corner line
"""

import cv2
import numpy as np
import time
from picamera2 import Picamera2

from vision.traffic_sign import TrafficSign
from vision.parking      import ParkingLotMarker
from vision.config       import HSV_RANGES, REAL_WIDTH_CM, FOCAL_LENGTH_PX


class Vision:
    """Full vision pipeline: capture → detect → track → registry."""

    def __init__(self, debug: bool = False):
        self.debug        = debug
        self.lap_number   = 1
        self.side_number  = 1
        self.max_laps     = 3
        self.max_sides    = 4

        # Per-lap/side object registries
        self.lap_memory           = []   # list of lists (one per side, Lap 1)
        self.current_side_registry = []
        self.current_lap_registry  = []

        # State flags
        self.registry_done       = False
        self.freeze_processing   = False
        self.side_initialized    = False
        self.detection_frames    = 0
        self.detection_frame_limit = 15

        self.sight          = []
        self.parking_marker = None   # single magenta marker from Lap 1
        self.picam2         = None   # lazy-initialised camera

    # ── Public API ────────────────────────────────────────────────────────────

    def init_camera_and_detect(self) -> None:
        """
        Initialise PiCamera2 on first call, capture one frame, run detection.
        Call this once per main-loop iteration.
        """
        if self.picam2 is None:
            self.picam2 = Picamera2()
            self.picam2.preview_configuration.main.size   = (3300, 2500)
            self.picam2.preview_configuration.main.format = "RGB888"
            self.picam2.configure("preview")
            self.picam2.start()
            time.sleep(1)   # warm-up

        frame = self.picam2.capture_array()
        frame = cv2.flip(frame, 1)
        self.detect_obstacles(frame)

    def detect_obstacles(self, frame: np.ndarray) -> list:
        """
        Run one detection pass on the given BGR frame.

        Updates self.current_side_registry and self.parking_marker.
        Returns the current side registry list.
        """
        allow_magenta_only = self.lap_number in [2, 3] and self.side_number == 1

        if self.registry_done and not allow_magenta_only:
            if self.debug:
                self._overlay_status(frame, frozen=True)
            return []

        frame_h, frame_w = frame.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

        all_detections = self._detect_colored(hsv, frame_w, frame_h)

        # ── Lap 1: build registry ─────────────────────────────────────────────
        if not self.side_initialized and self.lap_number == 1:
            self._register_new_objects(all_detections, frame_w, frame_h)
            self.detection_frames += 1
            if self.detection_frames >= self.detection_frame_limit:
                self.side_initialized = True

        # ── All laps: update existing object geometry ─────────────────────────
        else:
            self._update_existing_objects(all_detections, frame_w, frame_h)

        # ── Magenta parking marker (Side 1 only) ──────────────────────────────
        self._detect_magenta(hsv, frame_w, frame_h, frame)

        # ── Lap 2+ memory update ──────────────────────────────────────────────
        if self.lap_number >= 2 and self.side_number <= len(self.lap_memory):
            self.update_memory_objects(all_detections, frame_w, frame_h)

        if self.debug:
            self._draw_debug(frame, frame_w, frame_h)

        return self.current_side_registry

    def next_side(self) -> None:
        """
        Advance to the next side/lap. Call when the colour sensor detects a
        corner line (orange or blue).
        """
        if self.registry_done:
            return

        print(f"[VISION] Finished side {self.side_number} of lap {self.lap_number}")
        self.current_lap_registry.append(self.current_side_registry[:])
        self.side_number += 1
        self._reset_side_registry()

        if self.side_number > self.max_sides:
            print(f"[VISION] Completed lap {self.lap_number}")
            if self.lap_number == 1:
                self.lap_memory       = self.current_lap_registry[:]
                self.freeze_processing = True
            self.lap_number         += 1
            self.side_number         = 1
            self.current_lap_registry = []

            if self.lap_number > self.max_laps:
                print("[VISION] All 3 laps done — vision processing stopped.")
                self.registry_done = True

    def update_memory_objects(self, new_detections: list,
                               frame_w: int, frame_h: int) -> None:
        """Update Lap-1 memory objects with fresh geometry from current detections."""
        if self.lap_number < 2 or self.side_number > len(self.lap_memory):
            return
        for obj in self.lap_memory[self.side_number - 1]:
            obj.alive = False
            for det in new_detections:
                if det["color"] != obj.color:
                    continue
                if abs(det["x_px"] - obj.x_px) < 50 and abs(det["y_px"] - obj.y_px) < 50:
                    obj.update_position(det["x_px"], det["y_px"],
                                        det["bbox"], det["bbox_width_px"])
                    obj.calculate_geometry(frame_w, frame_h,
                                           REAL_WIDTH_CM[det["color"]], FOCAL_LENGTH_PX)
                    obj.alive = True
                    break

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _reset_side_registry(self):
        self.current_side_registry = []
        self.side_initialized      = False
        self.detection_frames      = 0

    def _detect_colored(self, hsv: np.ndarray,
                         frame_w: int, frame_h: int) -> list:
        """Detect red and green objects; return list of detection dicts."""
        detections = []
        kernel = np.ones((5, 5), np.uint8)
        for color in ("red", "red2", "green"):
            lo, hi = HSV_RANGES[color]
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(cnt) < 1000:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy     = x + w // 2, y + h // 2
                label      = "red" if "red" in color else "green"
                detections.append({
                    "color":        label,
                    "x_px":         cx,
                    "y_px":         cy,
                    "bbox":         (x, y, w, h),
                    "bbox_width_px": w,
                    "distance_cm":  (REAL_WIDTH_CM[label] * FOCAL_LENGTH_PX) / w,
                })
        return detections

    def _register_new_objects(self, detections: list,
                               frame_w: int, frame_h: int) -> None:
        """Add newly seen objects to the side registry (up to 2)."""
        for det in detections:
            if len(self.current_side_registry) >= 2:
                break
            matched = any(
                abs(det["x_px"] - o.x_px) < 50 and
                abs(det["y_px"] - o.y_px) < 50 and
                det["color"] == o.color
                for o in self.current_side_registry
            )
            if not matched:
                obj = TrafficSign(det["color"], det["x_px"], det["y_px"],
                                  det["bbox"], det["bbox_width_px"])
                obj.calculate_geometry(frame_w, frame_h,
                                       REAL_WIDTH_CM[det["color"]], FOCAL_LENGTH_PX)
                self.current_side_registry.append(obj)

    def _update_existing_objects(self, detections: list,
                                  frame_w: int, frame_h: int) -> None:
        """Update geometry of already-registered objects."""
        for obj in self.current_side_registry:
            obj.alive = False
            for det in detections:
                if (det["color"] == obj.color and
                        abs(det["x_px"] - obj.x_px) < 50 and
                        abs(det["y_px"] - obj.y_px) < 50):
                    obj.update_position(det["x_px"], det["y_px"],
                                        det["bbox"], det["bbox_width_px"])
                    obj.calculate_geometry(frame_w, frame_h,
                                           REAL_WIDTH_CM[det["color"]], FOCAL_LENGTH_PX)
                    obj.alive = True
                    break

    def _detect_magenta(self, hsv: np.ndarray,
                         frame_w: int, frame_h: int, frame: np.ndarray) -> None:
        """Detect and track the magenta parking marker (Side 1 only)."""
        if self.side_number != 1:
            return
        lo, hi = HSV_RANGES["magenta"]
        mask   = cv2.inRange(hsv, np.array(lo), np.array(hi))
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(cnt) < 1000:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            if self.parking_marker is None and self.lap_number == 1:
                self.parking_marker = ParkingLotMarker(cx, cy, (x, y, w, h),
                                                       w, self.lap_number)
            elif self.parking_marker:
                self.parking_marker.update_position(cx, cy, (x, y, w, h), w)
            if self.parking_marker:
                self.parking_marker.calculate_geometry(
                    frame_w, frame_h, REAL_WIDTH_CM["magenta"], FOCAL_LENGTH_PX)
            break  # only one magenta marker per frame

    def _draw_debug(self, frame: np.ndarray,
                    frame_w: int, frame_h: int) -> None:
        """Draw bounding boxes and labels on frame when debug=True."""
        for obj in self.current_side_registry:
            x, y, w, h = obj.bbox
            bgr = (0, 0, 255) if obj.color == "red" else (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
            cv2.putText(frame, f"{obj.color.upper()} {obj.distance_cm:.1f}cm",
                        (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
            cv2.putText(frame, f"Angle: {obj.angle_horizontal:+.1f}",
                        (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
        self._overlay_status(frame)

    def _overlay_status(self, frame: np.ndarray, frozen: bool = False) -> None:
        tag = " (Frozen)" if frozen else ""
        cv2.putText(frame,
                    f"Lap: {self.lap_number}/3 | Side: {self.side_number}/4{tag}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255) if frozen else (255, 255, 0), 2)
