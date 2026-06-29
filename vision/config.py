"""
config.py  (vision)
-------------------
Calibrated HSV colour ranges and camera parameters for the WRO track.

Tweak HSV_RANGES if colours look wrong under your track lighting.
Measure FOCAL_LENGTH_PX once with a target at a known distance:
    focal_px = (pixel_width * real_width_cm) / distance_cm
"""

# HSV ranges: [H_low, S_low, V_low],  [H_high, S_high, V_high]
HSV_RANGES = {
    "green":   ([45,  140, 150], [60,  180, 190]),
    "red":     ([0,   100,  80], [10,  255, 255]),   # lower red hue band
    "red2":    ([120, 240, 190], [180, 255, 255]),   # upper red hue band (calibrated)
    "magenta": ([140,  80,  70], [160, 255, 255]),
}

# Real-world pillar width in cm (used for distance estimation)
REAL_WIDTH_CM = {
    "red":     5.0,
    "green":   5.0,
    "magenta": 5.0,
}

# Focal length in pixels — calibrated for this camera + lens
# Formula: focal_px = (pixel_width * real_width_cm) / known_distance_cm
FOCAL_LENGTH_PX = 417.56
