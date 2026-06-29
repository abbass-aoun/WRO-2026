"""
colorsensor.py
--------------
TCS3200 colour sensor driver.

Outputs a frequency proportional to the intensity of each colour channel.
Uses GPIO interrupts via gpiozero to count transitions over a fixed window.

Wiring (BCM pins):
    S0 → GPIO 17,  S1 → GPIO 27
    S2 → GPIO 22,  S3 → GPIO 23   ← NOTE: original had S2=S3=GPIO9 (typo fixed)
    OUT → GPIO 9

Calibration (update FREQ_R/G/B for your lighting):
    MIN = frequency under black (low reflectivity)
    MAX = frequency under white (high reflectivity)

Usage:
    r, g, b = get_rgb()
    color   = classify_color(r, g, b)   # "red" | "green" | "blue" | "orange" | ...
    # Or use the high-level wrapper:
    color   = detect_main_color()
"""

from gpiozero import DigitalOutputDevice, DigitalInputDevice
import time

# GPIO pins
S0  = DigitalOutputDevice(17)
S1  = DigitalOutputDevice(27)
S2  = DigitalOutputDevice(22)   # fixed from original (was GPIO 9, same as OUT)
S3  = DigitalOutputDevice(23)
OUT = DigitalInputDevice(9)

# 20% output frequency scaling
S0.on()
S1.off()

# Calibration — measure under black (min) and white (max) surfaces
FREQ_R = {"min": 4431,  "max": 30908}
FREQ_G = {"min": 1789,  "max": 30504}
FREQ_B = {"min": 2592,  "max": 22133}


def map_freq(f: float, fmin: float, fmax: float) -> int:
    """Map raw frequency to 0–255 intensity (higher = more reflective)."""
    f = min(max(f, fmin), fmax)
    return int((fmax - f) * 255 / (fmax - fmin))


def read_frequency(s2_val: bool, s3_val: bool, sample_ms: int = 100) -> float:
    """
    Read the output frequency for one colour channel.

    Args:
        s2_val, s3_val : Channel select bits (see TCS3200 datasheet).
        sample_ms      : Counting window in milliseconds.

    Returns:
        Frequency in Hz.
    """
    S2.value = s2_val
    S3.value = s3_val
    time.sleep(0.01)   # settle

    count = 0
    end   = time.time() + sample_ms / 1000.0
    last  = OUT.value
    while time.time() < end:
        val = OUT.value
        if val != last:
            count += 1
            last = val
    return count / 2.0 / (sample_ms / 1000.0)   # transitions → Hz


def get_rgb() -> tuple:
    """Return (R, G, B) as 0–255 intensity values."""
    rf = read_frequency(False, False)
    gf = read_frequency(True,  True)
    bf = read_frequency(False, True)
    return (
        map_freq(rf, FREQ_R["min"], FREQ_R["max"]),
        map_freq(gf, FREQ_G["min"], FREQ_G["max"]),
        map_freq(bf, FREQ_B["min"], FREQ_B["max"]),
    )


def classify_color(r: int, g: int, b: int) -> str:
    """Classify floor colour from RGB intensities."""
    brightness = r + g + b
    if brightness < 100:
        return "black"
    if brightness > 700:
        return "white"
    if r > g and r > b:
        return "red"
    if g > r and g > b:
        return "green"
    if b > r and b > g:
        return "blue"
    return "unknown"


def detect_main_color() -> str:
    """One-call convenience wrapper."""
    return classify_color(*get_rgb())
