"""
encoder_test.py
---------------
Standalone script to verify wheel encoder wiring and pulse detection.
Run directly on the Raspberry Pi; rotate each wheel by hand to test.

Usage:
    python3 sensors/encoder_test.py
"""

from gpiozero import Button
from signal import pause
from config import Pins

left_count  = 0
right_count = 0

left_sensor  = Button(Pins.ENCODER_LEFT,  pull_up=True)
right_sensor = Button(Pins.ENCODER_RIGHT, pull_up=True)


def on_left_pulse():
    global left_count
    left_count += 1
    print(f"[LEFT ] Pulse #{left_count}")


def on_right_pulse():
    global right_count
    right_count += 1
    print(f"[RIGHT] Pulse #{right_count}")


left_sensor.when_pressed  = on_left_pulse
right_sensor.when_pressed = on_right_pulse

print("Encoder test running — rotate wheels to generate pulses.")
print("Press Ctrl+C to stop.\n")

pause()
