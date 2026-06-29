"""
imusensor.py
------------
MPU-6050 IMU sensor interface (I2C address 0x68).

Wiring:
    SDA → GPIO 2 (Pin 3)
    SCL → GPIO 3 (Pin 5)
    VCC → 3.3V or 5V

Requires:  pip install mpu6050-raspberrypi

The module-level `sensor` instance is imported by sensors.py.
"""

from mpu6050 import mpu6050

# Singleton sensor instance — import this object, don't instantiate again
sensor = mpu6050(0x68)
