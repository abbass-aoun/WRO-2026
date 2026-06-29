"""
alldistance.py
--------------
VL53L0X Time-of-Flight distance sensor driver using raw I2C (smbus2).

Supports up to 4 sensors on one I2C bus by controlling XSHUT pins to
assign each sensor a unique address at boot.

Wiring:
    All sensors share SDA (GPIO 2) and SCL (GPIO 3).
    Each sensor needs its own XSHUT GPIO pin (see config.py / pinschart.md).
    Default I2C address: 0x29  (before reassignment).

Usage:
    s = DistanceSensor(xshut_pin=4)
    s.change_address(0x30)
    mm = s.read()   # distance in mm, or None on error
"""

import smbus2
from gpiozero import DigitalOutputDevice
import time


class DistanceSensor:
    """Raw-I2C driver for a single VL53L0X sensor."""

    # Key register addresses
    SYSRANGE_START        = 0x00
    RESULT_RANGE_STATUS   = 0x14
    SYSTEM_INTERRUPT_CLEAR = 0x0B
    I2C_ADDR_REG          = 0x8A   # Register used to reassign I2C address

    def __init__(self, xshut_pin: int):
        self.bus     = smbus2.SMBus(1)
        self.address = 0x29          # factory default
        self.xshut   = DigitalOutputDevice(xshut_pin)
        # Boot sequence: on → off (sensor starts powered-off)
        self.turn_on()
        time.sleep(0.1)
        self.turn_off()

    # ── Power control ─────────────────────────────────────────────────────────

    def turn_off(self) -> None:
        self.xshut.off()
        time.sleep(0.1)

    def turn_on(self) -> None:
        self.xshut.on()
        time.sleep(0.1)

    # ── Address assignment ────────────────────────────────────────────────────

    def change_address(self, new_address: int) -> None:
        """
        Power the sensor on and reprogram its I2C address.

        Must be called before any other sensor on the bus is powered on,
        so each sensor gets a unique address in sequence.
        """
        self.turn_on()
        if new_address != self.address:
            self.bus.write_byte_data(self.address, self.I2C_ADDR_REG, new_address)
            self.address = new_address
        time.sleep(0.1)

    # ── Measurement ───────────────────────────────────────────────────────────

    def read(self) -> int | None:
        """
        Trigger a single ranging measurement and return distance in mm.

        Returns None on I2C error.
        """
        try:
            self.bus.write_byte_data(self.address, self.SYSRANGE_START, 0x01)
            time.sleep(0.05)
            data = self.bus.read_i2c_block_data(
                self.address, self.RESULT_RANGE_STATUS + 10, 2)
            distance = (data[0] << 8) + data[1]
            self.bus.write_byte_data(self.address, self.SYSTEM_INTERRUPT_CLEAR, 0x01)
            return distance
        except OSError as e:
            print(f"[DIST] Sensor 0x{self.address:X} error: {e}")
            return None
