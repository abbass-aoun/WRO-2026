"""
sensors.py
----------
Unified interface for all on-robot sensors.

Wraps:
    DistanceSensor  — 4× VL53L0X ToF sensors (I2C, via alldistance.py)
    colorsensor     — TCS3200 floor colour sensor
    imusensor       — MPU-6050 accelerometer + gyro
    SteeringEncoder — potentiometer-based steering angle encoder
    WheelEncoder    — left and right IR wheel encoders

Each sensor initialises independently; failures are logged but do not crash
the system. Use the ready_* flags to check which sensors are online.

Usage:
    s = Sensors()
    print(s.read_distances())      # [d0, d1, d2, d3] in mm
    print(s.read_color())          # "blue" | "orange" | "red" | ...
    print(s.read_imu())            # [accel_dict, gyro_dict]
    print(s.read_steering_angle()) # degrees
    print(s.read_wheel_encoders()) # {"left_distance", "right_distance", ...}
"""

from sensors.alldistance     import DistanceSensor
from sensors.colorsensor     import read_frequency, detect_main_color
from sensors.imusensor       import sensor as imu_sensor
from sensors.steeringencoder import SteeringEncoder
from sensors.wheelencoder    import WheelEncoder


class Sensors:
    """Aggregates all sensor subsystems with graceful degradation."""

    # VL53L0X XSHUT pins and target I2C addresses
    DIST_PINS      = [17, 20, 22, 23]
    DIST_ADDRESSES = [0x29, 0x30, 0x31, 0x32]

    def __init__(self):
        # Readiness flags — True only if init succeeded
        self.ready_distance  = False
        self.ready_color     = False
        self.ready_imu       = False
        self.ready_steering  = False
        self.ready_wheel     = False

        # Distance sensors
        try:
            self.dist_sensors = []
            for i in range(4):
                s = DistanceSensor(self.DIST_PINS[i])
                s.change_address(self.DIST_ADDRESSES[i])
                self.dist_sensors.append(s)
            self.ready_distance = True
        except Exception as e:
            print(f"[SENSORS] Distance sensors init failed: {e}")

        # Colour sensor (just try a read to confirm it works)
        try:
            read_frequency(False, False)
            self.ready_color = True
        except Exception as e:
            print(f"[SENSORS] Colour sensor init failed: {e}")

        # IMU
        try:
            imu_sensor.get_accel_data()
            imu_sensor.get_gyro_data()
            self.ready_imu = True
        except Exception as e:
            print(f"[SENSORS] IMU init failed: {e}")

        # Steering encoder
        try:
            self._steering = SteeringEncoder()
            self.ready_steering = True
        except Exception as e:
            print(f"[SENSORS] Steering encoder init failed: {e}")

        # Wheel encoders
        try:
            self._wheels = WheelEncoder()
            self.ready_wheel = True
        except Exception as e:
            print(f"[SENSORS] Wheel encoders init failed: {e}")

        self.ready = all([self.ready_distance, self.ready_color,
                          self.ready_imu, self.ready_steering, self.ready_wheel])
        print(f"[SENSORS] Ready: {self.ready} | "
              f"dist={self.ready_distance} color={self.ready_color} "
              f"imu={self.ready_imu} steer={self.ready_steering} wheel={self.ready_wheel}")

    # ── Distance ──────────────────────────────────────────────────────────────

    def read_distances(self) -> list:
        """Return list of 4 distances in mm. None on sensor failure."""
        if not self.ready_distance:
            return [None] * 4
        return [s.read() for s in self.dist_sensors]

    # ── Colour ────────────────────────────────────────────────────────────────

    def read_color(self) -> str:
        """Return floor colour string: "blue", "orange", "red", "green", or None."""
        if not self.ready_color:
            return None
        return detect_main_color()

    # ── IMU ───────────────────────────────────────────────────────────────────

    def read_imu(self) -> list:
        """Return [accel_data, gyro_data] dicts from MPU-6050."""
        if not self.ready_imu:
            return None
        return [imu_sensor.get_accel_data(), imu_sensor.get_gyro_data()]

    # ── Steering encoder ──────────────────────────────────────────────────────

    def read_steering_angle(self) -> float:
        """Return current steering angle in degrees."""
        if not self.ready_steering:
            return None
        return self._steering.get_angle()

    # ── Wheel encoders ────────────────────────────────────────────────────────

    def read_wheel_encoders(self) -> dict:
        """Return dict with left/right distance (cm) and pulse counts."""
        if not self.ready_wheel:
            return None
        d_l, d_r = self._wheels.get_distances()
        return {
            "left_distance":  d_l,
            "right_distance": d_r,
            "left_pulses":    self._wheels.left_count,
            "right_pulses":   self._wheels.right_count,
        }

    def get_wheel_speeds(self) -> tuple:
        """Return (v_left, v_right) in cm/s."""
        if not self.ready_wheel:
            return 0.0, 0.0
        return self._wheels.get_speeds()

    # ── Combined ──────────────────────────────────────────────────────────────

    def read_all(self) -> list:
        """Return [distances, color, imu, steering_angle, wheel_encoders]."""
        return [
            self.read_distances(),
            self.read_color(),
            self.read_imu(),
            self.read_steering_angle(),
            self.read_wheel_encoders(),
        ]
