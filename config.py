"""
config.py
---------
Central configuration for the WRO 2026 autonomous car.
All GPIO pins, PID gains, physical constants, and tuning
parameters live here — change values in one place only.
"""

# ── GPIO Pin Assignments (BCM numbering) ─────────────────────────────────────
class Pins:
    # I2C bus (shared by IMU and VL53L0X distance sensors)
    I2C_SDA            = 2
    I2C_SCL            = 3

    # VL53L0X distance sensor XSHUT pins (one per sensor)
    DIST_XSHUT_1       = 4
    DIST_XSHUT_2       = 18
    DIST_XSHUT_3       = 24
    DIST_XSHUT_4       = 25

    # Color sensor (TCS3200)
    COLOR_S0           = 17
    COLOR_S1           = 27
    COLOR_S2           = 22
    COLOR_S3           = 23
    COLOR_OUT          = 9

    # Wheel encoders (IR sensors)
    ENCODER_RIGHT      = 10   # D0 of right rear wheel encoder
    ENCODER_LEFT       = 26   # D0 of left rear wheel encoder

    # Steering servo (PWM)
    SERVO_STEERING     = 12

    # Drive motor (L298N / similar)
    MOTOR_IN1          = 14   # Direction pin 1
    MOTOR_IN2          = 15   # Direction pin 2
    MOTOR_ENA          = 21   # PWM speed pin

    # Start button
    START_BUTTON       = 16


# ── Physical Robot Parameters ─────────────────────────────────────────────────
class RobotParams:
    WHEELBASE_CM       = 18.0   # Distance between front and rear axles (cm)
    TRACK_WIDTH_CM     = 15.0   # Distance between left and right wheels (cm)
    WHEEL_RADIUS_CM    = 3.5    # Rear wheel radius (cm)
    ENCODER_PPR        = 20     # Pulses per revolution of wheel encoder
    CM_PER_PULSE       = (2 * 3.14159 * WHEEL_RADIUS_CM) / ENCODER_PPR

    # Servo limits (duty cycle %)
    SERVO_CENTER       = 7.5
    SERVO_LEFT_MAX     = 10.0
    SERVO_RIGHT_MAX    = 5.0


# ── PID Gains ────────────────────────────────────────────────────────────────
class PIDGains:
    # StopPID — used to brake the robot to a halt
    STOP_KP            = 1.2
    STOP_KI            = 0.0
    STOP_KD            = 0.2


# ── Vision / Detection Parameters ────────────────────────────────────────────
class VisionParams:
    CAMERA_WIDTH       = 640
    CAMERA_HEIGHT      = 480
    CAMERA_FPS         = 30

    # Focal length in pixels (calibrate for your lens)
    FOCAL_LENGTH_PX    = 600

    # Real-world pillar width in cm (used for distance estimation)
    PILLAR_WIDTH_CM    = 5.0

    # HSV color ranges  [H_low, S_low, V_low, H_high, S_high, V_high]
    HSV_RED_LOW        = [0,   120, 70,  10,  255, 255]
    HSV_RED_HIGH       = [170, 120, 70,  180, 255, 255]   # wraps around
    HSV_GREEN          = [40,  70,  70,  80,  255, 255]
    HSV_MAGENTA        = [140, 100, 100, 170, 255, 255]


# ── Track / Race Parameters ───────────────────────────────────────────────────
class TrackParams:
    TOTAL_LAPS         = 3
    SECTIONS_PER_LAP   = 4

    # How far off the trajectory before we recalculate (cm)
    TRACKING_ERROR_THRESHOLD = 5.0

    # Minimum seconds between section-change events (debounce)
    SECTION_DEBOUNCE_S = 1.0


# ── Trajectory Builder Parameters ────────────────────────────────────────────
class TrajParams:
    N_POINTS           = 100    # Number of sample points on each Bézier curve
    SEARCH_RADIUS_CM   = 10.0   # Nearest-point search radius on trajectory
    DEFAULT_SPEED      = 40     # Default motor speed (0-100)
    CORNER_SPEED       = 25     # Reduced speed through corners
