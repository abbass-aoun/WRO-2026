# WRO 2026 — Autonomous Car (Lebanon)

An autonomous robotic car built for the **Lebanese round of the World Robot
Olympiad (WRO) 2026 — Future Engineers** category.

The robot navigates a rectangular track for **3 laps**, avoids coloured traffic
pillars using computer vision, detects corners via a floor colour sensor, and
optionally executes a parking manoeuvre.

---

## Features

- **Computer vision pipeline** — detects red/green pillars and magenta parking
  markers using HSV colour segmentation (OpenCV).
- **Bézier-curve trajectory planner** — builds smooth paths around pillars
  following WRO rules (right of red, left of green).
- **Extended Kalman Filter (EKF)** — fuses wheel-encoder odometry with IMU
  yaw rate for accurate pose estimation.
- **PID braking controller** — brings the robot to a clean stop at the end.
- **Floor colour detection** — determines lap direction (CW/CCW) from orange
  and blue corner lines.
- **Modular, clean codebase** — each subsystem in its own package with full
  docstrings and type hints.

---

## Repository Structure

```
WRO-2026/
├── main.py                    # Entry point — race control loop
├── config.py                  # All GPIO pins, gains, and constants
├── requirements.txt           # Python dependencies
│
├── control/
│   ├── car_controller.py      # DC motor + steering servo driver
│   ├── stop_pid.py            # PID braking controller
│   └── trajectory_builder.py  # Bézier path planner
│
├── sensors/
│   ├── sensors.py             # Distance, colour, and encoder sensors
│   └── encoder_test.py        # Standalone encoder diagnostic tool
│
├── vision/
│   └── vision_system.py       # Camera capture + colour segmentation
│
├── localization/
│   └── ekf_estimator.py       # Extended Kalman Filter pose estimator
│
├── utils/
│   └── transforms.py          # Coordinate frame transforms & tracking helpers
│
└── docs/
    ├── pinschart.md           # Full GPIO wiring reference
    └── architecture.md        # System diagrams & data flow
```

---

## Hardware

| Component | Model / Notes |
|-----------|--------------|
| Compute   | Raspberry Pi 4 (or 3B+) |
| Camera    | Raspberry Pi Camera Module v2 |
| Drive motor | DC motor with L298N driver |
| Steering  | Servo motor (PWM on GPIO 12) |
| Distance  | 4× VL53L0X ToF sensors (I2C) |
| Floor colour | TCS3200 colour sensor |
| Wheel encoders | 2× IR hall-effect sensors |
| IMU       | MPU-6050 or similar (I2C) |
| Start button | Momentary push button |

See [`docs/pinschart.md`](docs/pinschart.md) for full GPIO wiring.

---

## Installation

### On the Raspberry Pi

```bash
# Clone the repository
git clone https://github.com/abbass-aoun/WRO-2026.git
cd WRO-2026

# Install Python dependencies
pip install -r requirements.txt

# (Optional) Enable I2C and camera via raspi-config
sudo raspi-config
```

### Dependencies

| Library | Purpose |
|---------|---------|
| `numpy` | Math, arrays |
| `scipy` | Optimisation helpers |
| `opencv-python` | Camera capture, colour segmentation |
| `gpiozero` | GPIO abstraction (motor, servo, buttons) |
| `RPi.GPIO` | Low-level GPIO for colour sensor |
| `smbus2` | I2C communication (sensors) |
| `keyboard` | Emergency keyboard quit |
| `picamera2` | Pi Camera v2 support |

---

## Usage

```bash
# Run the full autonomous race loop
python3 main.py

# Test wheel encoders only (diagnostic)
python3 sensors/encoder_test.py

# Press 'q' during a run to stop safely
```

---

## Configuration

All tunable parameters are in **`config.py`**:

- `Pins` — GPIO assignments (change here if you re-wire anything)
- `RobotParams` — wheelbase, wheel radius, encoder PPR
- `PIDGains` — braking PID kp/ki/kd
- `VisionParams` — camera resolution, focal length, HSV colour ranges
- `TrackParams` — laps, tracking error threshold, section debounce time
- `TrajParams` — Bézier sample points, search radius, speeds

---

## Calibration Steps

Before the first run:

1. **Colour sensor** — tune the thresholds in `sensors/sensors.py →
   read_color()` for your lighting conditions.
2. **Camera focal length** — measure `VisionParams.FOCAL_LENGTH_PX` using a
   known-distance target.
3. **Servo limits** — adjust `MAX` in `car_controller.py → set_steer()`.
4. **PID gains** — tune `PIDGains.STOP_KP/KI/KD` on a flat surface.
5. **VL53L0X addresses** — implement I2C address assignment in
   `sensors/sensors.py → read_distances()`.

---

## Status & TODOs

| Component | Status |
|-----------|--------|
| Main control loop | ✅ Complete |
| Vision / colour detection | ✅ Complete (calibration needed) |
| Trajectory builder | ✅ Complete |
| EKF localization | ✅ Skeleton complete (IMU integration needed) |
| Braking PID | ✅ Complete |
| Parking manoeuvre | 🚧 Stub — logic to be implemented |
| VL53L0X multi-sensor init | 🚧 Stub — I2C address init to be added |
| IMU integration | 🚧 Wire up MPU-6050 and feed yaw rate to EKF |

---

## Future Improvements

- Replace HSV segmentation with a lightweight YOLO model for more robust
  pillar detection under varying light.
- Add obstacle map building across multiple laps to refine trajectories.
- Implement proper VL53L0X multi-sensor I2C address assignment.
- Add a web dashboard (Flask) for live telemetry during testing.
- Implement the parking manoeuvre using distance sensor feedback.

---

## License

MIT — free to use and modify.
