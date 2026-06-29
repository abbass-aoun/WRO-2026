# System Architecture

## Software Stack

```
┌─────────────────────────────────────────────────┐
│                   main.py                        │
│          (Race loop & state machine)             │
└──────┬────────┬──────────┬────────────┬──────────┘
       │        │          │            │
  ┌────▼──┐ ┌──▼────┐ ┌───▼───┐ ┌─────▼──────┐
  │Vision │ │Sensors│ │Control│ │Localization│
  │System │ │(dist, │ │(motor,│ │(EKF pose   │
  │(camera│ │color, │ │servo, │ │ estimator) │
  │+OpenCV│ │encoder│ │PID,   │           │
  │)      │ │)      │ │traj.) │            │
  └───────┘ └───────┘ └───────┘ └────────────┘
```

## Data Flow

```
Camera Frame
    │
    ▼
Vision System → detected pillars (x_cm, y_cm, color)
    │
    ▼
Trajectory Builder → Bézier path (waypoints, steering, speed)
    │
    ▼
Tracking Error check (utils/transforms.py)
    │
    ▼
CarController.set_all(direction, speed, steer)
    │
    ▼ (feedback)
Wheel Encoders + IMU → EKF Estimator → updated pose
```

## Race State Machine

```
INIT → WAIT_FOR_BUTTON → RACE_LOOP ──────────────────────┐
                              │                           │
                    ┌─────────▼──────────┐               │
                    │  In straight section│               │
                    │  Follow trajectory  │               │
                    └─────────┬──────────┘               │
                              │ (floor color change)      │
                    ┌─────────▼──────────┐               │
                    │  Corner detected    │               │
                    │  Update direction   │               │
                    └─────────┬──────────┘               │
                              │ (3 laps done)             │
                    ┌─────────▼──────────┐               │
                    │  Final section      │◄──────────────┘
                    │  Park or stop       │
                    └────────────────────┘
```

## Coordinate Frames

```
Global frame:   Fixed world origin at robot start position.
Local frame:    Resets at each section start (local_reference pose).
Robot FPV:      Camera frame — x=right, y=forward (used by Vision).
```
