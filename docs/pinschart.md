# GPIO Pin Chart — WRO 2026 Car

All pin numbers use **BCM (Broadcom) numbering**.

## I2C Bus (shared)
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 2    | 3           | SDA — IMU + all VL53L0X sensors |
| 3    | 5           | SCL — IMU + all VL53L0X sensors |

## Distance Sensors (VL53L0X × 4)
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 4    | 7           | XSHUT — Sensor 1 (Front-Right 1) |
| 18   | 12          | XSHUT — Sensor 2 (Front-Right 2) |
| 24   | 18          | XSHUT — Sensor 3 (Front-Left 2)  |
| 25   | 22          | XSHUT — Sensor 4 (Front-Left 1)  |

> Each VL53L0X sensor shares the I2C bus (SDA/SCL). The XSHUT pins are
> pulled LOW one by one during boot to assign unique I2C addresses.

## Colour Sensor (TCS3200)
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 17   | 11          | S0 |
| 27   | 13          | S1 |
| 22   | 15          | S2 |
| 23   | 16          | S3 |
| 9    | 21          | OUT |

## Wheel Encoders (IR hall-effect)
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 10   | 19          | Right rear wheel encoder D0 |
| 26   | 37          | Left rear wheel encoder D0  |

## Steering
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 12   | 32          | Servo signal (PWM 50 Hz) |

## Drive Motor (L298N)
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 14   | 8           | IN1 — direction |
| 15   | 10          | IN2 — direction |
| 21   | 40          | ENA — speed (PWM) |

## Start Button
| GPIO | Physical Pin | Connected To |
|------|-------------|--------------|
| 16   | 36          | Push button signal (active LOW) |

## Power
| Pin  | Purpose |
|------|---------|
| 5V   | Servo, colour sensor, motor driver logic |
| 3.3V | VL53L0X sensors, encoders, IMU |
| GND  | All grounds |
