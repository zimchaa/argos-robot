# ARGOS Dev Log

## Session 1 — Phase 1: Core drivers

### What was done
- Established project structure: `argos/drivers/`, `argos/base/`, `argos/arm/`
- Read both vendor libraries before writing any code:
  - Waveshare Motor Driver HAT (`PCA9685.py` + `MotorDriver.py`)
  - SB Components MotorShield (GPIO-based, not I2C)
- Wrote `argos/drivers/pca9685.py` — unified PCA9685 driver using smbus2 only
  - `PCA9685`: low-level register access (set_pwm, set_duty, set_level, set_frequency)
  - `I2CMotor`: single-motor abstraction with `run(speed)` interface; motors on the same chip share a PCA9685 instance via module-level cache
- Wrote `argos/drivers/gpio_motor.py` — GPIO motor driver using RPi.GPIO
  - `GPIOMotor`: single-motor abstraction with identical `run(speed)` interface
  - Direction config uses neutral `pin_a`/`pin_b` names; swap in `_MOTOR_PINS` to reverse a motor without code changes
- Wrote `argos/base/tracks.py` — `TrackedBase` with `forward`, `backward`, `turn_left`, `turn_right`, `pivot_left`, `pivot_right`, `stop`
- Wrote `argos/arm/joints.py` — `RobotArm` with named joint attributes (`shoulder`, `elbow`, `wrist`, `gripper`), each a `GPIOMotor`

### Key design decisions
- Both `I2CMotor` and `GPIOMotor` expose identical `run(speed)` interface; speed is -100..100 signed int, no direction strings at driver level
- Waveshare HAT (I2C 0x40): motors 0 and 1 — tracks
- SB Components MotorShield (GPIO BOARD pins): motors 1–4 — arm joints
- SPI pins reused as GPIO on MotorShield (motors 3 and 4); safe with SPI disabled

### What worked
- Project structure and driver design complete and consistent
- No hardware testing yet (development platform: Pi 400, target: Pi 4 @ argos.local)

### Unresolved
- Hardware not yet connected; all code untested on physical hardware
- HAT 2 (SB Components) I2C address confirmed as N/A — it is GPIO-based
- GPIO pin conflict check between stacked HATs still TBD on hardware
- No i2cdetect run yet to confirm Waveshare HAT appears at 0x40
- `smbus2` and `RPi.GPIO` not yet installed on target Pi 4
