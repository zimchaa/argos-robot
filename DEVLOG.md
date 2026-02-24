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

---

## Session 2 — Hardware bring-up: tracks confirmed

### What was done
- Ran `i2cdetect -y 1` — Waveshare HAT confirmed at 0x40 (all-call at 0x70 also present, normal)
- Confirmed `smbus2` and `RPi.GPIO` installed on target Pi 4
- Ran `tests/test_tracks_manual.py` — both tracks spin and run in the correct direction at 70%
- Extended test script (renamed to `tests/test_all_motors_manual.py`) to cover arm joints (shoulder/elbow/wrist/gripper) at 25% / 0.5 s — no governor fitted yet

### What worked
- Waveshare Motor Driver HAT I2C comms verified (MODE1 = 0x11, oscillator running)
- Left track (Motor 0) and right track (Motor 1) both respond correctly, correct direction for positive speed
- `TrackedBase.forward()` drives both tracks together as expected

### Unresolved
- Arm joint directions not yet verified on hardware — run the extended test and note which direction each joint moves for +ve speed; swap `pin_a`/`pin_b` in `gpio_motor._MOTOR_PINS` for any that are reversed
- GPIO pin conflicts between stacked HATs not yet confirmed (tracks working suggests no conflict on I2C pins; GPIO pins still TBD)
- No governor / joint angle limits in place — arm testing must be done carefully at low speed

---

## Session 3 — Full motor test: all 6 motors verified

### What was done
- Ran `tests/test_all_motors_manual.py` with interactive reporting at 70% speed / 1s
- Confirmed all track and arm motor drivers communicate and respond correctly
- Discovered arm joint wiring did not match assumed motor ID order; corrected in `joints.py`

### What worked
- Both tracks spin forward with positive speed; correct direction confirmed
- Elbow (motor 2) and wrist (motor 3) correctly connected and correct polarity
- Gripper (physically on motor 1 terminal) responds and opens on positive speed
- Arm joint motor driver (GPIOMotor + MotorShield) confirmed functional

### Findings and fixes
- **Arm wiring swap**: motor 1 terminal → gripper, motor 4 terminal → shoulder
  (was assumed to be shoulder=1, gripper=4). Fixed in `argos/arm/joints.py` — no rewiring needed.
- **Track drift**: motor 1 (right track) appears mechanically slower than motor 0 at equal power.
  Noted for future compensation — will need feedback (IMU/encoder/visual) to correct automatically.
- **Shoulder (motor 4) no movement**: motor 4 terminal shows no movement. Possible causes:
  loose connector, mechanical bind, or faulty motor. Needs physical investigation before arm use.

### Unresolved
- Shoulder joint not moving — investigate connector and motor 4 terminal wiring
- Track drift to be quantified and compensated once odometry/feedback is available
- Joint directions (positive = which physical way) not yet fully characterised for all joints
